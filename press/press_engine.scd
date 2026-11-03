/*
FU Press Scene — 縮む部屋 (pantomime) sound engine
================================================
見えない壁が上下から迫り、筒がバウンスで壁を可視化するシーンの音。

  1. bounce layer : バウンス1発 = 打音 (Klank)。
                    壁の間隔が狭いほど高く短く鳴る (部屋が縮む)
                    → 終盤は密度もピッチも自然にクレッシェンド。
                    上の壁 = 明るめ / 下の壁 = 鈍め。定位はロボット位置。
  2. air layer    : 圧力ドローン。間隔が狭まるほど低い唸り + 鳴きが迫り上がる。
                    壁が消えた瞬間に断ち切られる。
  3. (ラトル層は廃止 2026-11-03: 作り物のジリジリはわざとらしい。
      潰しの音 = 実際のマイクロバウンスの連打そのもの)
  4. echo         : 壁がある間、バウンスの響きは封じられている (ゲート 0.1)。
                    /press/release でゲートが開き、溜まった残響が
                    revtime 9s の大伽藍となってあふれ出す → 自然減衰。
  6. roll         : CDJのループ半分ロール。隙間が半分になるたびレートが
                    倍になる量子化された刻み (3.2m:1Hz → 0.1m:32Hz)。
                    終盤は音程化し、静止(振幅0)と対になるライザー。
  5. drone        : 壁の位置に同期した2声のアンビエントグリッサンド。
                    上壁 = 高い声が降りてくる / 下壁 = 低い声が上がってくる。
                    潰し高さでほぼユニゾンに合流してうなり、release で消える。

OSC protocol (received on sclang default port 57120):
  /press/scene    int              1 = fade in, 0 = fade out
  /press/gap      gap_m, progress, top_z, bot_z
                  壁の中心間隔 [m], 閉じ具合 0..1, 上壁/下壁の高さ [m] (5 Hz)
  /press/bounce   idx, vel, gap, wall, freq_hz
                  打撃 (idx 0-based, vel 0..1, gap [m], wall 1=上/0=下,
                   freq_hz = 筒の個体音程。鳴るたびにモデル側で上がる。頭打ちなし)
  /press/squeeze  dur, gap         潰し開始 (現在は未使用。音は実バウンスの連打)
  /press/release  -                壁消滅: エコー解放
  /press/reset    -                次サイクル: エコーを再び封じる

設定・配置は territory/ の config.json / robots.json を共有。
territory_engine.scd の末尾からも load される (同一 sclang に /terr 系と同居、
環境変数は ~pr* で衝突しない)。単体起動:
  /Applications/SuperCollider.app/Contents/MacOS/sclang press_engine.scd
*/

(
var dir = thisProcess.nowExecutingPath.dirname;
var terrDir = dir.dirname +/+ "territory";
var config = File.readAllString(terrDir +/+ "config.json").parseJSON;
var robotsFile = File.readAllString(terrDir +/+ "robots.json").parseJSON;
var audioCfg = config["audio"];
var devStereo = (audioCfg["dev_stereo"].asString == "true");
var fadeTime = audioCfg["scene_fade_s"].asFloat;
var masterDb = audioCfg["master_gain_db"].asFloat;
var chanMap = audioCfg["speaker_channels"];
var hwChans = ["FL", "FR", "RL", "RR"].collect { |k| chanMap[k].asInteger };
var robots = robotsFile["robots"].collect { |r|
	(id: r["id"].asInteger, x: r["x"].asFloat, y: r["y"].asFloat)
};

~prNBots = robots.size;
~prFade = fadeTime;

// bilinear pan weights per robot, order [FL, FR, RL, RR]
~prPanW = robots.collect { |r|
	[ (1 - r.x) * (1 - r.y), r.x * (1 - r.y), (1 - r.x) * r.y, r.x * r.y ]
};

// ---------------- layer gains (tune here) ----------------
~prBounceGain = 0.55;   // バウンス打音
~prEchoSend   = 0.55;   // 打音からエコー槽への送り
~prAirGain    = 0.16;   // 圧力ドローン
~prDroneGain  = 0.45;   // 壁同期グリッサンドドローン
~prRollGain   = 0.0;    // CDJロール層 (全体で1個の刻みは自律感と逆行するのでミュート中。
                        //  序盤1Hzがメトロノームに聞こえる。戻すなら 0.3)
~prCrushZ     = 1.5;    // 両声が出会う高さ [m] (main.py の press_crush_z と合わせる)
~prEchoGain   = 0.9;    // エコー槽の出力 (解放時)
~prEchoClosed = 0.10;   // 壁がある間のエコー漏れ (密室感。0で完全に封じる)
~prRevClosed  = 3.0;    // 壁がある間の残響時間 [s]
~prRevOpen    = 9.0;    // 解放時の残響時間 [s]

s.options.numOutputBusChannels = if(devStereo, 2, max(4, hwChans.maxItem + 1));
s.options.numInputBusChannels = 0;
~prOutDevice = audioCfg["output_device"];
if(~prOutDevice.notNil and: { ~prOutDevice.asString != "" }) {
	s.options.outDevice = ~prOutDevice.asString;
	s.options.inDevice = ~prOutDevice.asString;
};
~prSr = "TERR_SAMPLE_RATE".getenv ? audioCfg["sample_rate"];
if(~prSr.notNil) { s.options.sampleRate = ~prSr.asFloat };

s.waitForBoot {
	// ---------------- buses ----------------
	~prMainBus = Bus.audio(s, 4);
	~prEchoBus = Bus.audio(s, 2);
	~prGapBus  = Bus.control(s, 1);   // 壁の中心間隔 [m]
	~prProgBus = Bus.control(s, 1);   // 閉じ具合 0..1
	~prTopBus  = Bus.control(s, 1);   // 上壁の高さ [m]
	~prBotBus  = Bus.control(s, 1);   // 下壁の高さ [m]
	~prGainBus = Bus.control(s, 1);   // master gain dB
	~prGapBus.set(3.0);
	~prProgBus.set(0.0);
	~prTopBus.set(3.45);
	~prBotBus.set(0.14);
	~prGainBus.set(masterDb);

	// ---------------- bounce: 1発の打音。狭いほど高く短い ----------------
	SynthDef(\prBounce, { |out = 0, echoOut = 0, amp = 0.5, vel = 0.7, gap = 2.0,
		wall = 1, wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25, echoSend = 0.5,
		freq = 0|
		var g = gap.clip(0.09, 2.9);
		// 音程 = 筒の個体音程 freq (鳴るたびにモデル側で上がる。頭打ちなし:
		// 不可聴域まで上がり続ける。25Hz/20kHzはDSP安全枠のみ)。
		// freq=0 のときだけ旧・空気柱式 f=c/(2L) にフォールバック
		var f = Select.kr(freq > 0, [
			(343.0 / (2.0 * g)).clip(55, 1715), freq ]).clip(25, 21000);
		var ring = g.linlin(0.30, 2.9, 0.16, 0.8);         // 広いうちは長く鳴る
		var bright = wall.linlin(0, 1, 0.82, 1.22);        // 上の壁は少し明るい
		var exc = PinkNoise.ar * Env.perc(0.0005, 0.012, 1, -4).ar;
		// 高音域では太鼓の倍音列がナイキストで折り返して上昇が止まって聞こえる。
		// 800Hz→6kHz で打体→純音ピングにモーフし、21kHz まで濁りなく上がって消える
		var hi = f.explin(800, 6000, 0, 1);
		var body = Klank.ar(
			`[ [1, 1.83, 2.76, 4.19], [1, 0.52, 0.28, 0.12], [1, 0.74, 0.55, 0.4] ],
			exc, freqscale: (f * bright).min(4800), decayscale: ring);
		var ping = SinOsc.ar(f) * EnvGen.ar(Env.perc(0.0008, 0.05 + (ring * 0.1), 1, -6));
		var sig = ((body * (1 - hi)) + (ping * hi * 0.9)).softclip
			* (amp * vel.linlin(0, 1, 0.45, 1.0));
		DetectSilence.ar(sig, 0.0008, 0.2, doneAction: 2);
		Out.ar(out, sig * [wFL, wFR, wRL, wRR]);
		Out.ar(echoOut, sig * echoSend * [wFL + wRL, wFR + wRR]);
	}).add;

	// ---------------- air: 圧力ドローン (狭いほど唸り + 鳴きが迫る) ----------------
	SynthDef(\prAir, { |out = 0, gapBus = 0, on = 1, airGain = 0.16|
		var gap = Lag.kr(In.kr(gapBus), 0.5).clip(0.09, 3.0);
		var close = gap.linlin(3.0, 0.4, 0.0, 1.0);
		var gate = LagUD.kr(on, 0.8, 0.04);                 // 解放では即座に断つ
		var rumble = LPF.ar(BrownNoise.ar, 85 + (close * 55));
		var cry = BPF.ar(PinkNoise.ar, (343.0 / (2.0 * gap)).clip(55, 1715), 0.22)
			* close.pow(2.0) * 2.2;   // 鳴きも同じ空気柱の共鳴に乗る
		var sig = ((rumble * 0.8) + cry) * close.pow(1.6) * airGain * gate;
		Out.ar(out, sig * [1.0, 0.92, 0.96, 0.88]);
	}).add;

	// ---------------- drone: 壁同期の2声グリッサンド (アンビエント) ----------------
	// 高さ→音高 (0.6 oct/m): 上壁の声は降り、下壁の声は上がり、
	// crushZ (潰し高さ) で両者とも ~110Hz に合流してうなる
	SynthDef(\prDrone, { |out = 0, echoOut = 0, amp = 0.24, topBus = 0, botBus = 0,
		crushZ = 1.5, on = 1|
		var top = Lag.kr(In.kr(topBus), 1.5);
		var bot = Lag.kr(In.kr(botBus), 1.5);
		var gate = LagUD.kr(on, 7.0, 2.5);   // ゆっくり立ち上がり、release で滑らかに消える
		// デチューンなし (2026-11-03 ユーザー指定「うねうね不要」): 純粋な倍音構成のみ
		var fTop = (110 * (2 ** ((top - crushZ) * 0.6))).clip(40, 900);
		var fBot = (110 * (2 ** ((bot - crushZ) * 0.6))).clip(40, 900);
		var vTop = LPF.ar(
			SinOsc.ar(fTop, 0, 0.9)
			+ LFTri.ar(fTop * 2.0, 0, 0.14),
			fTop * 3.0 + 150) * LFNoise1.kr(0.07).range(0.82, 1.0);
		// 下の声は低域(62→107Hz)だけだと埋もれる: 倍音×2,×3を足して
		// 上昇が中域(186→321Hz)でも聞こえるようにする
		var vBot = LPF.ar(
			SinOsc.ar(fBot, 0, 0.9)
			+ SinOsc.ar(fBot * 2.0, 0, 0.4) + SinOsc.ar(fBot * 3.0, 0, 0.2)
			+ LFTri.ar(fBot * 0.5, 0, 0.14),
			fBot * 6.0 + 150) * LFNoise1.kr(0.06).range(0.82, 1.0);
		var sig = ((vTop * 0.5) + (vBot * 0.5)) * amp * gate;
		// 上の声はやや前、下の声はやや後ろに置いて空間を分ける
		Out.ar(out, (vTop * 0.5 * amp * gate * [0.55, 0.5, 0.35, 0.33])
			+ (vBot * 0.5 * amp * gate * [0.35, 0.33, 0.55, 0.5]));
		Out.ar(echoOut, sig * 0.2 ! 2);
	}).add;

	// ---------------- roll: CDJのループ半分 (gap半減ごとにレート倍) ----------------
	SynthDef(\prRoll, { |out = 0, echoOut = 0, amp = 0.3, gapBus = 0, progBus = 0,
		on = 1, ref = 3.2, r0 = 1.0|
		var gap = In.kr(gapBus).clip(0.1, 3.4);
		var prog = Lag.kr(In.kr(progBus), 0.5).clip(0, 1);
		var gate = LagUD.kr(on, 1.0, 0.03);
		var oct = log2(ref / gap).floor;                 // 半分になるたび +1 (量子化)
		var rate = (r0 * (2 ** oct)).clip(r0, 64);
		var f = (343.0 / (2.0 * gap)).clip(55, 1715);    // 刻みの音色は太鼓と同じ空気柱
		var sig = Ringz.ar(Impulse.ar(rate) * 0.3, f * [1, 1.83], [0.08, 0.05]).sum;
		sig = (sig * (prog ** 1.5) * amp * gate).softclip;
		Out.ar(out, sig * [0.5, 0.5, 0.45, 0.45]);
		Out.ar(echoOut, sig * 0.3 ! 2);
	}).add;

	// ---------------- echo: 封じられた残響槽。解放で開く ----------------
	SynthDef(\prEcho, { |bus = 0, out = 0, open = 0.1, revtime = 3, echoGain = 0.9|
		var dry = In.ar(bus, 2);
		var wet = GVerb.ar(dry.sum * 0.5, roomsize: 80,
			revtime: Lag.kr(revtime, 0.2), damping: 0.35,
			inputbw: 0.6, drylevel: 0, earlyreflevel: 0.4, taillevel: 1.0,
			maxroomsize: 120);
		var g = Lag.kr(open, 0.3) * echoGain;
		ReplaceOut.ar(bus, Silent.ar(2));   // 槽への入力はここで消費
		Out.ar(out, [wet[0], wet[1], wet[1] * 0.9, wet[0] * 0.9] * g);
	}).add;

	// ---------------- output (territory エンジンと同形) ----------------
	SynthDef(\prOut4, { |mainBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 3,
		cFL = 0, cFR = 1, cRL = 2, cRR = 3|
		var sig  = In.ar(mainBus, 4);
		var gain = Lag.kr(In.kr(gainBus), 0.3).dbamp;
		var env  = EnvGen.kr(Env.asr(fadeTime, 1, fadeTime, \sine), gateLevel);
		sig = Limiter.ar(sig * gain * env, 0.95);
		Out.ar(cFL, sig[0]); Out.ar(cFR, sig[1]);
		Out.ar(cRL, sig[2]); Out.ar(cRR, sig[3]);
	}).add;

	SynthDef(\prOutStereo, { |mainBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 3|
		var sig  = In.ar(mainBus, 4);
		var gain = Lag.kr(In.kr(gainBus), 0.3).dbamp;
		var env  = EnvGen.kr(Env.asr(fadeTime, 1, fadeTime, \sine), gateLevel);
		sig = Limiter.ar(sig * gain * env, 0.95);
		Out.ar(0, [sig[0] + (sig[2] * 0.85), sig[1] + (sig[3] * 0.85)] * 0.7);
	}).add;

	s.sync;

	// ---------------- node graph ----------------
	~prSrcGroup = Group.new(s);
	~prAirSynth = Synth.new(\prAir, [
		\out, ~prMainBus, \gapBus, ~prGapBus, \on, 1, \airGain, ~prAirGain
	], ~prSrcGroup, \addToTail);
	~prRollSynth = Synth.new(\prRoll, [
		\out, ~prMainBus, \echoOut, ~prEchoBus,
		\amp, ~prRollGain, \gapBus, ~prGapBus, \progBus, ~prProgBus,
		\ref, 3.2, \r0, 1.0, \on, 1
	], ~prSrcGroup, \addToTail);
	~prDroneSynth = Synth.new(\prDrone, [
		\out, ~prMainBus, \echoOut, ~prEchoBus,
		\amp, ~prDroneGain, \topBus, ~prTopBus, \botBus, ~prBotBus,
		\crushZ, ~prCrushZ, \on, 1
	], ~prSrcGroup, \addToTail);
	~prEchoSynth = Synth.new(\prEcho, [
		\bus, ~prEchoBus, \out, ~prMainBus,
		\open, ~prEchoClosed, \revtime, ~prRevClosed, \echoGain, ~prEchoGain
	], ~prSrcGroup, \addAfter);
	~prOutSynth = Synth.new(
		if(devStereo, \prOutStereo, \prOut4),
		[ \mainBus, ~prMainBus, \gainBus, ~prGainBus,
		  \gateLevel, 0, \fadeTime, ~prFade,
		  \cFL, hwChans[0], \cFR, hwChans[1], \cRL, hwChans[2], \cRR, hwChans[3] ],
		~prEchoSynth, \addAfter);

	// ---------------- OSC receivers ----------------
	OSCdef(\prGap, { |msg|
		~prGapBus.set(msg[1].asFloat);
		~prProgBus.set(msg[2].asFloat.clip(0, 1));
		if(msg.size > 4) {
			~prTopBus.set(msg[3].asFloat);
			~prBotBus.set(msg[4].asFloat);
		};
	}, '/press/gap');

	OSCdef(\prBounceRx, { |msg|
		var i    = msg[1].asInteger;
		var vel  = msg[2].asFloat.clip(0, 1);
		var gap  = msg[3].asFloat;
		var wall = msg[4].asInteger;
		var freq = if(msg.size > 5) { msg[5].asFloat } { 0.0 };
		if((i >= 0) and: { i < ~prNBots }) {
			Synth.new(\prBounce, [
				\out, ~prMainBus, \echoOut, ~prEchoBus,
				\amp, ~prBounceGain, \vel, vel, \gap, gap, \wall, wall, \freq, freq,
				\wFL, ~prPanW[i][0], \wFR, ~prPanW[i][1],
				\wRL, ~prPanW[i][2], \wRR, ~prPanW[i][3],
				\echoSend, ~prEchoSend
			], ~prSrcGroup, \addToTail);
		};
	}, '/press/bounce');

	// /press/squeeze は受けるが何もしない (ラトル層は廃止。音は実バウンスの連打)

	OSCdef(\prRelease, { |msg|
		~prAirSynth.set(\on, 0);                            // 圧は断ち切る
		~prDroneSynth.set(\on, 0);                          // 壁が消えればグリスも消える
		~prRollSynth.set(\on, 0);                           // ロールも断ち切る
		~prEchoSynth.set(\revtime, ~prRevOpen, \open, 1.0); // 溜めた響きを解放
	}, '/press/release');

	OSCdef(\prReset, { |msg|
		~prAirSynth.set(\on, 1);
		~prDroneSynth.set(\on, 1);
		~prRollSynth.set(\on, 1);
		~prEchoSynth.set(\revtime, ~prRevClosed, \open, ~prEchoClosed);
	}, '/press/reset');

	OSCdef(\prScene, { |msg|
		~prOutSynth.set(\gateLevel, if(msg[1].asInteger > 0, 1, 0));
	}, '/press/scene');

	// ---------------- helpers ----------------
	~prSceneStart = { ~prOutSynth.set(\gateLevel, 1) };
	~prSceneStop  = { ~prOutSynth.set(\gateLevel, 0) };

	"FU press engine ready: % bots, % output, OSC on port %"
	.format(~prNBots, if(devStereo, "stereo (dev fold)", "4ch"), NetAddr.langPort).postln;
};
)

/* ---------------------------------------------------------------------------
manual test snippets (no Python needed) — evaluate line by line:

~prSceneStart.();

// 広い部屋のまばらなバウンス → 狭い部屋の連打
NetAddr.localAddr.sendMsg('/press/gap', 2.6, 0.1);
NetAddr.localAddr.sendMsg('/press/bounce', 3, 0.8, 2.6, 1);
fork { 20.do { |k| var gap = 2.6 - (k * 0.11);
	NetAddr.localAddr.sendMsg('/press/gap', gap, k / 20);
	NetAddr.localAddr.sendMsg('/press/bounce', 25.rand, 0.7, gap, 2.rand);
	(0.5 - (k * 0.02)).wait } };

// 潰し → 解放 (エコーがあふれる)
NetAddr.localAddr.sendMsg('/press/squeeze', 2.2, 0.36);
2.5.wait への代わりに手で:
NetAddr.localAddr.sendMsg('/press/release');

// 次のサイクル
NetAddr.localAddr.sendMsg('/press/reset');

~prSceneStop.();
--------------------------------------------------------------------------- */
