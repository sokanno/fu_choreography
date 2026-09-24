/*
FU Territory Scene — 陣取り (opinion dynamics) sound engine
===========================================================
29 ceiling robots = 29 drone voices + rotation ticks. The politics IS the
harmony: the opinion circle maps onto the circle of fifths, so opinion
distance = harmonic distance. Opposing camps sit a tritone apart; nearby
opinions are a fifth; consensus fuses to unison.

  1. drone layer : one persistent voice per robot.
                   opinion angle -> pitch class (circle of fifths, 3 registers)
                   conviction    -> voice amp + filter opening (undecided = breath)
                   fatigue       -> detune + tremor (burned-out voices crack)
                   height        -> proximity (louder + brighter when descended)
                   tilt          -> +down = open/projecting, -up = muffled/averted
                   |yaw vel|     -> air swish (you hear the tubes turning)
  2. tick layer  : a knock each time a tube's yaw crosses a half-turn.
                   scanning room = scattered clockwork; Kuramoto-synced
                   headings phase-lock the knocks into a march by themselves.
  3. mass layer  : dark unison sub that rises with R*C (the totalitarian mass)
  4. shift       : axis-jump gesture — low boom + air swell; drones re-tune
                   on their own (pitch lag = glissando into the new war)

OSC protocol (received on sclang default port 57120):
  /terr/op      29 x float  UNWRAPPED opinion angle rad (20 Hz, bundled)
  /terr/conv    29 x float  conviction 0..1                (20 Hz)
  /terr/fat     29 x float  fatigue 0..1                   (20 Hz)
  /terr/ht      29 x float  height 0=ceiling..1=descended  (20 Hz)
  /terr/tilt    29 x float  pitch tilt -1..+1 (= +-60 deg, + is down) (20 Hz)
  /terr/om      29 x float  yaw angular velocity rad/s     (20 Hz)
  /terr/global  5 x float   R, C, camps, mean_op_angle, master_gain_db (5 Hz)
  /terr/tick    3 x arg     robot idx (0-based), vel 0..1, pitch-class int
  /terr/shift   1 x int     1 = auto-triggered, 0 = manual
  /terr/scene   int         0 = fade out, 1 = fade in

Usage: open in SC IDE and evaluate the whole file (Cmd-A, Cmd-Enter),
or headless:  /Applications/SuperCollider.app/Contents/MacOS/sclang territory_engine.scd
Manual test snippets at the bottom.
*/

(
var dir = thisProcess.nowExecutingPath.dirname;
var config = File.readAllString(dir +/+ "config.json").parseJSON;
var robotsFile = File.readAllString(dir +/+ "robots.json").parseJSON;
var audioCfg = config["audio"];
var devStereo = (audioCfg["dev_stereo"].asString == "true");
var fadeTime = audioCfg["scene_fade_s"].asFloat;
var masterDb = audioCfg["master_gain_db"].asFloat;
var chanMap = audioCfg["speaker_channels"];
var hwChans = ["FL", "FR", "RL", "RR"].collect { |k| chanMap[k].asInteger };
var robots = robotsFile["robots"].collect { |r|
	(id: r["id"].asInteger, x: r["x"].asFloat, y: r["y"].asFloat)
};

~nBots = robots.size;   // robots.json の台数 (NMW 25 / FU 29)
~fadeTime = fadeTime;

// bilinear pan weights per robot, order [FL, FR, RL, RR]
~panW = robots.collect { |r|
	[ (1 - r.x) * (1 - r.y), r.x * (1 - r.y), (1 - r.x) * r.y, r.x * r.y ]
};

// ---------------- layer gains (tune here) ----------------
~droneGain = 0.16;
~sayGain   = 0.55;
~sayDurScale = 0.5; // しゃべり層の1音の長さの倍率・天井付近 (モデルは0.4〜2.8秒を出す。2026-09-24: 長すぎ→0.5)
~sayDurLow = 1.5;   // 同・最下(1.9m)まで降りたときの倍率。高さに比例して間をつなぐ (2026-09-24)
~htPitchSemis = 12; // 降りるほど音程を下げる量(半音)。ht 0=天井(4.3m)で0、ht 1=最下(1.9m)で -この値。0で無効 (2026-09-24)
~crescK = 2.0;      // 底に近づくほどのクレッシェンド: 音量 ×(1 + この値 × ht^3)。ht 1(最下)で ×3 (+9.5dB) (2026-09-24)
~landGain = 0.0;    // 筒が底に着いた瞬間の「ドン」。0で無効 (2026-09-24: 爆弾っぽいので無効。戻すなら0.6)
~cutHt = 0.85;      // 降りきった判定: ここで声がスパッと切れて残響だけ残る
~reopenHt = 0.35;   // 切れた声は、ここまで戻ったら(天井側)また鳴り出す。戻る途中は無音
~htSurface = 0.75;  // 降りてきた筒(=対立の前線)のドローンを浮上させる量。ht 0.5→1 で 0→この値までゲートが開き、低音(サブ)も太る。0で無効
~tickGain  = 0.0;   // ノック層は休眠中(声の指向性に置換、戻すなら0.5)
~massGain  = 0.4;
~shiftGain = 0.5;
~meltGain  = 0.0;   // 溶解の音 (ノイズが10秒かけて下降+落ちる音程)。爆弾の落下音に聞こえるので無効 (2026-09-24。戻すなら0.4)
~splitGain = 0.45;
~braamGain = 0.0;   // 結晶化の瞬間の「ドゥーン」(映画トレイラー風 BRAAAM)。爆発に聞こえるので無効 (2026-09-24。戻すなら0.55)
~revMix    = 0.5;
~baseMidi  = 39;    // register 0 root; registers at +0 / +12 / +24

s.options.numOutputBusChannels = if(devStereo, 2, max(4, hwChans.maxItem + 1));
s.options.numInputBusChannels = 0;

// 出力デバイス指定(config.json "output_device")。null=システム既定。
// Max経由構成では "BlackHole 16ch" にして、Maxがアグリゲートデバイスで受ける
~outDevice = audioCfg["output_device"];
if(~outDevice.notNil and: { ~outDevice.asString != "" }) {
	s.options.outDevice = ~outDevice.asString;
	// 入力は使わないが、未指定だとシステム既定入力(MOTU/RME)を開いてMaxと取り合うので同じデバイスに
	s.options.inDevice = ~outDevice.asString;
	("output device: " ++ ~outDevice).postln;
};
// サンプルレート(config.json "sample_rate")。null=デバイス任せ。
// Maxのアグリゲートと揃える(ずれるとBlackHoleのレートが途中で切り替わる)
// 環境変数 TERR_SAMPLE_RATE があればそちらを優先(start_territory_sound.sh 44100 で設定される)
~sr = "TERR_SAMPLE_RATE".getenv ? audioCfg["sample_rate"];
if(~sr.notNil) {
	s.options.sampleRate = ~sr.asFloat;
	("sample rate: " ++ s.options.sampleRate).postln;
};

s.waitForBoot {
	// ---------------- buses ----------------
	~opBus   = Bus.control(s, ~nBots);  // unwrapped opinion angle (rad)
	~convBus = Bus.control(s, ~nBots);  // conviction 0..1
	~fatBus  = Bus.control(s, ~nBots);  // fatigue 0..1
	~htBus   = Bus.control(s, ~nBots);  // height 0..1
	~tiltBus = Bus.control(s, ~nBots);  // tilt -1..1
	~omBus   = Bus.control(s, ~nBots);  // |yaw vel| rad/s
	~faceBus = Bus.control(s, ~nBots);  // 0..1 head points at the audience
	~vgBus   = Bus.control(s, ~nBots);  // 各筒の声のゲート (ドローンが書き、しゃべりが読む)

	~rBus      = Bus.control(s, 1);     // heading order parameter R
	~cBus      = Bus.control(s, 1);     // opinion coherence C
	~meanOpBus = Bus.control(s, 1);     // mean opinion angle (rad)
	~gainBus   = Bus.control(s, 1);     // master gain dB

	~mainBus = Bus.audio(s, 4);
	~recBus  = Bus.audio(s, 2);

	~convBus.setn(0.0 ! ~nBots);
	~htBus.setn(0.5 ! ~nBots);
	~faceBus.setn(0.5 ! ~nBots);
	~vgBus.setn(1.0 ! ~nBots);
	~gainBus.set(masterDb);
	~lastHt = 0.5 ! ~nBots;

	// ---------------- drone layer: one persistent voice per robot ----------------
	SynthDef(\terrDrone, {
		|out = 0, idx = 0, reg = 0, baseMidi = 39,
		wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25,
		opBus = 0, convBus = 0, fatBus = 0, htBus = 0, tiltBus = 0, omBus = 0,
		faceBus = 0, cBus = 0, droneGain = 0.16, htSemis = 5, htSurface = 0.75, crescK = 2,
		vgBus = 0, cutHt = 0.85, reopenHt = 0.35|

		var op   = In.kr(opBus + idx);
		var conv = Lag.kr(In.kr(convBus + idx), 0.25).clip(0, 1);
		var fat  = Lag.kr(In.kr(fatBus + idx), 0.6).clip(0, 1);
		var ht   = Lag.kr(In.kr(htBus + idx), 0.5).clip(0, 1);
		// 降下の声 (2026-09-24): 降りる間はクレッシェンドして底でピーク → 降りきった瞬間に
		// スパッと切れて残響だけ残る → 戻る(上がる)間は無音 → 天井側まで戻ったら再び鳴る
		var htRaw  = In.kr(htBus + idx).clip(0, 1);
		var cutOn  = SetResetFF.kr(htRaw > cutHt, htRaw < reopenHt);
		var rising = LagUD.kr((Slope.kr(Lag.kr(htRaw, 0.2)) < -0.02) * (htRaw > reopenHt), 0.05, 0.6);
		var vg     = LagUD.kr((1 - cutOn) * (1 - rising), 0.8, 0.025);   // 切れるのは一瞬、戻りはゆっくり
		var tilt = Lag.kr(In.kr(tiltBus + idx), 0.5).clip(-1, 1);
		var om   = Lag.kr(In.kr(omBus + idx).abs, 0.15);
		// voice directivity: the head is a beam — facing the audience the
		// voice opens, turned away it veils. rotation = the beam sweeping.
		var face = Lag.kr(In.kr(faceBus + idx), 0.1).clip(0, 1);

		// opinion angle -> circle-of-fifths pitch class, quantized then lagged:
		// the lag in midi space is the glissando between chord tones
		var p    = op / 2pi * 12;
		var pc   = (p.round.mod(12) * 7).mod(12);
		var midi = Lag.kr(baseMidi + (reg * 12) + pc, 0.5) - (htSemis * ht);
		var freq = midi.midicps;

		// fatigue -> detune spread + slow pitch waver + amplitude tremor
		var dd     = fat.linlin(0, 1, 0.0012, 0.014);
		var waver  = 1 + (fat * 0.006 * LFNoise2.kr(1.3));
		var f1     = freq * (1 + dd) * waver;
		var f2     = freq * (1 - dd);

		// timbre follows the COLOR: anchors on the same hue mapping as the
		// visuals (angMap: upper半円 red->yellow->green, lower red->magenta->blue)
		//   red 0        = saw      (hot, coarse)
		//   green 2pi/3  = square   (hollow, reedy)   yellow = saw+square blend
		//   blue 4pi/3   = sine     (clear, cold)
		//   magenta 5pi/3= inharmonic FM (artificial, tense)
		var aw   = op.wrap(-pi, pi);
		var hue  = ((aw >= 0) * aw * (2/3)) + ((aw < 0) * aw * (4/3));
		var wRd  = (1 - ((hue - 0).wrap(-pi, pi).abs / (2pi/3))).max(0);
		var wGn  = (1 - ((hue - (2pi/3)).wrap(-pi, pi).abs / (2pi/3))).max(0);
		var wBl  = (1 - ((hue - (4pi/3)).wrap(-pi, pi).abs / (pi/3))).max(0);
		var wMg  = (1 - ((hue - (5pi/3)).wrap(-pi, pi).abs / (pi/3))).max(0);
		var wSum = (wRd + wGn + wBl + wMg).max(0.01);

		var sawS  = (VarSaw.ar(f1, 0, 0.05) + VarSaw.ar(f2, 0.4, 0.05)) * 0.33;
		var sqrS  = (Pulse.ar(f1, 0.5) + Pulse.ar(f2, 0.5)) * 0.26;
		var pureS = (SinOsc.ar(f1) + SinOsc.ar(f2)) * 0.42
		            + (SinOsc.ar(f1 * 2) * 0.08);
		var metS  = (SinOsc.ar(f1, SinOsc.ar(f1 * 2.41, 0, 2.5))
		           + SinOsc.ar(f2, SinOsc.ar(f2 * 2.39, 0, 2.5))) * 0.3;

		// spectral handover: each timbre owns a HPF swept by its hue weight —
		// a rising timbre descends from the opened top (shimmer first, body
		// later), a fading one thins upward into air. mid-morph both sit
		// half-open, so the transition itself is audible as a filter gesture.
		var body = ((
			  (HPF.ar(sawS,  wRd.linexp(0, 1, 9000, 80)) * wRd)
			+ (HPF.ar(sqrS,  wGn.linexp(0, 1, 9000, 80)) * wGn)
			+ (HPF.ar(pureS, wBl.linexp(0, 1, 9000, 80)) * wBl)
			+ (HPF.ar(metS,  wMg.linexp(0, 1, 9000, 80)) * wMg)
		) / wSum) + (SinOsc.ar(freq * 0.5) * (0.3 + (0.5 * ((ht - 0.5) * 2).clip(0, 1))));
		var tremor = 1 - (fat * 0.45 * LFNoise2.kr(5.5).range(0, 1));

		// a tube whose opinion is in motion hums audibly while it transforms,
		// so the sweep is foregrounded exactly when the color slides
		var trans = LagUD.kr((Slope.kr(Lag.kr(op, 0.15)).abs / 1.5).clip(0, 1),
			0.1, 1.5);

		// conviction opens the filter; tilt down projects, tilt up muffles;
		// height = proximity (descended tubes are closer to the audience)
		var cutoff = (300 * (2 ** (conv * 2.3))
			* ht.linlin(0, 1, 0.85, 1.3)
			* (1 + (0.7 * tilt.max(0)))
			* face.linlin(0, 1, 0.75, 1.3)).clip(120, 9000);
		var tone   = RLPF.ar(body, cutoff, 0.55);

		// below ~0.3 conviction the voice dissolves into breath
		var voiced = conv.linlin(0.05, 0.45, 0, 1).clip(0, 1);
		var breath = BPF.ar(PinkNoise.ar, (freq * 2).clip(150, 2500), 0.35) * 2.2;

		// rotation is audible as the beam of air sweeping past the audience
		var swish  = BPF.ar(WhiteNoise.ar, 750 * ht.linlin(0, 1, 0.8, 1.3), 1.2)
		             * om.linlin(0.15, 2.5, 0, 0.4).clip(0, 0.4)
		             * face.linlin(0, 1, 0.4, 1.0);

		var amp = (conv ** 1.3).linlin(0, 1, 0.05, 1.0)
			* ht.linlin(0, 1, 0.45, 1.25) * (1 + (crescK * ht.cubed))   // 底へ向けてクレッシェンド
			* (1 - (0.35 * tilt.neg.max(0)))
			* face.linlin(0, 1, 0.65, 1.2)
			* tremor;

		// drones stay almost silent as individuals — they surface when the
		// room approaches unanimity (C high) OR while this tube's color is
		// actively sliding (trans), so the spectral morph is heard in motion
		var cGlob = Lag.kr(In.kr(cBus), 1.0).clip(0, 1);
		// 対立の前線が降りてくると(ht > 0.5)その筒のドローンが浮かび上がる:
		// 界面が降りる = トーンが下がって(htSemis)強くなる (2026-09-24)
		var gate = (0.08 + (0.92 * cGlob * cGlob)).max(0.65 * trans)
			.max(htSurface * ((ht - 0.5) * 2).clip(0, 1).squared);
		var sig = ((tone * voiced) + (breath * (1 - voiced) * 0.7) + swish)
			* amp * droneGain * gate * vg;
		ReplaceOut.kr(vgBus + idx, vg);                    // しゃべり層も同じ筒の声は同時に切る
		Out.ar(out, sig * [wFL, wFR, wRL, wRR]);
	}).add;

	// ---------------- speech layer: a tube takes the floor for a phrase ----------------
	SynthDef(\terrSay, {
		|out = 0, dur = 1, amp = 0.5, op = 0, conv = 0.7, fat = 0,
		ht = 0.5, tilt = 0, reg = 0, baseMidi = 39, idx = 0, faceBus = 0,
		wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25, sayGain = 0.55, htSemis = 5, htBus = 0, crescK = 2,
		vgBus = 0|
		var face = Lag.kr(In.kr(faceBus + idx), 0.1).clip(0, 1);

		var p    = op / 2pi * 12;
		var pc   = (p.round.mod(12) * 7).mod(12);
		// 高さは鳴っている間も追従: 降りながらしゃべると音程がスイープして下がる (2026-09-24)
		var htLive = Lag.kr(In.kr(htBus + idx), 0.3).clip(0, 1);
		var midi = baseMidi + (reg * 12) + pc - (htSemis * htLive);

		// prosody: rise, sustain, sag — fatigue lets the phrase end flat
		var contour = EnvGen.kr(Env(
			[0.985, 1.03, 0.99, 1 - (0.05 * fat)],
			[dur * 0.15, dur * 0.5, dur * 0.35], \sine));
		var freq = midi.midicps * contour;
		var dd   = fat.linlin(0, 1, 0.0015, 0.014);
		var f1   = freq * (1 + dd);
		var f2   = freq * (1 - dd);

		// same color->timbre anchors as the drone layer
		var aw   = op.wrap(-pi, pi);
		var hue  = ((aw >= 0) * aw * (2/3)) + ((aw < 0) * aw * (4/3));
		var wRd  = (1 - ((hue - 0).wrap(-pi, pi).abs / (2pi/3))).max(0);
		var wGn  = (1 - ((hue - (2pi/3)).wrap(-pi, pi).abs / (2pi/3))).max(0);
		var wBl  = (1 - ((hue - (4pi/3)).wrap(-pi, pi).abs / (pi/3))).max(0);
		var wMg  = (1 - ((hue - (5pi/3)).wrap(-pi, pi).abs / (pi/3))).max(0);
		var wSum = (wRd + wGn + wBl + wMg).max(0.01);

		var sawS  = (VarSaw.ar(f1, 0, 0.05) + VarSaw.ar(f2, 0.4, 0.05)) * 0.33;
		var sqrS  = (Pulse.ar(f1, 0.5) + Pulse.ar(f2, 0.5)) * 0.26;
		var pureS = (SinOsc.ar(f1) + SinOsc.ar(f2)) * 0.42;
		var metS  = (SinOsc.ar(f1, SinOsc.ar(f1 * 2.41, 0, 2.5))
		           + SinOsc.ar(f2, SinOsc.ar(f2 * 2.39, 0, 2.5))) * 0.3;
		// same spectral placement as the drones: intermediate hues speak
		// with the half-open, airy top of the morph
		var body = ((
			  (HPF.ar(sawS,  wRd.linexp(0, 1, 9000, 80)) * wRd)
			+ (HPF.ar(sqrS,  wGn.linexp(0, 1, 9000, 80)) * wGn)
			+ (HPF.ar(pureS, wBl.linexp(0, 1, 9000, 80)) * wBl)
			+ (HPF.ar(metS,  wMg.linexp(0, 1, 9000, 80)) * wMg)
		) / wSum);

		var cutoff = (300 * (2 ** (conv * 2.3))
			* ht.linlin(0, 1, 0.85, 1.3)
			* (1 + (0.7 * tilt.max(0)))
			* face.linlin(0, 1, 0.75, 1.3)).clip(120, 9000);
		var env  = EnvGen.kr(
			// 短い音でも立ち上がり/消えが急になりすぎない(プチ防止)
			Env.linen((dur * 0.12).clip(0.04, 0.15), (dur * 0.6).max(0.05), (dur * 0.28).clip(0.12, 0.6), 1, \sine),
			doneAction: 2);
		var trem = 1 - (fat * 0.5 * LFNoise2.kr(6).range(0, 1));

		// attack flair, two entrances by timbre brightness:
		//   bright voices (saw/square/FM) fall in through a highpass —
		//   air first, body lands after;
		//   dark voices (sine/blue) rise in through a lowpass —
		//   rumble first, opening upward.
		// convinced voices land almost at once, hesitant ones drift.
		var swDur   = 0.12 + (0.4 * (1 - conv));
		var hpSweep = XLine.kr(6500, 60, swDur);
		var lpSweep = XLine.kr(90, 7500, swDur);
		var bF      = wBl / wSum;   // blue share = low-harmonic voice
		var toned   = RLPF.ar(body, cutoff, 0.6);

		var sig = ((RHPF.ar(toned, hpSweep, 0.45) * (1 - bF))
		         + (RLPF.ar(toned, lpSweep, 0.45) * bF))
			* env * trem * amp * sayGain * htLive.linlin(0, 1, 0.5, 1.25)
			* (1 + (crescK * htLive.cubed))           // 鳴りながらでも底へ向けてクレッシェンド
			* In.kr(vgBus + idx)                      // 降りきったら/戻る間は同じ筒の声ごと切る
			* face.linlin(0, 1, 0.65, 1.2);
		Out.ar(out, sig * [wFL, wFR, wRL, wRR]);
	}).add;

	// ---------------- tick layer: knock on each half-turn ----------------
	SynthDef(\terrTick, {
		|out = 0, vel = 0.5, pc = 0, ht = 0.5,
		wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25, tickGain = 0.5|

		var t     = Impulse.ar(0);
		var f0    = (33 + pc).midicps * 3;   // knock body follows the camp's pitch class
		var click = BPF.ar(WhiteNoise.ar, 2600, 0.9)
		            * Decay2.ar(t, 0.0005, 0.010) * 3;
		var body  = (Ringz.ar(t, f0.clip(90, 1200), 0.07) * 0.8)
		          + (Ringz.ar(t, (f0 * 2.76).clip(200, 3200), 0.03) * 0.3);
		var sig   = (click + body)
			* vel.linlin(0, 1, 0.25, 1.0)
			* ht.linlin(0, 1, 0.5, 1.2) * tickGain;
		Line.kr(0, 0, 0.5, doneAction: 2);
		Out.ar(out, sig * [wFL, wFR, wRL, wRR]);
	}).add;

	// ---------------- mass layer: the unanimous sub ----------------
	SynthDef(\terrMass, {
		|out = 0, rBus = 0, cBus = 0, meanOpBus = 0, massGain = 0.4|
		var r   = Lag.kr(In.kr(rBus), 1.2).clip(0, 1);
		var c   = Lag.kr(In.kr(cBus), 1.2).clip(0, 1);
		var mo  = In.kr(meanOpBus);
		var pc  = ((mo / 2pi * 12).round.mod(12) * 7).mod(12);
		var midi = Lag.kr(27 + pc, 1.5);
		var f   = midi.midicps;
		var uni = ((r * c).clip(0, 1)) ** 2;
		var sig = (SinOsc.ar(f) * 0.6)
		        + (SinOsc.ar(f * 1.004) * 0.4)          // slow beating: the mass breathes
		        + (VarSaw.ar(f * 2.001, 0, 0.4) * 0.12);
		sig = LPF.ar(sig, 500) * Lag.kr(uni, 0.8) * massGain;
		Out.ar(out, sig * 0.28 ! 4);
	}).add;

	// ---------------- melt gesture: the border liquefies (no boom) ----------------
	SynthDef(\terrMelt, { |out = 0, amp = 0.4|
		var env  = EnvGen.kr(Env([0, 1, 0.55, 0], [1.8, 4.0, 4.2], \sine));
		var wash = BPF.ar(PinkNoise.ar, XLine.kr(2200, 260, 10), 0.5) * 2.2;
		var drip = SinOsc.ar(
			LFNoise2.kr(0.7).exprange(180, 900) * XLine.kr(1, 0.6, 10)) * 0.12;
		var sig  = (wash + drip) * env * amp;
		Line.kr(0, 0, 10.5, doneAction: 2);
		Out.ar(out, sig * 0.25 ! 4);
	}).add;

	// ---------------- schism gesture: creak (phase 0) / crack (phase 1) ----------------
	SynthDef(\terrSplit, { |out = 0, amp = 0.45, phase = 1|
		// dry fracture, no pitched sweep (a falling Ringz reads as a tom)
		var isCrack = phase > 0.5;
		var t = Impulse.ar(0);
		var snap     = HPF.ar(WhiteNoise.ar, 1800) * Decay2.ar(t, 0.001, 0.06) * 3;
		var crackle  = HPF.ar(WhiteNoise.ar, 900)
		               * Decay2.ar(Dust.ar(28) * Line.kr(1, 0, 0.9), 0.0005, 0.03)
		               * 2.2;
		var splinter = Ringz.ar(t, 1400, 0.04) * 0.25;
		var crack = (snap + crackle + splinter) * isCrack;
		var creak = LPF.ar(Saw.ar(LFNoise2.kr(3).exprange(48, 130)), 380)
		            * EnvGen.kr(Env([0, 0.5, 0], [1.4, 2.2], \sine))
		            * 0.5 * (1 - isCrack);
		var sig = (crack + creak) * amp;
		Line.kr(0, 0, 3.8, doneAction: 2);
		Out.ar(out, sig * 0.25 ! 4);
	}).add;

	// ---------------- braam: 結晶化の瞬間の「ドゥーン」 (2026-09-24 試作) ----------------
	// 割れた2陣営の音高を超低域の金管クラスターで同時に鳴らす(対立=トライトーン等の
	// 生の音程がそのまま和音になる)。デチューンした鋸波を重ね、歪ませ、フィルターを
	// 一気に開いてゆっくり閉じ、尾で1オクターブ沈み込む。
	SynthDef(\terrBraam, { |out = 0, amp = 0.55, pcA = 0, pcB = 6, dur = 5.5|
		var env   = EnvGen.kr(Env([0, 1, 0.55, 0], [0.04, dur * 0.35, dur * 0.65], [2, -2, -4]),
			doneAction: 2);
		var dive  = EnvGen.kr(Env([1, 1, 0.5], [dur * 0.3, dur * 0.7], [0, -3]));
		var voice = { |pc|
			var f = (27 + pc).midicps * dive;       // Eb1 基準の超低域
			var saws = Mix.fill(5, { |k|
				Saw.ar(f * (1 + ((k - 2) * 0.006)), 0.2) })
				+ (Mix.fill(3, { |k| Saw.ar(f * 2 * (1 + ((k - 1) * 0.004)), 0.12) }))
				+ (SinOsc.ar(f * 0.5) * 0.9);        // サブ
			saws
		};
		var cut  = EnvGen.kr(Env([150, 3200, 380], [0.07, dur * 0.8], [3, -4]));
		var sig  = voice.(pcA) + voice.(pcB);
		sig = RLPF.ar(sig, cut, 0.35);
		sig = (sig * 2.2).tanh;                    // 金管の歪み
		Out.ar(out, sig * env * amp * 0.25 ! 4);
	}).add;

	// ---------------- land: 筒が底に着いた瞬間の「ドン」 (2026-09-24) ----------------
	// その筒の位置から、その筒の(1オクターブ沈んだ)音程の超低域で短く打つ
	SynthDef(\terrLand, { |out = 0, amp = 0.6, midi = 27,
		wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25|
		var f    = midi.midicps;
		var env  = EnvGen.kr(Env.perc(0.008, 2.2, 1, -5), doneAction: 2);
		var body = Mix.fill(4, { |k| Saw.ar(f * (1 + ((k - 1.5) * 0.007))) }) * 0.35
		           + (SinOsc.ar(f * 0.5 * XLine.kr(1.5, 1, 0.12)) * 1.2);
		var cut  = EnvGen.kr(Env([120, 2400, 200], [0.01, 1.2], [3, -5]));
		var thud = LPF.ar(WhiteNoise.ar, 180) * EnvGen.kr(Env.perc(0.002, 0.25)) * 1.5;
		var sig  = ((RLPF.ar(body, cut, 0.4) * 1.8).tanh + thud) * env * amp;
		Out.ar(out, sig * [wFL, wFR, wRL, wRR]);
	}).add;

	// ---------------- shift gesture ----------------
	SynthDef(\terrShift, { |out = 0, amp = 0.5|
		var boom = SinOsc.ar(XLine.kr(72, 40, 1.4))
		           * EnvGen.kr(Env.perc(0.02, 2.2, 1, -5));
		var air  = LPF.ar(WhiteNoise.ar, 1400)
		           * EnvGen.kr(Env([0, 1, 0], [0.5, 2.6], \sine));
		var sig  = ((boom * 0.85) + (air * 0.22)) * amp;
		Line.kr(0, 0, 3.6, doneAction: 2);
		Out.ar(out, sig * 0.25 ! 4);
	}).add;

	// ---------------- reverb + output (same shape as the rain engine) ----------------
	SynthDef(\terrVerb, { |bus = 0, mix = 0.5, room = 0.88, damp = 0.45|
		var dry = In.ar(bus, 4);
		var wet = FreeVerb.ar(dry, 1.0, room, damp);
		ReplaceOut.ar(bus, (dry * (1 - mix)) + (wet * mix));
	}).add;

	SynthDef(\terrOut4, {
		|mainBus = 0, recBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 3,
		cFL = 0, cFR = 1, cRL = 2, cRR = 3|
		var sig  = In.ar(mainBus, 4);
		var gain = Lag.kr(In.kr(gainBus), 0.3).dbamp;
		var env  = EnvGen.kr(Env.asr(fadeTime, 1, fadeTime, \sine), gateLevel);
		sig = Limiter.ar(sig * gain * env, 0.95);
		Out.ar(cFL, sig[0]); Out.ar(cFR, sig[1]);
		Out.ar(cRL, sig[2]); Out.ar(cRR, sig[3]);
		Out.ar(recBus, [sig[0] + (sig[2] * 0.85), sig[1] + (sig[3] * 0.85)] * 0.7);
	}).add;

	SynthDef(\terrOutStereo, {
		|mainBus = 0, recBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 3|
		var sig  = In.ar(mainBus, 4);
		var gain = Lag.kr(In.kr(gainBus), 0.3).dbamp;
		var env  = EnvGen.kr(Env.asr(fadeTime, 1, fadeTime, \sine), gateLevel);
		var st;
		sig = Limiter.ar(sig * gain * env, 0.95);
		st = [sig[0] + (sig[2] * 0.85), sig[1] + (sig[3] * 0.85)] * 0.7;
		Out.ar(0, st);
		Out.ar(recBus, st);
	}).add;

	s.sync;

	// ---------------- node graph ----------------
	~srcGroup = Group.new(s);

	~voices = ~nBots.collect { |i|
		Synth.new(\terrDrone, [
			\out, ~mainBus, \idx, i, \reg, i % 3, \baseMidi, ~baseMidi,
			\wFL, ~panW[i][0], \wFR, ~panW[i][1], \wRL, ~panW[i][2], \wRR, ~panW[i][3],
			\opBus, ~opBus, \convBus, ~convBus, \fatBus, ~fatBus,
			\htBus, ~htBus, \tiltBus, ~tiltBus, \omBus, ~omBus,
			\faceBus, ~faceBus, \cBus, ~cBus, \droneGain, ~droneGain, \htSemis, ~htPitchSemis, \htSurface, ~htSurface, \crescK, ~crescK,
			\vgBus, ~vgBus, \cutHt, ~cutHt, \reopenHt, ~reopenHt
		], ~srcGroup, \addToTail)
	};

	~massSynth = Synth.new(\terrMass, [
		\out, ~mainBus, \rBus, ~rBus, \cBus, ~cBus,
		\meanOpBus, ~meanOpBus, \massGain, ~massGain
	], ~srcGroup, \addToTail);

	~reverbSynth = Synth.new(\terrVerb,
		[\bus, ~mainBus, \mix, ~revMix], ~srcGroup, \addAfter);

	~outSynth = Synth.new(
		if(devStereo, \terrOutStereo, \terrOut4),
		[ \mainBus, ~mainBus, \recBus, ~recBus, \gainBus, ~gainBus,
		  \gateLevel, 0, \fadeTime, ~fadeTime,
		  \cFL, hwChans[0], \cFR, hwChans[1], \cRL, hwChans[2], \cRR, hwChans[3] ],
		~reverbSynth, \addAfter);

	// ---------------- OSC receivers ----------------
	OSCdef(\terrOp, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~opBus.setn(v) };
	}, '/terr/op');

	OSCdef(\terrConv, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~convBus.setn(v) };
	}, '/terr/conv');

	OSCdef(\terrFat, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~fatBus.setn(v) };
	}, '/terr/fat');

	OSCdef(\terrHt, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~htBus.setn(v); ~lastHt = v };
	}, '/terr/ht');

	OSCdef(\terrTilt, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~tiltBus.setn(v) };
	}, '/terr/tilt');

	OSCdef(\terrOm, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~omBus.setn(v) };
	}, '/terr/om');

	OSCdef(\terrFace, { |msg|
		var v = msg[1..].collect(_.asFloat);
		if(v.size == ~nBots) { ~faceBus.setn(v) };
	}, '/terr/face');

	OSCdef(\terrGlobal, { |msg|
		~rBus.set(msg[1].asFloat.clip(0, 1));
		~cBus.set(msg[2].asFloat.clip(0, 1));
		// msg[3] = camps (unused by the engine, logged by the sender)
		~meanOpBus.set(msg[4].asFloat);
		~gainBus.set(msg[5].asFloat);
	}, '/terr/global');

	OSCdef(\terrTick, { |msg|
		var i   = msg[1].asInteger;
		var vel = msg[2].asFloat.clip(0, 1);
		var pc  = msg[3].asInteger;
		if((i >= 0) and: { i < ~nBots }) {
			Synth.new(\terrTick, [
				\out, ~mainBus, \vel, vel, \pc, pc, \ht, ~lastHt[i],
				\wFL, ~panW[i][0], \wFR, ~panW[i][1],
				\wRL, ~panW[i][2], \wRR, ~panW[i][3],
				\tickGain, ~tickGain
			], ~srcGroup, \addToTail);
		};
	}, '/terr/tick');

	OSCdef(\terrSay, { |msg|
		var i    = msg[1].asInteger;
		var htS  = msg[7].asFloat.clip(0, 1);
		var dur  = (msg[2].asFloat * (~sayDurScale + ((~sayDurLow - ~sayDurScale) * htS))).clip(0.1, 6);
		var amp  = msg[3].asFloat.clip(0, 1);
		var op   = msg[4].asFloat;
		var conv = msg[5].asFloat.clip(0, 1);
		var fat  = msg[6].asFloat.clip(0, 1);
		var ht   = msg[7].asFloat.clip(0, 1);
		var tilt = msg[8].asFloat.clip(-1, 1);
		if((i >= 0) and: { i < ~nBots }) {
			Synth.new(\terrSay, [
				\out, ~mainBus, \dur, dur, \amp, amp, \op, op,
				\conv, conv, \fat, fat, \ht, ht, \tilt, tilt,
				\reg, i % 3, \baseMidi, ~baseMidi,
				\idx, i, \faceBus, ~faceBus,
				\wFL, ~panW[i][0], \wFR, ~panW[i][1],
				\wRL, ~panW[i][2], \wRR, ~panW[i][3],
				\sayGain, ~sayGain, \htSemis, ~htPitchSemis, \htBus, ~htBus, \crescK, ~crescK,
				\vgBus, ~vgBus
			], ~srcGroup, \addToTail);
		};
	}, '/terr/say');

	OSCdef(\terrShift, { |msg|
		Synth.new(\terrShift, [\out, ~mainBus, \amp, ~shiftGain],
			~srcGroup, \addToTail);
	}, '/terr/shift');

	OSCdef(\terrMelt, { |msg|
		Synth.new(\terrMelt, [\out, ~mainBus, \amp, ~meltGain],
			~srcGroup, \addToTail);
	}, '/terr/melt');

	OSCdef(\terrSplit, { |msg|
		// phase 1 (secession crack) is intentionally SILENT — the event is
		// carried by the glissandi + spectral morph + drone surge alone.
		// only the soft phase-0 creak (faction forming) makes a sound.
		if(msg[1].asInteger == 0) {
			Synth.new(\terrSplit, [\out, ~mainBus, \amp, ~splitGain,
				\phase, 0], ~srcGroup, \addToTail);
		};
	}, '/terr/split');

	OSCdef(\terrLand, { |msg|
		var i  = msg[1].asInteger;
		var op = msg[2].asFloat;
		var pc = ((op / 2pi * 12).round.mod(12) * 7).mod(12);
		if((~landGain > 0) and: { (i >= 0) and: { i < ~nBots } }) {
			Synth.new(\terrLand, [\out, ~mainBus, \amp, ~landGain,
				\midi, ~baseMidi + ((i % 3) * 12) + pc - ~htPitchSemis - 12,
				\wFL, ~panW[i][0], \wFR, ~panW[i][1],
				\wRL, ~panW[i][2], \wRR, ~panW[i][3]], ~srcGroup, \addToTail);
		};
	}, '/terr/land');

	OSCdef(\terrBraam, { |msg|
		if(~braamGain > 0) {
			Synth.new(\terrBraam, [\out, ~mainBus, \amp, ~braamGain,
				\pcA, msg[1].asInteger, \pcB, msg[2].asInteger], ~srcGroup, \addToTail);
		};
	}, '/terr/braam');

	OSCdef(\terrScene, { |msg|
		~outSynth.set(\gateLevel, if(msg[1].asInteger > 0, 1, 0));
	}, '/terr/scene');

	OSCdef(\terrRecord, { |msg|
		if(msg[1].asInteger > 0) {
			~startRec.(if(msg.size > 2) { msg[2].asString } { nil });
			"recording started".postln;
		} {
			~stopRec.();
			"recording stopped".postln;
		};
	}, '/terr/record');

	// ---------------- helpers ----------------
	~sceneStart = { ~outSynth.set(\gateLevel, 1) };
	~sceneStop  = { ~outSynth.set(\gateLevel, 0) };
	~startRec = { |path|
		path = path ?? { dir +/+ "territory_check_%.aiff".format(Date.getDate.stamp) };
		s.record(path.standardizePath, bus: ~recBus.index, numChannels: 2);
	};
	~stopRec = { s.stopRecording };

	"FU territory engine ready: % voices, % output, OSC on port %"
	.format(~nBots, if(devStereo, "stereo (dev fold)", "4ch"), NetAddr.langPort).postln;
};
)

/* ---------------------------------------------------------------------------
manual test snippets (no Python needed) — evaluate line by line:

~sceneStart.();                              // fade in

// two camps: left half red-ish (angle 0), right half green-ish (angle pi),
// everyone fairly convinced -> should be a clean tritone standoff
~opBus.setn(~nBots.collect { |i| if(i < (~nBots div: 2), 0, pi) });
~convBus.setn(0.8 ! ~nBots);

// consensus: everyone to angle 0 -> unisons/fifths, mass drone rises
~opBus.setn(0 ! ~nBots); ~cBus.set(0.95); ~rBus.set(0.9); ~meanOpBus.set(0);

// undecided crowd: conviction to 0 -> the room becomes breath
~convBus.setn(0.05 ! ~nBots);

// burnout: high fatigue on convinced voices -> detuned, cracking
~convBus.setn(0.8 ! ~nBots); ~fatBus.setn(0.9 ! ~nBots);

// proximity: everyone descends / retreats
~htBus.setn(1.0 ! ~nBots);
~htBus.setn(0.1 ! ~nBots);

// tilt: address the audience vs turn away
~tiltBus.setn(0.9 ! ~nBots);
~tiltBus.setn(-0.9 ! ~nBots);

// a knock from robot 7, camp pc 0, then a "march" of all robots
~voices[7]; NetAddr.localAddr.sendMsg('/terr/tick', 7, 0.9, 0);
fork { 8.do { ~nBots.do { |i| NetAddr.localAddr.sendMsg('/terr/tick', i, 0.7, 0) }; 0.5.wait } };

// axis shift gesture
NetAddr.localAddr.sendMsg('/terr/shift', 0);

// mix tweaks
~voices.do(_.set(\droneGain, 0.2));
~massSynth.set(\massGain, 0.6);
~reverbSynth.set(\mix, 0.4);

~startRec.(); ~stopRec.();
~sceneStop.();
s.meter; s.scope(4);
--------------------------------------------------------------------------- */
