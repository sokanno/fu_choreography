/*
FU Rain Scene — synthesized multichannel rain engine
====================================================
29 ceiling robots = 29 rain voices. Each voice's drop rate is gated by the
robot's Kuramoto phase (sent from Python over OSC). 3 layers:

  1. impact layer  : 29 persistent voices, Poisson drop triggers, 4 materials
                     (0=concrete, 1=leaf, 2=metal roof, 3=water surface),
                     continuous material morph, bilinear 4ch panning
  2. wash layer    : per-channel decorrelated noise, level driven by the
                     local (speaker-weighted) instantaneous drop rate
  3. wind layer    : LPF noise + gust envelope propagating across channels
                     along wind_dir; gusts also boost local drop rates

OSC protocol (received on sclang default port 57120):
  /rain/phases    29 x float   robot phases [0, 2pi)          (30 Hz)
  /rain/rates     29 x float   base drop rates [drops/sec]    (30 Hz)
  /rain/heat      29 x int     occupancy under robot 0/1      (30 Hz)
  /rain/global    5 x float    depth, material, wind_speed, wind_dir, master_gain_db
  /rain/materials 29 x float   per-robot material override (-1 = follow global)
  /rain/heatmode  int, float   heat mode (0=umbrella 1=material), heat material
  /rain/scene     int          0=fade out, 1=fade in (fade time from config.json)

Usage: open in SC IDE and evaluate the whole file (Cmd-A, Cmd-Enter).
Manual test snippets are at the bottom of this file.
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
~robotXY = robots.collect { |r| [r.x, r.y] };

// ---------------- layer gains (tune here) ----------------
~voiceGain = 0.5;
~washGain  = 0.32;
~washGrain = 0.65;  // 0 = flat hiss, 1 = fully granular shimmer
~windGain  = 0.5;
~revMix    = 0.38;  // reverb wet amount (also: ~reverbSynth.set(\mix, ...))
~numSub    = 6;   // parallel drop generators per voice (lower if CPU-bound)

s.options.numOutputBusChannels = if(devStereo, 2, max(4, hwChans.maxItem + 1));
s.options.numInputBusChannels = 0;   // output-only; avoids in/out sample-rate mismatch on macOS

s.waitForBoot {
	// ---------------- buses ----------------
	~rateBus     = Bus.control(s, ~nBots);  // base drop rate per robot
	~gateBus     = Bus.control(s, ~nBots);  // 0.5 + 0.5*cos(phase), computed in sclang
	~heatBus     = Bus.control(s, ~nBots);  // occupancy 0/1
	~voiceMatBus = Bus.control(s, ~nBots);  // per-robot material, -1 = follow global

	~depthBus    = Bus.control(s, 1);
	~matBus      = Bus.control(s, 1);
	~windSpdBus  = Bus.control(s, 1);
	~windDirBus  = Bus.control(s, 1);
	~gainBus     = Bus.control(s, 1);       // master gain dB
	~heatModeBus = Bus.control(s, 1);       // 0=umbrella, 1=material
	~heatMatBus  = Bus.control(s, 1);       // material used in heat "material" mode
	~meanGateBus = Bus.control(s, 1);       // mean of gate01 (for wash pulsing)

	~washBus     = Bus.control(s, 4);       // local drop rate per speaker (sclang-computed)
	~gustBus     = Bus.control(s, 4);       // gust envelope per speaker (server-computed)

	~mainBus     = Bus.audio(s, 4);         // pre-master 4ch mix
	~recBus      = Bus.audio(s, 2);         // stereo fold for recording / dev monitoring

	~gateBus.setn(0.5 ! ~nBots);
	~voiceMatBus.setn(-1 ! ~nBots);
	~heatMatBus.set(3);                     // default heat material: water
	~gainBus.set(masterDb);
	~meanGateBus.set(0.5);

	// ---------------- impact layer: one persistent voice per robot ----------------
	SynthDef(\rainVoice, {
		|out = 0, idx = 0, xPos = 0.5, yPos = 0.5,
		wFL = 0.25, wFR = 0.25, wRL = 0.25, wRR = 0.25,
		rateBus = 0, gateBus = 0, heatBus = 0, voiceMatBus = 0,
		depthBus = 0, matBus = 0, windSpdBus = 0,
		heatModeBus = 0, heatMatBus = 0, gustBus = 0, voiceGain = 0.5|

		var depth    = Lag.kr(In.kr(depthBus), 0.15);
		var matG     = In.kr(matBus);
		var windSpd  = Lag.kr(In.kr(windSpdBus), 0.3);
		var heatMode = In.kr(heatModeBus);
		var heatMat  = In.kr(heatMatBus);
		var gate01   = Lag.kr(In.kr(gateBus + idx), 0.045);   // smooth 30 Hz steps
		var base     = Lag.kr(In.kr(rateBus + idx), 0.12);
		var heat     = Lag.kr(In.kr(heatBus + idx), 1.0);     // 1 s slew (sec 6)
		var vm       = In.kr(voiceMatBus + idx);
		var gusts    = In.kr(gustBus, 4);
		var gustLocal = (gusts * [wFL, wFR, wRL, wRR]).sum;

		// effective material: per-robot override > global, heat-material morph on top
		var matEff = Select.kr(vm >= 0, [matG, vm]);
		var rate, wConc, wLeaf, wMetal, wWater, white, pink, sig;
		matEff = matEff + ((heatMat - matEff) * heat * (heatMode > 0.5));
		matEff = Lag.kr(matEff, 0.7).clip(0, 3);

		// phase gate (sec 2) + umbrella ducking + gust boost
		rate = base * (1 - depth + (depth * gate01));
		rate = rate * (1 - (heat * (heatMode < 0.5)));
		rate = rate * (1 + (0.35 * gustLocal * windSpd));
		rate = rate.clip(0, 400);

		wConc  = (1 - (matEff - 0).abs).max(0);
		wLeaf  = (1 - (matEff - 1).abs).max(0);
		wMetal = (1 - (matEff - 2).abs).max(0);
		wWater = (1 - (matEff - 3).abs).max(0);

		white = WhiteNoise.ar;
		pink  = PinkNoise.ar;

		sig = Mix.fill(~numSub, {
			var trig = Dust.kr(rate / ~numSub);
			var ta   = T2A.ar(trig);

			// drop size: log-uniform, biased smaller as rate grows (sec 4.1);
			// at low rates favor fewer-but-bigger drops so the rain stays grainy
			var u         = TExpRand.kr(0.08, 1.0, trig);
			var smallBias = rate.linlin(0, 200, 0.65, 2.2);
			var size      = u ** smallBias;
			var amp       = size.linlin(0, 1, 0.2, 1.0) * TExpRand.kr(0.6, 1.0, trig);
			var fScale    = size.linexp(0.01, 1, 1.7, 0.6)
			                * (1 + (0.12 * gustLocal * windSpd));  // gusts brighten

			// 0: concrete — sharp filtered click
			var cDec = TExpRand.kr(0.005, 0.015, trig);
			var cf   = (TExpRand.kr(2000, 6000, trig) * fScale).clip(300, 12000);
			var conc = BPF.ar(white, cf, 0.5) * Decay2.ar(ta, 0.0004, cDec) * 6;

			// 1: leaf — soft broadband burst, slow attack
			var lDec = TExpRand.kr(0.010, 0.030, trig);
			var leaf = LPF.ar(pink, (2600 * fScale).clip(300, 8000))
			           * Decay2.ar(ta, 0.004, lDec) * 5;

			// 2: metal roof (corrugated sheet) — broadband slap + short dull
			// boxy body; rattly, NOT tuned-percussion (no long pure modes)
			var mRing  = TExpRand.kr(0.015, 0.05, trig);
			var mf0    = (TExpRand.kr(170, 520, trig) * fScale).clip(80, 1200);
			var thwack = BPF.ar(white, (2400 * fScale).clip(600, 7000), 0.8)
			             * Decay2.ar(ta, 0.0005, TExpRand.kr(0.004, 0.012, trig)) * 5;
			var metal = thwack + ((
				Ringz.ar(ta, mf0, mRing * 1.4)
				+ (Ringz.ar(ta, (mf0 * TRand.kr(2.4, 3.1, trig)).clip(120, 3500), mRing) * 0.55)
				+ (Ringz.ar(ta, (mf0 * TRand.kr(4.6, 5.8, trig)).clip(150, 6000), mRing * 0.6) * 0.3)
			) * 0.55);

			// 3: water — broadband "plip" splash for every drop; the tonal
			// Minnaert bubble chirp only on a fraction, or it turns melodic
			var f0w      = (TExpRand.kr(350, 1800, trig) * fScale).clip(150, 5000);
			var wDec     = TExpRand.kr(0.015, 0.045, trig);
			var chirpAmt = TRand.kr(0.2, 0.5, trig);
			var hasBub   = TRand.kr(0, 1, trig) < 0.28;
			var plip     = BPF.ar(white, (1400 * fScale).clip(400, 4000), 0.7)
			               * Decay2.ar(ta, 0.001, TExpRand.kr(0.006, 0.018, trig)) * 4;
			var bubFreq  = f0w * (1 + (chirpAmt * Sweep.ar(ta, wDec.reciprocal).clip(0, 1)));
			var water    = plip + (SinOsc.ar(bubFreq.clip(100, 8000))
			               * Decay2.ar(ta, 0.002, wDec) * 0.7 * hasBub);

			var drop = ((conc * wConc) + (leaf * wLeaf) + (metal * wMetal) + (water * wWater)) * amp;

			// per-drop pan jitter around the robot position (sec 5)
			var px = (xPos + TRand.kr(-0.05, 0.05, trig)).clip(0, 1);
			var py = (yPos + TRand.kr(-0.05, 0.05, trig)).clip(0, 1);
			drop * [ (1 - px) * (1 - py), px * (1 - py), (1 - px) * py, px * py ]
		});

		Out.ar(out, sig * voiceGain);
	}).add;

	// ---------------- wash layer ----------------
	SynthDef(\rainWash, {
		|out = 0, washBus = 0, matBus = 0, depthBus = 0, meanGateBus = 0,
		windSpdBus = 0, gustBus = 0, washGain = 0.45, washGrain = 0.65|

		var local    = Lag.kr(In.kr(washBus, 4), 0.25);   // drops/sec near each speaker
		var matG     = Lag.kr(In.kr(matBus), 0.7).clip(0, 3);
		var depth    = Lag.kr(In.kr(depthBus), 0.15);
		var meanGate = Lag.kr(In.kr(meanGateBus), 0.045);
		var windSpd  = Lag.kr(In.kr(windSpdBus), 0.3);
		var gusts    = Lag.kr(In.kr(gustBus, 4), 0.1);

		var wConc  = (1 - (matG - 0).abs).max(0);
		var wLeaf  = (1 - (matG - 1).abs).max(0);
		var wMetal = (1 - (matG - 2).abs).max(0);
		var wWater = (1 - (matG - 3).abs).max(0);

		// independent noise per channel (decorrelated, sec 9), kept dark —
		// the impact layer owns the highs, the wash only fills underneath
		var noise = { WhiteNoise.ar } ! 4;
		var conc  = (BPF.ar(noise, 3200, 1.0) * 0.8) + (HPF.ar(noise, 1500) * 0.2);
		var leaf  = LPF.ar(noise, 1800) * 0.9;
		var metal = (BPF.ar(noise, 2100, 0.7) * 0.9) + (BPF.ar(noise, 450, 1.5) * 0.4);
		var water = BPF.ar(noise,
			1100 * ({ LFNoise2.kr(0.3).range(0.8, 1.25) } ! 4), 1.0) * 1.2;

		var shaped = (conc * wConc) + (leaf * wLeaf) + (metal * wMetal) + (water * wWater);
		// fast random amplitude grain so the wash shimmers instead of hissing flat
		// (clip before ** — LFNoise2 overshoots its range and negatives would NaN)
		var grain = (({ LFNoise2.kr(13).range(0, 1).clip(0, 1) } ! 4) ** 1.5)
		            * ({ LFNoise2.kr(3.1).range(0.55, 1.0).clip(0, 1) } ! 4);

		// silent until the rain is genuinely dense — the wash only fills the gaps
		// between drops in heavy rain; too early and it smears the granularity
		var amp = (((local - 120).max(0)) / 350).min(1) ** 0.8;
		// pulse with the phase mean when depth is up
		var pulse = 1 - depth + (depth * meanGate);

		shaped = LPF.ar(shaped, 4200) * (1 - washGrain + (washGrain * grain));

		Out.ar(out, shaped * amp * pulse * (1 + (0.25 * gusts * windSpd)) * washGain);
	}).add;

	// ---------------- wind layer ----------------
	SynthDef(\rainWind, {
		|out = 0, windSpdBus = 0, windDirBus = 0, gustBus = 0, windGain = 0.5|

		var windSpd = Lag.kr(In.kr(windSpdBus), 0.5);
		var windDir = Lag.kr(In.kr(windDirBus), 0.5);

		// gust master envelope, 0.1-0.5 Hz smooth random (sec 4.3)
		// (clip before ** — LFNoise2 overshoots and negatives would NaN)
		var master = ((LFNoise2.kr(0.18).range(0, 1) * 0.65)
			+ (LFNoise2.kr(0.45).range(0, 1) * 0.35)).clip(0, 1) ** 1.5;

		// propagate gusts across speakers along wind_dir; faster wind = shorter delays
		var spkPos = [[0, 0], [1, 0], [0, 1], [1, 1]];   // FL FR RL RR in [0,1]^2
		var delayScale = windSpd.linlin(0, 1, 1.5, 0.2);
		var proj = spkPos.collect { |p| (cos(windDir) * p[0]) + (sin(windDir) * p[1]) };
		var pmin = proj[0].min(proj[1]).min(proj[2]).min(proj[3]);
		var gust = proj.collect { |pr|
			DelayN.kr(master, 2.2, Lag.kr((pr - pmin) * delayScale, 0.5))
		};

		var noise = { WhiteNoise.ar } ! 4;
		var sig = LPF.ar(noise, (220 + (600 * gust)).clip(100, 900))
		          * (gust ** 2) * windSpd * windGain;

		ReplaceOut.kr(gustBus, gust);   // voices + wash read local gusts from here
		Out.ar(out, sig);
	}).add;

	// ---------------- reverb (the room) ----------------
	SynthDef(\rainVerb, { |bus = 0, mix = 0.38, room = 0.72, damp = 0.55|
		var dry = In.ar(bus, 4);
		var wet = FreeVerb.ar(dry, 1.0, room, damp);   // 4 independent mono verbs
		ReplaceOut.ar(bus, (dry * (1 - mix)) + (wet * mix));
	}).add;

	// ---------------- output stage ----------------
	SynthDef(\rainOut4, {
		|mainBus = 0, recBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 2,
		cFL = 0, cFR = 1, cRL = 2, cRR = 3|
		var sig  = In.ar(mainBus, 4);
		var gain = Lag.kr(In.kr(gainBus), 0.3).dbamp;
		var env  = EnvGen.kr(Env.asr(fadeTime, 1, fadeTime, \sine), gateLevel);
		sig = Limiter.ar(sig * gain * env, 0.95);
		Out.ar(cFL, sig[0]); Out.ar(cFR, sig[1]);
		Out.ar(cRL, sig[2]); Out.ar(cRR, sig[3]);
		Out.ar(recBus, [sig[0] + (sig[2] * 0.85), sig[1] + (sig[3] * 0.85)] * 0.7);
	}).add;

	SynthDef(\rainOutStereo, {
		|mainBus = 0, recBus = 0, gainBus = 0, gateLevel = 0, fadeTime = 2|
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

	~windSynth = Synth.new(\rainWind, [
		\out, ~mainBus, \windSpdBus, ~windSpdBus, \windDirBus, ~windDirBus,
		\gustBus, ~gustBus, \windGain, ~windGain
	], ~srcGroup, \addToHead);   // first: writes gustBus for everyone else

	~voices = ~nBots.collect { |i|
		Synth.new(\rainVoice, [
			\out, ~mainBus, \idx, i,
			\xPos, ~robotXY[i][0], \yPos, ~robotXY[i][1],
			\wFL, ~panW[i][0], \wFR, ~panW[i][1], \wRL, ~panW[i][2], \wRR, ~panW[i][3],
			\rateBus, ~rateBus, \gateBus, ~gateBus, \heatBus, ~heatBus,
			\voiceMatBus, ~voiceMatBus, \depthBus, ~depthBus, \matBus, ~matBus,
			\windSpdBus, ~windSpdBus, \heatModeBus, ~heatModeBus,
			\heatMatBus, ~heatMatBus, \gustBus, ~gustBus, \voiceGain, ~voiceGain
		], ~srcGroup, \addToTail)
	};

	~washSynth = Synth.new(\rainWash, [
		\out, ~mainBus, \washBus, ~washBus, \matBus, ~matBus, \depthBus, ~depthBus,
		\meanGateBus, ~meanGateBus, \windSpdBus, ~windSpdBus, \gustBus, ~gustBus,
		\washGain, ~washGain, \washGrain, ~washGrain
	], ~srcGroup, \addToTail);

	~reverbSynth = Synth.new(\rainVerb,
		[\bus, ~mainBus, \mix, ~revMix], ~srcGroup, \addAfter);

	~outSynth = Synth.new(
		if(devStereo, \rainOutStereo, \rainOut4),
		[ \mainBus, ~mainBus, \recBus, ~recBus, \gainBus, ~gainBus,
		  \gateLevel, 0, \fadeTime, ~fadeTime,
		  \cFL, hwChans[0], \cFR, hwChans[1], \cRL, hwChans[2], \cRR, hwChans[3] ],
		~reverbSynth, \addAfter);

	// ---------------- OSC receivers ----------------
	~lastRates = 0.0 ! ~nBots;
	~lastGate  = 0.5 ! ~nBots;
	~lastDepth = 0.0;

	// local (speaker-weighted) effective rate -> wash bus; field math stays in sclang
	~updateWash = {
		var local = [0.0, 0.0, 0.0, 0.0];
		~nBots.do { |i|
			var eff = ~lastRates[i] * (1 - ~lastDepth + (~lastDepth * ~lastGate[i]));
			4.do { |c| local[c] = local[c] + (eff * ~panW[i][c]) };
		};
		~washBus.setn(local);
	};

	OSCdef(\rainPhases, { |msg|
		var gate = msg[1..].collect { |p| 0.5 + (0.5 * cos(p.asFloat)) };
		if(gate.size == ~nBots) {
			~lastGate = gate;
			~gateBus.setn(gate);
			~meanGateBus.set(gate.sum / ~nBots);
			~updateWash.();
		};
	}, '/rain/phases');

	OSCdef(\rainRates, { |msg|
		var rates = msg[1..].collect(_.asFloat);
		if(rates.size == ~nBots) {
			~lastRates = rates;
			~rateBus.setn(rates);
			~updateWash.();
		};
	}, '/rain/rates');

	OSCdef(\rainHeat, { |msg|
		var heat = msg[1..].collect(_.asFloat);
		if(heat.size == ~nBots) { ~heatBus.setn(heat) };
	}, '/rain/heat');

	OSCdef(\rainGlobal, { |msg|
		~lastDepth = msg[1].asFloat;
		~depthBus.set(msg[1].asFloat);
		~matBus.set(msg[2].asFloat.clip(0, 3));
		~windSpdBus.set(msg[3].asFloat.clip(0, 1));
		~windDirBus.set(msg[4].asFloat);
		~gainBus.set(msg[5].asFloat);
		~updateWash.();
	}, '/rain/global');

	OSCdef(\rainMaterials, { |msg|
		var mats = msg[1..].collect { |m| m.asFloat.clip(-1, 3) };
		if(mats.size == ~nBots) { ~voiceMatBus.setn(mats) };
	}, '/rain/materials');

	OSCdef(\rainHeatMode, { |msg|
		~heatModeBus.set(msg[1].asFloat.clip(0, 1));
		~heatMatBus.set(msg[2].asFloat.clip(0, 3));
	}, '/rain/heatmode');

	OSCdef(\rainScene, { |msg|
		~outSynth.set(\gateLevel, if(msg[1].asInteger > 0, 1, 0));
	}, '/rain/scene');

	// ---------------- helpers ----------------
	~sceneStart = { ~outSynth.set(\gateLevel, 1) };
	~sceneStop  = { ~outSynth.set(\gateLevel, 0) };
	~startRec = { |path|
		path = path ?? { dir +/+ "rain_check_%.aiff".format(Date.getDate.stamp) };
		s.record(path.standardizePath, bus: ~recBus.index, numChannels: 2);
	};
	~stopRec = { s.stopRecording };

	"FU rain engine ready: % voices, % output, OSC on port %"
	.format(~nBots, if(devStereo, "stereo (dev fold)", "4ch"), NetAddr.langPort).postln;
};
)

/* ---------------------------------------------------------------------------
manual test snippets (no Python needed) — evaluate line by line:

~sceneStart.();                       // fade in
~rateBus.setn(15 ! ~nBots);               // light rain everywhere
~rateBus.setn(80 ! ~nBots);               // heavy rain
~matBus.set(0);                       // concrete
~matBus.set(1);                       // leaf
~matBus.set(2);                       // metal roof
~matBus.set(3);                       // water surface
~matBus.set(2.5);                     // morph metal<->water

// half-space downpour: left half 120 drops/s, right half 8
~rateBus.setn(~robotXY.collect { |p| if(p[0] < 0.5, 120, 8) });

// wind
~windSpdBus.set(0.7); ~windDirBus.set(0.0);

// mix tweaks while listening
~reverbSynth.set(\mix, 0.25);                 // リバーブ量 (0=dry)
~reverbSynth.set(\room, 0.8, \damp, 0.4);     // 部屋の大きさ / 高域減衰
~washSynth.set(\washGain, 0);                 // ウォッシュだけ消して切り分け
~washSynth.set(\washGain, 0.32, \washGrain, 0.8);  // 粒状感の強さ

// fake sync anomaly by hand (normally Python drives phases at 30 Hz)
(
fork {
	var phase = 0;
	~depthBus.set(1); ~lastDepth = 1;
	200.do {
		phase = (phase + (2pi * 0.8 / 30)) % 2pi;   // 0.8 Hz common pulse
		~gateBus.setn((0.5 + (0.5 * cos(phase))) ! ~nBots);
		~meanGateBus.set(0.5 + (0.5 * cos(phase)));
		~updateWash.();
		(1/30).wait;
	};
	~depthBus.set(0); ~lastDepth = 0; ~updateWash.();
};
)

// heat interaction: someone standing under robots 7 and 12
~heatModeBus.set(0);                  // 0=umbrella  1=material
~heatBus.setn(~nBots.collect { |i| [7, 12].includes(i).asInteger });
~heatBus.setn(0 ! ~nBots);                // everyone leaves

// recording (stereo fold of the 4ch mix)
~startRec.();
~stopRec.();

~sceneStop.();                        // fade out
s.meter; s.scope(4);
--------------------------------------------------------------------------- */
