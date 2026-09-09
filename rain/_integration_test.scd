// headless integration test: boot engine, wait for OSC from rain_test_sender.py,
// probe output amplitude, report, exit. Run alongside:
//   python3 rain_test_sender.py steady --duration 10 --max-rate 80
(
var dir = thisProcess.nowExecutingPath.dirname;
(dir +/+ "rain_engine.scd").load;
fork {
	var tries = 0;
	while { ~outSynth.isNil and: { tries < 45 } } { 1.wait; tries = tries + 1 };
	if(~outSynth.isNil) { "BOOT: FAILED".postln; 1.exit };
	"BOOT: OK, % synths".format(s.numSynths).postln;

	~probeBus = Bus.control(s, 1);
	SynthDef(\probe, { |in = 0, out = 0|
		Out.kr(out, Amplitude.kr(In.ar(in, 2).sum, 0.01, 1.5))
	}).add;
	s.sync;
	Synth.tail(s.defaultGroup, \probe, [\in, ~recBus, \out, ~probeBus]);

	~startRec.("/tmp/rain_check.aiff");

	"WAITING-FOR-PYTHON".postln;
	12.do { |i|
		2.wait;
		~probeBus.get { |v| "PROBE %: peak=%  cpu=%\\%".format(i, v.round(0.0001), s.avgCPU.round(0.1)).postln };
	};
	~stopRec.();
	1.wait;
	"TEST: DONE".postln;
	0.exit;
};
)
