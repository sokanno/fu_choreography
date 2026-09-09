// headless syntax check: compile rain_engine.scd without executing it
(
var path = thisProcess.nowExecutingPath.dirname +/+ "rain_engine.scd";
var func = thisProcess.interpreter.compile(File.readAllString(path));
if(func.isNil) {
	"SYNTAX-CHECK: FAILED".postln;
} {
	"SYNTAX-CHECK: OK".postln;
};
0.exit;
)
