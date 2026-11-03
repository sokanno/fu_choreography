// headless syntax check: compile press_engine.scd without executing it
(
var path = thisProcess.nowExecutingPath.dirname +/+ "press_engine.scd";
var func = thisProcess.interpreter.compile(File.readAllString(path));
if(func.isNil) {
	"SYNTAX-CHECK: FAILED".postln;
} {
	"SYNTAX-CHECK: OK".postln;
};
0.exit;
)
