import std.stdio;
import std.math;
import std.random;

void main() {
	double[string] test = ["hi" : 1.5, "middle" : 2, "best": 3];
	double* p;
	p = ("hi" in test);
	writeln((("hi" in test) !is null));
	p = ("by" in test);
	writeln((p !is null));
}