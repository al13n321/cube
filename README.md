# cube
Rigid body physics experimentation

The goal is to simulate this thing: http://raffaello.name/projects/cubli/ , presumably as a system of 4 rigid bodies, and then try to build control software that would balance it (I'm have no connection to that project, just saw a youtube video). The simulation is going to be a straightforward force-based simulator along the lines of https://www.cs.cmu.edu/~baraff/sigcourse/notesd1.pdf , supporting joints, engines and friction, but without any collision detection, and probably with O(n^3) complexity.

Some boilerplate code is copied from https://github.com/al13n321/fract