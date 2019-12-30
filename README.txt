Noah Attwood
Summer 2018
Rollercoaster Simulator


============ Compiling ============

the program can be compiled with the following command:
gcc -o rc_sim.o rollercoaster_sim.c -lGL -lGLU -lglut -lm


============= Running =============

please run the program by passing a set of control points via stdin, such as with the following command:
./rc_sim.o < track-points/cp_sky_circle.txt

The control points will be used to generate the rollercoaster track.


============ Controls =============

The following controls can be used at any point in the simulation:

c - alternates the camera between being on the rollercoaster and rotating around the rollercoaster 
p - turn the rendering of control points on/off
q - quit the simulation


While the camera is rotating around the rollercoaster, the following controls can also be used:

up arrow - zoom in
down arrow - zoom out
left arrow - increase clockwise rotation speed
right arrow - increase counter-clockwise rotation speed
shift + up arrow - increase camera pitch
shift + down arrow - decrease camera pitch
