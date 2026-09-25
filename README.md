# EENG350 Mini Project

A camera on the Raspberry Pi acts as the user interface for a two wheel system.
The quadrant of the image containing an ArUco marker sets the goal position of
each wheel, the Pi sends that goal to the Arduino, and the Arduino holds each
wheel at its goal with integral control.

| Quadrant | Left Wheel | Right Wheel |
|----------|------------|-------------|
| NE       | 0          | 0           |
| NW       | 0          | 1           |
| SW       | 1          | 1           |
| SE       | 1          | 0           |

Each wheel has two tape marks, 0 and 1, 180 degrees apart. Both wheels start
with 0 facing up.

## Organization

- `vision/`: Pi code for marker detection, LCD display, and communication with the Arduino
- `control/`: Arduino position controller plus Simulink/MATLAB models and step response scripts
- `resources/`: assignment handout and reference material
