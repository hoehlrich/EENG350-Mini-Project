# Computer Vision Subsystem

Runs on the Raspberry Pi.

## Goals

- Detect the marker in the camera image and determine which quadrant (NE, NW,
  SW, SE) it is in
- Show a live camera view with the marker position indicated
- Map the quadrant to goal positions for the left and right wheels
- Send the goal positions to the Arduino
- Display `Goal Position: L R` on the LCD, updated from a separate thread so
  the LCD does not slow the main loop
