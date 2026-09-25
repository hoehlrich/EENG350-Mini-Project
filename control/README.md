# Control Subsystem

Runs on the Arduino.

## Goals

- Receive goal positions (0 or 1 for each wheel) from the Pi
- Turn each wheel to its goal independently, where 0 and 1 are 180 degrees
  apart
- Use a PI position loop wrapped around the existing velocity loop so the
  wheels return to position when disturbed by hand
- Include anti-windup on the integrator
- Provide Simulink models and MATLAB scripts to simulate open and closed loop
  step responses and compare them to experimental data
