# Control Subsystem

Runs on the Arduino. Receives a goal (0 or 1) for each wheel from the Pi over
I2C and holds each wheel at its goal with a PI position loop wrapped around
the proportional velocity loop from assignment 2. Goal 0 is the position at
reset (tape mark 0 up) and goal 1 is pi rad from there (tape mark 1 up).

## Layout

| Path | What it is |
|------|------------|
| `position_control/position_control.ino` | Main sketch for the demo. Uncomment `#define STEP_TEST` to record a position step response instead of listening to the Pi. |
| `step_response/step_response.ino` | Open loop voltage step on both motors, used to measure K and sigma |
| `tests/i2c_echo/i2c_echo.ino` | Arduino end of the I2C link on its own: prints every message it receives |
| `tests/i2c_send_goals.py` | Pi end of the I2C link without the vision code: cycles through the quadrants or sends typed goals |
| `matlab/motor_params.m` | Every model and controller parameter in one place |
| `matlab/readArduinoData.m` | Records one experiment from the serial port into a `.mat` file |
| `matlab/openloop_sim.slx`, `matlab/run_openloop.m` | Open loop model, comparison with experiment, and a fit of K and sigma |
| `matlab/position_control.slx`, `matlab/run_position.m` | Closed loop position model, comparison with experiment, and a disturbance rejection demo |
| `matlab/build_models.m` | Rebuilds both `.slx` files from code |

## Hardware

| Connection | Arduino pin |
|------------|-------------|
| Motor driver enable | 4 |
| Left motor (motor 1) sign / PWM | 7 / 9 |
| Right motor (motor 2) sign / PWM | 8 / 10 |
| Left encoder A (yellow) / B (white) | 2 / 5 |
| Right encoder A (yellow) / B (white) | 3 / 6 |
| Encoder Vcc (blue) / GND (green) | 5 V / GND |
| Pi GPIO2 SDA (header pin 3) | A4 (SDA) |
| Pi GPIO3 SCL (header pin 5) | A5 (SCL) |
| Pi GND (header pin 6) | GND |

The counts must increase when the sign pins are HIGH. If a wheel runs away,
swap that motor's leads at the driver. The sketches turn off the Arduino's
internal I2C pull-ups so the bus stays at the Pi's 3.3 V.

## I2C protocol

The Arduino is an I2C slave at address `0x08` on the Pi's bus 1.

| Direction | Bytes | Python (`smbus2`) |
|-----------|-------|-------------------|
| Pi → Arduino | `[0, left_goal, right_goal]`, each goal 0 or 1 | `bus.write_i2c_block_data(0x08, 0, [left, right])` |
| Arduino → Pi | `[left_goal, right_goal, at_goal_flags]` | `bus.read_i2c_block_data(0x08, 0, 3)` |

In `at_goal_flags`, bit 0 is set when the left wheel is within 0.05 rad of its goal, and bit 1 is the same for the right wheel. The Arduino ignores any message that isn't a valid goal message. Sending the same goal again does no harm, but the Pi only needs to send when the quadrant changes.

## Controller

Every 10 ms, for each wheel:

```
pos_error      = desired_pos - actual_pos
integral_error = integral_error + pos_error*Ts
desired_speed  = Kp_pos*pos_error + Ki_pos*integral_error     limited to ±MAX_SPEED
Voltage        = Kp_vel*(desired_speed - actual_speed)        limited to ±Battery_Voltage
```

For anti-windup, the sketch undoes that pass's integration whenever `desired_speed` saturates (PID lecture, section 9.3). The Simulink PID block does the same thing with its output limit and clamping anti-windup.

| Gain | Value | Source |
|------|-------|--------|
| `Kp_vel` | 3.0 V per rad/s | assignment 2 |
| `Kp_pos` | 10 (rad/s) per rad | `run_position.m` |
| `Ki_pos` | 10 (rad/s) per rad·s | `run_position.m` |
| `MAX_SPEED` | 6 rad/s | `run_position.m` |

The simulation uses K = 2.2 and sigma = 5. It predicts about 1.4% overshoot and a 0.7 s settling time (2%) for a pi rad step. A 3 V disturbance is removed in about 2 s. With Ki_pos = 0 the same disturbance would leave a 0.1 rad error.

These gains are a starting point. Re-measure K and sigma on the real wheels, then retune:
1. Set `Ki_pos = 0`.
2. Raise `Kp_pos` until the step is fast without overshoot.
3. Add `Ki_pos` until a wheel turned by hand comes back quickly without much overshoot.

Keep `motor_params.m` and the constants at the top of `position_control.ino` in sync.

## Running things

**Measure the motors.**
1. Lift the wheels and upload `step_response/`.
2. In MATLAB, from `matlab/`, run `readArduinoData("openloop_data.mat", 5)`.
3. Run `run_openloop`. It prints fitted K and sigma for each wheel.
4. Copy them into `motor_params.m`.

**Position step response.**
1. Uncomment `#define STEP_TEST` in `position_control.ino`.
2. Set both wheels to 0 and upload.
3. Run `readArduinoData("position_data.mat", 5)`, then `run_position`.

After the recording, the wheels are still held at their goal, so you can disturb them by hand. Change `STEP_GOAL` to `{1, 0}` to step the wheels to different goals.

**I2C link on its own.**
1. Upload `tests/i2c_echo/` and open the serial monitor at 115200.
2. On the Pi, check that `i2cdetect -y 1` shows `08`.
3. Run `python3 tests/i2c_send_goals.py` (or `-i` to type goals).

**Full system.**
1. Set both wheels to 0.
2. Upload `position_control/` with `STEP_TEST` commented out.
3. Start the vision code, or `i2c_send_goals.py`.

The serial monitor shows time, goals, positions, and voltages 10 times a second.

Opening the USB serial port resets the Uno, which zeroes the encoders. Always put both wheels back to 0 before opening the serial monitor or running `readArduinoData`.

**Simulation only.**
- `run_openloop` and `run_position` plot only the simulation if there is no data file yet.
- Change `motor_params.m`, not the block parameters.
- Run `build_models` only after changing the model structure.
- To make a report, use `publish('run_position.m', 'pdf')`.
