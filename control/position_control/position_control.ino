/*
 * EENG350 Mini Project - Two wheel position control.
 *
 * Holds each wheel at a goal position set by the Raspberry Pi over I2C.
 * Each wheel has two tape marks, 0 and 1, 180 degrees apart, and 0 faces up
 * when the Arduino is reset. A goal of 0 means 0 rad and a goal of 1 means
 * pi rad, measured from the position at reset. Returning to 0 unwinds back
 * to 0 rad rather than continuing on to 2*pi.
 *
 * Control law, run every 10 ms for each wheel i (mini project handout
 * section 6). A PI position loop is wrapped around the velocity loop from
 * assignment 2:
 *
 *     pos_error      = desired_pos - actual_pos
 *     integral_error = integral_error + pos_error*Ts
 *     desired_speed  = Kp_pos*pos_error + Ki_pos*integral_error
 *     Voltage        = Kp_vel*(desired_speed - actual_speed)
 *
 * The integral term makes the controller pull a wheel back to its goal
 * with no steady state error after it is turned by hand.
 *
 * Anti-windup (PID lecture section 9.3): desired_speed is limited to
 * +/- MAX_SPEED. When it saturates, that pass's integration is undone, so
 * the integral cannot grow while the wheel is already being driven as
 * fast as allowed. The Simulink model position_control.slx uses the same
 * limit with clamping anti-windup.
 *
 * I2C protocol (Arduino is the slave at address 0x08)
 *   Pi -> Arduino, 3 bytes: [0, left_goal, right_goal]. Each goal is 0 or 1.
 *     From Python: bus.write_i2c_block_data(0x08, 0, [left, right])
 *     Any other message is ignored.
 *   Arduino -> Pi, 3 bytes: [left_goal, right_goal, at_goal_flags]
 *     at_goal_flags bit 0 is set when the left wheel is within
 *     AT_GOAL_TOL_RAD of its goal, bit 1 the same for the right wheel.
 *     From Python: bus.read_i2c_block_data(0x08, 0, 3)
 *
 * Modes
 *   Normal (STEP_TEST not defined): goals come from the Pi. A status line
 *   is printed over Serial every 100 ms for debugging.
 *
 *   STEP_TEST defined: I2C is off. Both goals are 0 until t = 1 s, then
 *   STEP_GOAL. Prints "Ready!", then time (s), left voltage (V), left
 *   position (rad), right voltage (V), right position (rad) in five
 *   tab-separated columns every 10 ms for RUN_TIME_S, then "Finished". The
 *   wheels keep being held after that, so they can be disturbed by hand.
 *   Record the run with control/matlab/readArduinoData.m and compare it
 *   with the simulation using run_position.m.
 *
 * Note: opening the USB serial port resets the Uno, which also resets the
 * encoder counts. Set both wheels to 0 before opening the serial monitor.
 *
 * Hardware
 *   Pololu dual MC33926 motor driver shield: pin 4 enable, pins 7/8 motor
 *   1/2 voltage sign, pins 9/10 motor 1/2 PWM.
 *   Motor 1 = LEFT wheel.  Encoder A (yellow) -> pin 2, B (white) -> pin 5
 *   Motor 2 = RIGHT wheel. Encoder A (yellow) -> pin 3, B (white) -> pin 6
 *   Encoder Vcc (blue) -> 5 V, encoder GND (green) -> GND
 *   Raspberry Pi GPIO2 (SDA, pin 3)  -> Arduino A4 (SDA)
 *   Raspberry Pi GPIO3 (SCL, pin 5)  -> Arduino A5 (SCL)
 *   Raspberry Pi GND (pin 6)         -> Arduino GND
 *
 * Sign convention (assignment 2 tutorial section 2.2): the counts must
 * increase when pins 7 and 8 are HIGH. If they don't, swap that motor's red
 * and black leads at the motor driver. Otherwise the feedback is positive
 * and the wheel runs away.
 */

#include <Wire.h>

// Uncomment to run a position step response instead of listening to the Pi
// #define STEP_TEST

// ---- I2C ----
const uint8_t I2C_ADDRESS = 0x08;

// ---- Motor driver pins, index 0 = left (motor 1), 1 = right (motor 2) ----
const uint8_t ENABLE_PIN = 4;
const uint8_t SIGN_PIN[2] = {7, 8};
const uint8_t PWM_PIN[2] = {9, 10};

// ---- Encoder pins ----
const uint8_t ENC_A[2] = {2, 3};
const uint8_t ENC_B[2] = {5, 6};

const float COUNTS_PER_REV = 3200;   // 64 counts per motor rev * 50:1 gearbox

// ---- Controller settings (keep in sync with control/matlab/motor_params.m) ----
const float Kp_vel = 3.0;            // velocity loop gain (V per rad/s) - from assignment 2
const float Kp_pos = 10.0;           // position proportional gain ((rad/s) per rad)
const float Ki_pos = 10.0;           // position integral gain ((rad/s) per rad*s)
const float MAX_SPEED = 6.0;         // limit on desired_speed (rad/s)
const float Battery_Voltage = 7.8;   // read the actual value off the voltage monitor
const float AT_GOAL_TOL_RAD = 0.05;  // error counted as "at goal" for the Pi (rad)

// ---- Step test settings ----
#ifdef STEP_TEST
const uint8_t STEP_GOAL[2] = {1, 1}; // goals after the step (left, right)
const float STEP_TIME_S = 1.0;       // time the goals step (s)
const float RUN_TIME_S = 5.0;        // length of the recording (s)
bool finished = false;               // true once "Finished" has been printed
#else
const unsigned int PRINT_EVERY = 10; // print a status line every 10th pass
unsigned int pass_count = 0;
#endif

// ---- Timing ----
unsigned long desired_Ts_ms = 10;    // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

// ---- Controller state ----
uint8_t goal[2] = {0, 0};            // goal tape mark for each wheel, 0 or 1
float last_pos[2] = {0, 0};          // wheel positions on the previous pass (rad)
float integral_error[2] = {0, 0};    // integral of the position error (rad*s)

// ---- Values shared with the I2C interrupt handlers ----
// Single bytes, so each read or write is atomic on the AVR. The two goals
// are still copied with interrupts off so a message can't land between them.
volatile uint8_t i2c_goal[2] = {0, 0};
volatile uint8_t at_goal_flags = 0;

// ---- Encoders ----
// Channel A of each encoder is on an interrupt pin. The interrupt sees two of
// the four transitions in each quadrature cycle, so it counts by twos.
// myEnc() adds the missing +/-1 if channel B has changed since then, which
// gives the full 3200 counts per wheel revolution. Same method as
// assignments 1 and 2.
volatile long counter[2] = {0, 0};
volatile bool lastA[2], lastB[2];

// Shared by both encoder interrupts
void updateEncoder(uint8_t i) {
  bool a = digitalRead(ENC_A[i]);
  bool b = digitalRead(ENC_B[i]);

  if (a != b) {
    counter[i] += 2;
  } else {
    counter[i] -= 2;
  }

  lastA[i] = a;
  lastB[i] = b;
}

// Runs on every edge of the left encoder channel A
void handleA1() {
  updateEncoder(0);
}

// Runs on every edge of the right encoder channel A
void handleA2() {
  updateEncoder(1);
}

// Returns the current position of wheel i (0 = left, 1 = right) in counts
long myEnc(uint8_t i) {
  long count;
  bool a, b, snapA, snapB;

  // A long is 4 bytes, so an interrupt could change it halfway through a
  // read. Turn interrupts off while taking a snapshot.
  noInterrupts();
  a = digitalRead(ENC_A[i]);
  b = digitalRead(ENC_B[i]);
  count = counter[i];
  snapA = lastA[i];
  snapB = lastB[i];
  interrupts();

  // Only B can have changed since the last interrupt. If it did, add one
  // more count in the direction we were going.
  if (a == snapA && b != snapB) {
    if (snapA != snapB) {
      count += 1;
    } else {
      count -= 1;
    }
  }

  return count;
}

// ---- I2C handlers (run in interrupt context, so keep them short) ----

// Called when the Pi writes. Accepts only [0, left, right] with goals 0 or 1.
void receiveEvent(int howMany) {
  uint8_t msg[3];
  int n = 0;

  // Read everything so the buffer is empty for the next message
  while (Wire.available()) {
    uint8_t c = Wire.read();
    if (n < 3) {
      msg[n] = c;
    }
    n++;
  }

  if (n == 3 && msg[0] == 0 && msg[1] <= 1 && msg[2] <= 1) {
    i2c_goal[0] = msg[1];
    i2c_goal[1] = msg[2];
  }
}

// Called when the Pi reads. Reports the goals and whether each wheel is there.
void requestEvent() {
  uint8_t reply[3] = {i2c_goal[0], i2c_goal[1], at_goal_flags};
  Wire.write(reply, 3);
}

// Applies Voltage to motor i: sets the sign pin and the PWM duty cycle
void driveMotor(uint8_t i, float Voltage) {
  unsigned int PWM;

  if (Voltage > 0) {
    digitalWrite(SIGN_PIN[i], HIGH);
  } else {
    digitalWrite(SIGN_PIN[i], LOW);
  }

  PWM = 255 * fabs(Voltage) / Battery_Voltage;
  analogWrite(PWM_PIN[i], min(PWM, 255));
}

void setup() {
  // Motor driver
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);   // turn the motor driver on
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(SIGN_PIN[i], OUTPUT);
    pinMode(PWM_PIN[i], OUTPUT);
    analogWrite(PWM_PIN[i], 0);
  }

  // Encoders
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(ENC_A[i], INPUT_PULLUP);
    pinMode(ENC_B[i], INPUT_PULLUP);
    lastA[i] = digitalRead(ENC_A[i]);
    lastB[i] = digitalRead(ENC_B[i]);
  }
  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), handleA1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), handleA2, CHANGE);

#ifndef STEP_TEST
  // I2C slave. Wire.begin() turns on the internal pull-ups, which would pull
  // the bus up to 5 V. The Pi's pins are 3.3 V and the Pi already has
  // pull-ups, so turn the Arduino's off.
  Wire.begin(I2C_ADDRESS);
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
#endif

  Serial.begin(115200); // Set the baud rate fast so that we can display the results
  Serial.println("Ready!");
#ifndef STEP_TEST
  Serial.println("time\tgoalL\tgoalR\tposL\tposR\tVL\tVR");
#endif
  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;
}

void loop() {
  const float Ts = (float)desired_Ts_ms / 1000;   // sample time (s)
  float actual_pos[2];
  float actual_speed[2];
  float desired_pos[2];
  float pos_error[2];
  float desired_speed[2];
  float error[2];
  float Voltage[2];
  uint8_t flags = 0;

  // timestamp in seconds
  current_time = (float)(last_time_ms - start_time_ms) / 1000;

  // Pick up the goals for this pass
#ifdef STEP_TEST
  for (uint8_t i = 0; i < 2; i++) {
    goal[i] = (current_time >= STEP_TIME_S) ? STEP_GOAL[i] : 0;
  }
#else
  noInterrupts();
  goal[0] = i2c_goal[0];
  goal[1] = i2c_goal[1];
  interrupts();
#endif

  for (uint8_t i = 0; i < 2; i++) {
    // Position in radians, and velocity = change in position / sample time
    actual_pos[i] = 2 * PI * (float)myEnc(i) / COUNTS_PER_REV;
    actual_speed[i] = (actual_pos[i] - last_pos[i]) / Ts;
    last_pos[i] = actual_pos[i];

    // Outer loop: PI position controller gives the desired speed
    desired_pos[i] = goal[i] * PI;
    pos_error[i] = desired_pos[i] - actual_pos[i];
    integral_error[i] = integral_error[i] + pos_error[i] * Ts;
    desired_speed[i] = Kp_pos * pos_error[i] + Ki_pos * integral_error[i];

    // Anti-windup: saturate the desired speed and undo this pass's integration
    if (fabs(desired_speed[i]) > MAX_SPEED) {
      desired_speed[i] = (desired_speed[i] > 0) ? MAX_SPEED : -MAX_SPEED;
      integral_error[i] = integral_error[i] - pos_error[i] * Ts;
    }

    // Inner loop: proportional velocity controller from assignment 2
    error[i] = desired_speed[i] - actual_speed[i];
    Voltage[i] = Kp_vel * error[i];
    Voltage[i] = constrain(Voltage[i], -Battery_Voltage, Battery_Voltage);
    driveMotor(i, Voltage[i]);

    if (fabs(pos_error[i]) < AT_GOAL_TOL_RAD) {
      flags |= (1 << i);
    }
  }
  at_goal_flags = flags;

#ifdef STEP_TEST
  if (current_time < RUN_TIME_S) {
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(Voltage[0]);
    Serial.print("\t");
    Serial.print(actual_pos[0]);
    Serial.print("\t");
    Serial.print(Voltage[1]);
    Serial.print("\t");
    Serial.print(actual_pos[1]);
    Serial.println("");
  } else if (!finished) {
    Serial.println("Finished");
    finished = true;
  }
#else
  // Status line for debugging, 10 times a second
  pass_count++;
  if (pass_count >= PRINT_EVERY) {
    pass_count = 0;
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(goal[0]);
    Serial.print("\t");
    Serial.print(goal[1]);
    Serial.print("\t");
    Serial.print(actual_pos[0]);
    Serial.print("\t");
    Serial.print(actual_pos[1]);
    Serial.print("\t");
    Serial.print(Voltage[0]);
    Serial.print("\t");
    Serial.print(Voltage[1]);
    Serial.println("");
  }
#endif

  while (millis() < last_time_ms + desired_Ts_ms) {
    // wait until desired time passes to go top of the loop
  }
  last_time_ms = millis();
}
