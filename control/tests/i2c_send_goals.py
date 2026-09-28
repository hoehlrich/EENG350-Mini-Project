#!/usr/bin/env python3
"""EENG350 Mini Project - I2C link test, Pi side.

Sends wheel goal positions to the Arduino without the vision code, to test
the Arduino (position_control.ino or tests/i2c_echo/i2c_echo.ino) on its own.

Modes
    python3 i2c_send_goals.py            cycle NE, NW, SW, SE every 3 s
    python3 i2c_send_goals.py -p 5       same, 5 s per quadrant
    python3 i2c_send_goals.py -i         type goals such as "0 1", or q to quit

After every write the Arduino is read back and its reply is printed:
the goals it accepted and which wheels are at their goal.

I2C protocol (see control/README.md)
    Pi -> Arduino: [0, left_goal, right_goal], goals 0 or 1
    Arduino -> Pi: [left_goal, right_goal, at_goal_flags]
        at_goal_flags bit 0 = left wheel at goal, bit 1 = right wheel

Hardware
    Raspberry Pi GPIO2 (SDA, pin 3) -> Arduino A4 (SDA)
    Raspberry Pi GPIO3 (SCL, pin 5) -> Arduino A5 (SCL)
    Raspberry Pi GND (pin 6)        -> Arduino GND
    I2C must be enabled on the Pi (raspi-config -> Interface Options).
    `i2cdetect -y 1` should show 08. Needs the smbus2 package.
"""

import argparse
import time

from smbus2 import SMBus

I2C_BUS = 1
ARDUINO_ADDRESS = 0x08

# Marker quadrant -> (left goal, right goal), from the assignment handout
QUADRANT_GOALS = {
    "NE": (0, 0),
    "NW": (0, 1),
    "SW": (1, 1),
    "SE": (1, 0),
}


def send_goals(bus, left, right):
    """Write the two goals to the Arduino."""
    bus.write_i2c_block_data(ARDUINO_ADDRESS, 0, [left, right])


def read_status(bus):
    """Read back (left_goal, right_goal, left_at_goal, right_at_goal)."""
    left, right, flags = bus.read_i2c_block_data(ARDUINO_ADDRESS, 0, 3)
    return left, right, bool(flags & 1), bool(flags & 2)


def print_status(bus):
    left, right, left_ok, right_ok = read_status(bus)
    print(f"  Arduino goals: {left} {right}   at goal: left={left_ok} right={right_ok}")


def cycle(bus, period):
    """Step through the four quadrants forever."""
    while True:
        for quadrant, (left, right) in QUADRANT_GOALS.items():
            print(f"{quadrant}: Goal Position: {left} {right}")
            send_goals(bus, left, right)
            time.sleep(0.05)
            print_status(bus)
            time.sleep(period)
            print_status(bus)


def interactive(bus):
    """Send goals typed at the keyboard."""
    while True:
        text = input("left right (e.g. 0 1), s for status, q to quit: ").strip().lower()
        if text == "q":
            return
        if text == "s":
            print_status(bus)
            continue
        parts = text.split()
        if len(parts) != 2 or any(p not in ("0", "1") for p in parts):
            print("  enter two goals, each 0 or 1")
            continue
        send_goals(bus, int(parts[0]), int(parts[1]))
        time.sleep(0.05)
        print_status(bus)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="type goals instead of cycling through quadrants")
    parser.add_argument("-p", "--period", type=float, default=3.0,
                        help="seconds per quadrant when cycling (default 3)")
    args = parser.parse_args()

    with SMBus(I2C_BUS) as bus:
        try:
            if args.interactive:
                interactive(bus)
            else:
                cycle(bus, args.period)
        except KeyboardInterrupt:
            pass
        finally:
            # Leave both wheels at 0 so the system is ready for the next run
            send_goals(bus, 0, 0)
            print("\nSent 0 0")


if __name__ == "__main__":
    main()
