# Lab 3: Sharp IR Sensor and Servo Motors

## Overview

This lab uses Sharp IR distance sensors for two behaviors:

- Wall following, in a two-state version and a three-state version
- Looking for open space with a distance sensor mounted on a servo

Like Lab 2, everything is in `main.py`, and you pick the behavior with `MODE` at the top of the file.

## Hardware

- KIPR Wombat controller
- Two drive motors (left on port 3, right on port 0)
- A Sharp IR sensor facing left for wall following, on analog port 5
- A Sharp IR sensor on the front servo for scanning, on analog port 4
- A servo on port S0

With the Sharp IR, a higher reading means the object is closer. Below about 10 cm the reading drops again, so the code avoids getting that close. I measured both sensors at 10, 15, 20 and 30 cm and wrote the values in the code comments.

## How It Works

**Two-state wall following:** The robot tries to stay about 15 cm from the wall on its left. If it's too close, it steers right. If it's too far, it steers left.

**Three-state wall following:** This is the same idea with a third "just right" state for roughly 14 to 16 cm, where the robot drives straight instead of always turning.

Both versions steer in short pulses and then go straight for a moment, so the robot doesn't over-turn before the sensor catches up. The front sensor also watches for a wall straight ahead, like in a corner. If it sees one, the robot spins away from the wall until the path is clear.

**Open space:** The robot stops and points the servo in five directions: left, half left, center, half right and right. It reads the distance at each one and picks the farthest direction, with clear space on both sides of it. Ties go to the straightest direction. Then it turns that way and drives forward. While driving, it slows down as things get closer and stops at about 30 cm. It also glances to the sides every second or so. Every time it stops, it scans again.

Turns are timed. I measured how many milliseconds of spinning it takes to turn one degree and used that to turn toward each direction.

## Setup

1. Connect to the Wombat and open the KIPR IDE at `192.168.125.1:8888`.
2. Create a Python project and add `main.py`.
3. Run these test modes first:
   - `test_buttons` checks that A and B work
   - `test_motors` checks the wheel directions
   - `test_servo` finds the servo's left, center and right positions
4. Run `calibrate_ir` to measure the sensors, and `test_turn` to measure turning.
5. Set `MODE` and run:

```python
MODE = "open_space"   # or "wall_follow_2state", "wall_follow_3state"
```

Press A on the Wombat to start and B to stop. The motors and servo always turn off at the end.

## Notes

- Open space mode only allows 5 seconds of driving per run. You can change this with `MOVE_TIME_LIMIT_MS`.
- Wall following keeps going until B is pressed.
- The scan angles and some distance thresholds (like 14 and 16 cm) are estimates, not measurements, so turns may be a little off.
- Only the left side is set up for wall following.
