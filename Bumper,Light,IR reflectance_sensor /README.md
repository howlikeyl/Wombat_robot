# Lab 2: Bumper, Light and IR Reflectance Sensors

## Overview

This lab was about learning to use the Wombat's sensors. The robot does four things:

- Avoids obstacles using bump sensors
- Drives toward a flashlight using a light sensor
- Seeks light while still avoiding obstacles (sensor fusion)
- Follows a black tape line using an IR reflectance sensor

Everything is in one file, `main.py`. You choose what the robot does by changing `MODE` at the top of the file.

The handout asked for two light sensors, but I used one. With only one sensor the robot can't compare left and right, so it spins in a circle to find the brightest direction instead.

## Hardware

- KIPR Wombat controller
- Two drive motors (left on port 3, right on port 0)
- Three bumper switches: left (digital 9), front (digital 2), right (digital 0)
- One light sensor on analog port 3
- One IR reflectance sensor on analog port 0
- A flashlight as the light source

## How It Works

**Obstacle avoidance:** The robot drives forward until a bumper is pressed. If the left bumper hits, it backs up and turns right. If the right bumper hits, it backs up and turns left. A front hit makes it turn the opposite way from its last turn. If several bumpers are pressed at once, or it gets hit three times within a few seconds, it does a bigger turn to get out of a corner.

**Light seeking:** The robot spins one full circle while recording the light level. Then it turns back toward the brightest spot and drives toward it. It slows down as the light gets brighter and stops when it's close. If the light drops off while driving, or after 10 seconds, it scans again. The turns are based on timing, so I measured how long one full spin takes with the `measure_spin` mode.

**Sensor fusion:** This combines the two behaviors above. Bumpers always come first. After hitting something, the robot drives straight for a moment so the light doesn't pull it right back into the obstacle, then starts a new scan.

**Line following:** With one sensor, the robot follows the left edge of the tape instead of the middle. It reads how dark the surface is. Too much black means turn left and too much white means turn right, and the steering gets stronger the farther it is from the edge. If it loses the line, it drives forward a bit, then sweeps side to side to find it again, and stops if it can't.

There are also calibration and test modes for checking the bumpers, light sensor, IR sensor and each motor on its own.

## Setup

1. Connect to the Wombat and open the KIPR IDE at `192.168.125.1:8888`.
2. Create a Python project and add `main.py`.
3. Run `test_left_motor` and `test_right_motor` to check that the wheels spin the right way.
4. Run `calibrate` and copy the printed light and IR values into the top of the file.
5. Run `measure_spin` a few times and set `LIGHT_SCAN_DURATION_MS` to the average.
6. Set `MODE` to the behavior you want and run it:

```python
MODE = "fusion"   # or "obstacle", "light", "line"
```

Press Stop in the IDE to end the program. The motors always shut off when it stops.

## Files

- [main.py](main.py) is the full program
- [F26 - COMP4520 - Lab 2.pdf](F26%20-%20COMP4520%20-%20Lab%202.pdf) is the lab handout
- [Bumper_sensor.MOV](Bumper_sensor.MOV) is the obstacle avoidance demo
- [LightSeeking.MOV](LightSeeking.MOV) is the light seeking demo
- [line_following.MOV](line_following.MOV) is the line following demo

The video descriptions come from the filenames.

## Notes

- Light seeking is slow because one full scan takes about 15 seconds.
- The IR sensor's white and black readings were very close together, so line following can be sensitive to the sensor height and the surface.
- Turns are timed, so battery level and the floor surface affect how accurate they are.
