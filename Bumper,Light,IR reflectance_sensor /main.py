


#   "obstacle"     - Part 1: bumper obstacle avoidance
#   "light"        - Part 2: single-sensor scanning light seeking
#   "fusion"       - Part 2: light seeking + obstacle avoidance
#   "line"         - Part 3: single-sensor line following
#   "calibrate"    - guided calibration of the light sensor and IR sensor
#   "measure_spin" - spin at LIGHT_SCAN_SPEED; press A after exactly one
#                    full turn to measure LIGHT_SCAN_DURATION_MS
#   "monitor"      - print every sensor continuously (no motors)
#   "test_bumpers", "test_light", "test_ir",
#   "test_left_motor", "test_right_motor"  - individual component tests
MODE = "fusion"

LEFT_MOTOR_PORT = 3       # motor port of the LEFT drive wheel
RIGHT_MOTOR_PORT = 0      # motor port of the RIGHT drive wheel

LEFT_BUMPER_PORT = 9      # digital port of the LEFT bumper switch
FRONT_BUMPER_PORT = 2     # digital port of the FRONT bumper switch
RIGHT_BUMPER_PORT = 0     # digital port of the RIGHT bumper switch

LIGHT_SENSOR_PORT = 3     # analog port of the single light sensor

IR_SENSOR_PORT = 0        # analog port of the IR reflectance (line) sensor

# ---------------------------------------------------------------------------
# Motor directions
# ---------------------------------------------------------------------------
# Run "test_left_motor" / "test_right_motor". The test commands the wheel
# FORWARD first. If a wheel spins backwards, set its flag to True.
LEFT_MOTOR_REVERSED = True   # TODO: verify with test_left_motor
RIGHT_MOTOR_REVERSED = True  # TODO: verify with test_right_motor

# ---------------------------------------------------------------------------
# Bumper activation value
# ---------------------------------------------------------------------------
# digital() returns 0 or 1. Run "test_bumpers" and press each switch to see
# which value means "pressed" on your wiring.
BUMPER_PRESSED_VALUE = 1  # verify with test_bumpers

# ---------------------------------------------------------------------------
# Motor speeds (percent, -100 .. 100)
# ---------------------------------------------------------------------------
FORWARD_SPEED = 30
REVERSE_SPEED = 50        # magnitude; the code applies the minus sign
TURN_SPEED = 40           # wheel speed for in-place turns

# ---------------------------------------------------------------------------
# Obstacle avoidance timing (milliseconds)
# ---------------------------------------------------------------------------
REVERSE_DURATION_MS = 2000          # back-up time after a single bumper hit
TURN_DURATION_MS = 500             # in-place turn after a single bumper hit
RECOVERY_REVERSE_DURATION_MS = 900 # back-up time when several bumpers hit
RECOVERY_TURN_DURATION_MS = 1000   # bigger turn (roughly 180 deg) for recovery

# Repeated-collision protection: if REPEAT_BUMP_LIMIT hits happen within
# REPEAT_BUMP_WINDOW_S seconds, the next maneuver uses the recovery durations
# so the robot gets out of corners instead of bouncing in place.
REPEAT_BUMP_WINDOW_S = 4.0
REPEAT_BUMP_LIMIT = 3

LOOP_DELAY_MS = 20  # pause between sensor reads in every control loop

# ---------------------------------------------------------------------------
# Light sensor calibration  (fill in from MODE = "calibrate")
# ---------------------------------------------------------------------------
# DARK  = reading with the sensor shaded / pointed away from any light
# BRIGHT = reading with the flashlight shining straight into the sensor
# The code does NOT assume brighter = higher or lower: normalization uses
# whichever direction these two numbers go.
LIGHT_DARK = 4053
LIGHT_BRIGHT = 64

# ---------------------------------------------------------------------------
# Light seeking behavior
# ---------------------------------------------------------------------------
# One sensor cannot compare left and right, so the robot SCANS: it spins one
# full turn while sampling the light, turns to the brightest direction, drives
# toward it for a while, then scans again. "Level" below means the normalized
# light reading: 0.0 = calibrated dark, 1.0 = calibrated bright.

# Wheel speed while spinning in place to scan.
LIGHT_SCAN_SPEED = 30

# Time in milliseconds for the robot to spin exactly ONE full turn (360 deg)
# at LIGHT_SCAN_SPEED. The scan uses it to convert "when the brightest
# reading was seen" into "how far to turn back". Measure it on your robot
# with MODE = "measure_spin"; it depends on battery, wheels and floor.
LIGHT_SCAN_DURATION_MS = 14557  # from measure_spin

LIGHT_DRIVE_SPEED = 40   # forward speed while driving toward the light
LIGHT_DRIVE_MS = 10000    # drive this long, then scan again to correct the heading

# While driving toward the light, if the level falls this far below the
# brightest level seen so far (e.g. best 0.60, now below 0.40), the robot has
# drifted off course, so it stops driving and scans again immediately.
LIGHT_LOST_DROP = 0.40

LIGHT_STOP_LEVEL = 0.90    # level at which the robot has reached the light and stops
LIGHT_SLOW_LEVEL = 0.70    # above this level, approach at reduced speed
LIGHT_SLOW_FACTOR = 0.5    # speed multiplier while approaching slowly
LIGHT_DARK_LEVEL = 0.10    # brightest scan level below this means no light was found
LIGHT_DARK_WAIT_MS = 1000  # when no light was found, wait this long, then scan again

# Sensor fusion: after backing away from an obstacle, drive straight this long
# (still watching the bumpers) before light seeking takes over again. Without
# it the light pulls the robot straight back into the same obstacle.
FUSION_ESCAPE_FORWARD_MS = 800

# ---------------------------------------------------------------------------
# IR reflectance sensor calibration  (fill in from MODE = "calibrate")
# ---------------------------------------------------------------------------
IR_WHITE_VALUE = 3604  # reading over white paper
IR_BLACK_VALUE = 3794  # reading over black electrical tape
# Fraction of the white-black range treated as the "edge" dead zone around
# the midpoint (0.2 = middle 20 %).
IR_DEAD_ZONE = 0.2

# ---------------------------------------------------------------------------
# Line following behavior (one IR sensor, edge following)
# ---------------------------------------------------------------------------
LINE_EDGE_SIDE = "left"     # which edge of the tape to follow: "left" or "right"
LINE_BASE_SPEED = 5        # forward speed while on the edge
LINE_STEER_GAIN = 40        # how hard to steer per unit of error
LINE_LOST_MS = 400          # continuously on white this long -> line lost
LINE_GAP_FORWARD_MS = 300   # recovery step 1: drive straight to bridge small gaps
LINE_SEARCH_TURN_MS = 700   # recovery step 2: sweep toward the line side
LINE_SEARCH_SPEED = 35      # wheel speed during the recovery sweep

# ---------------------------------------------------------------------------
# Calibration / sensor reading
# ---------------------------------------------------------------------------
ANALOG_SAMPLES = 5          # readings averaged per analog measurement
ANALOG_MIN = 0              # Wombat analog range is 12-bit
ANALOG_MAX = 4095
CALIBRATION_SAMPLES = 50    # readings averaged per calibration capture
MONITOR_PRINT_MS = 250      # print interval in monitor/test modes
MOTOR_TEST_MS = 500         # how long each motor test spins the wheel
MOTOR_TEST_SPEED = 15       # low speed for test_left_motor / test_right_motor
MEASURE_SPIN_MAX_MS = 15000 # measure_spin stops by itself after this long

# ======================================================================
# ROBOT_CONTROL
# ======================================================================
"""
robot_control.py - Motor control and shared helpers.

Only these _kipr functions are used anywhere in the project. They are the
Python (SWIG) bindings of libkipr's motor.h, digital.h, analog.h, time.h and
button.h:  k.motor, k.off, k.ao, k.digital, k.analog, k.msleep, k.a_button
"""
import sys
import time

# Import the Wombat library the same way the KIPR IDE template does. On a
# normal computer it is missing; k stays None so the pure decision-making
# functions can still be unit-tested (see test_logic.py).
sys.path.append("/usr/lib")
try:
    import _kipr as k
except ImportError:
    k = None



def log(*parts):
    """print() with flush so text shows up immediately in the KIPR console."""
    print(*parts)
    sys.stdout.flush()


def require_kipr():
    if k is None:
        raise RuntimeError("_kipr library not found - this must run on the Wombat.")


def wait_ms(milliseconds):
    """Pause using the KIPR msleep (falls back to time.sleep off the robot)."""
    if k is not None:
        k.msleep(int(milliseconds))
    else:
        time.sleep(milliseconds / 1000.0)


def now_seconds():
    """Monotonic time in seconds, used for all timers."""
    return time.monotonic()


def clamp(value, low, high):
    return max(low, min(high, value))


def check_settings(settings):
    """
    settings: list of (name, value) pairs from config.py.
    Returns the names that are still None placeholders.
    """
    return [name for name, value in settings if value is None]


# ---------------------------------------------------------------------------
# Motors
# ---------------------------------------------------------------------------
def set_motor(port, speed, reversed_motor):
    """Drive one motor at -100..100, flipping the sign if it is mounted backwards."""
    require_kipr()
    speed = int(clamp(speed, -100, 100))
    if reversed_motor:
        speed = -speed
    k.motor(port, speed)


def set_wheel_speeds(left_speed, right_speed):
    """
    Set both drive wheels (-100..100). Positive always means "robot forward";
    the *_MOTOR_REVERSED flags in config.py fix wheels mounted backwards.
    """
    set_motor(LEFT_MOTOR_PORT, left_speed, LEFT_MOTOR_REVERSED)
    set_motor(RIGHT_MOTOR_PORT, right_speed, RIGHT_MOTOR_REVERSED)


def drive_forward(speed=FORWARD_SPEED):
    set_wheel_speeds(speed, speed)


def drive_reverse(speed=REVERSE_SPEED):
    set_wheel_speeds(-speed, -speed)


def turn_left(speed=TURN_SPEED):
    """Spin in place to the left (left wheel back, right wheel forward)."""
    set_wheel_speeds(-speed, speed)


def turn_right(speed=TURN_SPEED):
    """Spin in place to the right (left wheel forward, right wheel back)."""
    set_wheel_speeds(speed, -speed)


def stop_motors():
    """Turn every motor off. Safe to call even if ports are not configured."""
    if k is not None:
        k.ao()

# ======================================================================
# SENSORS
# ======================================================================
"""
sensors.py - Reading bumpers, light sensors and the IR reflectance sensor.

Raw reads go through small wrappers that average analog samples and reject
values outside the Wombat's 0-4095 range, so one glitchy reading does not
make the robot jerk.
"""


def read_analog(port, samples=ANALOG_SAMPLES):
    """
    Average several analog readings. Out-of-range samples are discarded.
    Returns None if every sample was invalid (sensor unplugged/faulty).
    """
    require_kipr()
    valid = []
    for _ in range(samples):
        value = k.analog(port)
        if ANALOG_MIN <= value <= ANALOG_MAX:
            valid.append(value)
    if not valid:
        log("WARNING: no valid reading from analog port", port)
        return None
    return sum(valid) / len(valid)


def is_bumper_pressed(port):
    require_kipr()
    return k.digital(port) == BUMPER_PRESSED_VALUE


def read_bumpers():
    """Returns (left, front, right) as booleans: True = pressed."""
    return (is_bumper_pressed(LEFT_BUMPER_PORT),
            is_bumper_pressed(FRONT_BUMPER_PORT),
            is_bumper_pressed(RIGHT_BUMPER_PORT))


def read_light_raw():
    """Raw reading of the single light sensor; None on a read error."""
    return read_analog(LIGHT_SENSOR_PORT)


def read_ir_raw():
    return read_analog(IR_SENSOR_PORT)


def normalize(raw, low_end, high_end):
    """
    Map a raw reading to 0.0 .. 1.0 where low_end -> 0.0 and high_end -> 1.0.

    Works whichever way the sensor goes: for a light sensor pass
    (dark, bright); if your sensor reads LOWER in bright light, bright < dark
    and the math still gives 1.0 for bright. Result is clamped to 0..1.
    """
    if raw is None or low_end is None or high_end is None or high_end == low_end:
        return 0.0
    fraction = (raw - low_end) / float(high_end - low_end)
    return max(0.0, min(1.0, fraction))

# ======================================================================
# OBSTACLE_AVOIDANCE
# ======================================================================
"""
obstacle_avoidance.py - Part 1: bumper-based obstacle avoidance.

Behavior (reactive, rule-based):
    no bumper          -> drive forward
    left bumper        -> back up, turn RIGHT (away from the hit)
    right bumper       -> back up, turn LEFT
    front bumper       -> back up, turn the opposite way to the previous turn
    2+ bumpers at once -> back up farther, make a big recovery turn
    repeated hits      -> treated like a multi-bumper hit (escape the corner)

The decision is made by choose_avoidance_maneuver(), a pure function with no
hardware calls, so it can be tested on a computer (test_logic.py).
"""

OBSTACLE_REQUIRED_SETTINGS = [
    ("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT), ("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT),
    ("LEFT_BUMPER_PORT", LEFT_BUMPER_PORT), ("FRONT_BUMPER_PORT", FRONT_BUMPER_PORT),
    ("RIGHT_BUMPER_PORT", RIGHT_BUMPER_PORT),
]


def choose_avoidance_maneuver(left, front, right, previous_turn, recent_bump_count):
    """
    Decide how to escape from a collision.

    left/front/right   : bumper states (True = pressed)
    previous_turn      : "left" or "right" - direction of the last escape turn
    recent_bump_count  : hits within the repeat window, including this one

    Returns None if no bumper is pressed, otherwise a tuple
        (reverse_ms, turn_direction, turn_ms, reason)
    """
    pressed_count = int(left) + int(front) + int(right)
    if pressed_count == 0:
        return None

    alternate_turn = "left" if previous_turn == "right" else "right"

    # Several bumpers at once (e.g. hit a wall head-on, or wedged in a corner)
    # or many hits in a short time: one side-turn is not enough, so back up
    # farther and turn about 180 degrees. Alternating the direction stops the
    # robot from repeating the exact same failed escape.
    if pressed_count >= 2 or recent_bump_count >= REPEAT_BUMP_LIMIT:
        if left and not right:
            direction = "right"      # left side still involved -> go right
        elif right and not left:
            direction = "left"
        else:
            direction = alternate_turn
        reason = "multiple bumpers" if pressed_count >= 2 else "repeated collisions"
        return (RECOVERY_REVERSE_DURATION_MS, direction, RECOVERY_TURN_DURATION_MS, reason)

    if left:
        return (REVERSE_DURATION_MS, "right", TURN_DURATION_MS, "left bumper")
    if right:
        return (REVERSE_DURATION_MS, "left", TURN_DURATION_MS, "right bumper")

    # Front only: no side information, so alternate the turn direction.
    return (REVERSE_DURATION_MS, alternate_turn, TURN_DURATION_MS, "front bumper")


class CollisionHistory:
    """Remembers recent hit times so repeated collisions can be detected."""

    def __init__(self):
        self.hit_times = []
        self.previous_turn = "left"

    def record_hit(self, current_time):
        self.hit_times.append(current_time)
        # Forget hits older than the repeat window.
        self.hit_times = [t for t in self.hit_times
                          if current_time - t <= REPEAT_BUMP_WINDOW_S]
        return len(self.hit_times)


def perform_maneuver(maneuver):
    """Execute (reverse_ms, direction, turn_ms, reason) on the motors."""
    reverse_ms, direction, turn_ms, reason = maneuver
    log("Bump:", reason, "-> reverse", reverse_ms, "ms, turn", direction, turn_ms, "ms")
    drive_reverse()
    wait_ms(reverse_ms)
    if direction == "left":
        turn_left()
    else:
        turn_right()
    wait_ms(turn_ms)
    stop_motors()


def avoid_if_bumped(history):
    """
    Check the bumpers once. If any is pressed, perform the escape maneuver.
    Returns True if a maneuver was performed. Shared with sensor_fusion.py.
    """
    left, front, right = read_bumpers()
    if not (left or front or right):
        return False
    recent = history.record_hit(now_seconds())
    maneuver = choose_avoidance_maneuver(left, front, right, history.previous_turn, recent)
    perform_maneuver(maneuver)
    history.previous_turn = maneuver[1]
    if maneuver[3] == "repeated collisions":
        history.hit_times = []  # escaped the corner; start counting again
    return True


def run_obstacle_avoidance():
    """Main loop for Part 1: drive forward, escape whenever a bumper is hit."""
    log("Obstacle avoidance running. Press Stop in the IDE to end.")
    history = CollisionHistory()
    while True:
        if not avoid_if_bumped(history):
            drive_forward()
        wait_ms(LOOP_DELAY_MS)

# ======================================================================
# LIGHT_SEEKING
# ======================================================================
"""
light_seeking.py - Part 2: light seeking with ONE light sensor (scanning).

With a single sensor the robot cannot compare left and right readings at the
same moment, so a two-sensor Braitenberg vehicle is not possible. Instead the
robot compares readings taken at different ORIENTATIONS:

    SCAN   spin clockwise in place for exactly one full turn
           (LIGHT_SCAN_DURATION_MS, measured with MODE = "measure_spin"),
           recording (time into the scan, light level) every loop.
    TURN   turn to the heading where the level was highest.
    DRIVE  drive toward the light for LIGHT_DRIVE_MS, then SCAN again to
           correct the heading. Rescan early if the light drops (drifted off).
    ARRIVED  level >= LIGHT_STOP_LEVEL: stop; rescan if the light moves away.
    DARK   nothing brighter than LIGHT_DARK_LEVEL was seen: wait, rescan.

Heading bookkeeping: spinning at a constant speed, time is proportional to
angle. A sample taken t seconds into the scan lies t/T of a turn clockwise
from where the scan started (T = full-turn time). When the scan ends the
robot has turned e_end seconds' worth (about one full turn, slightly more
because the loop runs every LOOP_DELAY_MS). To face the brightest sample at
e_best it must turn clockwise by (e_best - e_end) mod T, or counter-clockwise
by T minus that - whichever is shorter.

Light levels are normalized readings: 0.0 = calibrated dark, 1.0 = calibrated
bright, so the code works whether brighter light gives higher or lower raw
values (decided by calibration, not assumed).

LightScanner contains no hardware calls, so it can be tested on a computer.
"""

LIGHT_REQUIRED_SETTINGS = [
    ("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT), ("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT),
    ("LIGHT_SENSOR_PORT", LIGHT_SENSOR_PORT),
    ("LIGHT_DARK", LIGHT_DARK), ("LIGHT_BRIGHT", LIGHT_BRIGHT),
    ("LIGHT_SCAN_DURATION_MS", LIGHT_SCAN_DURATION_MS),
]

SPIN_CLOCKWISE = (LIGHT_SCAN_SPEED, -LIGHT_SCAN_SPEED)          # same as turn_right
SPIN_COUNTERCLOCKWISE = (-LIGHT_SCAN_SPEED, LIGHT_SCAN_SPEED)   # same as turn_left
STOPPED = (0, 0)


def plan_turn_to_best(best_elapsed, end_elapsed, full_turn_time):
    """
    Shortest turn from the scan's end heading to the brightest heading.
    Returns (direction, seconds) with direction "right" (clockwise) or "left".
    """
    clockwise_time = (best_elapsed - end_elapsed) % full_turn_time
    if clockwise_time <= full_turn_time / 2.0:
        return "right", clockwise_time
    return "left", full_turn_time - clockwise_time


class LightScanner:
    """Scan / turn / drive state machine. step() returns wheel speeds."""

    def __init__(self, full_turn_ms=LIGHT_SCAN_DURATION_MS):
        self.full_turn_time = full_turn_ms / 1000.0
        self.restart()

    def restart(self):
        """Forget the current plan (e.g. after an obstacle moved the robot)."""
        self.state = "start"
        self.samples = []
        self.best_level = 0.0
        self.best_elapsed = 0.0
        self.turn_direction = "right"
        self.deadline = 0.0

    def step(self, level, now):
        """level: normalized light 0..1 at this moment; now: time in seconds."""
        # A state change re-dispatches immediately so the new state's command
        # is returned in the same step (at most a few hops per call).
        for _ in range(4):
            command = getattr(self, "_" + self.state)(level, now)
            if command is not None:
                return command
        return STOPPED

    def _begin_scan(self, now):
        self.state = "scan"
        self.scan_start = now
        self.samples = []

    def _start(self, level, now):
        self._begin_scan(now)

    def _scan(self, level, now):
        elapsed = now - self.scan_start
        if elapsed < self.full_turn_time:
            self.samples.append((elapsed, level))
            return SPIN_CLOCKWISE
        # Full turn done: pick the brightest heading (first one if tied).
        self.best_elapsed, self.best_level = max(self.samples, key=lambda s: s[1])
        if self.best_level < LIGHT_DARK_LEVEL:
            self.state = "dark"
            self.deadline = now + LIGHT_DARK_WAIT_MS / 1000.0
            return None
        self.turn_direction, turn_time = plan_turn_to_best(
            self.best_elapsed, elapsed, self.full_turn_time)
        self.state = "turn"
        self.deadline = now + turn_time
        return None

    def _turn(self, level, now):
        # The turn is timed, not sensor-guided: a dim, distant light changes
        # so little near its peak that "close to the best reading" can still
        # be many degrees off. If timing errors (battery, wheel slip) leave
        # the robot off course, the DRIVE state notices the light dropping
        # and rescans.
        if now >= self.deadline:
            self.state = "drive"
            self.deadline = now + LIGHT_DRIVE_MS / 1000.0
            return None
        return SPIN_CLOCKWISE if self.turn_direction == "right" else SPIN_COUNTERCLOCKWISE

    def _drive(self, level, now):
        self.best_level = max(self.best_level, level)  # light grows as we approach
        if level >= LIGHT_STOP_LEVEL:
            self.state = "arrived"
            return None
        if level < self.best_level - LIGHT_LOST_DROP or now >= self.deadline:
            self._begin_scan(now)  # drifted off course, or time to re-check
            return None
        speed = LIGHT_DRIVE_SPEED
        if level >= LIGHT_SLOW_LEVEL:
            speed = int(speed * LIGHT_SLOW_FACTOR)  # close to the light: slow down
        return (speed, speed)

    def _arrived(self, level, now):
        if level < LIGHT_SLOW_LEVEL:  # light moved away: look for it again
            self._begin_scan(now)
            return None
        return STOPPED

    def _dark(self, level, now):
        if now >= self.deadline:
            self._begin_scan(now)
            return None
        return STOPPED


def read_light_level():
    """Normalized light level 0..1 (0 on a read error)."""
    return normalize(read_light_raw(), LIGHT_DARK, LIGHT_BRIGHT)


def light_seeking_step(scanner):
    """One control step on the robot. Returns (level, left_speed, right_speed)."""
    level = read_light_level()
    left_speed, right_speed = scanner.step(level, now_seconds())
    set_wheel_speeds(left_speed, right_speed)
    return level, left_speed, right_speed


def run_light_seeking():
    """Main loop for Part 2 light seeking. Prints status for the lab report."""
    log("Light seeking (single sensor, scanning). Press Stop in the IDE to end.")
    scanner = LightScanner()
    last_print = 0.0
    last_state = None
    while True:
        level, left_speed, right_speed = light_seeking_step(scanner)
        if scanner.state != last_state or now_seconds() - last_print >= MONITOR_PRINT_MS / 1000.0:
            log("%-7s light=%.2f best=%.2f  motors L=%d R=%d"
                % (scanner.state, level, scanner.best_level, left_speed, right_speed))
            last_print = now_seconds()
            last_state = scanner.state
        wait_ms(LOOP_DELAY_MS)

# ======================================================================
# SENSOR_FUSION
# ======================================================================
"""
sensor_fusion.py - Part 2: light seeking with obstacle avoidance.

Priority-based arbitration (subsumption style). Every loop iteration the
highest-priority behavior that wants control wins:

    1. AVOID   - any bumper pressed: interrupt everything, back up and turn
                 away (same maneuver logic as Part 1, all three bumpers).
    2. ESCAPE  - for FUSION_ESCAPE_FORWARD_MS after an avoidance, drive
                 straight ahead so the robot actually moves past the obstacle
                 instead of being pulled straight back into it by the light.
                 Bumpers are still checked every iteration (priority 1).
    3. SEEK    - otherwise, single-sensor scanning light seeking.

Bumpers are checked before every light-seeking step, so a bump interrupts a
scan, a turn or a drive toward the light. After an avoidance the robot's
heading has changed, so the scanner's remembered "brightest direction" is
wrong; it is restarted and does a fresh scan once the escape is over.
"""

FUSION_REQUIRED_SETTINGS = OBSTACLE_REQUIRED_SETTINGS + LIGHT_REQUIRED_SETTINGS[2:]


def run_sensor_fusion():
    log("Light seeking + obstacle avoidance running. Press Stop to end.")
    history = CollisionHistory()
    scanner = LightScanner()
    escape_until = 0.0
    last_print = 0.0

    while True:
        # Priority 1: avoid_if_bumped() reads all three bumpers and, if any is
        # pressed, performs the full escape maneuver before returning True.
        if avoid_if_bumped(history):
            escape_until = now_seconds() + FUSION_ESCAPE_FORWARD_MS / 1000.0
            scanner.restart()  # heading changed: old light direction is invalid
            behavior = "avoid"
        elif now_seconds() < escape_until:
            drive_forward()  # priority 2
            behavior = "escape"
        else:
            light_seeking_step(scanner)  # priority 3
            behavior = "seek/" + scanner.state

        if now_seconds() - last_print >= MONITOR_PRINT_MS / 1000.0:
            log("behavior:", behavior)
            last_print = now_seconds()
        wait_ms(LOOP_DELAY_MS)

# ======================================================================
# LINE_FOLLOWING
# ======================================================================
"""
line_following.py - Part 3: follow a black line with ONE IR reflectance sensor.

Why edge following: one sensor cannot tell whether the line drifted left or
right - it only knows "black" or "white". So instead of riding the centre of
the tape, the robot rides one EDGE (LINE_EDGE_SIDE). Over black it steers
off the tape; over white it steers back toward it. Readings in the dead zone
around the midpoint mean "right on the edge", which is where we want to be.

Steering is proportional: the IR reading is normalized to a "darkness"
0.0 (white) .. 1.0 (black); error = darkness - 0.5 sets how hard to turn.
This gives a smoother path than a pure bang-bang left/right zig-zag.

Line-lost recovery (sharp corners, gaps in the tape):
    1. drive straight briefly  - bridges small gaps in the line
    2. sweep toward the line side
    3. sweep the other way (twice as long, to cover the start + other side)
    4. still nothing -> stop the robot
"""

LINE_REQUIRED_SETTINGS = [
    ("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT), ("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT),
    ("IR_SENSOR_PORT", IR_SENSOR_PORT),
    ("IR_WHITE_VALUE", IR_WHITE_VALUE), ("IR_BLACK_VALUE", IR_BLACK_VALUE),
]


def classify_surface(darkness):
    """darkness 0..1 -> "white", "black" or "edge" (inside the dead zone)."""
    half_zone = IR_DEAD_ZONE / 2.0
    if darkness < 0.5 - half_zone:
        return "white"
    if darkness > 0.5 + half_zone:
        return "black"
    return "edge"


def line_side():
    """Direction from the sensor to the tape while the sensor is on white."""
    return "right" if LINE_EDGE_SIDE == "left" else "left"


def line_steering(darkness):
    """
    Proportional edge follower. Returns (left_speed, right_speed).
    Following the LEFT edge, the tape is to the robot's right, so too much
    black means turn left and too much white means turn right.
    """
    error = darkness - 0.5            # + : too far onto the tape
    turn = LINE_STEER_GAIN * error    # + : turn left (for the left edge)
    if LINE_EDGE_SIDE == "right":
        turn = -turn
    return int(LINE_BASE_SPEED - turn), int(LINE_BASE_SPEED + turn)


def read_darkness():
    """IR reading normalized to 0 (white) .. 1 (black); None on read error."""
    raw = read_ir_raw()
    if raw is None:
        return None
    return normalize(raw, IR_WHITE_VALUE, IR_BLACK_VALUE)


def search_for_line(motion, duration_ms):
    """Run a motion until the sensor leaves white or the time runs out."""
    motion()
    end_time = now_seconds() + duration_ms / 1000.0
    while now_seconds() < end_time:
        darkness = read_darkness()
        if darkness is not None and classify_surface(darkness) != "white":
            stop_motors()
            return True
        wait_ms(LOOP_DELAY_MS)
    stop_motors()
    return False


def recover_line():
    """Try to find the line again. Returns True if it was found."""
    toward, away = (turn_right, turn_left) if line_side() == "right" else (turn_left, turn_right)
    log("Line lost - trying straight ahead (gap)")
    if search_for_line(lambda: drive_forward(LINE_BASE_SPEED), LINE_GAP_FORWARD_MS):
        return True
    log("Line lost - sweeping toward the", line_side())
    if search_for_line(lambda: toward(LINE_SEARCH_SPEED), LINE_SEARCH_TURN_MS):
        return True
    log("Line lost - sweeping the other way")
    if search_for_line(lambda: away(LINE_SEARCH_SPEED), 2 * LINE_SEARCH_TURN_MS):
        return True
    return False


def run_line_following():
    log("Line following (%s edge) running. Press Stop to end." % LINE_EDGE_SIDE)
    white_since = None
    last_print = 0.0
    while True:
        darkness = read_darkness()
        if darkness is None:
            stop_motors()  # sensor error: do not drive blind
            wait_ms(LOOP_DELAY_MS)
            continue

        surface = classify_surface(darkness)
        if surface == "white":
            if white_since is None:
                white_since = now_seconds()
            elif now_seconds() - white_since >= LINE_LOST_MS / 1000.0:
                if not recover_line():
                    log("Line could not be found. Stopping.")
                    stop_motors()
                    return
                white_since = None
                continue
        else:
            white_since = None

        left_speed, right_speed = line_steering(darkness)
        set_wheel_speeds(left_speed, right_speed)

        if now_seconds() - last_print >= MONITOR_PRINT_MS / 1000.0:
            log("IR darkness=%.2f (%s)  motors L=%d R=%d"
                % (darkness, surface, left_speed, right_speed))
            last_print = now_seconds()
        wait_ms(LOOP_DELAY_MS)

# ======================================================================
# CALIBRATION
# ======================================================================
"""
calibration.py - Sensor monitor and guided calibration.

monitor:   prints every configured sensor plus the min/max seen so far
           (useful for lab questions 2a-2f and 3a). Motors are not used.
calibrate: step-by-step capture of light DARK/BRIGHT values and IR
           WHITE/BLACK values. Press the Wombat's on-screen A button at each
           step. Prints the lines to copy into config.py.
measure_spin: spins the robot clockwise at LIGHT_SCAN_SPEED; press A when it
           has turned exactly once. Prints LIGHT_SCAN_DURATION_MS.
"""


def wait_for_a_button(prompt):
    """Show a prompt and wait for a full press-and-release of the A button."""
    require_kipr()
    log("")
    log(prompt)
    log("  -> press the A button on the Wombat screen when ready")
    while not k.a_button():
        wait_ms(20)
    while k.a_button():
        wait_ms(20)


def capture_average(port):
    """Average CALIBRATION_SAMPLES readings from an analog port."""
    readings = []
    for _ in range(CALIBRATION_SAMPLES):
        value = read_analog(port)
        if value is not None:
            readings.append(value)
        wait_ms(10)
    if not readings:
        log("ERROR: port", port, "gave no valid readings - check the cable.")
        return None
    return int(round(sum(readings) / len(readings)))


def describe_direction(name, low_label, low_value, high_label, high_value):
    """Report which direction the sensor moves and warn about a weak contrast."""
    if low_value is None or high_value is None:
        return
    difference = high_value - low_value
    direction = "HIGHER" if difference > 0 else "LOWER"
    log("%s: %s reads %s than %s (difference %d)"
        % (name, high_label, direction, low_label, abs(difference)))
    if abs(difference) < 0.05 * ANALOG_MAX:
        log("  WARNING: very small difference - check mounting/shielding/height.")


def calibrate_light_sensor():
    if LIGHT_SENSOR_PORT is None:
        log("Skipping light calibration: set LIGHT_SENSOR_PORT first.")
        return
    wait_for_a_button("LIGHT 1/2: shade the light sensor (flashlight off / pointed away).")
    dark = capture_average(LIGHT_SENSOR_PORT)
    log("  dark=%s" % dark)
    wait_for_a_button("LIGHT 2/2: shine the flashlight straight into the light sensor (close).")
    bright = capture_average(LIGHT_SENSOR_PORT)
    log("  bright=%s" % bright)
    describe_direction("Light sensor", "dark", dark, "bright", bright)
    log("")
    log("Copy into config.py:")
    log("LIGHT_DARK = %s" % dark)
    log("LIGHT_BRIGHT = %s" % bright)


def calibrate_ir_sensor():
    if IR_SENSOR_PORT is None:
        log("Skipping IR calibration: set IR_SENSOR_PORT first.")
        return
    wait_for_a_button("IR 1/2: place the robot so the IR sensor is over WHITE paper.")
    white = capture_average(IR_SENSOR_PORT)
    log("  white=%s" % white)
    wait_for_a_button("IR 2/2: place the IR sensor over the middle of the BLACK tape.")
    black = capture_average(IR_SENSOR_PORT)
    log("  black=%s" % black)
    describe_direction("IR sensor", "white", white, "black", black)
    log("")
    log("Copy into config.py:")
    log("IR_WHITE_VALUE = %s" % white)
    log("IR_BLACK_VALUE = %s" % black)


def run_calibration():
    log("=== Guided sensor calibration ===")
    calibrate_light_sensor()
    calibrate_ir_sensor()
    log("")
    log("Calibration finished. Edit config.py, then re-run.")


def run_measure_spin():
    """
    Measure LIGHT_SCAN_DURATION_MS: the time for one full clockwise turn at
    LIGHT_SCAN_SPEED (the same motion the light scan uses). THE ROBOT MOVES.
    """
    log("=== Measure one full spin at speed %d ===" % LIGHT_SCAN_SPEED)
    log("Put the robot on the floor where it can spin freely. Mark the")
    log("direction it faces (e.g. a piece of tape in front of it).")
    wait_for_a_button("Press A to START spinning; press A AGAIN when it faces the mark again.")
    start = now_seconds()
    turn_right(LIGHT_SCAN_SPEED)
    while not k.a_button():
        if (now_seconds() - start) * 1000.0 >= MEASURE_SPIN_MAX_MS:
            stop_motors()
            log("No button press within %d ms - stopped. Nothing measured." % MEASURE_SPIN_MAX_MS)
            return
        wait_ms(10)
    elapsed_ms = int(round((now_seconds() - start) * 1000.0))
    stop_motors()
    log("One full turn took %d ms." % elapsed_ms)
    log("")
    log("Copy into config.py (repeat 2-3 times and use the average):")
    log("LIGHT_SCAN_DURATION_MS = %d" % elapsed_ms)


def run_sensor_monitor():
    """Print raw values of every configured sensor with running min/max."""
    log("Sensor monitor (motors off). Press Stop to end.")
    analog_ports = [("light", LIGHT_SENSOR_PORT), ("IR", IR_SENSOR_PORT)]
    bumper_ports = [("bumpL", LEFT_BUMPER_PORT), ("bumpF", FRONT_BUMPER_PORT), ("bumpR", RIGHT_BUMPER_PORT)]
    analog_ports = [(n, p) for n, p in analog_ports if p is not None]
    bumper_ports = [(n, p) for n, p in bumper_ports if p is not None]
    if not analog_ports and not bumper_ports:
        log("No sensor ports are set in config.py yet.")
        return
    lowest, highest = {}, {}
    while True:
        parts = []
        for name, port in bumper_ports:
            parts.append("%s=%d" % (name, 1 if is_bumper_pressed(port) else 0))
        for name, port in analog_ports:
            value = read_analog(port)
            if value is None:
                parts.append("%s=ERR" % name)
                continue
            lowest[name] = min(value, lowest.get(name, value))
            highest[name] = max(value, highest.get(name, value))
            parts.append("%s=%d [%d..%d]" % (name, value, lowest[name], highest[name]))
        log("  ".join(parts))
        wait_ms(MONITOR_PRINT_MS)

# ======================================================================
# HARDWARE_TESTS
# ======================================================================
"""
hardware_tests.py - Test each component on its own (runs on the Wombat).

These only print what the hardware does; YOU judge whether it is correct.
"""

TEST_REQUIRED_SETTINGS = {
    "test_bumpers": [("LEFT_BUMPER_PORT", LEFT_BUMPER_PORT), ("FRONT_BUMPER_PORT", FRONT_BUMPER_PORT),
                     ("RIGHT_BUMPER_PORT", RIGHT_BUMPER_PORT)],
    "test_light": [("LIGHT_SENSOR_PORT", LIGHT_SENSOR_PORT)],
    "test_ir": [("IR_SENSOR_PORT", IR_SENSOR_PORT)],
    "test_left_motor": [("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT)],
    "test_right_motor": [("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT)],
}


def test_bumpers():
    """Press each bumper (and pairs of them) and check the printed names."""
    log("Bumper test: press left, front, right, then two at once. Stop to end.")
    while True:
        left, front, right = read_bumpers()
        pressed = [name for name, state in (("LEFT", left), ("FRONT", front), ("RIGHT", right)) if state]
        label = ", ".join(pressed) if pressed else "none"
        if len(pressed) >= 2:
            label += "   <- MULTIPLE"
        log("pressed:", label)
        wait_ms(MONITOR_PRINT_MS)


def test_light_sensor():
    """Raw value of the light sensor; normalized level once calibrated."""
    calibrated = LIGHT_DARK is not None and LIGHT_BRIGHT is not None
    log("Light test: shade the sensor, then shine the flashlight on it. Stop to end.")
    while True:
        raw = read_light_raw()
        line = "light raw=%s" % raw
        if calibrated and raw is not None:
            line += "  level=%.2f (0 = dark, 1 = bright)" % normalize(raw, LIGHT_DARK, LIGHT_BRIGHT)
        log(line)
        wait_ms(MONITOR_PRINT_MS)


def test_ir_sensor():
    calibrated = IR_WHITE_VALUE is not None and IR_BLACK_VALUE is not None
    log("IR test: slide the sensor between white paper and black tape. Stop to end.")
    while True:
        raw = read_ir_raw()
        line = "IR raw=%s" % raw
        if calibrated and raw is not None:
            darkness = normalize(raw, IR_WHITE_VALUE, IR_BLACK_VALUE)
            line += "  darkness=%.2f  -> %s" % (darkness, classify_surface(darkness))
        log(line)
        wait_ms(MONITOR_PRINT_MS)


def test_one_motor(name, port, reversed_motor):
    """Spin one wheel FORWARD, then BACKWARD. Lift the robot off the table first."""
    log("%s motor test (port %d). Wheel should first roll the robot FORWARD." % (name, port))
    set_motor(port, MOTOR_TEST_SPEED, reversed_motor)
    wait_ms(MOTOR_TEST_MS)
    stop_motors()
    wait_ms(500)
    log("...now BACKWARD.")
    set_motor(port, -MOTOR_TEST_SPEED, reversed_motor)
    wait_ms(MOTOR_TEST_MS)
    stop_motors()
    log("Done. If the directions were opposite, flip %s_MOTOR_REVERSED in config.py." % name.upper())


def test_left_motor():
    test_one_motor("Left", LEFT_MOTOR_PORT, LEFT_MOTOR_REVERSED)


def test_right_motor():
    test_one_motor("Right", RIGHT_MOTOR_PORT, RIGHT_MOTOR_REVERSED)

# ======================================================================
# MAIN
# ======================================================================
"""
main.py - COMP 4520 Lab 2 entry point.

Choose the behavior with MODE in config.py, then compile and run in the KIPR
IDE. Before any motor moves, the program checks that every setting the
chosen behavior needs has been filled in, and lists the ones still missing.
"""
import os, sys
sys.path.append("/usr/lib")
# Let Python find the other project files sitting next to this one.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import signal


# mode name -> (function to run, settings that must not be None)
MODES = {
    "obstacle": (run_obstacle_avoidance, OBSTACLE_REQUIRED_SETTINGS),
    "light": (run_light_seeking, LIGHT_REQUIRED_SETTINGS),
    "fusion": (run_sensor_fusion, FUSION_REQUIRED_SETTINGS),
    "line": (run_line_following, LINE_REQUIRED_SETTINGS),
    "calibrate": (run_calibration, []),
    "monitor": (run_sensor_monitor, []),
    "measure_spin": (run_measure_spin, [("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT), ("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT)]),
    "test_bumpers": (test_bumpers, TEST_REQUIRED_SETTINGS["test_bumpers"]),
    "test_light": (test_light_sensor, TEST_REQUIRED_SETTINGS["test_light"]),
    "test_ir": (test_ir_sensor, TEST_REQUIRED_SETTINGS["test_ir"]),
    "test_left_motor": (test_left_motor, TEST_REQUIRED_SETTINGS["test_left_motor"]),
    "test_right_motor": (test_right_motor, TEST_REQUIRED_SETTINGS["test_right_motor"]),
}


def handle_stop_signal(signum, frame):
    # The IDE's Stop button terminates the program; turning the signal into
    # SystemExit makes the finally-block below run so the motors stop.
    raise SystemExit(0)


def main():
    signal.signal(signal.SIGTERM, handle_stop_signal)
    signal.signal(signal.SIGINT, handle_stop_signal)

    if MODE not in MODES:
        log("Unknown MODE '%s'. Choose one of: %s" % (MODE, ", ".join(sorted(MODES))))
        return
    behavior, required_settings = MODES[MODE]

    missing = check_settings(required_settings)
    if missing:
        log("Cannot run '%s' - fill in these settings in config.py:" % MODE)
        for name in missing:
            log("   ", name)
        return

    require_kipr()
    log("Running mode:", MODE)
    try:
        behavior()
    finally:
        stop_motors()
        log("Motors stopped.")


if __name__ == "__main__":
    main()
