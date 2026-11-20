#!/usr/bin/python3
"""
lab3.py - COMP 4520 Lab 3: Sharp IR sensors and servo motors.

Single standalone file for the KIPR IDE. Pick what the robot does with MODE,
then compile and run.

Part I:  two-state and three-state wall following (side Sharp IR, analog 5).
Part II: open-space navigation (front Sharp IR on the servo, analog 4).

Safety: every movement mode waits for A, stops on B, has a time limit, stops
if a sensor reading is invalid, and always turns the motors and servo off
when it ends.
"""
import sys
import signal

# ===========================================================================
# CONFIGURATION - every setting lives here
# ===========================================================================

# Which behavior to run:
#   "test_ir_front" - print the front (servo) Sharp IR on analog 4. No movement.
#   "test_ir_side"  - print the side (wall) Sharp IR on analog 5. No movement.
#   "test_servo"    - move the servo in small steps with the buttons to find
#                     its safe left / center / right positions.
#   "test_motors"   - short, slow test of each wheel, driving and turning.
#   "test_scan"     - press A: the servo visits the five scan positions and
#                     prints the front IR at each one. The robot does not drive.
#   "test_turn"     - press A: spins left and right to calibrate turning.
#   "test_buttons"  - check that the Wombat's A and B buttons work. Prints a
#                     message for each press; B ends it. No movement.
#   "calibrate_ir"  - record both Sharp IR sensors at 10, 15, 20 and 30 cm
#                     after a countdown for each (no A button needed) and
#                     print a table. No movement.
#   "wall_follow_2state" - Part I, two states: too close / too far.
#   "wall_follow_3state" - Part I, three states: too close / correct / too far.
#   "open_space"    - Part II: scan five directions, turn toward the most
#                     open one, drive a short step, repeat.
# Set for the supervised open_space test. It waits for A before anything
# moves; B stops it. (test_ir_side moves nothing at all.)
MODE = "open_space"

# --- Safety: shared by every mode that drives ---
# Every driving mode stops by itself this long after you press A, and B
# stops it at any moment. (test_motors runs fixed 0.6 s steps instead.)
MOVE_TIME_LIMIT_MS = 5000

# --- Motors (copied from the working Lab 2 program) ---
LEFT_MOTOR_PORT = 3
RIGHT_MOTOR_PORT = 0

LEFT_MOTOR_REVERSED = True
RIGHT_MOTOR_REVERSED = True

# --- Sharp IR sensors (analog ports) ---
FRONT_SHARP_IR_PORT = 4   # on the servo, used for scanning
WALL_SHARP_IR_PORT = 5    # on the side, used for wall following

# --- Servo ---
FRONT_SERVO_PORT = 0      # servo cable is in S0

# --- Wall configuration ---
WALL_SIDE = "left"        # the side Sharp IR faces the robot's LEFT

# --- Calibration values (fill in after measuring - do NOT guess) ---
# Side Sharp IR, measured on the robot:
#   10 cm -> 2558
#   15 cm -> 1979
#   20 cm -> 1449
#   30 cm -> 1074
# So a HIGHER reading means the wall is CLOSER (in this range).
WALL_TOO_CLOSE = 2558     # reading at or above this = too close (< ~10 cm)
WALL_TOO_FAR = 1074       # reading at or below this = too far  (> ~30 cm)
# (open_space uses these two; in wall following WALL_TOO_CLOSE is only the
# ~10 cm safety line. The wall-following states use the targets below.)

# Wall-following distance targets: 15 cm from the left wall is ideal.
# Two-state: one target at 15 cm (the measured 15 cm reading).
#   reading >  WALL_2_TARGET -> TOO_CLOSE (closer than 15 cm)
#   reading <= WALL_2_TARGET -> TOO_FAR   (15 cm or farther)
WALL_2_TARGET = 1979
# Three-state: 14-16 cm band around 15 cm. 14 and 16 cm were NOT measured:
# these are ESTIMATES, 1/5 of the way from the measured 15 cm (1979) toward
# 10 cm (2558) and toward 20 cm (1449).
#   reading >  WALL_3_NEAR -> TOO_CLOSE (closer than 14 cm)
#   WALL_3_FAR..WALL_3_NEAR -> CORRECT  (14-16 cm, both ends included)
#   reading <  WALL_3_FAR  -> TOO_FAR   (farther than 16 cm)
WALL_3_NEAR = 2095
WALL_3_FAR = 1873
# Caution zone: ~13 cm. At or above it, the robot does
# not drive forward at all; it corrects by spinning in place AWAY from the
# wall. A robot angled toward the wall keeps drifting closer while it turns
# in a forward arc, and the slanted side beam reads long, so reacting only at
# 10 cm always crossed 10 cm in simulation. None = no caution zone.
# It must be closer than the wall-following targets (15 cm), or the robot
# would spin inside its own target band. 2211 is an ESTIMATE (2/5 of the way
# from the measured 15 cm, 1979, to 10 cm, 2558); 13 cm was NOT measured.
# (~12 cm, 2326, let one simulated angled approach reach 9.9 cm.)
WALL_CAUTION = 2211

# --- Wall following (both the two-state and three-state modes) ---
WALL_MOTORS_ON = True     # False = dry run (print only). True = really drive.
WALL_FAST_SPEED = 30      # outer wheel while steering (percent, keep <= 40)
WALL_SLOW_SPEED = 15      # inner wheel while steering (percent)
WALL_STRAIGHT_SPEED = 25  # both wheels in the three-state CORRECT state
WALL_RUN_MS = None        # None = follow the wall until B is pressed
                          # (a number = also stop by itself after this many ms)
WALL_LOOP_MS = 50         # pause between sensor checks while driving
# Steering is done in short pulses: steer for WALL_STEER_PULSE_MS, then drive
# straight for WALL_STRAIGHT_PULSE_MS, then read the sensor again. This stops
# the robot from swinging round too far before the reading catches up.
# WALL_STEER_PULSE_MS = 0 turns pulses off (steer until the state changes).
WALL_STEER_PULSE_MS = 200
WALL_STRAIGHT_PULSE_MS = 300
# Turn limit. The robot counts its pulses: +1 toward the wall, -1 away from
# it. That count is a rough guess of its angle to the wall. Once the count
# reaches +WALL_MAX_PULSES (or -WALL_MAX_PULSES) it stops steering further
# that way and drives straight instead. Without this, a sideways sensor
# reads "far" while the robot is angled at the wall, so it keeps turning
# into it. NOT tuned on the robot yet.
WALL_MAX_PULSES = 6
# Turn limit release. While the limit blocks steering TOWARD the wall, the
# robot drives straight. If it really is angled toward the wall, the side
# reading rises (wall gets closer). If the reading has NOT risen after this
# long, the count is wrong (e.g. after a WALL AHEAD spin), so the count goes
# down by 1 and one more pulse toward the wall is allowed. Only used with the
# front guard (WALL_USE_FRONT_IR = True). NOT tuned yet.
WALL_LIMIT_RELEASE_MS = 1000
# Front guard. The front Sharp IR is held straight ahead (servo at
# SERVO_CENTER, never scanning) while wall following. If it sees something
# closer than ~30 cm (reading above FRONT_OPEN), the robot spins in place AWAY
# from the wall no matter what the side sensor says. This catches the case the
# side sensor cannot: the robot angled steeply toward the wall, and walls
# ahead at inside corners. False = side sensor only, servo stays off.
WALL_USE_FRONT_IR = True
# When the front guard sees the wall, the robot stops and spins in place away
# from it until the front IR clears, for at most this long. If it still sees
# a wall after that, the robot stops (probably boxed in).
WALL_AHEAD_MAX_SPIN_MS = 2000

# --- Servo positions (fill in after test_servo - do NOT guess) ---
# Measured with test_servo.
SERVO_LEFT = 24          # sensor points to the robot's left
SERVO_CENTER = 874       # sensor points straight ahead
SERVO_RIGHT = 1999       # sensor points to the robot's right

# --- test_servo settings ---
# The servo starts at SERVO_TEST_START (the measured center) and the test
# refuses to move outside SERVO_TEST_MIN..SERVO_TEST_MAX, which are the
# measured left and right limits.
SERVO_TEST_START = 874
SERVO_TEST_STEP = 25      # how far one button press moves the servo
SERVO_TEST_MIN = 24
SERVO_TEST_MAX = 1999

# --- Front Sharp IR (analog 4), measured on the robot ---
#   10 cm -> 2915
#   15 cm -> 2599
#   20 cm -> 1897
#   30 cm -> 926
# A HIGHER reading means the object is CLOSER (from ~10 cm outwards). Closer
# than ~10 cm a Sharp IR reading DROPS again and can look far away, so the
# robot only ever drives toward space it measured as farther than ~30 cm.
FRONT_BLOCKED = 2915      # reading at or above this = blocked (< ~10 cm)
FRONT_OPEN = 926          # reading at or below this = open    (> ~30 cm)
# open_space only drives toward a direction whose NEIGHBOURING scan
# directions are farther than ~20 cm too (reading at or below this, the
# measured 20 cm value). Otherwise the robot can clip a corner or wall just
# beside its path, which neither sensor sees while driving.
FRONT_SIDE_CLEAR = 1897

# --- Open-space scanning ---
# Five servo positions from the robot's LEFT to its RIGHT. They are the
# measured limits, the center, and the halfway points in between.
SCAN_POSITIONS = [SERVO_LEFT,
                  (SERVO_LEFT + SERVO_CENTER) // 2,
                  SERVO_CENTER,
                  (SERVO_CENTER + SERVO_RIGHT) // 2,
                  SERVO_RIGHT]
SCAN_NAMES = ["LEFT", "LEFT-MID", "CENTER", "RIGHT-MID", "RIGHT"]

# Which way each scan position points, in degrees.
# 0 = straight ahead, POSITIVE = robot's LEFT, negative = robot's RIGHT.
# ESTIMATES chosen by the user, NOT measured: turns may be off by the error.
SCAN_ANGLES_DEG = [90, 45, 0, -45, -90]

SERVO_SETTLE_MS = 500     # wait after each servo move before reading
SCAN_TEST_HOLD_MS = 1500  # test_scan holds each position this long

# --- Turning (spin in place) ---
TURN_SPEED = 25           # wheel speed while turning (percent)
TURN_MS_PER_DEG = 66.67   # ms of spinning per degree, measured with test_turn
                          # (None = measure again: TURN_TEST_MS / degrees turned)
TURN_TEST_MS = 1000       # test_turn timing run: spins this long in each direction
TURN_TEST_ANGLE_DEG = 45  # test_turn check: one turn of this many degrees (+ = left)

# --- Open-space driving ---
OPEN_DRIVE_SPEED = 25     # forward cruising speed (percent)
OPEN_SLOW_SPEED = 15      # slowest approach speed, just before stopping at ~30 cm
# Slowing down: full speed while the front reads at or below FRONT_SLOW_START,
# then slower and slower until FRONT_OPEN (~30 cm, measured), where it stops.
# 600 is an ESTIMATE of the ~40 cm reading (curve fitted to the 20 and 30 cm
# measurements). It only changes the speed; the stop is still at 30 cm.
FRONT_SLOW_START = 600
OPEN_FORWARD_MS = 1500    # (unused by open_space now: it drives until something
                          # needs a decision; kept for drive_forward_watching's default)
# While driving, about every OPEN_GLANCE_MS the servo glances at one 45 degree
# diagonal (RIGHT-MID, then LEFT-MID, ...) WITHOUT stopping, at
# OPEN_SLOW_SPEED. If that diagonal is closer than ~20 cm (FRONT_SIDE_CLEAR),
# the robot stops and scans. This catches walls and corners beside the path.
OPEN_GLANCE_MS = 1000     # 1500 let the fast simulated robot get within 9.8 cm
OPEN_LOOP_MS = 50         # how often the front IR is checked while driving
OPEN_RUN_MS = MOVE_TIME_LIMIT_MS   # safety: total MOTOR-ON time (turning +
                          # driving) per run. Scanning (robot standing still,
                          # only the servo moves) does not count. A turn is only
                          # started if it fits in the time that is left.
OPEN_MAX_STUCK = 3        # stop after this many scans in a row with no driving
# Side corridor entry (LEFT side only: the side IR faces left). While driving
# straight, the side IR reading at or below WALL_TOO_FAR (> ~30 cm) for this
# many checks in a row, after a wall was seen, marks the start of an opening.
OPEN_OPENING_CHECKS = 3
OPEN_MAX_SIDE_TURNS = 1   # at most this many corridor entries per run
TEST_RUN_MS = 30000       # time limit for test_scan (servo only, no driving)

# --- test_buttons settings ---
BUTTON_TEST_MS = 60000    # give up after this long with no button press at all
BUTTON_REMINDER_MS = 10000  # print a "still waiting" line this often

# --- calibrate_ir settings ---
CALIBRATE_DISTANCES_CM = [10, 15, 20, 30]   # distances to record, in order
CALIBRATE_SAMPLES = 5     # averaged readings combined into one recorded value
CALIBRATE_COUNTDOWN_S = 15  # seconds to place the object before each recording

# --- test_motors settings (slow and short on purpose) ---
MOTOR_TEST_SPEED = 25     # percent of full power
MOTOR_TEST_MS = 600       # how long each test step runs

# --- Sensor reading ---
ANALOG_SAMPLES = 10       # readings averaged per measurement (Sharp IR is noisy)
ANALOG_MIN = 0            # Wombat analog range is 12-bit
ANALOG_MAX = 4095
PRINT_INTERVAL_MS = 250   # how often the test modes print


# ===========================================================================
# KIPR LIBRARY
# ===========================================================================
# Lab 2 imported the Wombat library as _kipr from /usr/lib, and that worked on
# this robot. Newer Wombat firmware also offers it as "kipr", so try both.
sys.path.append("/usr/lib")
try:
    import _kipr as k
except ImportError:
    try:
        import kipr as k
    except ImportError:
        k = None


def log(*parts):
    """print() that shows up immediately in the KIPR console."""
    print(*parts)
    sys.stdout.flush()


def wait_ms(milliseconds):
    """Pause for this many milliseconds."""
    k.msleep(int(milliseconds))


def clamp(value, low, high):
    """Keep value between low and high."""
    return max(low, min(high, value))


class StopRequested(Exception):
    """Raised when B is pressed or a mode's time limit runs out."""


def check_stop(deadline):
    """Raise StopRequested if B is pressed or the deadline (k.seconds()) passed."""
    if k.b_button():
        raise StopRequested("B pressed")
    if deadline is not None and k.seconds() >= deadline:
        raise StopRequested("time limit reached")


def sleep_checking(milliseconds, deadline):
    """Like wait_ms(), but notices B and the time limit every 10 ms."""
    remaining = int(milliseconds)
    while remaining > 0:
        check_stop(deadline)
        step = min(10, remaining)
        wait_ms(step)
        remaining -= step


def deadline_after(milliseconds):
    """The k.seconds() value that is this many ms from now."""
    return k.seconds() + milliseconds / 1000.0


def wait_for_a_to_start():
    """Wait until A is pressed (and released). B quits instead."""
    log("Press A to start (B = quit).")
    while not k.a_button():
        if k.b_button():
            raise StopRequested("B pressed before start")
        wait_ms(10)
    while k.a_button():
        wait_ms(10)


# ===========================================================================
# MOTORS (same logic as Lab 2's robot_control.py)
# ===========================================================================
def set_motor(port, speed, reversed_motor):
    """Drive one motor at -100..100, flipping the sign if it is mounted backwards."""
    speed = int(clamp(speed, -100, 100))
    if reversed_motor:
        speed = -speed
    k.motor(port, speed)


def set_wheel_speeds(left_speed, right_speed):
    """Set both wheels. Positive always means 'roll the robot forward'."""
    set_motor(LEFT_MOTOR_PORT, left_speed, LEFT_MOTOR_REVERSED)
    set_motor(RIGHT_MOTOR_PORT, right_speed, RIGHT_MOTOR_REVERSED)


def drive_forward(speed):
    set_wheel_speeds(speed, speed)


def drive_reverse(speed):
    set_wheel_speeds(-speed, -speed)


def turn_left(speed):
    """Spin in place to the left (left wheel back, right wheel forward)."""
    set_wheel_speeds(-speed, speed)


def turn_right(speed):
    """Spin in place to the right (left wheel forward, right wheel back)."""
    set_wheel_speeds(speed, -speed)


def stop_motors():
    """Turn every motor off."""
    k.ao()


# ===========================================================================
# SENSORS
# ===========================================================================
def read_analog_stats(port, samples=ANALOG_SAMPLES):
    """
    Take several readings from an analog port.
    Returns (average, lowest, highest), or None if every reading was invalid.
    The lowest/highest show how much the sensor jitters.
    """
    valid = []
    for _ in range(samples):
        value = k.analog(port)
        if ANALOG_MIN <= value <= ANALOG_MAX:
            valid.append(value)
        wait_ms(2)
    if not valid:
        return None
    return sum(valid) / float(len(valid)), min(valid), max(valid)


# ===========================================================================
# SERVO
# ===========================================================================
def servo_start(position):
    """
    Turn on the servo at a known position. The position is set BEFORE
    enabling, so the servo goes straight there instead of to an old value.
    """
    k.set_servo_position(FRONT_SERVO_PORT, int(position))
    k.enable_servos()


def servo_stop():
    """Turn servo power off so it no longer pushes against anything."""
    k.disable_servos()


def servo_move(position):
    """
    Move the servo, but ONLY inside the measured safe range
    SERVO_LEFT..SERVO_RIGHT. Anything outside is refused with an error,
    because it means a setting is wrong.
    """
    low, high = min(SERVO_LEFT, SERVO_RIGHT), max(SERVO_LEFT, SERVO_RIGHT)
    if not low <= position <= high:
        raise ValueError("servo position %d is outside the safe range %d..%d"
                         % (position, low, high))
    k.set_servo_position(FRONT_SERVO_PORT, int(position))


# ===========================================================================
# TURNING
# ===========================================================================
def turn_by_angle(angle_deg, deadline=None):
    """
    Spin in place by roughly angle_deg degrees. POSITIVE = left.
    There is no compass, so this is timing only:
        spin time = |angle| * TURN_MS_PER_DEG
    The result is only as good as the TURN_MS_PER_DEG measurement.
    """
    spin_ms = int(abs(angle_deg) * TURN_MS_PER_DEG)
    if spin_ms == 0:
        return
    if angle_deg > 0:
        turn_left(TURN_SPEED)
    else:
        turn_right(TURN_SPEED)
    try:
        sleep_checking(spin_ms, deadline)
    finally:
        stop_motors()


# ===========================================================================
# TEST MODES
# ===========================================================================
def test_ir(name, port):
    """Print one Sharp IR sensor forever. The robot does not move."""
    log("%s Sharp IR test (analog port %d). The robot will NOT move." % (name, port))
    log("Hold a flat object at measured distances and write down the avg value.")
    log("Press Stop to end.")
    while True:
        stats = read_analog_stats(port)
        if stats is None:
            log("%s IR: no valid reading - check the cable on analog %d" % (name, port))
        else:
            avg, low, high = stats
            log("%s IR  avg=%4d   min=%4d   max=%4d   (spread %d)"
                % (name, avg, low, high, high - low))
        wait_ms(PRINT_INTERVAL_MS)


def test_ir_front():
    test_ir("FRONT", FRONT_SHARP_IR_PORT)


def test_ir_side():
    test_ir("SIDE", WALL_SHARP_IR_PORT)


def wait_for_button_release():
    while k.a_button() or k.b_button() or k.c_button():
        wait_ms(10)


def test_servo():
    """
    Move the servo by hand with the Wombat buttons:
        A = step one way (lower number)
        C = step the other way (higher number)
        B = finish
    Write down the position numbers where the sensor points left, straight
    ahead and right, and where it starts to strain (buzz or stop moving).
    """
    position = clamp(SERVO_TEST_START, SERVO_TEST_MIN, SERVO_TEST_MAX)
    log("Servo test on servo port %d." % FRONT_SERVO_PORT)
    log("A = lower number, C = higher number, B = finish.")
    log("Each press moves %d. Limits for this test: %d .. %d"
        % (SERVO_TEST_STEP, SERVO_TEST_MIN, SERVO_TEST_MAX))
    log("If the servo buzzes, strains or the sensor hits anything, press B at once.")
    servo_start(position)
    log("Servo at %d. Which way does the sensor point?" % position)
    try:
        while True:
            if k.b_button():
                break
            new_position = position
            if k.a_button():
                new_position = position - SERVO_TEST_STEP
            elif k.c_button():
                new_position = position + SERVO_TEST_STEP

            if new_position != position:
                if SERVO_TEST_MIN <= new_position <= SERVO_TEST_MAX:
                    position = new_position
                    k.set_servo_position(FRONT_SERVO_PORT, position)
                    log("Servo position:", position)
                else:
                    log("Limit reached (%d). Not moving further." % position)
                wait_for_button_release()
            wait_ms(20)
        log("Last servo position:", position)
    finally:
        servo_stop()
        log("Servo power off.")


def motor_step(description, left_speed, right_speed):
    """Run one motor test step, then stop and pause so you can watch.
    B stops it at any moment (raises StopRequested)."""
    log(description)
    set_wheel_speeds(left_speed, right_speed)
    try:
        sleep_checking(MOTOR_TEST_MS, None)
    finally:
        stop_motors()
    sleep_checking(1000, None)


def test_motors():
    """
    Short, slow checks of the drive motors. Put the robot on the floor with
    room around it (or lift it so the wheels spin freely). Press A to start.
    """
    s = MOTOR_TEST_SPEED
    log("Motor test: speed %d, %d ms per step." % (s, MOTOR_TEST_MS))
    log("Press A to start.")
    while not k.a_button():
        wait_ms(10)
    wait_for_button_release()

    log("B = emergency stop.")
    try:
        motor_step("1. LEFT wheel only - should roll FORWARD", s, 0)
        motor_step("2. RIGHT wheel only - should roll FORWARD", 0, s)
        motor_step("3. BOTH wheels - robot should drive FORWARD", s, s)
        motor_step("4. BOTH wheels - robot should drive BACKWARD", -s, -s)
        motor_step("5. Spin LEFT in place (counter-clockwise from above)", -s, s)
        motor_step("6. Spin RIGHT in place (clockwise from above)", s, -s)
        log("Done. Tell me which steps did NOT match their description.")
    except StopRequested as reason:
        log("Stopped:", reason)
    finally:
        stop_motors()


# ===========================================================================
# WALL FOLLOWING - TWO STATES
# ===========================================================================
TOO_CLOSE = "TOO_CLOSE"
TOO_FAR = "TOO_FAR"
CORRECT = "CORRECT"       # three-state only


def choose_wall_state(reading, previous_state):
    """
    Decide the state from one side-sensor reading.

    A higher reading means the wall is closer. One target (15 cm):
      reading >  WALL_2_TARGET -> TOO_CLOSE (closer than 15 cm)
      reading <= WALL_2_TARGET -> TOO_FAR   (15 cm or farther)
    previous_state is not needed; it is accepted so both algorithms can
    share the same driving loop. Noise near the target cannot flip the
    robot quickly: each decision is followed by a full steering pulse.
    """
    if reading > WALL_2_TARGET:
        return TOO_CLOSE
    return TOO_FAR


def steer_for_state(state):
    """
    Return (left_speed, right_speed, direction) for a state.
    With the wall on the LEFT:
      TOO_CLOSE -> steer RIGHT (left wheel faster)
      TOO_FAR   -> steer LEFT  (right wheel faster)
    If WALL_SIDE is "right", everything is mirrored.
    """
    if state == TOO_CLOSE:
        left, right, direction = WALL_FAST_SPEED, WALL_SLOW_SPEED, "RIGHT"
    else:
        left, right, direction = WALL_SLOW_SPEED, WALL_FAST_SPEED, "LEFT"
    if WALL_SIDE == "right":
        left, right = right, left
        direction = "LEFT" if direction == "RIGHT" else "RIGHT"
    return left, right, direction


def choose_wall_state_3(reading, previous_state=None):
    """
    Three-state version. A higher reading means the wall is closer:
      reading >  WALL_3_NEAR -> TOO_CLOSE (closer than 14 cm)
      reading <  WALL_3_FAR  -> TOO_FAR   (farther than 16 cm)
      anything in between    -> CORRECT   (14-16 cm, both ends included)
    previous_state is not needed; it is accepted so both algorithms can
    share the same driving loop.
    """
    if reading > WALL_3_NEAR:
        return TOO_CLOSE
    if reading < WALL_3_FAR:
        return TOO_FAR
    return CORRECT


def steer_for_state_3(state):
    """Like steer_for_state(), plus CORRECT -> drive straight."""
    if state == CORRECT:
        return WALL_STRAIGHT_SPEED, WALL_STRAIGHT_SPEED, "STRAIGHT"
    return steer_for_state(state)


def spin_away_from_wall(deadline):
    """
    Spin in place away from the wall until the front IR no longer sees
    anything within ~30 cm. Returns True when the front is clear, False if
    it is still blocked after WALL_AHEAD_MAX_SPIN_MS (or the reading failed).
    """
    spin = turn_right if WALL_SIDE == "left" else turn_left
    spun_ms = 0
    try:
        while spun_ms < WALL_AHEAD_MAX_SPIN_MS:
            spin(TURN_SPEED)
            sleep_checking(WALL_LOOP_MS, deadline)
            spun_ms += WALL_LOOP_MS
            front = read_analog_stats(FRONT_SHARP_IR_PORT)
            if front is None:
                log("No valid front IR reading while spinning.")
                return False
            if front[0] <= FRONT_OPEN:
                log("Front clear after %d ms of spinning (front IR=%d)." % (spun_ms, front[0]))
                return True
        return False
    finally:
        stop_motors()


def wall_follow_2state():
    """Part I, two states: too close -> steer away, too far -> steer toward."""
    if not WALL_TOO_FAR < WALL_2_TARGET < WALL_TOO_CLOSE:
        log("WALL_2_TARGET must be between WALL_TOO_FAR and WALL_TOO_CLOSE.")
        return
    run_wall_follow("Two-state", choose_wall_state, steer_for_state,
                    "Too close (< 15 cm): reading > %d   Too far: reading <= %d"
                    % (WALL_2_TARGET, WALL_2_TARGET))


def wall_follow_3state():
    """Part I, three states: adds CORRECT -> drive straight."""
    if not WALL_TOO_FAR < WALL_3_FAR < WALL_3_NEAR < WALL_TOO_CLOSE:
        log("Need WALL_TOO_FAR < WALL_3_FAR < WALL_3_NEAR < WALL_TOO_CLOSE.")
        return
    run_wall_follow("Three-state", choose_wall_state_3, steer_for_state_3,
                    "Too close (< 14 cm): reading > %d   Correct (14-16 cm): %d-%d"
                    "   Too far (> 16 cm): reading < %d"
                    % (WALL_3_NEAR, WALL_3_FAR, WALL_3_NEAR, WALL_3_FAR))


def run_wall_follow(title, choose_state, steer, thresholds):
    """
    The wall-following loop shared by both algorithms.
    choose_state(reading, previous_state) picks the state;
    steer(state) gives (left_speed, right_speed, direction);
    thresholds is the line printed to explain the states.
    WALL_MOTORS_ON = False: dry run, prints decisions only, servo off. B = finish.
    WALL_MOTORS_ON = True:  press A to start; drives until B is pressed
                            (or WALL_RUN_MS, if it is a number). It never
                            ends by itself after a correction. With WALL_USE_FRONT_IR the
                            servo is held at SERVO_CENTER (never scans);
                            without it the servo stays off.
    """
    if WALL_SIDE not in ("left", "right"):
        log('WALL_SIDE must be "left" or "right", not %r.' % (WALL_SIDE,))
        return
    if WALL_TOO_CLOSE <= WALL_TOO_FAR:
        log("WALL_TOO_CLOSE must be bigger than WALL_TOO_FAR. Check calibration.")
        return

    if WALL_MOTORS_ON:
        log("%s wall following - MOTORS ON. Wall side: %s" % (title, WALL_SIDE))
        log("Speeds: fast %d, slow %d. Runs %s. B = stop."
            % (WALL_FAST_SPEED, WALL_SLOW_SPEED,
               "until B" if WALL_RUN_MS is None else "%d ms" % WALL_RUN_MS))
        log("Place the robot next to the wall, then press A to start.")
        while not k.a_button():
            wait_ms(10)
        wait_for_button_release()
        loop_ms = WALL_LOOP_MS
    else:
        log("%s wall following - DRY RUN, motors OFF. Wall side: %s" % (title, WALL_SIDE))
        log("Slide the robot by hand toward and away from a wall. B = finish.")
        loop_ms = PRINT_INTERVAL_MS
    log(thresholds)

    front_guard = WALL_MOTORS_ON and WALL_USE_FRONT_IR
    pulsing = WALL_MOTORS_ON and WALL_STEER_PULSE_MS > 0
    net_pulses = 0           # + = turned toward the wall, - = away (estimate)
    limit_base = None        # side reading when the turn limit started blocking
    limit_since = None       # k.seconds() at that moment
    state = TOO_FAR          # start by heading toward the wall to find it
    deadline = None
    try:
        if front_guard:
            # Hold the front IR straight ahead. The servo does not move again.
            log("Front guard ON: servo held at %d, wall ahead if front IR > %d"
                % (SERVO_CENTER, FRONT_OPEN))
            servo_start(SERVO_CENTER)
            wait_ms(SERVO_SETTLE_MS)
        start = k.seconds()
        timed = WALL_MOTORS_ON and WALL_RUN_MS is not None
        if timed:
            deadline = start + WALL_RUN_MS / 1000.0
        while not k.b_button():
            if timed and (k.seconds() - start) * 1000 >= WALL_RUN_MS:
                log("Time limit reached.")
                break
            stats = read_analog_stats(WALL_SHARP_IR_PORT)
            if stats is None:
                # Never drive blind: stop until the sensor works again.
                if WALL_MOTORS_ON:
                    stop_motors()
                log("No valid side IR reading - stopped. Check analog %d"
                    % WALL_SHARP_IR_PORT)
                wait_ms(loop_ms)
                continue

            reading = stats[0]
            state = choose_state(reading, state)
            left, right, direction = steer(state)
            verb = "drive" if direction == "STRAIGHT" else "steer"

            # Turn limit: which way would this pulse turn, and is that
            # already as far as we allow?
            turn = 0
            if direction != "STRAIGHT":
                turn = 1 if state == TOO_FAR else -1
            limited = pulsing and ((turn > 0 and net_pulses >= WALL_MAX_PULSES) or
                                   (turn < 0 and net_pulses <= -WALL_MAX_PULSES))

            front_text = ""
            wall_ahead = False
            if front_guard:
                front = read_analog_stats(FRONT_SHARP_IR_PORT)
                if front is None:
                    stop_motors()
                    log("No valid front IR reading - stopped. Check analog %d"
                        % FRONT_SHARP_IR_PORT)
                    wait_ms(loop_ms)
                    continue
                front_text = "   front IR=%4d" % front[0]
                if front[0] > FRONT_OPEN:
                    # Something within ~30 cm ahead: spin away from the
                    # wall, ignoring the side sensor and the turn limit.
                    wall_ahead = True

            # Caution zone (closer than ~13 cm): no forward motion at all,
            # only short in-place spins away from the wall. A spin lasts as
            # long as a steering pulse but turns faster: a pulse turns at a
            # rate set by the wheel-speed difference (FAST - SLOW), a spin by
            # 2 * TURN_SPEED. So one spin counts as that many pulses in the
            # turn count, and the usual turn limit stops the spins.
            spin_pulses = 2.0 * TURN_SPEED / (WALL_FAST_SPEED - WALL_SLOW_SPEED)
            caution = (WALL_MOTORS_ON and WALL_CAUTION is not None
                       and reading >= WALL_CAUTION
                       and net_pulses > -WALL_MAX_PULSES)
            if (WALL_MOTORS_ON and WALL_CAUTION is not None
                    and reading >= WALL_CAUTION and not caution):
                limited = True       # turned away as far as allowed: go straight
            # Closer than ~10 cm and already turned away as far as allowed:
            # the turn count must be wrong, so keep steering AWAY from the
            # wall in a forward arc (ignoring the limit) instead of going
            # straight or stopping.
            give_up = limited and turn < 0 and reading >= WALL_TOO_CLOSE
            away = "RIGHT" if WALL_SIDE == "left" else "LEFT"
            # Wheel speeds that will really be sent (for the printout).
            spin = (TURN_SPEED, -TURN_SPEED) if away == "RIGHT" else (-TURN_SPEED, TURN_SPEED)
            if WALL_MOTORS_ON and (wall_ahead or caution):
                sent = spin
            elif WALL_MOTORS_ON and limited and not give_up:
                sent = (WALL_STRAIGHT_SPEED, WALL_STRAIGHT_SPEED)
            else:
                sent = (left, right)
            # The release timer only runs while the limit blocks a turn
            # toward the wall and the robot drives straight.
            if not (WALL_MOTORS_ON and limited and turn > 0
                    and not wall_ahead and not caution):
                limit_base = None

            if not WALL_MOTORS_ON:
                action = "would %s %s" % (verb, direction)
            elif wall_ahead:
                action = "WALL AHEAD - spin away until the front is clear"
            elif caution:
                action = "CAUTION (< ~13 cm) - spin %s in place" % away
            elif give_up:
                action = "STILL < ~10 cm after turning away - steer %s (pulse)" % away
            elif limited:
                action = "turn limit - drive STRAIGHT"
            elif pulsing and turn != 0:
                action = "%s %s (pulse)" % (verb, direction)
            else:
                action = "%s %s" % (verb, direction)
            log("side IR=%4d%s   state=%-9s   %s   L=%d R=%d   turn count=%+d"
                % (reading, front_text, state, action, sent[0], sent[1], net_pulses))

            if not WALL_MOTORS_ON:
                wait_ms(loop_ms)
            elif wall_ahead:
                if not spin_away_from_wall(deadline):
                    # Motors are already stopped. Do not drive: wait, then
                    # read both sensors again (the next loop spins again if
                    # the front is still blocked).
                    log("Front still blocked after %d ms of spinning - stopped,"
                        " checking again." % WALL_AHEAD_MAX_SPIN_MS)
                    sleep_checking(loop_ms, deadline)
                    continue
                # Still angled toward the wall: no more steering toward it
                # until the robot has steered away again.
                net_pulses = WALL_MAX_PULSES
            elif caution:
                # Turning in place does not move the robot toward the wall.
                net_pulses -= spin_pulses
                if away == "RIGHT":
                    turn_right(TURN_SPEED)
                else:
                    turn_left(TURN_SPEED)
                try:
                    sleep_checking(WALL_STEER_PULSE_MS, deadline)
                finally:
                    stop_motors()
            elif give_up:
                # Forward arc away from the wall (left, right = TOO_CLOSE
                # steering), then read the sensor again.
                set_wheel_speeds(left, right)
                sleep_checking(max(WALL_STEER_PULSE_MS, loop_ms), deadline)
            elif limited:
                set_wheel_speeds(WALL_STRAIGHT_SPEED, WALL_STRAIGHT_SPEED)
                wait_ms(loop_ms)
                if turn > 0 and front_guard:
                    # Limit toward the wall: is the wall really getting closer?
                    # Only with the front guard: angled steeply, the side beam
                    # can miss the wall and read a steady "far"; then the
                    # front IR sees the wall and spins the robot away.
                    now = k.seconds()
                    if limit_base is None:
                        limit_base, limit_since = reading, now
                    elif (now - limit_since) * 1000 >= WALL_LIMIT_RELEASE_MS:
                        if reading <= limit_base:
                            net_pulses -= 1
                            log("No closer to the wall after %d ms straight - allow one"
                                " more pulse toward it." % WALL_LIMIT_RELEASE_MS)
                        limit_base = None       # start a new check
            elif pulsing and turn != 0:
                # Short steering pulse, then straight, then read again.
                net_pulses += turn
                set_wheel_speeds(left, right)
                sleep_checking(WALL_STEER_PULSE_MS, deadline)
                set_wheel_speeds(WALL_STRAIGHT_SPEED, WALL_STRAIGHT_SPEED)
                sleep_checking(WALL_STRAIGHT_PULSE_MS, deadline)
            else:
                set_wheel_speeds(left, right)
                wait_ms(loop_ms)
    except StopRequested:
        if deadline is not None and k.seconds() >= deadline:
            log("Time limit reached.")
        else:
            log("Stopped: B pressed.")
    finally:
        if WALL_MOTORS_ON:
            stop_motors()
            log("Motors stopped.")
        if front_guard:
            servo_stop()
    log("Wall following finished.")


# ===========================================================================
# OPEN SPACE - FIVE-POSITION SCAN
# ===========================================================================
def classify_front(reading):
    """
    Put one front-IR reading into a category:
      None                  -> INVALID (no usable reading)
      >= FRONT_BLOCKED      -> BLOCKED (closer than ~10 cm)
      <= FRONT_OPEN         -> OPEN    (farther than ~30 cm)
      in between            -> NEAR    (about 10-30 cm)
    Only OPEN directions are ever chosen.
    """
    if reading is None:
        return "INVALID"
    if reading >= FRONT_BLOCKED:
        return "BLOCKED"
    if reading <= FRONT_OPEN:
        return "OPEN"
    return "NEAR"


def direction_is_safe(readings, i):
    """
    A direction is safe to choose only if it is OPEN (> ~30 cm) AND each
    neighbouring direction is valid and farther than ~20 cm (reading at or
    below FRONT_SIDE_CLEAR). This keeps clear space on both sides of the
    path. It also guards against the Sharp IR's close-range problem: under
    ~8-10 cm a reading DROPS and looks far away, so an "open" reading next
    to a blocked or near one may really be something touching the robot.
    """
    if classify_front(readings[i]) != "OPEN":
        return False
    for j in (i - 1, i + 1):
        if 0 <= j < len(readings):
            if readings[j] is None or readings[j] > FRONT_SIDE_CLEAR:
                return False
    return True


def choose_open_direction(readings, allowed=None, last_turn=0):
    """
    Pick a clear scan direction, or None if no direction is safe.
    Only safe directions count (see direction_is_safe), and only those with
    allowed[i] True if allowed is given (e.g. the turn fits in the movement
    time left).
      - Choose the FARTHEST direction (lowest reading; for the Sharp IR a
        lower reading means farther away), even if straight ahead is clear.
      - A tie is broken by the direction closest to straight ahead (the
        smaller turn), then by not reversing the last turn (last_turn:
        +1 = left, -1 = right, 0 = none).
    """
    center = len(readings) // 2
    candidates = [i for i in range(len(readings))
                  if direction_is_safe(readings, i) and (allowed is None or allowed[i])]
    if not candidates:
        return None

    def preference(i):
        side = 1 if i < center else -1           # left = +1
        reverses = 1 if last_turn and i != center and side == -last_turn else 0
        return (readings[i], abs(i - center), reverses)
    return min(candidates, key=preference)


def scan_five(deadline, hold_ms=SERVO_SETTLE_MS):
    """
    The motors must already be stopped. Visit each SCAN_POSITIONS entry,
    wait for the servo to settle, and read the front IR there.
    Returns a list of five averaged readings (None = invalid reading).
    """
    readings = []
    for position in SCAN_POSITIONS:
        servo_move(position)
        sleep_checking(hold_ms, deadline)
        stats = read_analog_stats(FRONT_SHARP_IR_PORT)
        readings.append(None if stats is None else stats[0])
    return readings


def print_scan(readings, best, allowed=None):
    for i, reading in enumerate(readings):
        shown = "----" if reading is None else "%4d" % reading
        if i == best:
            marker = "  <-- chosen"
        elif classify_front(reading) == "OPEN" and not direction_is_safe(readings, i):
            marker = "  (a neighbour is under ~20 cm - not used)"
        elif allowed is not None and not allowed[i] and direction_is_safe(readings, i):
            marker = "  (turn too long for the time left)"
        else:
            marker = ""
        log("  %-9s servo=%4d  IR=%s  %-7s%s"
            % (SCAN_NAMES[i], SCAN_POSITIONS[i], shown, classify_front(reading), marker))


def scan_settings_problem():
    """Return a message if the scan settings are unsafe or incomplete, else None."""
    if FRONT_BLOCKED <= FRONT_OPEN:
        return "FRONT_BLOCKED must be bigger than FRONT_OPEN. Check calibration."
    if len(SCAN_POSITIONS) != 5 or len(SCAN_NAMES) != 5:
        return "SCAN_POSITIONS and SCAN_NAMES must each have 5 entries."
    low, high = min(SERVO_LEFT, SERVO_RIGHT), max(SERVO_LEFT, SERVO_RIGHT)
    for position in SCAN_POSITIONS:
        if not low <= position <= high:
            return "Scan position %d is outside the safe range %d..%d." % (position, low, high)
    if not FRONT_SLOW_START < FRONT_OPEN:
        return "FRONT_SLOW_START must be smaller than FRONT_OPEN (farther away)."
    return None


def approach_speed(front_reading):
    """
    Forward speed for a front IR reading (higher reading = closer):
      at or below FRONT_SLOW_START (farther than ~40 cm): OPEN_DRIVE_SPEED
      between FRONT_SLOW_START and FRONT_OPEN: slower the closer it gets,
        down to OPEN_SLOW_SPEED just before ~30 cm
    (Above FRONT_OPEN the robot does not drive at all.)
    """
    if front_reading <= FRONT_SLOW_START:
        return OPEN_DRIVE_SPEED
    closeness = (front_reading - FRONT_SLOW_START) / float(FRONT_OPEN - FRONT_SLOW_START)
    closeness = min(1.0, closeness)
    return int(round(OPEN_DRIVE_SPEED - closeness * (OPEN_DRIVE_SPEED - OPEN_SLOW_SPEED)))


def scan_angles_problem():
    """
    Check SCAN_ANGLES_DEG: five measured angles, left (+) to right (-),
    center exactly 0. A wrong order or sign would turn the robot the wrong
    way, so the mode refuses to run. Returns a message, or None if OK.
    """
    if SCAN_ANGLES_DEG is None:
        return "SCAN_ANGLES_DEG is not measured yet (use test_scan)."
    if len(SCAN_ANGLES_DEG) != 5:
        return "SCAN_ANGLES_DEG must have 5 numbers, one per scan position."
    if SCAN_ANGLES_DEG[2] != 0:
        return "SCAN_ANGLES_DEG: the CENTER angle must be 0."
    if not all(a > b for a, b in zip(SCAN_ANGLES_DEG, SCAN_ANGLES_DEG[1:])):
        return ("SCAN_ANGLES_DEG must go from LEFT (positive) to RIGHT (negative):"
                " [left, left-mid, 0, -right-mid, -right].")
    if any(abs(a) > 180 for a in SCAN_ANGLES_DEG):
        return "SCAN_ANGLES_DEG values must be between -180 and 180."
    return None


def new_side_tracker():
    """
    Remembers what the LEFT side IR saw while the robot drives straight
    (reset after every turn). Distances are kept as "cruise ms": ms of
    driving at OPEN_DRIVE_SPEED, so slowing down is accounted for:
      straight_ms - distance driven straight since the last turn
      wall        - a wall closer than ~30 cm has been seen on the left
      far_checks  - side checks in a row that saw nothing within ~30 cm
      opening_at  - straight_ms when a side opening started (or None)
      gap_ms      - distance driven across the opening, once known
    """
    return {"straight_ms": 0, "wall": False, "far_checks": 0,
            "opening_at": None, "gap_ms": None}


def track_side(tracker, side_reading, now_ms):
    """
    Update the tracker with one side IR reading taken now_ms into straight
    driving. Returns True when an opening has just ended (the wall is back),
    so the robot should stop there.
    """
    if side_reading > WALL_TOO_FAR:                  # wall within ~30 cm
        tracker["far_checks"] = 0
        if tracker["opening_at"] is not None:
            tracker["gap_ms"] = now_ms - tracker["opening_at"]
            log("Side opening ended (%d ms of driving wide)." % tracker["gap_ms"])
            return True
        tracker["wall"] = True
        return False
    tracker["far_checks"] += 1
    if (tracker["wall"] and tracker["opening_at"] is None
            and tracker["far_checks"] >= OPEN_OPENING_CHECKS):
        tracker["opening_at"] = now_ms
        log("Side opening starts on the LEFT.")
    return False


def drive_forward_watching(deadline, max_ms=None, tracker=None, glance=False):
    """
    Drive forward without stopping for up to max_ms (default OPEN_FORWARD_MS)
    with the servo facing forward. Before every OPEN_LOOP_MS step it checks:
      - front IR: stop if something is closer than ~30 cm (> FRONT_OPEN);
        otherwise set the speed with approach_speed() (slower when closer)
      - side IR:  stop if the wall is closer than ~10 cm (>= WALL_TOO_CLOSE)
      - either reading invalid: stop
      - with a tracker: stop where a side opening ends (see track_side); if
        the front blocks the way inside an opening, the opening ends there.
      - with glance: about every OPEN_GLANCE_MS, look at a 45 degree
        diagonal while still rolling slowly; stop if it is under ~20 cm.
    B and the deadline stop it at once. Returns how many ms the motors ran.
    """
    if max_ms is None:
        max_ms = OPEN_FORWARD_MS
    # The wheels keep turning while both sensors are read, so leave room for
    # that reading time; otherwise the robot would drive past max_ms.
    check_ms = 2 * ANALOG_SAMPLES * 2
    start = None
    front_blocked = False
    speed = 0                # speed currently commanded (0 = not moving yet)
    since = None             # when that speed was commanded
    cruise_ms = 0.0          # distance so far, as ms at OPEN_DRIVE_SPEED
    slowing_logged = False
    last_glance = None       # when the last diagonal glance finished
    glance_right = True      # the right side has no side sensor: look there first
    glance_ms = 2 * SERVO_SETTLE_MS + ANALOG_SAMPLES * 2

    def add_distance():
        # Distance driven at the current speed since it was commanded.
        if since is None:
            return 0.0
        return (k.seconds() - since) * 1000 * speed / float(OPEN_DRIVE_SPEED)

    try:
        while True:
            check_stop(deadline)
            driven = 0 if start is None else (k.seconds() - start) * 1000
            if driven + check_ms + 1 > max_ms:
                break
            stats = read_analog_stats(FRONT_SHARP_IR_PORT)
            if stats is None:
                log("Front IR: no valid reading - stopping.")
                break
            if stats[0] > FRONT_OPEN:
                log("Front IR=%d: something closer than ~30 cm - stopping." % stats[0])
                front_blocked = True
                break
            side = read_analog_stats(WALL_SHARP_IR_PORT)
            if side is None:
                log("Side IR: no valid reading - stopping.")
                break
            if side[0] >= WALL_TOO_CLOSE:
                log("Side IR=%d: wall closer than ~10 cm - stopping." % side[0])
                break
            if tracker is not None:
                if track_side(tracker, side[0],
                              tracker["straight_ms"] + cruise_ms + add_distance()):
                    break
            new_speed = approach_speed(stats[0])
            if new_speed < OPEN_DRIVE_SPEED and not slowing_logged:
                log("Front IR=%d: something ahead - slowing down." % stats[0])
                slowing_logged = True
            if start is None:
                start = k.seconds()
                last_glance = start
            cruise_ms += add_distance()
            driven = (k.seconds() - start) * 1000
            if (glance and (k.seconds() - last_glance) * 1000 >= OPEN_GLANCE_MS
                    and driven + glance_ms + check_ms + 1 < max_ms):
                # The front just read farther than ~30 cm, so rolling slowly
                # for the ~1 s the servo looks aside is safe.
                speed, since = OPEN_SLOW_SPEED, k.seconds()
                drive_forward(speed)
                index = 3 if glance_right else 1
                servo_move(SCAN_POSITIONS[index])
                sleep_checking(SERVO_SETTLE_MS, deadline)
                look = read_analog_stats(FRONT_SHARP_IR_PORT)
                servo_move(SERVO_CENTER)
                sleep_checking(SERVO_SETTLE_MS, deadline)
                last_glance = k.seconds()
                glance_right = not glance_right
                if look is None or look[0] > FRONT_SIDE_CLEAR:
                    shown = "invalid" if look is None else "%d" % look[0]
                    log("Glance %s IR=%s: closer than ~20 cm beside the path - stopping."
                        % (SCAN_NAMES[index], shown))
                    break
                continue                   # check the front again right away
            speed, since = new_speed, k.seconds()
            drive_forward(speed)
            left_ms = max_ms - check_ms - (k.seconds() - start) * 1000
            sleep_checking(max(1, min(OPEN_LOOP_MS, int(left_ms))), deadline)
    finally:
        cruise_ms += add_distance()
        stop_motors()
    driven_ms = 0 if start is None else int(round((k.seconds() - start) * 1000))
    if tracker is not None:
        tracker["straight_ms"] += int(round(cruise_ms))
        if front_blocked and tracker["opening_at"] is not None and tracker["gap_ms"] is None:
            tracker["gap_ms"] = tracker["straight_ms"] - tracker["opening_at"]
            log("Way ahead blocked inside the side opening (%d ms of driving wide)."
                % tracker["gap_ms"])
    return driven_ms


def enter_side_opening(tracker, budget_ms):
    """
    The robot has just driven past a LEFT side opening that was
    tracker["gap_ms"] of driving wide. Back up half of that (along the path
    it has just driven, so known to be clear) to the middle of the opening,
    then turn LEFT by SCAN_ANGLES_DEG[0]. Returns the motor-on ms used, or
    0 if the whole move does not fit in budget_ms (then nothing moves).
    The next scan decides if the corridor is really clear to drive into.
    """
    back_ms = tracker["gap_ms"] // 2
    angle = SCAN_ANGLES_DEG[0]
    turn_ms = abs(angle) * TURN_MS_PER_DEG
    if back_ms + turn_ms >= budget_ms:
        log("Side opening found, but backing up %d ms and turning %d ms does not fit"
            " in the %d ms of movement left." % (back_ms, turn_ms, budget_ms))
        return 0
    log("Entering the side opening: back up %d ms to its middle, then turn LEFT"
        " about %d degrees." % (back_ms, angle))
    start = k.seconds()
    deadline = deadline_after(budget_ms)
    drive_reverse(OPEN_DRIVE_SPEED)
    try:
        sleep_checking(back_ms, deadline)
    finally:
        stop_motors()
    turn_by_angle(angle, deadline)     # B here ends the mode: no driving after
    return int(round((k.seconds() - start) * 1000))


def open_space():
    """
    Part II. Repeats until B, the time limit, or no safe direction:
      1. stop, scan five directions (at the start, and after every stop)
      2. choose the farthest safe direction (tie: the one closest to straight)
      3. turn toward it, point the servo forward again
      4. drive CONTINUOUSLY, slowing down as something gets near, and stop
         only when something is within ~30 cm ahead or ~10 cm on the left,
         a reading is invalid, or a LEFT side opening ends
      5. if a LEFT side opening was passed, back up to its middle and turn
         into it (at most OPEN_MAX_SIDE_TURNS times)
    """
    problem = scan_settings_problem() or scan_angles_problem()
    if problem:
        log(problem)
        return

    log("Open-space navigation. Drive speed %d, step %d ms, movement limit %d ms."
        % (OPEN_DRIVE_SPEED, OPEN_FORWARD_MS, OPEN_RUN_MS))
    log("Scanning does not count toward the limit; turning and driving do.")
    stop_motors()
    budget_ms = OPEN_RUN_MS      # motor-on time still allowed
    stuck = 0
    last_turn = 0                # +1 = last turn was left, -1 = right
    tracker = new_side_tracker() # what the left side IR saw since the last turn
    side_turns = 0               # corridor entries so far
    try:
        wait_for_a_to_start()
        servo_start(SERVO_CENTER)                   # nothing moves before A
        sleep_checking(SERVO_SETTLE_MS, None)
        # Less movement time than one drive step plus its sensor checks is
        # treated as used up (no tiny useless moves, no endless scanning).
        min_move_ms = OPEN_LOOP_MS + 2 * ANALOG_SAMPLES * 2
        while True:
            if budget_ms < min_move_ms:
                log("Stopped: time limit reached (%d ms of movement used)." % OPEN_RUN_MS)
                break
            stop_motors()
            readings = scan_five(None)              # robot stands still; B works
            # A turn is only allowed if ALL of it fits in the time left, so a
            # turn is never cut off part way.
            fits = [abs(a) * TURN_MS_PER_DEG < budget_ms for a in SCAN_ANGLES_DEG]
            best = choose_open_direction(readings, fits, last_turn)
            log("Scan (%d ms of movement left):" % budget_ms)
            print_scan(readings, best, fits)
            # Say why the robot turns even though straight ahead reads clear.
            center = len(readings) // 2
            if best is not None and best != center and direction_is_safe(readings, center):
                log("Front clear, but %s reads farther: turning toward it." % SCAN_NAMES[best])
            elif best is not None and best != center and classify_front(readings[center]) == "OPEN":
                log("Front reads clear, but something beside the path (45 degree diagonal)"
                    " is closer than ~20 cm: turning.")
            if best is None:
                if choose_open_direction(readings) is None:
                    log("No safe direction (nothing measured farther than ~30 cm). Stopping.")
                else:
                    log("Open directions need a longer turn than the time left. Stopping.")
                break

            angle = SCAN_ANGLES_DEG[best]
            if angle == 0:
                log("Going straight.")
            else:
                log("Turning %s about %d degrees. (%d ms)"
                    % ("LEFT" if angle > 0 else "RIGHT", abs(angle),
                       abs(angle) * TURN_MS_PER_DEG))
                turn_start = k.seconds()
                # If this is interrupted (B), StopRequested ends the mode: the
                # robot never drives after a turn that did not finish.
                turn_by_angle(angle, deadline_after(budget_ms))
                budget_ms -= int(round((k.seconds() - turn_start) * 1000))
                last_turn = 1 if angle > 0 else -1
                tracker = new_side_tracker()        # new heading: start over
            servo_move(SERVO_CENTER)                # face forward again
            sleep_checking(SERVO_SETTLE_MS, None)
            if budget_ms < min_move_ms:
                log("Stopped: time limit reached (%d ms of movement used)." % OPEN_RUN_MS)
                break

            # Drive continuously: until something needs a new decision (front
            # blocked, left wall too close, invalid reading, end of a side
            # opening) or the movement time is used up. Only then scan again.
            # The deadline is only a backstop (motors start ~40 ms after the
            # first sensor checks); the drive itself keeps within budget_ms.
            driven_ms = drive_forward_watching(deadline_after(budget_ms + 200), budget_ms,
                                               tracker, glance=True)
            budget_ms -= driven_ms
            log("Drove forward %d ms." % driven_ms)
            stuck = stuck + 1 if driven_ms == 0 else 0
            if tracker["gap_ms"] is not None:
                if side_turns < OPEN_MAX_SIDE_TURNS:
                    used_ms = enter_side_opening(tracker, budget_ms)
                    if used_ms:
                        budget_ms -= used_ms
                        side_turns += 1
                        last_turn = 1
                        stuck = 0
                tracker = new_side_tracker()
            if stuck >= OPEN_MAX_STUCK:
                log("Could not drive after %d scans in a row. Stopping." % stuck)
                break
    except StopRequested as reason:
        log("Stopped:", reason)
    finally:
        stop_motors()
        servo_stop()
    log("Open-space navigation finished.")


# ===========================================================================
# CALIBRATION MODES FOR PART II
# ===========================================================================
def test_scan():
    """
    The robot does NOT drive. Each A press: the servo visits the five scan
    positions (holding each one for SCAN_TEST_HOLD_MS so you can measure its
    angle), prints the front IR readings and which direction it would pick.
    B = finish.
    """
    problem = scan_settings_problem()
    if problem:
        log(problem)
        return
    log("Scan test. The robot will NOT drive. Positions:", SCAN_POSITIONS)
    log("Measure the angle of each position for SCAN_ANGLES_DEG (left = +).")
    stop_motors()
    servo_start(SERVO_CENTER)
    try:
        deadline = deadline_after(TEST_RUN_MS)
        while True:
            log("A = scan, B = finish.")
            while not k.a_button():
                check_stop(deadline)
                wait_ms(10)
            while k.a_button():
                wait_ms(10)
            readings = scan_five(deadline, hold_ms=SCAN_TEST_HOLD_MS)
            best = choose_open_direction(readings)
            print_scan(readings, best)
            if best is None:
                log("No OPEN direction - open_space would stop here.")
            servo_move(SERVO_CENTER)
    except StopRequested as reason:
        log("Stopped:", reason)
    finally:
        servo_stop()
        log("Servo power off.")


def test_turn():
    """
    Press A. With TURN_MS_PER_DEG = None: spins LEFT then RIGHT for
    TURN_TEST_MS each so you can measure the angle turned.
    With TURN_MS_PER_DEG set: ONE turn of TURN_TEST_ANGLE_DEG (+ = left),
    then stop. No turn back. Refuses a turn that would not fit in
    MOVE_TIME_LIMIT_MS, so a turn is never cut off halfway.
    """
    log("Turn test at speed %d. Give the robot room to spin. B = stop." % TURN_SPEED)
    if TURN_MS_PER_DEG is not None:
        spin_ms = abs(TURN_TEST_ANGLE_DEG) * TURN_MS_PER_DEG
        if spin_ms >= MOVE_TIME_LIMIT_MS:
            log("A %d degree turn needs %d ms, more than the %d ms limit. Not turning."
                % (TURN_TEST_ANGLE_DEG, spin_ms, MOVE_TIME_LIMIT_MS))
            return
        log("Will turn %s %d degrees (%d ms at %.2f ms/degree), then stop."
            % ("LEFT" if TURN_TEST_ANGLE_DEG > 0 else "RIGHT", abs(TURN_TEST_ANGLE_DEG),
               spin_ms, TURN_MS_PER_DEG))
    try:
        wait_for_a_to_start()
        deadline = deadline_after(MOVE_TIME_LIMIT_MS)
        if TURN_MS_PER_DEG is None:
            for i, (name, spin) in enumerate((("LEFT", turn_left), ("RIGHT", turn_right))):
                if i > 0:
                    sleep_checking(1500, deadline)    # time to measure
                log("Spinning %s for %d ms. Measure the angle." % (name, TURN_TEST_MS))
                spin(TURN_SPEED)
                sleep_checking(TURN_TEST_MS, deadline)
                stop_motors()
            log("Then set TURN_MS_PER_DEG = %d / (degrees turned)." % TURN_TEST_MS)
        else:
            angle = TURN_TEST_ANGLE_DEG
            log("Turning %s %d degrees." % ("LEFT" if angle > 0 else "RIGHT", abs(angle)))
            turn_by_angle(angle, deadline)     # stops both motors when done
            log("Turn finished, motors stopped. Measure the angle.")
            log("Too far? Lower TURN_MS_PER_DEG. Not far enough? Raise it.")
    except StopRequested as reason:
        log("Stopped:", reason)
    finally:
        stop_motors()


def test_buttons():
    """
    Check the A and B buttons without moving anything (no motors, no servo).
    Each A press prints one message (holding A down counts once).
    B prints a message and ends the test.
    If no button is pressed for BUTTON_TEST_MS, the test ends by itself:
    that usually means the buttons are not available, e.g. because the
    program was started from the IDE instead of the Wombat's touchscreen.
    """
    log("Button test. The robot will NOT move. Motors and servo stay OFF.")
    log("Look at the Wombat screen: are A, B and C buttons shown?")
    log("  - Press A: a message should appear here for each press.")
    log("  - Press B: a message appears and the test ends.")
    log("If nothing happens, stop the program with the IDE's Stop button.")
    a_presses = 0
    a_was_down = False
    waited_ms = 0          # time since the last press of any button
    since_reminder_ms = 0
    while True:
        if k.b_button():
            log("B pressed - B works. Button test finished (A pressed %d time(s))."
                % a_presses)
            return
        a_down = bool(k.a_button())
        if a_down and not a_was_down:
            a_presses += 1
            log("A pressed (#%d) - A works." % a_presses)
            waited_ms = 0
        a_was_down = a_down
        if waited_ms >= BUTTON_TEST_MS:
            if a_presses == 0:
                log("No button press seen in %d s. The buttons are probably not"
                    " available in this launch mode - do NOT run driving modes."
                    % (BUTTON_TEST_MS // 1000))
            else:
                log("A works, but B was not pressed within %d s. Check B before"
                    " running driving modes." % (BUTTON_TEST_MS // 1000))
            return
        if since_reminder_ms >= BUTTON_REMINDER_MS:
            log("Still waiting for A or B... (A pressed %d time(s) so far)" % a_presses)
            since_reminder_ms = 0
        wait_ms(10)
        waited_ms += 10
        since_reminder_ms += 10


def record_ir(port):
    """
    Combine CALIBRATE_SAMPLES averaged readings from one sensor.
    Returns (average, spread), or None if no reading was valid.
    spread = highest single sample - lowest single sample (the jitter).
    """
    averages, lows, highs = [], [], []
    for _ in range(CALIBRATE_SAMPLES):
        stats = read_analog_stats(port)
        if stats is not None:
            averages.append(stats[0])
            lows.append(stats[1])
            highs.append(stats[2])
    if not averages:
        return None
    return sum(averages) / len(averages), max(highs) - min(lows)


def format_ir(result):
    """'avg  spread' text for one recorded or live reading, or dashes if invalid."""
    if result is None:
        return "  ----    ----"
    return "%6d  %6d" % (result[0], result[1])


def print_calibration_table(rows):
    log("")
    log("IR calibration table (higher reading = closer, if the sensor behaves normally)")
    log("distance |  FRONT avg  spread |   SIDE avg  spread")
    for distance, front, side in rows:
        log("  %3d cm |     %s |     %s" % (distance, format_ir(front), format_ir(side)))
    if len(rows) < 2:
        return
    for name, column in (("FRONT", 1), ("SIDE", 2)):
        values = [row[column][0] for row in rows if row[column] is not None]
        if len(values) < 2:
            continue
        if all(a > b for a, b in zip(values, values[1:])):
            log("%s: readings go DOWN as distance goes up, as expected." % name)
        else:
            log("%s: readings do NOT go down steadily with distance - check this sensor." % name)


def calibrate_ir():
    """
    Record both Sharp IR sensors at each distance in CALIBRATE_DISTANCES_CM.
    No buttons are needed: before each distance there is a countdown of
    CALIBRATE_COUNTDOWN_S seconds (with live readings every second) to place
    the object, then the readings are recorded automatically.
    The motors are never used and the servo stays off.
    B finishes early if the Wombat's buttons are available; otherwise use
    the IDE's Stop button. The table of recorded rows is printed at the end.
    """
    log("IR calibration. The robot will NOT move. The servo is OFF:")
    log("point the front sensor straight ahead by hand before starting.")
    log("Distances: %s cm. You get %d s to place the object before each one;"
        % (CALIBRATE_DISTANCES_CM, CALIBRATE_COUNTDOWN_S))
    log("then it records automatically. Keep the object STILL while it records.")
    log("To stop early: Stop in the IDE (or B if the Wombat shows its buttons).")
    rows = []
    try:
        for distance in CALIBRATE_DISTANCES_CM:
            log("")
            log("=== Put a flat object %d cm from the sensor(s) now ===" % distance)
            for seconds_left in range(CALIBRATE_COUNTDOWN_S, 0, -1):
                front = read_analog_stats(FRONT_SHARP_IR_PORT)
                side = read_analog_stats(WALL_SHARP_IR_PORT)
                front_live = None if front is None else (front[0], front[2] - front[1])
                side_live = None if side is None else (side[0], side[2] - side[1])
                log("  %2d cm: recording in %2d s   FRONT %s   SIDE %s"
                    % (distance, seconds_left, format_ir(front_live), format_ir(side_live)))
                sleep_checking(1000, None)          # B stops it, if available
            log("  Recording %d cm - hold still..." % distance)
            row = (distance, record_ir(FRONT_SHARP_IR_PORT), record_ir(WALL_SHARP_IR_PORT))
            rows.append(row)
            log("Recorded %d cm:   FRONT %s   SIDE %s"
                % (distance, format_ir(row[1]), format_ir(row[2])))
        log("")
        log("All distances recorded.")
    except StopRequested as reason:
        log("Stopped:", reason)
    print_calibration_table(rows)


# ===========================================================================
# MAIN
# ===========================================================================
# mode name -> (function to run, list of (setting name, value) it needs)
MODES = {
    "test_ir_front": (test_ir_front, [("FRONT_SHARP_IR_PORT", FRONT_SHARP_IR_PORT)]),
    "test_ir_side": (test_ir_side, [("WALL_SHARP_IR_PORT", WALL_SHARP_IR_PORT)]),
    "test_servo": (test_servo, [("FRONT_SERVO_PORT", FRONT_SERVO_PORT)]),
    "test_motors": (test_motors, [("LEFT_MOTOR_PORT", LEFT_MOTOR_PORT),
                                  ("RIGHT_MOTOR_PORT", RIGHT_MOTOR_PORT)]),
    "wall_follow_2state": (wall_follow_2state,
                           [("WALL_SHARP_IR_PORT", WALL_SHARP_IR_PORT),
                            ("WALL_TOO_CLOSE", WALL_TOO_CLOSE),
                            ("WALL_TOO_FAR", WALL_TOO_FAR),
                            ("WALL_2_TARGET", WALL_2_TARGET)]),
    "wall_follow_3state": (wall_follow_3state,
                           [("WALL_SHARP_IR_PORT", WALL_SHARP_IR_PORT),
                            ("WALL_TOO_CLOSE", WALL_TOO_CLOSE),
                            ("WALL_TOO_FAR", WALL_TOO_FAR),
                            ("WALL_3_NEAR", WALL_3_NEAR),
                            ("WALL_3_FAR", WALL_3_FAR)]),
    "test_scan": (test_scan, [("SERVO_LEFT", SERVO_LEFT),
                              ("SERVO_CENTER", SERVO_CENTER),
                              ("SERVO_RIGHT", SERVO_RIGHT)]),
    "test_turn": (test_turn, []),
    "test_buttons": (test_buttons, []),
    "calibrate_ir": (calibrate_ir, [("FRONT_SHARP_IR_PORT", FRONT_SHARP_IR_PORT),
                                    ("WALL_SHARP_IR_PORT", WALL_SHARP_IR_PORT)]),
    "open_space": (open_space, [("SERVO_LEFT", SERVO_LEFT),
                                ("SERVO_CENTER", SERVO_CENTER),
                                ("SERVO_RIGHT", SERVO_RIGHT),
                                ("FRONT_BLOCKED", FRONT_BLOCKED),
                                ("FRONT_OPEN", FRONT_OPEN),
                                ("SCAN_ANGLES_DEG", SCAN_ANGLES_DEG),
                                ("TURN_MS_PER_DEG", TURN_MS_PER_DEG)]),
}


def handle_stop_signal(signum, frame):
    # The IDE's Stop button ends the program; turning that into SystemExit
    # makes the finally-block in main() run, so motors and servo turn off.
    raise SystemExit(0)


def main():
    signal.signal(signal.SIGTERM, handle_stop_signal)
    signal.signal(signal.SIGINT, handle_stop_signal)

    if k is None:
        log("KIPR library not found - this program must run on the Wombat.")
        return
    if MODE not in MODES:
        log("Unknown MODE '%s'. Choose one of: %s" % (MODE, ", ".join(sorted(MODES))))
        return

    behavior, required_settings = MODES[MODE]
    missing = [name for name, value in required_settings if value is None]
    if missing:
        log("Cannot run '%s' - fill in these settings first:" % MODE)
        for name in missing:
            log("   ", name)
        return

    log("Running mode:", MODE)
    try:
        behavior()
    finally:
        stop_motors()
        k.disable_servos()
        log("Motors and servo stopped.")


if __name__ == "__main__":
    main()
