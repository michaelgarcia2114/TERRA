"""Reactive obstacle avoidance with timed soil sampling and escape recovery.

The Raspberry Pi:
- Controls obstacle avoidance.
- Counts only time spent driving forward.
- Requests a soil sample after enough forward-driving time.
- Waits for the Arduino to finish the sampling cycle.
- Escapes persistent obstacles with a timed reverse and committed turn.

Command protocol sent to the Arduino:
    F = forward
    B = backward
    L = turn left
    R = turn right
    S = stop
    P = perform the complete sampling cycle

The Arduino must reply with a newline-terminated DONE or ABORTED after P.
"""

import time

import serial
from rplidar import RPLidar


# ------------------------------------------------------------------
# CONNECTIONS
# ------------------------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600


# ------------------------------------------------------------------
# OBSTACLE AVOIDANCE
# ------------------------------------------------------------------
STOP_DISTANCE_MM = 500
FORWARD_CONE_DEG = 30

CHECK_LEFT_DEG = (240, 300)
CHECK_RIGHT_DEG = (60, 120)

# The rover must remain front-blocked this long before escaping.
BLOCKED_BEFORE_ESCAPE_SECONDS = 1.5

# Escape sequence: reverse briefly, then commit to one turn. Keeping
# these actions timed prevents rapid left/right command oscillation.
REVERSE_SECONDS = 0.8
COMMITTED_TURN_SECONDS = 1.2

# Reverse only when the LIDAR has a valid, clear rear measurement.
REAR_CENTER_DEG = 180
REAR_CONE_DEG = 30
REAR_CLEARANCE_MM = 600

# IMPORTANT: 2-D LIDAR sees obstacles, not a drop at a table edge.
# Leave False for tabletop testing. Set True only when testing on the
# floor or in another area where reversing cannot cause a fall.
ENABLE_AUTOMATIC_REVERSE = False


# ------------------------------------------------------------------
# SOIL SAMPLING
# ------------------------------------------------------------------
SAMPLE_AFTER_FORWARD_SECONDS = 15
SAMPLE_CLEARANCE_MM = 800
SAMPLE_TIMEOUT_SECONDS = 15


# ------------------------------------------------------------------
# GLOBAL STATE
# ------------------------------------------------------------------
arduino = None
last_command = None

sampling_active = False
sample_started_at = None

forward_drive_time = 0.0
last_timer_update = None

serial_buffer = b""

# Escape phases are None, "reverse", or "turn".
blocked_since = None
escape_phase = None
escape_phase_ends_at = None
escape_turn_direction = None


# ------------------------------------------------------------------
# ARDUINO COMMUNICATION
# ------------------------------------------------------------------
def send_command(command):
    """Send a command only when it differs from the previous command."""
    global last_command

    if command == last_command:
        return

    arduino.write(command.encode())
    arduino.flush()

    last_command = command
    print(f"[PI] Sent: {command}")


def move_forward():
    send_command("F")


def move_backward():
    send_command("B")


def turn_left():
    send_command("L")


def turn_right():
    send_command("R")


def stop():
    send_command("S")


def sample_cycle():
    global sampling_active
    global sample_started_at

    send_command("P")
    sampling_active = True
    sample_started_at = time.monotonic()
    print("[PI] Sampling cycle requested")


def read_arduino_messages():
    """Read complete, newline-terminated Arduino messages without blocking."""
    global serial_buffer

    bytes_waiting = arduino.in_waiting
    if bytes_waiting > 0:
        serial_buffer += arduino.read(bytes_waiting)

    result = None

    while b"\n" in serial_buffer:
        raw_line, serial_buffer = serial_buffer.split(b"\n", 1)
        line = raw_line.decode(errors="ignore").strip()

        if not line:
            continue

        print(f"[ARDUINO] {line}")

        if line == "DONE":
            result = "done"
        elif line == "ABORTED":
            result = "aborted"

    return result


# ------------------------------------------------------------------
# FORWARD-DRIVING TIMER
# ------------------------------------------------------------------
def update_forward_timer():
    """Add time only while the most recent command is forward."""
    global forward_drive_time
    global last_timer_update

    current_time = time.monotonic()

    if last_timer_update is None:
        last_timer_update = current_time
        return

    elapsed = current_time - last_timer_update
    last_timer_update = current_time

    if last_command == "F" and not sampling_active:
        forward_drive_time += elapsed


# ------------------------------------------------------------------
# LIDAR SCAN ANALYSIS
# ------------------------------------------------------------------
def angle_in_range(angle, lo, hi):
    """Return whether an angle is in a range that may wrap around zero."""
    lo %= 360
    hi %= 360

    if lo <= hi:
        return lo <= angle <= hi

    return angle >= lo or angle <= hi


def get_min_distance_in_cone(scan, center_deg, half_width_deg):
    """Return the closest valid distance in a cone, or None."""
    lo = center_deg - half_width_deg
    hi = center_deg + half_width_deg

    distances = [
        distance
        for _quality, angle, distance in scan
        if angle_in_range(angle, lo, hi) and distance > 0
    ]

    return min(distances) if distances else None


def get_side_clearances(scan):
    left_min = get_min_distance_in_cone(
        scan,
        sum(CHECK_LEFT_DEG) / 2,
        (CHECK_LEFT_DEG[1] - CHECK_LEFT_DEG[0]) / 2,
    )
    right_min = get_min_distance_in_cone(
        scan,
        sum(CHECK_RIGHT_DEG) / 2,
        (CHECK_RIGHT_DEG[1] - CHECK_RIGHT_DEG[0]) / 2,
    )

    # A side with no returns is treated as open. If this proves unreliable
    # in the test area, replace infinity with zero to require valid returns.
    left_clearance = left_min if left_min is not None else float("inf")
    right_clearance = right_min if right_min is not None else float("inf")

    return left_clearance, right_clearance


def choose_turn_direction(scan):
    """Choose the clearer side once, then retain it through the escape turn."""
    left_clearance, right_clearance = get_side_clearances(scan)

    if left_clearance > right_clearance:
        return "left"

    return "right"


def decide_normal_action(scan):
    """Choose a normal movement action and track persistent front blockage."""
    global blocked_since

    front_distance = get_min_distance_in_cone(scan, 0, FORWARD_CONE_DEG)

    if front_distance is None or front_distance > STOP_DISTANCE_MM:
        blocked_since = None
        return "forward"

    if blocked_since is None:
        blocked_since = time.monotonic()

    return choose_turn_direction(scan)


def rear_is_clear(scan):
    """Require a valid rear measurement before allowing reverse motion."""
    rear_distance = get_min_distance_in_cone(
        scan,
        REAR_CENTER_DEG,
        REAR_CONE_DEG,
    )

    return rear_distance is not None and rear_distance > REAR_CLEARANCE_MM


def safe_to_sample(scan, action):
    """Allow sampling only with a valid, clear front measurement."""
    if action != "forward":
        return False

    front_distance = get_min_distance_in_cone(scan, 0, FORWARD_CONE_DEG)
    return front_distance is not None and front_distance > SAMPLE_CLEARANCE_MM


# ------------------------------------------------------------------
# ESCAPE STATE MACHINE
# ------------------------------------------------------------------
def escape_is_due():
    return (
        blocked_since is not None
        and time.monotonic() - blocked_since >= BLOCKED_BEFORE_ESCAPE_SECONDS
    )


def begin_escape(scan):
    """Start a safe reverse, or turn immediately when reverse is unavailable."""
    global blocked_since
    global escape_phase
    global escape_phase_ends_at
    global escape_turn_direction

    now = time.monotonic()
    escape_turn_direction = choose_turn_direction(scan)
    blocked_since = None

    if ENABLE_AUTOMATIC_REVERSE and rear_is_clear(scan):
        escape_phase = "reverse"
        escape_phase_ends_at = now + REVERSE_SECONDS
        print(
            f"[ESCAPE] Reversing for {REVERSE_SECONDS:.1f}s, then turning "
            f"{escape_turn_direction}"
        )
    else:
        escape_phase = "turn"
        escape_phase_ends_at = now + COMMITTED_TURN_SECONDS

        if ENABLE_AUTOMATIC_REVERSE:
            print("[ESCAPE] Rear is not confirmed clear; skipping reverse")
        else:
            print("[ESCAPE] Automatic reverse disabled; skipping reverse")

        print(
            f"[ESCAPE] Committing to {escape_turn_direction} turn for "
            f"{COMMITTED_TURN_SECONDS:.1f}s"
        )


def update_escape(scan):
    """Return the current escape action, or None when the escape is complete."""
    global escape_phase
    global escape_phase_ends_at
    global escape_turn_direction

    if escape_phase is None:
        return None

    now = time.monotonic()

    if escape_phase == "reverse":
        # Recheck every scan so a newly detected rear obstacle stops reverse.
        if now < escape_phase_ends_at and rear_is_clear(scan):
            return "backward"

        escape_phase = "turn"
        escape_phase_ends_at = now + COMMITTED_TURN_SECONDS
        print(
            f"[ESCAPE] Committing to {escape_turn_direction} turn for "
            f"{COMMITTED_TURN_SECONDS:.1f}s"
        )
        return escape_turn_direction

    if escape_phase == "turn" and now < escape_phase_ends_at:
        return escape_turn_direction

    print("[ESCAPE] Escape sequence complete")
    escape_phase = None
    escape_phase_ends_at = None
    escape_turn_direction = None
    return None


def perform_action(action):
    if action == "forward":
        move_forward()
    elif action == "backward":
        move_backward()
    elif action == "left":
        turn_left()
    elif action == "right":
        turn_right()
    else:
        stop()


# ------------------------------------------------------------------
# MAIN LOOP
# ------------------------------------------------------------------
def main():
    global arduino
    global sampling_active
    global sample_started_at
    global forward_drive_time
    global last_timer_update

    lidar = None

    try:
        print("[ARDUINO] Connecting...")
        arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0)

        # Opening the serial port may reset the Arduino.
        time.sleep(2)
        print("[ARDUINO] Connected")

        print("[LIDAR] Connecting...")
        lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)
        print("[LIDAR] Connected")

        print(f"Device info: {lidar.get_info()}")
        print(f"Device health: {lidar.get_health()}")

        lidar.stop()
        time.sleep(0.5)
        lidar.reset()
        time.sleep(1)
        lidar.clean_input()

        print("[ROVER] Starting obstacle avoidance")
        print(
            f"[ROVER] Sampling every {SAMPLE_AFTER_FORWARD_SECONDS} seconds "
            "of forward motion"
        )
        print(f"[ROVER] Automatic reverse: {ENABLE_AUTOMATIC_REVERSE}")

        last_timer_update = time.monotonic()

        for scan in lidar.iter_scans():
            update_forward_timer()
            arduino_status = read_arduino_messages()

            if arduino_status == "done":
                sampling_active = False
                sample_started_at = None
                forward_drive_time = 0.0
                print("[PI] Sample completed")
                print("[PI] Forward timer reset")

            elif arduino_status == "aborted":
                raise RuntimeError("Arduino aborted the sampling cycle")

            # Continue consuming scans while the Arduino owns the motors.
            if sampling_active:
                elapsed_sampling_time = time.monotonic() - sample_started_at

                if elapsed_sampling_time > SAMPLE_TIMEOUT_SECONDS:
                    stop()
                    raise TimeoutError(
                        "Arduino did not report DONE before timeout"
                    )

                continue

            escape_action = update_escape(scan)
            if escape_action is not None:
                perform_action(escape_action)
                continue

            action = decide_normal_action(scan)

            if escape_is_due():
                begin_escape(scan)
                escape_action = update_escape(scan)
                perform_action(escape_action)
                continue

            if (
                forward_drive_time >= SAMPLE_AFTER_FORWARD_SECONDS
                and safe_to_sample(scan, action)
            ):
                print(f"[PI] Forward time: {forward_drive_time:.1f} seconds")
                sample_cycle()
                continue

            perform_action(action)

    except KeyboardInterrupt:
        print("\n[ROVER] Stopping due to user interrupt")

    except Exception as error:
        print(f"[ERROR] {error}")

    finally:
        if arduino is not None and arduino.is_open:
            stop()
            time.sleep(0.25)
            arduino.close()

        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass

        print("[ROVER] Shut down safely")


if __name__ == "__main__":
    main()