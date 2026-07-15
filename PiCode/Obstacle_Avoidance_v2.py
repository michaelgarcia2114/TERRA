"""
Reactive obstacle avoidance with timed soil sampling.

The Pi:
- Controls obstacle avoidance.
- Counts only time spent driving forward.
- Requests a sample after enough forward-driving time.
- Waits for Arduino to finish the sampling cycle.
"""

from rplidar import RPLidar
import time
import serial


# ---------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600

STOP_DISTANCE_MM = 500
FORWARD_CONE_DEG = 30

CHECK_LEFT_DEG = (60, 120)
CHECK_RIGHT_DEG = (240, 300)

# Request a sample after this much accumulated forward-driving time.
SAMPLE_AFTER_FORWARD_SECONDS = 15

# The front must be this clear before lowering the sensor.
SAMPLE_CLEARANCE_MM = 800

# Arduino's simulated cycle takes about 7.5 seconds.
SAMPLE_TIMEOUT_SECONDS = 15


# ---------------------------------------------------------------
# GLOBAL STATE
# ---------------------------------------------------------------
arduino = None
last_command = None

sampling_active = False
sample_started_at = None

forward_drive_time = 0.0
last_timer_update = None

serial_buffer = b""


# ---------------------------------------------------------------
# ARDUINO COMMUNICATION
# ---------------------------------------------------------------
def send_command(command):
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
    """
    Reads complete lines from the Arduino without blocking.

    Returns:
        "done" if Arduino reports DONE
        "aborted" if Arduino reports ABORTED
        None otherwise
    """
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


# ---------------------------------------------------------------
# FORWARD-DRIVING TIMER
# ---------------------------------------------------------------
def update_forward_timer():
    """
    Adds time only while the most recent command is F.
    Turning, stopping, and sampling do not count.
    """
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


# ---------------------------------------------------------------
# SCAN ANALYSIS
# ---------------------------------------------------------------
def angle_in_range(angle, lo, hi):
    """Handles ranges that wrap around 0/360."""
    lo = lo % 360
    hi = hi % 360

    if lo <= hi:
        return lo <= angle <= hi

    return angle >= lo or angle <= hi


def get_min_distance_in_cone(scan, center_deg, half_width_deg):
    """Returns the closest distance in the cone, or None."""
    lo = center_deg - half_width_deg
    hi = center_deg + half_width_deg

    distances = [
        distance
        for quality, angle, distance in scan
        if angle_in_range(angle, lo, hi) and distance > 0
    ]

    return min(distances) if distances else None


def decide_action(scan):
    """Decides how the rover should move based on the current scan."""
    front_distance = get_min_distance_in_cone(
        scan,
        0,
        FORWARD_CONE_DEG
    )

    if front_distance is None or front_distance > STOP_DISTANCE_MM:
        return "forward"

    left_min = get_min_distance_in_cone(
        scan,
        sum(CHECK_LEFT_DEG) / 2,
        (CHECK_LEFT_DEG[1] - CHECK_LEFT_DEG[0]) / 2
    )

    right_min = get_min_distance_in_cone(
        scan,
        sum(CHECK_RIGHT_DEG) / 2,
        (CHECK_RIGHT_DEG[1] - CHECK_RIGHT_DEG[0]) / 2
    )

    if left_min is None:
        left_min = float("inf")

    if right_min is None:
        right_min = float("inf")

    if left_min > right_min:
        return "left"

    return "right"


def safe_to_sample(scan, action):
    """
    Sampling is allowed only while the rover would otherwise
    drive forward and the LIDAR has a valid, clear front reading.
    """
    if action != "forward":
        return False

    front_distance = get_min_distance_in_cone(
        scan,
        0,
        FORWARD_CONE_DEG
    )

    if front_distance is None:
        return False

    return front_distance > SAMPLE_CLEARANCE_MM


# ---------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------
def main():
    global arduino
    global sampling_active
    global sample_started_at
    global forward_drive_time
    global last_timer_update

    lidar = None

    try:
        print("[ARDUINO] Connecting...")

        arduino = serial.Serial(
            ARDUINO_PORT,
            ARDUINO_BAUD,
            timeout=0
        )

        # Opening the serial port may reset the Arduino.
        time.sleep(2)

        print("[ARDUINO] Connected")

        print("[LIDAR] Connecting...")

        lidar = RPLidar(
            LIDAR_PORT,
            baudrate=LIDAR_BAUD,
            timeout=3
        )

        print("[LIDAR] Connected")

        info = lidar.get_info()
        print(f"Device info: {info}")

        health = lidar.get_health()
        print(f"Device health: {health}")

        lidar.stop()
        time.sleep(0.5)
        lidar.reset()
        time.sleep(1)
        lidar.clean_input()

        print("[ROVER] Starting obstacle avoidance")
        print(
            f"[ROVER] Sampling every "
            f"{SAMPLE_AFTER_FORWARD_SECONDS} seconds of forward motion"
        )

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
                raise RuntimeError(
                    "Arduino aborted the sampling cycle"
                )
            # Keep consuming LIDAR scans, but do not send movement
            # commands while Arduino controls the sampling cycle.
            if sampling_active:
                elapsed_sampling_time = (
                    time.monotonic() - sample_started_at
                )

                if elapsed_sampling_time > SAMPLE_TIMEOUT_SECONDS:
                    stop()

                    raise TimeoutError(
                        "Arduino did not report DONE before timeout"
                    )

                continue

            action = decide_action(scan)

            if (
                forward_drive_time >= SAMPLE_AFTER_FORWARD_SECONDS
                and safe_to_sample(scan, action)
            ):
                print(
                    f"[PI] Forward time: "
                    f"{forward_drive_time:.1f} seconds"
                )

                sample_cycle()
                continue

            if action == "forward":
                move_forward()

            elif action == "left":
                turn_left()

            elif action == "right":
                turn_right()

    except KeyboardInterrupt:
        print("\n[ROVER] Stopping due to user interrupt")

    except Exception as error:
        print(f"[ERROR_test] {error}")

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
