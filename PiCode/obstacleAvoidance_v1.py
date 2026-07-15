"""
Phase 2: Reactive obstacle avoidance using RPLidar S1/S2.

No mapping or localization yet - this just reacts to whatever is
directly in front of the robot right now, using a forward-facing
"cone" of LIDAR angles.

Motor control functions are stubs (they just print) - fill them in
once you have motors wired up. Everything else is ready to go.
"""

from rplidar import RPLidar
import time
import serial

# ---------------------------------------------------------------
# CONFIG - tune these once you're testing with real motors
# ---------------------------------------------------------------
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUD = 460800          # S1/S2 baud rate

ARDUINO_PORT = "/dev/ttyACM0"
ARDUINO_BAUD = 9600

arduino = None
last_command = None

STOP_DISTANCE_MM = 500        # if something is closer than this, react
FORWARD_CONE_DEG = 30         # +/- degrees around "front" (0 deg) to check

CHECK_LEFT_DEG = (60, 120)    # angle range considered "left side"
CHECK_RIGHT_DEG = (240, 300)  # angle range considered "right side"


# ---------------------------------------------------------------
# MOTOR CONTROL STUBS - replace the print() calls with real motor
# code once you have a motor driver wired up (e.g. GPIO pins on a
# Raspberry Pi via RPi.GPIO or gpiozero)
# ---------------------------------------------------------------
def send_command(command):
        global last_command
        
        if command == last_command:
                return
                
        arduino.write(command.encode())
        last_command = command
        print(f"Sent: {command}")
        
def move_forward():
    send_command("F")
    
def turn_left():
    send_command("L")
    
def turn_right():
    send_command("R")
    
def stop():
    send_command("S")


# ---------------------------------------------------------------
# SCAN ANALYSIS
# ---------------------------------------------------------------
def angle_in_range(angle, lo, hi):
    """Handles ranges that wrap around 0/360, e.g. -30 to 30."""
    lo = lo % 360
    hi = hi % 360
    if lo <= hi:
        return lo <= angle <= hi
    return angle >= lo or angle <= hi


def get_min_distance_in_cone(scan, center_deg, half_width_deg):
    """Returns the closest distance (mm) found within the cone, or None."""
    lo = center_deg - half_width_deg
    hi = center_deg + half_width_deg
    distances = [
        distance
        for (quality, angle, distance) in scan
        if angle_in_range(angle, lo, hi) and distance > 0
    ]
    return min(distances) if distances else None


def decide_action(scan):
    """Looks at the current scan and decides what the robot should do."""
    front_distance = get_min_distance_in_cone(scan, 0, FORWARD_CONE_DEG)

    if front_distance is None or front_distance > STOP_DISTANCE_MM:
        return "forward"

    # Something is close in front - figure out which side has more room
    left_min = get_min_distance_in_cone(
        scan, sum(CHECK_LEFT_DEG) / 2, (CHECK_LEFT_DEG[1] - CHECK_LEFT_DEG[0]) / 2
    )
    right_min = get_min_distance_in_cone(
        scan, sum(CHECK_RIGHT_DEG) / 2, (CHECK_RIGHT_DEG[1] - CHECK_RIGHT_DEG[0]) / 2
    )

    left_min = left_min if left_min is not None else float("inf")
    right_min = right_min if right_min is not None else float("inf")

    if left_min > right_min:
        return "left"
    else:
        return "right"


# ---------------------------------------------------------------
# MAIN LOOP
# ---------------------------------------------------------------
def main():
    global arduino
    
    arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout = 1)
    time.sleep(2)
    
    print("[LIDAR] Connecting...")
    lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)
    print("[LIDAR] Connected.")

    try:
        info = lidar.get_info()
        print(f"Device Info: {info}")
        health = lidar.get_health()
        print(f"Device health: {health}")

        print("[LIDAR] Streaming. Starting obstacle avoidance loop...")

        # Clear any leftover bytes from the info/health commands before
        # starting the scan stream - prevents "descriptor length mismatch"
        lidar.clean_input()

        for scan in lidar.iter_scans():
            action = decide_action(scan)

            if action == "forward":
                move_forward()
            elif action == "left":
                turn_left()
            elif action == "right":
                turn_right()

            # Small delay so prints are readable while testing.
            # Remove or shrink this once real motors are attached.
            #time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping due to user interrupt...")
    except Exception as e:
        print(f"An error occured: {e}")
    finally:
        stop()
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
    time.sleep(0.5)

if __name__ == "__main__":
    main()
