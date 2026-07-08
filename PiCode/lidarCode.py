from rplidar import RPLidar
import time
import sys
print (sys.executable)

LIDAR_PORT   = "/dev/ttyUSB0"
LIDAR_BAUD   = 460800
LIDAR_PWM    = 660
LIDAR_WARMUP = 2.0

#lidar initialize
print("[LIDAR] Connecting …")
lidar = RPLidar(LIDAR_PORT, baudrate=LIDAR_BAUD, timeout=3)
print("[LIDAR] Connected.")

#lidar.set_motor_pwm(LIDAR_PWM)
#time.sleep(LIDAR_WARMUP)
#lidar.stop()
#time.sleep(0.5)
#scan_gen = lidar.start_scan()
#Lidar scanning
try:
    info = lidar.get_info()
    print(f"Device Info: {info}")
    health = lidar.get_health()
    print(f"Device health: {health}")
    
    print("[LIDAR] Streaming.")
    for i, scan in enumerate(lidar.iter_scans()):
        print(f"Scan #{i}: Got {len(scan)} measurements")
        #print first few data points
        for quality, angle, distance in scan[:3]:
            print(f" -> Angle: {angle:.2f}, Distance {distance:.2f}mm (Quality {quality})")
            
        if i >= 10:
            break
    

except KeyboardInterrupt:
    print("stopping due to user interrupt...")
except Exception as e:
    print(f"An error occured: {e}")
finally:
        lidar.stop()
        lidar.stop_motor()
      #  lidar.set_motor_pwm(0)
        lidar.disconnect()