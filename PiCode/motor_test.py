from gpiozero import DigitalOutputDevice
from time import sleep

IN1 = DigitalOutputDevice(23)
IN2 = DigitalOutputDevice(24)
IN3 = DigitalOutputDevice(27)
IN4 = DigitalOutputDevice(22)


def stop():
	IN1.off()
	IN2.off()
	IN3.off()
	IN4.off()

print("left motor forward")
IN1.on()
IN2.off()
sleep(2)
stop()
sleep(1)

print("right motor forward")
IN3.on()
IN4.off()
sleep(2)
stop()

print("done")
