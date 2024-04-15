import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)

print("sleeping")

sleep(3)

print("awake")


GPIO.setup(3, GPIO.OUT)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)



pwm=GPIO.PWM(7, 100)


pwm.start(0)



GPIO.output(3, True)
GPIO.output(5, False)


print("running...")
pwm.ChangeDutyCycle(50)

GPIO.output(7, True)

print("sleeping...")

sleep(2)

print("awake")

GPIO.output(7, False)


GPIO.output(3, False)
GPIO.output(5, True)


pwm.ChangeDutyCycle(75)

GPIO.output(7, True)

sleep(3)
GPIO.output(7, False)
pwm.stop()
GPIO.cleanup()

