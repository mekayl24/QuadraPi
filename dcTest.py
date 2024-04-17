import RPi.GPIO as GPIO
from time import sleep



GPIO.setmode(GPIO.BOARD)

print("sleeping")

sleep(3)

print("awake")


#setting pins as outputs, add whatever gpio pins are outs to here
enable_pin = 7
input1_pin = 3
input2_pin = 5


GPIO.setup(input1_pin, GPIO.OUT)   
GPIO.setup(input2_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)


# Set enable pin to high to enable motor
GPIO.output(enable_pin, True)

# Function to drive motor forward
def forward():
    GPIO.output(input1_pin, True)
    GPIO.output(input2_pin, False)

# Function to drive motor backward
def backward():
    GPIO.output(input1_pin, False)
    GPIO.output(input2_pin, True)

# Move motor forward for 2 seconds
forward()
time.sleep(2)

# Stop motor
GPIO.output(input1_pin, False)
GPIO.output(input2_pin, False)

# Clean up GPIO
GPIO.cleanup()






"""
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

"""