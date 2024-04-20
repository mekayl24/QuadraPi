import RPi.GPIO as GPIO
import time



GPIO.setmode(GPIO.BOARD)

print("sleeping")

time.sleep(3)

print("awake")


#setting pins as outputs, add whatever gpio pins are outs to here
###Front wheels

enable_pin = 18
enable2_pin = 3
input1_pin = 16
input2_pin = 12
input3_pin = 5
input4_pin = 7

##Rear Wheels

enable3_pin = 38
enable4_pin = 33
input5_pin = 36
input6_pin = 32
input7_pin = 35
input8_pin = 37




GPIO.setup(input1_pin, GPIO.OUT)   
GPIO.setup(input2_pin, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)

GPIO.setup(input3_pin, GPIO.OUT)   
GPIO.setup(input4_pin, GPIO.OUT)
GPIO.setup(enable2_pin, GPIO.OUT)



GPIO.setup(input5_pin, GPIO.OUT)   
GPIO.setup(input6_pin, GPIO.OUT)
GPIO.setup(enable3_pin, GPIO.OUT)

GPIO.setup(input7_pin, GPIO.OUT)   
GPIO.setup(input8_pin, GPIO.OUT)
GPIO.setup(enable4_pin, GPIO.OUT)



# Set enable pin to high to enable motor
GPIO.output(enable_pin, True)
GPIO.output(enable2_pin, True)
GPIO.output(enable3_pin, True)
GPIO.output(enable4_pin, True)

# Function to drive motor forward
def forward():
    GPIO.output(input1_pin, True)
    GPIO.output(input2_pin, False)
    GPIO.output(input3_pin, True)
    GPIO.output(input4_pin, False)
    
    GPIO.output(input5_pin, True)
    GPIO.output(input6_pin, False)
    GPIO.output(input7_pin, True)
    GPIO.output(input8_pin, False)
    
    

# Function to drive motor backward
def backward():
    GPIO.output(input1_pin, False)
    GPIO.output(input2_pin, True)
    GPIO.output(input3_pin, False)
    GPIO.output(input4_pin, True)
    
    
    GPIO.output(input5_pin, False)
    GPIO.output(input6_pin, True)
    GPIO.output(input7_pin, False)
    GPIO.output(input8_pin, True)
    
def stop():
    GPIO.output(input1_pin, False)
    GPIO.output(input2_pin, False)
    GPIO.output(input3_pin, False)
    GPIO.output(input4_pin, False)
    
    GPIO.output(input5_pin, False)
    GPIO.output(input6_pin, False)
    GPIO.output(input7_pin, False)
    GPIO.output(input8_pin, False)
# Move motor forward for 2 seconds
forward()
time.sleep(4)

stop()

# Stop motor


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
