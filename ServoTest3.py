from time import sleep
import board
import busio
from adafruit_servokit import ServoKit

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PCA9685 module
kit = ServoKit(channels=16)

# Define the minimum and maximum pulse lengths for your servo motors
# These values may need to be adjusted based on your servo specifications
servo_min = 0  # Min pulse length out of 4096
servo_max = 180  # Max pulse length out of 4096



kit.servo[0].angle = 175
print("Moving to 175 deg")
sleep(1)
kit.servo[0].angle = 45
print("Moved")

