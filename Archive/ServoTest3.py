import time
import board
import busio
from adafruit_servokit import ServoKit

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize PCA9685 servo controller
kit = ServoKit(channels=16, i2c=i2c)
  
# Define the servo number (0-15) connected to the PCA9685
servo_number = 0

# Set servo angle to 45 degrees
kit.servo[servo_number].angle = 45

# Wait for the servo to reach the desired position
time.sleep(1)

# Release the servo to stop applying torque (optional)
kit.servo[servo_number].angle = None

