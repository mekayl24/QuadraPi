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
print("Moved to 45 deg")
sleep(1)

# Function to map angle to PWM pulse
def angle_to_pulse(angle):
    pulse = int((angle / 180) * (servo_max - servo_min) + servo_min)
    return pulse

# Function to move servo to a specific angle
def set_servo_angle(servo, angle):
    pulse = angle_to_pulse(angle)
    servo.angle = pulse

# Main function
def main():
    try:
        # Get servo objects
        print("running main")
        servo1 = kit.servo[0]
        servo2 = kit.servo[1]

        # Move servo 1 to 90 degrees
        set_servo_angle(servo1, 90)

        # Move servo 2 to 45 degrees
        set_servo_angle(servo2, 45)

        # Wait for 2 seconds
        sleep(2)

        # Move servo 1 to 180 degrees
        set_servo_angle(servo1, 180)

        # Move servo 2 to 135 degrees
        set_servo_angle(servo2, 135)

        # Wait for 2 seconds
        sleep(2)

        # Move servo 1 back to 0 degrees
        set_servo_angle(servo1, 0)

        # Move servo 2 back to 90 degrees
        set_servo_angle(servo2, 90)
        
        return
    except KeyboardInterrupt:
        print("Keyboard Interrupt. Stopping servo control.")
        
if __name__ == "__main__":
    main()
