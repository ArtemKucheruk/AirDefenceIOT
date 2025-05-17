import time
import wiringpi
from stepper_motor.config import *



def setup_motors():
    """
    Setup function to initialize the motor pins, configure wiringPi, and enable the motors.
    """
    # Initialize wiringPi
    wiringpi.wiringPiSetup()

    # Set the pin modes for each motor
    for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
        wiringpi.pinMode(pin, 1)  # Set pin mode to OUTPUT

    # Enable both motors (LOW = enabled)
    wiringpi.digitalWrite(ENA1, 0)
    wiringpi.digitalWrite(ENA2, 0)
    print("Motors initialized and enabled.")

def run_motor_1(x_offset):
    """
    Function to control motor 1 based on the x offset.
    """
    delay = 0.002  # Initial delay
    duration = 3  # Duration for each motor step

    # Determine motor direction based on x offset
    if x_offset > 0:
        dir1 = 1  # Move motor 1 CW (clockwise)
    else:
        dir1 = 0  # Move motor 1 CCW (counterclockwise)

    # Control the first motor
    wiringpi.digitalWrite(DIR1, dir1)
    steps = int(duration / (delay * 2))

    for i in range(steps):
        wiringpi.digitalWrite(PUL1, 1)
        time.sleep(delay)
        wiringpi.digitalWrite(PUL1, 0)
        time.sleep(delay)



def run_motor_2(y_offset):
    """
    Function to control motor 2 based on the y offset.
    """
    delay = 0.002  # Initial delay
    duration = 3  # Duration for each motor step

    # Determine motor direction based on y offset
    if y_offset > 0:
        dir2 = 1  # Move motor 2 CW (clockwise)
    else:
        dir2 = 0  # Move motor 2 CCW (counterclockwise)

    # Control the second motor
    wiringpi.digitalWrite(DIR2, dir2)
    steps = int(duration / (delay * 2))

    for i in range(steps):
        wiringpi.digitalWrite(PUL2, 1)
        time.sleep(delay)
        wiringpi.digitalWrite(PUL2, 0)
        time.sleep(delay)


def disableBothMotors():
    wiringpi.digitalWrite(ENA1, 1)
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors stopped and disabled.")