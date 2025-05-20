import numpy as np


# Camera settings
CAMERA_ID = 1
FRAME_WIDTH = 840
FRAME_HEIGHT = 880
FLIP_CAMERA = True  # Set to False if you don't need 180째 flip

# Camera optical parameters
# These determine the actual field of view angles
CAMERA_HORIZONTAL_FOV = 90  # degrees - typical for Raspberry Pi camera
CAMERA_VERTICAL_FOV = 90    # degrees - typical for Raspberry Pi camera

# Face detection settings
FACE_DETECTION_INTERVAL = 1  # Process every 3rd frame for detection
MIN_FACE_SIZE = (100, 100)  # Larger minimum size for faster processing

# Laser control pin (WiringPi numbering)
LASER_PIN = 16

# Motor control settings
# Motor 1 pins (X-axis) - WiringPi numbers
PUL1 = 2
DIR1 = 3
ENA1 = 11

# Motor 2 pins (Y-axis) - WiringPi numbers
PUL2 = 4
DIR2 = 5
ENA2 = 6

# Motor characteristics
STEPS_PER_REVOLUTION = 200  # Standard for NEMA 17 (1.8째 per step)
MICROSTEPS = 16  # Depends on your TB6600 configuration
TOTAL_STEPS_PER_REV = STEPS_PER_REVOLUTION * MICROSTEPS

MIN_ANGLE_THRESHOLD = 0.05
# Motor mechanical constraints
MAX_ANGLE_X = 45  # Maximum rotation angle in degrees
MAX_ANGLE_Y = 45  # Maximum rotation angle in degrees

# Movement speed settings - adjusted for better Y-axis performance
MAX_SPEED = 5000  # Maximum steps per second (reduced from 2000)
MIN_SPEED = 2000   # Minimum steps per second (increased from 100)
Y_SPEED_FACTOR = 7  # Slow down Y-axis for stability


axis_config = {
    'X': {'kp': 0.8, 'kd': 0.1, 'threshold': 0.05, 'speed_limit': 100},
    'Y': {'kp': 1.2, 'kd': 0.2, 'threshold': 0.05, 'speed_limit': 80}
}


CAMERA_MATRIX = np.array([[500, 0, FRAME_WIDTH/2],
                         [0, 500, FRAME_HEIGHT/2],
                         [0, 0, 1]])
DIST_COEFFS = np.array([-0.15, 0.1, 0, 0])