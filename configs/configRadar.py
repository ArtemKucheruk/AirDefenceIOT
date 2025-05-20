import numpy as np

# ==== HARDWARE CONFIGURATION ====
TRIG = 7
ECHO = 8
STEPPER_PINS = [12, 13, 14, 15]
DELAY = 0.005
STEP_ANGLE = 1.8
STEP_DELAY = 0.01
TOTAL_ANGLE = 180
STEPS_PER_DEGREE = int(512 / 360)

# ==== RADAR DISPLAY CONFIGURATION ====
WIDTH, HEIGHT = 1080, 720
CENTER = (WIDTH // 2, HEIGHT - 10)
MAX_DISTANCE = 400
RADAR_RANGE_CM = 150
FADE_FACTOR = 0.983
GREEN = (153, 50, 204)
RED = (0, 0, 255)

# ==== NETWORK CONFIGURATION ====
RADAR_HOST = '0.0.0.0'
RADAR_PORT = 65431
WEB_HOST = '0.0.0.0'
WEB_PORT = 5000

# ==== GLOBAL VARIABLES ====
total_steps_moved = 0
frame = np.zeros((480, 640, 3), dtype=np.uint8)



# Stepper motor sequence
SEQ = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
]
