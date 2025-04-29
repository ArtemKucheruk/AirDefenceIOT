import RPi.GPIO as GPIO
import time

# Pins (BCM numbering)
STEP_PIN = 6    # GPIO17 (Physical pin 11)
DIR_PIN = 2     # GPIO27 (Physical pin 13)
ENABLE_PIN = 5  # GPIO22 (Physical pin 15)

# Configure GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(STEP_PIN, GPIO.OUT)
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)

# Enable motor (active low, depending on TB6600 version)
GPIO.output(ENABLE_PIN, GPIO.LOW)
