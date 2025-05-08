import wiringpi
import time

# WiringPi pins
PUL_PIN = 2    # w2 = GPIO12 = Physical Pin 7
DIR_PIN = 3    # w3 = GPIO13 = Physical Pin 8
ENA_PIN = 11   # w11 = GPIO19 = Physical Pin 35

# Setup
wiringpi.wiringPiSetup()  # Use WiringPi pin numbering
wiringpi.pinMode(PUL_PIN, 1)  # Set as OUTPUT
wiringpi.pinMode(DIR_PIN, 1)
wiringpi.pinMode(ENA_PIN, 1)

# Enable motor (ENA+ LOW = enable)
wiringpi.digitalWrite(ENA_PIN, 0)

def run_motor(delay, duration, direction):
    wiringpi.digitalWrite(DIR_PIN, direction)  # 1 = CW, 0 = CCW
    steps = int(duration / (delay * 2))
    print(f"Running {'CW' if direction else 'CCW'} for {duration}s at delay {delay}s")
    for i in range(steps):
        wiringpi.digitalWrite(PUL_PIN, 1)
        time.sleep(delay)
        wiringpi.digitalWrite(PUL_PIN, 0)
        time.sleep(delay)

try:
    run_motor(delay=0.002, duration=5, direction=1)   # Slow CW
    run_motor(delay=0.001, duration=5, direction=1)   # Faster CW
    run_motor(delay=0.0005, duration=5, direction=1)  # Fastest CW
    run_motor(delay=0.0005, duration=5, direction=0)  # Fastest CCW

finally:
    wiringpi.digitalWrite(ENA_PIN, 1)  # Disable motor
    print("Motor test finished.")
