import wiringpi
import time

# Motor 1 pins (WiringPi numbers)
PUL1 = 2     # w2 = GPIO12
DIR1 = 3     # w3 = GPIO13
ENA1 = 11    # w11 = GPIO19

# Motor 2 pins (WiringPi numbers)
PUL2 = 4     # w4 = GPIO16
DIR2 = 5     # w5 = GPIO1
ENA2 = 6     # w6 = GPIO4

# Setup
wiringpi.wiringPiSetup()

# Set all pins as output
for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
    wiringpi.pinMode(pin, 1)

# Enable both motors
wiringpi.digitalWrite(ENA1, 0)
wiringpi.digitalWrite(ENA2, 0)

def run_motor(pul_pin, dir_pin, delay, duration, direction):
    wiringpi.digitalWrite(dir_pin, direction)
    steps = int(duration / (delay * 2))
    for _ in range(steps):
        wiringpi.digitalWrite(pul_pin, 1)
        time.sleep(delay)
        wiringpi.digitalWrite(pul_pin, 0)
        time.sleep(delay)

try:
    print("Motor 1 CW, Motor 2 CCW (slow)")
    run_motor(PUL1, DIR1, 0.002, 5, 1)
    run_motor(PUL2, DIR2, 0.002, 5, 0)

    print("Motor 1 CW, Motor 2 CCW (fast)")
    run_motor(PUL1, DIR1, 0.0005, 5, 1)
    run_motor(PUL2, DIR2, 0.0005, 5, 0)

finally:
    wiringpi.digitalWrite(ENA1, 1)
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors disabled.")