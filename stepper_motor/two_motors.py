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
for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
    wiringpi.pinMode(pin, 1)

# Enable both motors (LOW = enabled)
wiringpi.digitalWrite(ENA1, 0)
wiringpi.digitalWrite(ENA2, 0)

def run_both_motors(delay, duration, dir1, dir2):
    wiringpi.digitalWrite(DIR1, dir1)
    wiringpi.digitalWrite(DIR2, dir2)
    steps = int(duration / (delay * 2))

    for i in range(steps):
        wiringpi.digitalWrite(PUL1, 1)
        wiringpi.digitalWrite(PUL2, 1)
        time.sleep(delay)
        wiringpi.digitalWrite(PUL1, 0)
        wiringpi.digitalWrite(PUL2, 0)
        time.sleep(delay)

try:
    print("Both motors CW (slow)")
    run_both_motors(delay=0.002, duration=3, dir1=1, dir2=1)

    print("Both motors CW (faster)")
    run_both_motors(delay=0.001, duration=3, dir1=1, dir2=1)

    print("Both motors CW (faster)")
    run_both_motors(delay=0.0005, duration=3, dir1=1, dir2=1)

    print("Both motors CCW (fastest)")
    run_both_motors(delay=0.0002, duration=3, dir1=1, dir2=1)

    print("Both motors CCW (fastest x2)")
    run_both_motors(delay=0.0001, duration=3, dir1=1, dir2=1)

finally:
    wiringpi.digitalWrite(ENA1, 1)
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors stopped and disabled.")