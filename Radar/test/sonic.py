import time
import wiringpi as wp

# Use wPi numbering (from gpio readall)
TRIG = 0    # wPi 6 = Physical Pin 11 (GPIO114)
ECHO = 1    # wPi 7 = Physical Pin 12 (GPIO115) *CONFIRM THIS!*

# Initialize WiringPi
wp.wiringPiSetup()  # Use wPi numbering
wp.pinMode(TRIG, wp.OUTPUT)
wp.pinMode(ECHO, wp.INPUT)

def get_distance():
    wp.digitalWrite(TRIG, wp.LOW)
    time.sleep(0.002)

    wp.digitalWrite(TRIG, wp.HIGH)
    time.sleep(0.00001)  # 10µs
    wp.digitalWrite(TRIG, wp.LOW)

    timeout = time.time() + 1
    while wp.digitalRead(ECHO) == 0:
        if time.time() > timeout:
            return -1
    pulse_start = time.time()

    timeout = time.time() + 1
    while wp.digitalRead(ECHO) == 1:
        if time.time() > timeout:
            return -1
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

try:
    while True:
        dist = get_distance()
        print(f"Distance: {dist} cm")
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
