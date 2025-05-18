import socket
import time
import math
import wiringpi as wp

# ==== CONFIGURATION ====
HOST = '0.0.0.0'      # Listen on all interfaces
PORT = 65432          # Port to listen on

TRIG = 0              # GPIO.5 (BCM 24) -> HC-SR04 TRIG
ECHO = 1              # GPIO.4 (BCM 23) -> HC-SR04 ECHO

STEPPER_PINS = [13, 14, 15, 16]  # WiringPi pin numbers

DELAY = 0.005          # Motor delay
STEP_ANGLE = 1.8       # Degree per step
STEP_DELAY = 0.01      # Delay between steps
TOTAL_ANGLE = 180      # 180-degree sweep
STEPS_PER_DEGREE = int(512 / 360)  # 512 steps = 360 degrees

# ==== SETUP ====
wp.wiringPiSetup()

# Setup ultrasonic
wp.pinMode(TRIG, 1)
wp.pinMode(ECHO, 0)

# Setup stepper motor pins
for pin in STEPPER_PINS:
    wp.pinMode(pin, 1)

# Full-step sequence (4-step)
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


def move_stepper(steps, direction=1):
    """Rotate the stepper motor a number of steps"""
    for _ in range(steps):
        for halfstep in range(8):
            for pin in range(4):
                wp.digitalWrite(STEPPER_PINS[pin], SEQ[::direction][halfstep][pin])
            time.sleep(DELAY)


def get_distance():
    """Measure distance using ultrasonic sensor"""
    wp.digitalWrite(TRIG, 0)
    time.sleep(0.000002)
    wp.digitalWrite(TRIG, 1)
    time.sleep(0.00001)
    wp.digitalWrite(TRIG, 0)

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 0:
        if time.time() - timeout_start > 0.05:
            return -1
    start_time = time.time()

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 1:
        if time.time() - timeout_start > 0.05:
            return -1
    end_time = time.time()

    duration = end_time - start_time
    distance = duration * 34300 / 2
    return round(distance, 1)


def serve_radar():
    """Main server loop with sweeping logic"""
    print(f"Starting radar server on {HOST}:{PORT}...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()

        print(f"Client connected: {addr}")
        with conn:
            angle = 0
            direction = 1  # 1 for increasing, -1 for decreasing

            while True:
                # Step the motor
                steps = STEPS_PER_DEGREE
                move_stepper(steps, direction)

                # Get distance and send to client
                distance = get_distance()
                if distance > 0:
                    msg = f"{distance},{angle}\n"
                    conn.sendall(msg.encode())

                # Update angle
                angle += direction
                if angle >= TOTAL_ANGLE or angle <= 0:
                    direction *= -1  # Reverse sweep

                time.sleep(STEP_DELAY)


if __name__ == "__main__":
    try:
        serve_radar()
    except KeyboardInterrupt:
        print("Shutting down.")
