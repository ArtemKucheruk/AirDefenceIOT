import socket
import time
import math
import wiringpi as wp

# ==== CONFIGURATION ====
HOST = '0.0.0.0'
PORT = 65432

TRIG = 7
ECHO = 8

STEPPER_PINS = [12, 13, 14, 15]

DELAY = 0.005
STEP_ANGLE = 1.8
STEP_DELAY = 0.01
TOTAL_ANGLE = 180
STEPS_PER_DEGREE = int(512 / 360)

# ==== SETUP ====
wp.wiringPiSetup()

wp.pinMode(TRIG, 1)
wp.pinMode(ECHO, 0)

for pin in STEPPER_PINS:
    wp.pinMode(pin, 1)

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

def disable_motor():
    for pin in STEPPER_PINS:
        wp.digitalWrite(pin, 0)

def move_stepper(steps, direction=1):
    try:
        for _ in range(steps):
            for halfstep in range(8):
                for pin in range(4):
                    wp.digitalWrite(STEPPER_PINS[pin], SEQ[::direction][halfstep][pin])
                time.sleep(DELAY)
    except Exception as e:
        print(f"Stepper error: {e}")
        disable_motor()

def get_distance():
    wp.digitalWrite(TRIG, 0)
    time.sleep(0.000002)
    wp.digitalWrite(TRIG, 1)
    time.sleep(0.00001)
    wp.digitalWrite(TRIG, 0)

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 0:
        if time.time() - timeout_start > 0.02:
            return -1

    start_time = time.time()

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 1:
        if time.time() - timeout_start > 0.02:
            return -1

    end_time = time.time()

    duration = end_time - start_time
    distance = duration * 34300 / 2
    return round(distance, 1)

# ==== GLOBAL step counter ====
total_steps_moved = 0

def serve_radar():
    global total_steps_moved
    print(f"Starting radar server on {HOST}:{PORT}...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        conn, addr = s.accept()

        print(f"Client connected: {addr}")
        with conn:
            angle = 0
            direction = 1

            while True:
                steps = STEPS_PER_DEGREE
                move_stepper(steps, direction)
                total_steps_moved += steps * direction

                distance = get_distance()
                msg = f"{distance},{angle}\n"
                conn.sendall(msg.encode())

                angle += direction
                if angle >= TOTAL_ANGLE or angle <= 0:
                    direction *= -1

                time.sleep(STEP_DELAY)

def return_to_origin():
    print(f"Returning to origin: {abs(total_steps_moved)} steps back")
    if total_steps_moved != 0:
        move_stepper(abs(total_steps_moved), direction=-1 if total_steps_moved > 0 else 1)
    disable_motor()

if __name__ == "__main__":
    try:
        serve_radar()
    except KeyboardInterrupt:
        print("Shutting down.")
        return_to_origin()

