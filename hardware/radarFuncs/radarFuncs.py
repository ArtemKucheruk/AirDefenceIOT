import socket
import wiringpi as wp
import time
import numpy as np
import cv2
import configs.configRadar as radar_config

# ==== HARDWARE SETUP ====
def setup_hardware(TRIG, ECHO, STEPPER_PINS):
    """
    Initialize GPIO pins for the ultrasonic sensor and stepper motor.
    """
    wp.wiringPiSetup()
    wp.pinMode(TRIG, 1)  # Set TRIG as output
    wp.pinMode(ECHO, 0)  # Set ECHO as input
    for pin in STEPPER_PINS:
        wp.pinMode(pin, 1)  # Set stepper motor pins as output

def disable_motor(STEPPER_PINS):
    """
    Disable the stepper motor by setting all control pins low.
    """
    for pin in STEPPER_PINS:
        wp.digitalWrite(pin, 0)

def move_stepper(STEPPER_PINS, SEQ, DELAY, steps, direction=1):
    """
    Move the stepper motor a given number of steps in the specified direction.
    SEQ is the step sequence, DELAY is the time between steps.
    """
    try:
        for _ in range(steps):
            for halfstep in range(8):
                for pin in range(4):
                    wp.digitalWrite(STEPPER_PINS[pin], SEQ[::direction][halfstep][pin])
                time.sleep(DELAY)
    except Exception as e:
        print(f"Stepper error: {e}")
        disable_motor(STEPPER_PINS)

def get_distance(TRIG, ECHO):
    """
    Measure distance using the ultrasonic sensor.
    Returns the distance in centimeters, or -1 on timeout.
    """
    wp.digitalWrite(TRIG, 0)
    time.sleep(0.000002)
    wp.digitalWrite(TRIG, 1)
    time.sleep(0.00001)
    wp.digitalWrite(TRIG, 0)

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 0:
        if time.time() - timeout_start > 0.02:
            return -1  # Timeout waiting for echo

    start_time = time.time()

    timeout_start = time.time()
    while wp.digitalRead(ECHO) == 1:
        if time.time() - timeout_start > 0.02:
            return -1  # Timeout waiting for echo end

    end_time = time.time()

    duration = end_time - start_time
    distance = duration * 34300 / 2  # Speed of sound: 34300 cm/s
    return round(distance, 1)

def return_to_origin(STEPPER_PINS, SEQ, DELAY):
    """
    Move the stepper motor back to its origin (zero) position.
    """
    global total_steps_moved
    print(f"Returning to origin: {abs(total_steps_moved)} steps back")
    if total_steps_moved != 0:
        move_stepper(STEPPER_PINS, SEQ, DELAY, abs(total_steps_moved), direction=-1 if total_steps_moved > 0 else 1)
    disable_motor(STEPPER_PINS)
    total_steps_moved = 0

# ==== RADAR DISPLAY FUNCTIONS ====
def polar_to_cartesian(angle_deg, length, CENTER):
    """
    Convert polar coordinates (angle in degrees, length) to Cartesian (x, y) coordinates.
    Used for drawing radar sweeps and detections.
    """
    rotated_angle = angle_deg - 90
    rad = np.radians(rotated_angle)
    x = int(CENTER[0] + length * np.sin(rad))
    y = int(CENTER[1] - length * np.cos(rad))
    return (x, y)

def initialize_radar_display(CENTER, MAX_DISTANCE, HEIGHT, WIDTH, GREEN, RADAR_RANGE_CM):
    """
    Create the base radar display image with circles and angle/distance labels.
    """
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    # Draw range circles
    for radius in range(50, MAX_DISTANCE + 1, MAX_DISTANCE // 4):
        cv2.circle(img, CENTER, radius, GREEN, 1)
    # Draw angle lines and labels
    for angle in range(0, 181, 30):
        pt = polar_to_cartesian(angle, MAX_DISTANCE, CENTER)
        cv2.line(img, CENTER, pt, GREEN, 1)
        label_pt = polar_to_cartesian(angle, MAX_DISTANCE + 20, CENTER)
        offset_x = -15 if angle < 90 else 5
        cv2.putText(img, f"{angle}Â°", (label_pt[0] + offset_x, label_pt[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1)
    # Draw distance labels
    for i, dist_cm in enumerate(range(10, RADAR_RANGE_CM + 1, RADAR_RANGE_CM // 4)):
        radius = int(MAX_DISTANCE * dist_cm / RADAR_RANGE_CM)
        label_pos = (CENTER[0] - radius - 40, CENTER[1] + 5)
        cv2.putText(img, f"{dist_cm}cm", label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)
    return img

def update_display(base_img, trace_img, distance, angle, CENTER, MAX_DISTANCE, RADAR_RANGE_CM, GREEN, RED):
    """
    Draw the radar sweep and detected object on the radar display images.
    """
    angle = max(0, min(180, angle))
    sweep_end = polar_to_cartesian(angle, MAX_DISTANCE, CENTER)
    cv2.line(base_img, CENTER, sweep_end, GREEN, 2)
    if 0 < distance < RADAR_RANGE_CM:
        dist_px = int(np.interp(distance, [0, RADAR_RANGE_CM], [0, MAX_DISTANCE]))
        detected_pt = polar_to_cartesian(angle, dist_px, CENTER)
        cv2.line(trace_img, sweep_end, detected_pt, RED, 3)
        cv2.circle(trace_img, detected_pt, 6, RED, -1)

# ==== RADAR CONTROL FUNCTIONS ====
def serve_radar(TRIG, ECHO, RADAR_HOST, RADAR_PORT, STEPS_PER_DEGREE, TOTAL_ANGLE, STEP_DELAY, STEPPER_PINS, SEQ, DELAY):
    """
    Start the radar server, move the stepper, and send distance/angle data to the client.
    """
    print(f"Starting radar server on {RADAR_HOST}:{RADAR_PORT}...")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((RADAR_HOST, RADAR_PORT))
        s.listen(1)
        conn, addr = s.accept()

        print(f"Client connected: {addr}")
        with conn:
            angle = 0
            direction = 1

            while True:
                steps = STEPS_PER_DEGREE
                move_stepper(STEPPER_PINS, SEQ, DELAY, steps, direction)
                radar_config.total_steps_moved += steps * direction

                distance = get_distance(TRIG, ECHO)
                msg = f"{distance},{angle}\n"
                conn.sendall(msg.encode())

                angle += direction
                if angle >= TOTAL_ANGLE or angle <= 0:
                    direction *= -1

                time.sleep(STEP_DELAY)

def radar_data_loop(CENTER, MAX_DISTANCE, HEIGHT, WIDTH, GREEN, RADAR_RANGE_CM, RADAR_HOST, RADAR_PORT, FADE_FACTOR, RED):
    """
    Connect to the radar server, receive distance/angle data, and update the radar display frame.
    """
    base_img = initialize_radar_display(CENTER, MAX_DISTANCE, HEIGHT, WIDTH, GREEN, RADAR_RANGE_CM)
    trace_img = np.zeros_like(base_img)

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                print(f"Connecting to radar at {RADAR_HOST}:{RADAR_PORT}...")
                s.connect((RADAR_HOST, RADAR_PORT))
                print("Connected to radar.")

                buffer = ""
                while True:
                    data = s.recv(1024).decode()
                    if not data:
                        break
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        try:
                            distance, angle = map(float, line.strip().split(','))
                            # Fade previous traces for a trailing effect
                            trace_img[:] = (trace_img * FADE_FACTOR).astype(np.uint8)
                            base = base_img.copy()
                            # Draw current sweep and detection
                            update_display(base, trace_img, distance, angle, CENTER, MAX_DISTANCE, RADAR_RANGE_CM, GREEN, RED)
                            # Update the global radar frame for streaming
                            radar_config.frame = cv2.addWeighted(base, 1, trace_img, 1, 0)
                        except ValueError:
                            continue
        except Exception as e:
            print(f"Radar connection error: {e}")
            time.sleep(2)
