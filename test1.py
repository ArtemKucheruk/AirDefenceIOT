import cv2
import imutils
import time
import threading
import wiringpi
import sys
import signal

# Motor pins (WiringPi numbers)
PUL1 = 2
DIR1 = 3
ENA1 = 11

PUL2 = 4
DIR2 = 5
ENA2 = 6

# Motor control params
MOTOR_DELAY = 0.002  # seconds per half-step pulse
MAX_STEPS_X = 30
MAX_STEPS_Y = 15
DEADZONE_STEPS = 1

# OpenCV camera device
CAMERA_DEVICE = "/dev/video0"

# Setup wiringpi pins and enable motors
def setup_motors():
    wiringpi.wiringPiSetup()
    for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
        wiringpi.pinMode(pin, 1)
    wiringpi.digitalWrite(ENA1, 0)  # Enable motor 1 (LOW = enabled)
    wiringpi.digitalWrite(ENA2, 0)  # Enable motor 2

def disable_motors():
    wiringpi.digitalWrite(ENA1, 1)
    wiringpi.digitalWrite(ENA2, 1)

def step_motor(pul_pin, dir_pin, steps):
    wiringpi.digitalWrite(dir_pin, 1 if steps >= 0 else 0)
    steps = abs(steps)
    for _ in range(steps):
        wiringpi.digitalWrite(pul_pin, 1)
        time.sleep(MOTOR_DELAY)
        wiringpi.digitalWrite(pul_pin, 0)
        time.sleep(MOTOR_DELAY)

class TurretController:
    def __init__(self):
        self.current_x = 0
        self.current_y = 0
        self.target_x = 0
        self.target_y = 0
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self.motor_loop)
        self.thread.start()

    def update_target(self, x, y):
        with self.lock:
            self.target_x = max(min(x, MAX_STEPS_X), -MAX_STEPS_X)
            self.target_y = max(min(y, MAX_STEPS_Y), -MAX_STEPS_Y)
        print(f"Updated target: X={self.target_x}, Y={self.target_y}")

    def motor_loop(self):
        while self.running:
            with self.lock:
                dx = self.target_x - self.current_x
                dy = self.target_y - self.current_y

            if abs(dx) > DEADZONE_STEPS:
                step_x = 1 if dx > 0 else -1
                step_motor(PUL2, DIR2, step_x)  # Motor 2 controls X axis
                self.current_x += step_x

            if abs(dy) > DEADZONE_STEPS:
                step_y = 1 if dy > 0 else -1
                step_motor(PUL1, DIR1, step_y)  # Motor 1 controls Y axis
                self.current_y += step_y

            time.sleep(0.01)  # slight delay to avoid hogging CPU

    def stop(self):
        self.running = False
        self.thread.join()

def get_best_contour(thresh, min_area=5000):
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_cnt = None
    best_area = min_area
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > best_area:
            best_area = area
            best_cnt = cnt
    return best_cnt

def main():
    setup_motors()
    turret = TurretController()

    cap = cv2.VideoCapture(CAMERA_DEVICE)
    if not cap.isOpened():
        print(f"Error: Could not open camera {CAMERA_DEVICE}")
        return

    first_frame = None
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if first_frame is None:
                first_frame = gray
                print("Calibrating background... please wait")
                continue

            frame_delta = cv2.absdiff(first_frame, gray)
            thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)

            cnt = get_best_contour(thresh)
            if cnt is not None:
                (x, y, w, h) = cv2.boundingRect(cnt)
                cx = x + w // 2
                cy = y + h // 2

                # Draw rectangle and center on frame for visualization
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                # Calculate offsets from center
                frame_center_x = frame.shape[1] // 2
                frame_center_y = frame.shape[0] // 2

                offset_x = cx - frame_center_x
                offset_y = cy - frame_center_y

                # Map pixel offset to motor steps (simple linear scaling)
                # Adjust divisor to tune sensitivity
                scale_x = MAX_STEPS_X / float(frame.shape[1] // 2)
                scale_y = MAX_STEPS_Y / float(frame.shape[0] // 2)

                target_step_x = int(offset_x * scale_x)
                target_step_y = int(offset_y * scale_y)

                # Update turret motors
                turret.update_target(target_step_x, target_step_y)

            cv2.imshow("Turret View", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        turret.stop()
        disable_motors()
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    def handler(signum, frame):
        print("Exiting gracefully")
        sys.exit(0)
    signal.signal(signal.SIGINT, handler)
    main()

