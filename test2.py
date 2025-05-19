import cv2
import numpy as np
import time
import threading
import sys
import termios
import contextlib
import imutils
import RPi.GPIO as GPIO

### Hardware Configuration ###
# Motor 1 (X-axis) - BCM GPIO numbers
PUL1 = 12
DIR1 = 13
ENA1 = 19

# Motor 2 (Y-axis) - BCM GPIO numbers
PUL2 = 16
DIR2 = 1
ENA2 = 4

# Motor control settings
MOTOR_DELAY = 0.002  # Delay between steps
MOTOR_DURATION = 3    # Duration for each step sequence

# Camera configuration
CAMERA_DEVICE = "/dev/video0"

# Other parameters
RELAY_PIN = 22
MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False
MAX_STEPS_X = 30
MAX_STEPS_Y = 15

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor control pins setup
for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.setup(RELAY_PIN, GPIO.OUT)
GPIO.output(RELAY_PIN, GPIO.LOW)

### Helper Classes and Functions ###
@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)

class Motor:
    def __init__(self, pul, dir, ena):
        self.pul = pul
        self.dir = dir
        self.ena = ena
        self.current_steps = 0
        GPIO.output(self.ena, GPIO.HIGH)  # Enable motor

    def step(self, steps, direction):
        GPIO.output(self.dir, direction)
        for _ in range(steps):
            GPIO.output(self.pul, GPIO.HIGH)
            time.sleep(MOTOR_DELAY)
            GPIO.output(self.pul, GPIO.LOW)
            time.sleep(MOTOR_DELAY)
        self.current_steps += steps if direction else -steps

class VideoUtils:
    @staticmethod
    def live_video():
        video_capture = cv2.VideoCapture(CAMERA_DEVICE)
        while True:
            ret, frame = video_capture.read()
            if not ret:
                break
            cv2.imshow('Video', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, show_video=False):
        camera = cv2.VideoCapture(CAMERA_DEVICE)
        firstFrame = None
        while True:
            grabbed, frame = camera.read()
            if not grabbed:
                break
            
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            if firstFrame is None:
                firstFrame = gray
                continue

            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]
            thresh = cv2.dilate(thresh, None, iterations=2)
            contour = VideoUtils.get_best_contour(thresh, 5000)

            if contour is not None:
                (x, y, w, h) = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(contour, frame)

            if show_video:
                cv2.imshow("Motion Feed", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        contours, _ = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt = None
        max_area = threshold
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area:
                max_area = area
                best_cnt = cnt
        return best_cnt

class Turret:
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode
        self.motor_x = Motor(PUL1, DIR1, ENA1)
        self.motor_y = Motor(PUL2, DIR2, ENA2)
        self.lock = threading.Lock()

    def calibrate(self):
        print("Calibrate X-axis with A/D keys")
        self.__calibrate_axis(self.motor_x, 'a', 'd')
        print("Calibrate Y-axis with W/S keys")
        self.__calibrate_axis(self.motor_y, 'w', 's')
        print("Calibration complete")

    def __calibrate_axis(self, motor, left_key, right_key):
        with raw_mode(sys.stdin):
            while True:
                ch = sys.stdin.read(1)
                if ch == '\n':
                    break
                steps = 5
                if ch == left_key:
                    motor.step(steps, MOTOR_X_REVERSED if motor == self.motor_x else MOTOR_Y_REVERSED)
                elif ch == right_key:
                    motor.step(steps, not (MOTOR_X_REVERSED if motor == self.motor_x else MOTOR_Y_REVERSED))

    def motion_detection(self, show_video=False):
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)
        
        target_x = (2 * MAX_STEPS_X * (x + w/2) / v_w) - MAX_STEPS_X
        target_y = (2 * MAX_STEPS_Y * (y + h/2) / v_h) - MAX_STEPS_Y

        with self.lock:
            dx = target_x - self.motor_x.current_steps
            dy = target_y - self.motor_y.current_steps

            if dx != 0:
                dir_x = MOTOR_X_REVERSED if dx < 0 else not MOTOR_X_REVERSED
                self.motor_x.step(abs(dx), dir_x)

            if dy != 0:
                dir_y = MOTOR_Y_REVERSED if dy < 0 else not MOTOR_Y_REVERSED
                self.motor_y.step(abs(dy), dir_y)

            if not self.friendly_mode and abs(dx) <= 2 and abs(dy) <= 2:
                Turret.fire()

    @staticmethod
    def fire():
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(RELAY_PIN, GPIO.LOW)

    def cleanup(self):
        GPIO.output(ENA1, GPIO.LOW)
        GPIO.output(ENA2, GPIO.LOW)
        GPIO.cleanup()

if __name__ == "__main__":
    turret = Turret(friendly_mode=False)
    try:
        mode = input("Choose mode (1: Motion Detection, 2: Interactive): ")
        if mode == "1":
            turret.calibrate()
            turret.motion_detection(show_video=True)
        elif mode == "2":
            turret.interactive()
    finally:
        turret.cleanup()
