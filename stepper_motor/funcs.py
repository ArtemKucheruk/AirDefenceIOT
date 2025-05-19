import wiringpi
import time
import threading
from stepper_motor.config import *


def set_up_motors():
    wiringpi.wiringPiSetup()
    for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
        wiringpi.pinMode(pin, 1)
    wiringpi.digitalWrite(ENA1, 0)  # Enable motor 1 (LOW = enabled)
    wiringpi.digitalWrite(ENA2, 0)

def run_motor1(steps):
    wiringpi.digitalWrite(DIR1, 1 if steps >= 0 else 0)
    steps = abs(steps)
    print(f"run_motor1: Moving {steps} steps {'forward' if steps >= 0 else 'backward'}")
    for i in range(steps):
        print(f"Motor1 pulse HIGH step {i+1}")
        wiringpi.digitalWrite(PUL1, 1)
        time.sleep(MOTOR_DELAY)
        wiringpi.digitalWrite(PUL1, 0)
        print(f"Motor1 pulse LOW step {i+1}")
        time.sleep(MOTOR_DELAY)

def run_motor2(steps):
    wiringpi.digitalWrite(DIR2, 1 if steps >= 0 else 0)
    steps = abs(steps)
    print(f"run_motor2: Moving {steps} steps {'forward' if steps >= 0 else 'backward'}")
    for i in range(steps):
        print(f"Motor2 pulse HIGH step {i+1}")
        wiringpi.digitalWrite(PUL2, 1)
        time.sleep(MOTOR_DELAY)
        wiringpi.digitalWrite(PUL2, 0)
        print(f"Motor2 pulse LOW step {i+1}")
        time.sleep(MOTOR_DELAY)


def pixel_offset_to_steps(offset, frame_dim, fov_deg=60, step_angle_deg=1.8, deadzone=5):
    if abs(offset) < deadzone:
        return 0
    angle_per_pixel = fov_deg / frame_dim
    angle = offset * angle_per_pixel
    steps = int(round(angle / step_angle_deg))
    return steps



class MotorController:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_pos_x = 0
        self.current_pos_y = 0
        self.target_pos_x = 0
        self.target_pos_y = 0
        self.running = True
        self.deadzone_steps = 1

    def update_target(self, steps_x, steps_y):
        with self.lock:
            self.target_pos_x = steps_x
            self.target_pos_y = steps_y
        print(f"Updated target position: X={steps_x}, Y={steps_y}")

    def motor_thread(self):
        while self.running:
            with self.lock:
                dx = self.target_pos_x - self.current_pos_x
                dy = self.target_pos_y - self.current_pos_y

            print(f"Motor thread: Current X={self.current_pos_x}, Target X={self.target_pos_x}, dx={dx}")
            print(f"Motor thread: Current Y={self.current_pos_y}, Target Y={self.target_pos_y}, dy={dy}")

            if abs(dx) > 0:
                step_dir_x = 1 if dx > 0 else -1
                print(f"Stepping motor 2 one step {'forward' if step_dir_x == 1 else 'backward'}")
                run_motor2(step_dir_x)
                self.current_pos_x += step_dir_x
            if abs(dy) > 0:
                step_dir_y = 1 if dy > 0 else -1
                print(f"Stepping motor 1 one step {'forward' if step_dir_y == 1 else 'backward'}")
                run_motor1(step_dir_y)
                self.current_pos_y += step_dir_y

            time.sleep(MOTOR_DELAY)


    def start(self):
        self.thread = threading.Thread(target=self.motor_thread, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()