import socket
import threading
import time
import wiringpi
from stepper_motor.funcs import MotorController, pixel_offset_to_steps, set_up_motors, ENA1, ENA2

HOST = "127.0.0.1"
PORT = 65432

def parse_coordinates(message):
    try:
        parts = message.strip().split(',')
        x = float(parts[0].split('=')[1])
        y = float(parts[1].split('=')[1])
        return x, y
    except Exception as e:
        print(f"Failed to parse coordinates: {message} | Error: {e}")
        return None, None

class OffsetSmoother:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.smoothed_x = 0
        self.smoothed_y = 0
        self.initialized = False

    def smooth(self, x, y):
        if not self.initialized:
            self.smoothed_x = x
            self.smoothed_y = y
            self.initialized = True
        else:
            self.smoothed_x = self.alpha * x + (1 - self.alpha) * self.smoothed_x
            self.smoothed_y = self.alpha * y + (1 - self.alpha) * self.smoothed_y
        return self.smoothed_x, self.smoothed_y

def run_server():
    frame_width = 320
    frame_height = 240

    set_up_motors()
    motor_controller = MotorController()
    motor_controller.deadzone_steps = 3
    motor_controller.start()

    smoother = OffsetSmoother(alpha=0.3)

    # Shared latest target coords for thread-safe updating
    latest_target_lock = threading.Lock()
    latest_target_steps = (0, 0)

    def update_latest_target(steps_x, steps_y):
        nonlocal latest_target_steps
        with latest_target_lock:
            latest_target_steps = (steps_x, steps_y)

    def motor_target_updater():
        # Thread to regularly update motor target from latest buffered steps
        while motor_controller.running:
            with latest_target_lock:
                tx, ty = latest_target_steps
            motor_controller.update_target(tx, ty)
            time.sleep(0.05)  # update motor target at 20Hz

    # Start motor target updater thread
    updater_thread = threading.Thread(target=motor_target_updater, daemon=True)
    updater_thread.start()

    # Start socket server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")

            while True:
                data = conn.recv(1024)
                if not data:
                    print("Client disconnected")
                    break
                message = data.decode('utf-8').strip()
                x, y = parse_coordinates(message)
                if x is None or y is None:
                    continue

                x_smooth, y_smooth = smoother.smooth(x, y)

                steps_x = pixel_offset_to_steps(x_smooth, frame_width)
                steps_y = pixel_offset_to_steps(y_smooth, frame_height)

                # Update latest target thread-safely, without blocking
                update_latest_target(steps_x, steps_y)

    motor_controller.stop()
    wiringpi.digitalWrite(ENA1, 1)
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors disabled, server shutting down.")

if __name__ == "__main__":
    try:
        run_server()
    finally:
        wiringpi.digitalWrite(ENA1, 1)
        wiringpi.digitalWrite(ENA2, 1)
        print("Motors disabled, server shutting down.")
