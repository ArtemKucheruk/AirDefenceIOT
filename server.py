import socket
import wiringpi
import time
from stepper_motor.funcs import MotorController, pixel_offset_to_steps, set_up_motors, ENA1, ENA2

HOST = "127.0.0.1"  # Localhost
PORT = 65432

def parse_coordinates(message):
    # Example message: "x=156.0,y=28.0"
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
    frame_width = 320   # Must match client resized frame width
    frame_height = 240  # Must match client resized frame height

    motor_controller = MotorController()
    motor_controller.deadzone_steps = 3  # increase deadzone to reduce jitter
    motor_controller.start()

    set_up_motors()  # Initialize GPIO pins and enable motors

    smoother = OffsetSmoother(alpha=0.3)

    min_update_interval = 0.15  # seconds, update motor target every 150ms
    last_update_time = 0

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
                    print("Client disconnected.")
                    break

                message = data.decode('utf-8').strip()
                x, y = parse_coordinates(message)
                if x is None or y is None:
                    continue

                # Smooth the incoming offset data
                x_smooth, y_smooth = smoother.smooth(x, y)

                # Convert smoothed offsets to motor steps
                steps_x = pixel_offset_to_steps(x_smooth, frame_width)
                steps_y = pixel_offset_to_steps(y_smooth, frame_height)

                now = time.time()
                if now - last_update_time >= min_update_interval:
                    motor_controller.update_target(steps_x, steps_y)
                    last_update_time = now

    motor_controller.stop()
    wiringpi.digitalWrite(ENA1, 1)  # Disable motors on exit
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors disabled, server shutting down.")

if __name__ == "__main__":
    run_server()
