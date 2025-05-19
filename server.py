import socket
import wiringpi
import time
from stepper_motor.funcs import *

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

def run_server():
    HOST = "127.0.0.1"
    PORT = 65432

    # Assuming client resizes frame to these dimensions
    frame_width = 320
    frame_height = 240

    motor_controller = MotorController()
    motor_controller.start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            set_up_motors()
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                message = data.decode('utf-8').strip()
                x, y = parse_coordinates(message)
                if x is None or y is None:
                    continue

                # Convert pixel offsets to motor steps
                steps_x = pixel_offset_to_steps(x, frame_width)
                steps_y = pixel_offset_to_steps(y, frame_height)

                # Update motor targets
                motor_controller.update_target(steps_x, steps_y)

    motor_controller.stop()
    wiringpi.digitalWrite(ENA1, 1)  # Disable motors on exit
    wiringpi.digitalWrite(ENA2, 1)
    print("Motors disabled, server shutting down.")

if __name__ == "__main__":
    run_server()
    