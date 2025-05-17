import socket
from stepper_motor.funcs import setup_motors, run_motor_1, run_motor_2, disableBothMotors

HOST = "127.0.0.1"
PORT = 65432


def start_server():
    setup_motors()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")

        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            buffer = ""

            while True:
                data = conn.recv(1024).decode('utf-8')
                if not data:
                    break

                buffer += data

                while "\n" in buffer:
                    message, buffer = buffer.split("\n", 1)
                    message = message.strip()

                    if not message:
                        continue

                    print(f"Received: {message}")

                    try:
                        x_offset, y_offset = map(int, message.split(','))
                        print(f"Moving motors: x={x_offset}, y={y_offset}")

                        print(f"Moving motors: x={x_offset}, y={y_offset}")
                        run_motor_1(x_offset)
                        run_motor_2(y_offset)
                        print("Motors moved.")

                        conn.sendall(b"ACK\n")

                    except ValueError as e:
                        print(f"Error parsing: {e}")
                        conn.sendall(b"ERROR\n")


if __name__ == "__main__":
    try:
        start_server()
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        disableBothMotors()
        print("Motors disabled")