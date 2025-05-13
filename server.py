import socket
from stepper_motor.funcs import setup_motors, run_motor_1, run_motor_2  # Import motor control functions

# Server settings
HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

def start_server():
    # Set up motors before starting server
    setup_motors()

    # Set up server socket to receive data from the client
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
                    break

                # Decode the received data and parse the x and y offsets
                data = data.decode('utf-8')
                print(f"Received data: {data}")

                # Extract the x and y offset from the message (e.g., "x=5,y=-3")
                try:
                    x_offset, y_offset = map(int, data.split(','))
                    print(f"Parsed offsets - x_offset: {x_offset}, y_offset: {y_offset}")

                    # Control the motors based on the received offsets
                    run_motor_1(x_offset)  # Control motor 1 with the x offset
                    run_motor_2(y_offset)  # Control motor 2 with the y offset

                except ValueError:
                    print("Invalid data format received.")

                # Optionally, send a confirmation message back to the client
                response = f"Moved motors with x_offset={x_offset}, y_offset={y_offset}"
                conn.sendall(response.encode('utf-8'))

# Run the server
if __name__ == "__main__":
    start_server()
