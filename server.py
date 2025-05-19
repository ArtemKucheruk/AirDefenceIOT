import wiringpi
import socket
import json
from threading import Thread

# Motor GPIO Configuration (WiringPi numbering)
MOTORS = {
    'X': {'PUL': 2, 'DIR': 3, 'ENA': 11},
    'Y': {'PUL': 4, 'DIR': 5, 'ENA': 6}
}

# Motor control parameters
STEP_DELAY = 0.001  # Adjust for motor speed
STEPS_PER_PIXEL = 2  # Steps per pixel offset (adjust based on your mechanics)

def setup_gpio():
    wiringpi.wiringPiSetup()
    for axis in MOTORS.values():
        wiringpi.pinMode(axis['PUL'], wiringpi.OUTPUT)
        wiringpi.pinMode(axis['DIR'], wiringpi.OUTPUT)
        wiringpi.pinMode(axis['ENA'], wiringpi.OUTPUT)
        wiringpi.digitalWrite(axis['ENA'], wiringpi.HIGH)  # Enable motors

def move_motor(axis, direction, steps):
    wiringpi.digitalWrite(MOTORS[axis]['DIR'], direction)
    for _ in range(steps):
        wiringpi.digitalWrite(MOTORS[axis]['PUL'], wiringpi.HIGH)
        wiringpi.delayMicroseconds(int(STEP_DELAY * 1000000))
        wiringpi.digitalWrite(MOTORS[axis]['PUL'], wiringpi.LOW)
        wiringpi.delayMicroseconds(int(STEP_DELAY * 1000000))

def handle_client(conn):
    while True:
        try:
            data = conn.recv(1024).decode()
            if not data:
                break
            dx, dy = json.loads(data)
            
            # Calculate steps needed
            x_steps = int(abs(dx) * STEPS_PER_PIXEL)
            y_steps = int(abs(dy) * STEPS_PER_PIXEL)
            
            # Move motors in threads
            if dx != 0:
                Thread(target=move_motor, args=('X', dx > 0, x_steps)).start()
            if dy != 0:
                Thread(target=move_motor, args=('Y', dy < 0, y_steps)).start()
                
        except Exception as e:
            print(f"Error: {e}")
            break
    conn.close()

def main():
    setup_gpio()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('0.0.0.0', 65432))
    server.listen(1)
    print("Motor server listening on port 65432...")
    
    try:
        while True:
            conn, _ = server.accept()
            Thread(target=handle_client, args=(conn,)).start()
    finally:
        server.close()
        wiringpi.digitalWrite(MOTORS['X']['ENA'], wiringpi.LOW)
        wiringpi.digitalWrite(MOTORS['Y']['ENA'], wiringpi.LOW)

if __name__ == "__main__":
    main()