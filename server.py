import socket
from flask import Flask, render_template, Response
import threading
import cv2
import numpy as np
from stepper_motor.funcs import setup_motors, run_motor_1, run_motor_2, disableBothMotors

# Initialize Flask app
app = Flask(__name__)

# Server configuration
HOST = "127.0.0.1"
PORT = 65432

# Global variables for frame sharing
current_frame = None
frame_lock = threading.Lock()

def generate_frames():
    """Video streaming generator function."""
    global current_frame, frame_lock
    while True:
        with frame_lock:
            if current_frame is None:
                continue
            ret, buffer = cv2.imencode('.jpg', current_frame)
            frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

def start_flask():
    """Start Flask web server in a separate thread."""
    app.run(host='0.0.0.0', port=5000, threaded=True)

def socket_server():
    """Main socket server for motor control."""
    setup_motors()
    
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Socket server listening on {HOST}:{PORT}")

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
                        
                        run_motor_1(x_offset)
                        run_motor_2(y_offset)
                        
                        conn.sendall(b"ACK\n")

                    except ValueError as e:
                        print(f"Error parsing: {e}")
                        conn.sendall(b"ERROR\n")

def update_frame(frame):
    """Update the current frame for Flask streaming."""
    global current_frame, frame_lock
    with frame_lock:
        current_frame = frame.copy()

if __name__ == "__main__":
    # Start Flask in a separate thread
    flask_thread = threading.Thread(target=start_flask)
    flask_thread.daemon = True
    flask_thread.start()

    # Start socket server in main thread
    try:
        socket_server()
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        disableBothMotors()
        print("Motors disabled")