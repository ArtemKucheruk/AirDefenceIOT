import cv2
import wiringpi
import time

# GPIO PIN CONFIGURATION (WiringPi numbering)
MOTOR_PINS = {
    'x': {'PUL': 2, 'DIR': 3, 'ENA': 11},
    'y': {'PUL': 4, 'DIR': 5, 'ENA': 6}
}

# MOTOR MOVEMENT PARAMETERS
STEP_DELAY = 0.001   # Seconds between steps (controls speed)
STEPS_PER_MOVE = 5   # Number of steps to move per correction (tune as needed)
FACE_MARGIN = 40     # Pixels margin around center before movement occurs

def setup_motor(axis):
    pins = MOTOR_PINS[axis]
    for key in pins:
        wiringpi.pinMode(pins[key], 1)  # 1 = OUTPUT
        wiringpi.digitalWrite(pins[key], 0)  # Disable motor initially

def enable_motor(axis, enable=True):
    pins = MOTOR_PINS[axis]
    wiringpi.digitalWrite(pins['ENA'], 0 if enable else 1)

def move_motor(axis, direction, steps):
    pins = MOTOR_PINS[axis]
    # Set direction: 1 or 0
    wiringpi.digitalWrite(pins['DIR'], 1 if direction == 'pos' else 0)
    enable_motor(axis, True)
    for _ in range(steps):
        wiringpi.digitalWrite(pins['PUL'], 1)
        time.sleep(STEP_DELAY)
        wiringpi.digitalWrite(pins['PUL'], 0)
        time.sleep(STEP_DELAY)
    enable_motor(axis, False)

def get_face_center(gray, face_cascade):
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    if len(faces) == 0:
        return None, None, None
    # Pick the largest face
    x, y, w, h = max(faces, key=lambda rect: rect[2] * rect[3])
    cx = x + w // 2
    cy = y + h // 2
    return (cx, cy), (x, y, w, h), faces

def process_camera_and_move_motors():
    # Setup motors
    setup_motor('x')
    setup_motor('y')

    # Load OpenCV Haar cascade for face detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    if face_cascade.empty():
        print("Failed to load Haar cascade classifier.")
        return

    # Open camera (Logitech C270)
    cap = cv2.VideoCapture('/dev/video1', cv2.CAP_V4L2)
    if not cap.isOpened():
        print("Cannot open camera /dev/video1")
        return

    print("Starting face tracking. Press 'q' to quit.")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break

            # Get frame shape for center calculation
            h_frame, w_frame = frame.shape[:2]
            center_x, center_y = w_frame // 2, h_frame // 2

            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            face_center, face_rect, faces = get_face_center(gray, face_cascade)

            # Draw rectangle and center point if face found
            if face_center:
                x, y, w, h = face_rect
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, face_center, 5, (0, 0, 255), -1)
                dx = face_center[0] - center_x
                dy = face_center[1] - center_y

                # Print coordinates for debugging/telemetry
                print(f"Face center: ({face_center[0]}, {face_center[1]}) | Frame center: ({center_x}, {center_y}) | dX: {dx}, dY: {dy}")

                # Move X-axis (left/right)
                if abs(dx) > FACE_MARGIN:
                    direction = 'pos' if dx > 0 else 'neg'
                    move_motor('x', direction, STEPS_PER_MOVE)
                    print(f"Moving X {'right' if dx > 0 else 'left'}")

                # Move Y-axis (up/down)
                if abs(dy) > FACE_MARGIN:
                    direction = 'pos' if dy > 0 else 'neg'
                    move_motor('y', direction, STEPS_PER_MOVE)
                    print(f"Moving Y {'down' if dy > 0 else 'up'}")

            else:
                print("No face detected.")

            # Draw frame center
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            # Show frame (for debugging, optional)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        # Disable motors
        enable_motor('x', False)
        enable_motor('y', False)

if __name__ == "__main__":
    try:
        wiringpi.wiringPiSetup()
        process_camera_and_move_motors()
    except Exception as e:
        print(f"Error: {e}")