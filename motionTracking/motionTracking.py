import cv2
import socket
import json

# Camera setup
CAMERA_DEVICE = 1  # /dev/video1
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Network settings
SERVER_IP = '127.0.0.1'
SERVER_PORT = 65432

def detect_face():
    cap = cv2.VideoCapture(CAMERA_DEVICE)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    
    # Load face detection classifier
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    
    # Connect to motor server
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((SERVER_IP, SERVER_PORT))
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, 1.1, 4)
            
            if len(faces) > 0:
                (x, y, w, h) = faces[0]
                face_center = (x + w//2, y + h//2)
                frame_center = (FRAME_WIDTH//2, FRAME_HEIGHT//2)
                
                dx = frame_center[0] - face_center[0]
                dy = frame_center[1] - face_center[1]
                
                # Send offset data to server
                sock.sendall(json.dumps((dx, dy)).encode())
                
                # Visualization
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.circle(frame, face_center, 5, (0, 255, 0), -1)
                cv2.circle(frame, frame_center, 5, (0, 0, 255), -1)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    finally:
        cap.release()
        cv2.destroyAllWindows()
        sock.close()

if __name__ == "__main__":
    detect_face()