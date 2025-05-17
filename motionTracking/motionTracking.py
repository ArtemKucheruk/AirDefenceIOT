import os
import cv2
import socket
from motpy import Detection, MultiObjectTracker, NpImage
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track
from urllib.request import urlretrieve
from typing import Sequence
from server import update_frame

# Face detector configuration
WEIGHTS_URL = 'https://github.com/opencv/opencv_3rdparty/raw/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel'
WEIGHTS_PATH = '/tmp/opencv_face_detector.caffemodel'
CONFIG_URL = 'https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt'
CONFIG_PATH = '/tmp/deploy.prototxt'

class FaceDetector(BaseObjectDetector):
    def __init__(self, conf_threshold=0.5):
        super().__init__()
        if not os.path.exists(WEIGHTS_PATH) or not os.path.exists(CONFIG_PATH):
            self._download_models()
        self.net = cv2.dnn.readNetFromCaffe(CONFIG_PATH, WEIGHTS_PATH)
        self.conf_threshold = conf_threshold

    def _download_models(self):
        print("Downloading face detection models...")
        urlretrieve(WEIGHTS_URL, WEIGHTS_PATH)
        urlretrieve(CONFIG_URL, CONFIG_PATH)

    def process_image(self, image: NpImage) -> Sequence[Detection]:
        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), (104, 117, 123), False, False)
        self.net.setInput(blob)
        detections = self.net.forward()
        return self._post_process(detections, image.shape)

    def _post_process(self, detections, image_shape):
        out_detections = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                xmin = int(detections[0, 0, i, 3] * image_shape[1])
                ymin = int(detections[0, 0, i, 4] * image_shape[0])
                xmax = int(detections[0, 0, i, 5] * image_shape[1])
                ymax = int(detections[0, 0, i, 6] * image_shape[0])
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence))
        return out_detections

def calculate_offsets(frame, track):
    frame = cv2.flip(frame, 2)# to flip camera
    frame_center = (frame.shape[1] // 2, frame.shape[0] // 2)
    track_center = (
        (track.box[0] + track.box[2]) // 2,
        (track.box[1] + track.box[3]) // 2
    )
    return track_center, (
        track_center[0] - frame_center[0],
        track_center[1] - frame_center[1]
    )

def run():
    # Initialize components
    tracker = MultiObjectTracker(dt=1/15, model_spec={
        'order_pos': 1, 'dim_pos': 2,
        'order_size': 0, 'dim_size': 2,
        'q_var_pos': 5000., 'r_var_pos': 0.1
    })
    
    cap = cv2.VideoCapture(0)
    detector = FaceDetector()
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        client_socket.connect(("127.0.0.1", 65432))
        print("Connected to control server")

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Update web stream
            update_frame(frame)

            # Process frame
            frame = cv2.resize(frame, None, fx=0.5, fy=0.5)
            detections = detector.process_image(frame)
            tracker.step(detections)
            tracks = tracker.active_tracks(min_steps_alive=3)

            # Draw visualizations
            for det in detections:
                draw_detection(frame, det)
            for track in tracks:
                draw_track(frame, track)
                _, (dx, dy) = calculate_offsets(frame, track)
                message = f"{int(dx)},{int(dy)}\n"
                client_socket.sendall(message.encode())
                client_socket.recv(1024)  # Wait for ACK

            # Show preview
            cv2.imshow('Tracking', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        client_socket.close()
        print("Tracking stopped")

if __name__ == "__main__":
    run()