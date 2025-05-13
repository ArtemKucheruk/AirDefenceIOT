import os
from typing import Sequence
from urllib.request import urlretrieve
import socket 
import cv2
from motpy import Detection, MultiObjectTracker, NpImage
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track
from motpy.utils import ensure_packages_installed

ensure_packages_installed(['cv2'])

logger = setup_logger(__name__, 'DEBUG', is_main=True)

# OpenCV Face Detector files (download if missing)
WEIGHTS_URL = 'https://github.com/opencv/opencv_3rdparty/raw/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel'
WEIGHTS_PATH = '/tmp/opencv_face_detector.caffemodel'
CONFIG_URL = 'https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt'
CONFIG_PATH = '/tmp/deploy.prototxt'

class FaceDetector(BaseObjectDetector):
    def __init__(self,
                 weights_url: str = WEIGHTS_URL,
                 weights_path: str = WEIGHTS_PATH,
                 config_url: str = CONFIG_URL,
                 config_path: str = CONFIG_PATH,
                 conf_threshold: float = 0.5) -> None:
        super(FaceDetector, self).__init__()

        if not os.path.isfile(weights_path) or not os.path.isfile(config_path):
            logger.debug('downloading model...')
            urlretrieve(weights_url, weights_path)
            urlretrieve(config_url, config_path)

        self.net = cv2.dnn.readNetFromCaffe(config_path, weights_path)
        self.conf_threshold = conf_threshold

    def process_image(self, image: NpImage) -> Sequence[Detection]:
        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], False, False)
        self.net.setInput(blob)
        detections = self.net.forward()

        out_detections = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                xmin = int(detections[0, 0, i, 3] * image.shape[1])
                ymin = int(detections[0, 0, i, 4] * image.shape[0])
                xmax = int(detections[0, 0, i, 5] * image.shape[1])
                ymax = int(detections[0, 0, i, 6] * image.shape[0])
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence))
        return out_detections

def calculate_offsets(frame, track):
    """Returns (x, y) center of track and (dx, dy) offsets from frame center."""
    frame_center_x = frame.shape[1] // 2
    frame_center_y = frame.shape[0] // 2

    # Track's center (x, y)
    track_center_x = (track.box[0] + track.box[2]) // 2
    track_center_y = (track.box[1] + track.box[3]) // 2

    # Offset from frame center
    dx = track_center_x - frame_center_x  # +ve if target is right of center
    dy = track_center_y - frame_center_y  # +ve if target is below center

    return (track_center_x, track_center_y), (dx, dy)

import socket

def run():
    # Tracker setup
    model_spec = {'order_pos': 1, 'dim_pos': 2,
                  'order_size': 0, 'dim_size': 2,
                  'q_var_pos': 5000., 'r_var_pos': 0.1}
    dt = 1 / 15.0  # assume 15 fps
    tracker = MultiObjectTracker(dt=dt, model_spec=model_spec)

    # Camera setup
    cap = cv2.VideoCapture(0)
    face_detector = FaceDetector()

    # Socket setup
    SERVER_HOST = "127.0.0.1"  # Server's IP address
    SERVER_PORT = 65432        # Server's port
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SERVER_HOST, SERVER_PORT))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.resize(frame, dsize=None, fx=0.5, fy=0.5)

            # Detect faces and update tracker
            detections = face_detector.process_image(frame)
            tracker.step(detections)
            tracks = tracker.active_tracks(min_steps_alive=3)  # Only stable tracks

            # Draw detections and tracks
            for det in detections:
                draw_detection(frame, det)
            for track in tracks:
                draw_track(frame, track)

                # Calculate and print coordinates/offsets
                (x, y), (dx, dy) = calculate_offsets(frame, track)
                print(f"Target Center: (x={x}, y={y}) | Offset from center: (dx={dx}, dy={dy})")

                # Send coordinates to the server
                message = f"x={dx},y={dy}"
                client_socket.sendall(message.encode('utf-8'))

            # Draw frame center (for visualization)
            cv2.circle(frame, (frame.shape[1]//2, frame.shape[0]//2), 5, (0, 255, 0), -1)
            cv2.imshow('frame', frame)

            if cv2.waitKey(int(1000 * dt)) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        client_socket.close()


if __name__ == "__main__":
    run()