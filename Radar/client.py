import socket
import numpy as np
import cv2
import time

# Network configuration
SERVER_IP = '192.168.137.2'  # Replace with your radar server IP
SERVER_PORT = 65432
BUFFER_SIZE = 1024
RECONNECT_DELAY = 3  # seconds

# Image configuration
WIDTH = 1080  # horizontal size
HEIGHT = 720  # vertical size
CENTER = (WIDTH // 2, HEIGHT - 10)  # bottom center, 10 px above bottom edge

# Radar parameters
MAX_DISTANCE = 500  # max pixel radius for 40 cm
RADAR_RANGE_CM = 80  # max distance in cm

# Colors (BGR)
BLACK = (0, 0, 0)
GREEN = (153,50,204)
RED = (0, 0, 255)

# Fade factor for ~3 seconds at 10 ms per frame (300 frames)
FADE_FACTOR = 0.983


def polar_to_cartesian(angle_deg, length):
    """
    Convert polar to Cartesian coordinates.
    Rotated 90 degrees left:
    We subtract 90 degrees from the angle to rotate CCW by 90°.
    0 degrees points straight UP (before rotation),
    after rotation it points LEFT.
    """
    rotated_angle = angle_deg - 90
    rad = np.radians(rotated_angle)
    x = int(CENTER[0] + length * np.sin(rad))
    y = int(CENTER[1] - length * np.cos(rad))
    return (x, y)


def initialize_radar_display():
    """Create and return a blank radar image with base drawings"""
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

    # Draw distance circles
    for radius in range(50, MAX_DISTANCE + 1, MAX_DISTANCE // 4):
        cv2.circle(img, CENTER, radius, GREEN, 1)

    # Draw sweep lines every 30 degrees from 0 to 180
    for angle in range(0, 181, 30):
        pt = polar_to_cartesian(angle, MAX_DISTANCE)
        cv2.line(img, CENTER, pt, GREEN, 1)

    # Draw angle labels rotated accordingly
    for deg in range(0, 181, 30):
        pos = polar_to_cartesian(deg, MAX_DISTANCE + 20)
        # Adjust label position so text doesn't overlap with radar edge
        text_offset_x = -15 if deg < 90 else 5
        text_offset_y = 5
        cv2.putText(img, f"{deg}°", (pos[0] + text_offset_x, pos[1] + text_offset_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1)

    # Draw distance labels on the "left" axis (previously vertical)
    for i, dist_cm in enumerate(range(10, RADAR_RANGE_CM + 1, RADAR_RANGE_CM // 4)):
        radius = int(MAX_DISTANCE * dist_cm / RADAR_RANGE_CM)
        label_pos = (CENTER[0] - radius - 40, CENTER[1] + 5)
        cv2.putText(img, f"{dist_cm}cm", label_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)

    return img


def update_display(img, trace_img, distance, angle):
    """Update radar image and trace image with new distance at given angle"""
    # Clamp angle to 0-180 degrees (radar sweep range)
    angle = max(0, min(180, angle))

    # Draw sweep line in green on base image
    sweep_end = polar_to_cartesian(angle, MAX_DISTANCE)
    cv2.line(img, CENTER, sweep_end, GREEN, 2)

    # Draw detection if distance valid on trace image (so it persists)
    if 0 < distance < RADAR_RANGE_CM:
        dist_px = int(np.interp(distance, [0, RADAR_RANGE_CM], [0, MAX_DISTANCE]))
        detected_pt = polar_to_cartesian(angle, dist_px)
        cv2.line(trace_img, sweep_end, detected_pt, RED, 3)
        cv2.circle(trace_img, detected_pt, 6, RED, -1)


def connect_to_server():
    """Connect to the radar server and receive data"""
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(5)
                print(f"Connecting to {SERVER_IP}:{SERVER_PORT}...")
                s.connect((SERVER_IP, SERVER_PORT))
                print("Connected!")

                buffer = ""
                base_img = initialize_radar_display()
                trace_img = np.zeros_like(base_img)  # trace layer to keep old points

                while True:
                    data = s.recv(BUFFER_SIZE).decode()
                    if not data:
                        raise ConnectionError("Server disconnected")

                    buffer += data

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        if ',' in line:
                            try:
                                distance, angle = map(float, line.split(','))

                                # Fade trace layer slowly
                                trace_img = (trace_img * FADE_FACTOR).astype(np.uint8)

                                # Update display images
                                img = base_img.copy()
                                update_display(img, trace_img, distance, angle)

                                # Blend trace image on top of base image
                                output_img = cv2.addWeighted(img, 1, trace_img, 1, 0)

                                cv2.imshow('Radar Display', output_img)
                                if cv2.waitKey(10) & 0xFF == ord('q'):
                                    raise KeyboardInterrupt
                            except ValueError:
                                pass

        except (ConnectionRefusedError, socket.gaierror) as e:
            print(f"Connection failed: {e}. Retrying in {RECONNECT_DELAY}s...")
            time.sleep(RECONNECT_DELAY)
        except socket.timeout:
            print("Connection timeout, reconnecting...")
        except ConnectionError as e:
            print(e)
        except KeyboardInterrupt:
            print("Exiting...")
            break
        except Exception as e:
            print(f"Unexpected error: {e}, reconnecting...")
            time.sleep(RECONNECT_DELAY)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    connect_to_server()

