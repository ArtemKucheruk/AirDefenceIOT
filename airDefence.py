from flask import Flask, Response, render_template
import cv2
import time
from configs.configAirDefence import *
from hardware.airDefenceClasses.airDefenceClass import *
from AirDefenceFuncs.airDefenceFuncs import *
import time
import cv2
from threading import Thread
from configs.configRadar import *
from hardware.radarFuncs.radarFuncs import *







app = Flask(__name__)

# ===== Combined Routes =====

@app.route('/')
def index():
    """Main combined interface"""
    return render_template('index.html')

@app.route('/radar_video')
def radar_video():
    """Radar video streaming route"""
    def generate():
        while True:
            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_video')
def camera_video():
    """Camera video streaming route"""
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

# Radar control endpoints
@app.route('/radar/start', methods=['POST'])
def start_scan():
    return "Radar scan started", 200

@app.route('/radar/stop', methods=['POST'])
def stop_scan():
    return "Radar scan stopped", 200

@app.route('/radar/save', methods=['POST'])
def save_data():
    return "Radar data saved", 200

# Tracking system endpoints
@app.route('/tracking/center')
def center():
    tracking_system.center_position()
    return "Motors centered"

@app.route('/tracking/toggle_tracking')
def toggle_tracking():
    tracking_system.enable_tracking(not tracking_system.tracking_enabled)
    status = "enabled" if tracking_system.tracking_enabled else "disabled"
    return f"Tracking {status}"

@app.route('/tracking/toggle_laser')
def toggle_laser():
    return tracking_system.toggle_laser()

@app.route('/tracking/calibrate')
def calibrate():
    threading.Thread(target=tracking_system.calibrate).start()
    return "Calibration started. Follow instructions in the console."

@app.route('/tracking/debug_toggle')
def debug_toggle():
    tracking_system.enable_debug_mode(not tracking_system.debug_mode)
    status = "enabled" if tracking_system.debug_mode else "disabled"
    return f"Debug mode {status}"

def run_web(WEB_HOST, WEB_PORT):
    app.run(host=WEB_HOST, port=WEB_PORT, threaded=True)






# Initialize tracking system
tracking_system = TrackingSystem()

# Initialize camera thread
camera_thread = CameraThread(tracking_system)
camera_thread.start()


def generate_frames():
    """Generate video frames for streaming"""
    while True:
        frame, _ = camera_thread.get_frame_with_faces()
        if frame is None:
            time.sleep(0.01)
            continue
            
        # Encode frame for streaming
        ret, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')




if __name__ == '__main__':
    try:
        # Center motors at startup
        print("Starting Angle-Based Face Tracking System...")
        print("Centering motors...")
        tracking_system.center_position()
        setup_hardware(TRIG, ECHO, STEPPER_PINS)
        
        # Start radar server in a separate thread
        radar_thread = Thread(
            target=serve_radar,
            args=(TRIG, ECHO, RADAR_HOST, RADAR_PORT, STEPS_PER_DEGREE, TOTAL_ANGLE, STEP_DELAY, STEPPER_PINS, SEQ, DELAY),
            daemon=True
        )
        radar_thread.start()
        
        # Start radar data processing in a separate thread
        data_thread = Thread(target=radar_data_loop, args=(CENTER, MAX_DISTANCE, HEIGHT, WIDTH, GREEN, RADAR_RANGE_CM, RADAR_HOST, RADAR_PORT, FADE_FACTOR, RED), daemon=True)
        data_thread.start()
        print("Laser enabled at startup")
        print("System ready! Visit http://[YOUR-IP]:8000 to access the interface")
        app.run(host='0.0.0.0', port=8000, threaded=True)
    finally:
        camera_thread.stop()
        camera_thread.join()
        tracking_system.cleanup()
