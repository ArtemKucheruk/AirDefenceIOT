
import wiringpi
from configs.configAirDefence import *
import time
import threading
import cv2


class AngleCalculator:
    """Handles angle calculations based on camera parameters and pixel positions"""
    
    def __init__(self, frame_width, frame_height, h_fov, v_fov):
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.h_fov = h_fov
        self.v_fov = v_fov
        
        # Calculate pixel-to-angle conversion factors
        self.x_degrees_per_pixel = h_fov / frame_width
        self.y_degrees_per_pixel = v_fov / frame_height
        
        # Center points
        self.center_x = frame_width // 2
        self.center_y = frame_height // 2
        
    def get_angle_from_pixel(self, x, y):
        """Convert pixel position to angles relative to center"""
        dx = x - self.center_x
        dy = y - self.center_y
        
        # Calculate angle offsets
        angle_x = dx * self.x_degrees_per_pixel
        angle_y = dy * self.y_degrees_per_pixel
        
        return angle_x, angle_y


class LaserController:
    """Handles laser control"""
    
    def __init__(self, pin):
        self.pin = pin
        self.is_on = False
        self.setup()
        
    def setup(self):
        wiringpi.pinMode(self.pin, wiringpi.OUTPUT)
        self.on()  # Turn on laser at startup
        
    def on(self):
        wiringpi.digitalWrite(self.pin, wiringpi.LOW)  # LOW turns laser ON
        self.is_on = True
        
    def off(self):
        wiringpi.digitalWrite(self.pin, wiringpi.HIGH)  # HIGH turns laser OFF
        self.is_on = False
        
    def toggle(self):
        if self.is_on:
            self.off()
        else:
            self.on()


class MotorController:
    """Handles a single stepper motor with precise angle control"""
    
    def __init__(self, pul_pin, dir_pin, ena_pin, steps_per_rev, max_angle, inverted=False, is_y_axis=False):
        self.pul_pin = pul_pin
        self.dir_pin = dir_pin
        self.ena_pin = ena_pin
        self.steps_per_rev = steps_per_rev
        self.max_angle = max_angle
        self.inverted = inverted
        self.is_y_axis = is_y_axis  # Flag for Y-axis specific handling
        
        self.setup_gpio()
        
        # Current position state
        self.current_angle = 0.0  # degrees
        self.current_steps = 0    # steps from center
        
        # Constants for conversion
        self.steps_per_degree = steps_per_rev / 360.0
        
    def setup_gpio(self):
        wiringpi.pinMode(self.pul_pin, wiringpi.OUTPUT)
        wiringpi.pinMode(self.dir_pin, wiringpi.OUTPUT)
        wiringpi.pinMode(self.ena_pin, wiringpi.OUTPUT)
        wiringpi.digitalWrite(self.ena_pin, wiringpi.LOW)  # Enable the motor
        
    def set_direction(self, direction):
        """Set direction pin accounting for inverted configuration"""
        # If motor is inverted, flip the direction
        actual_dir = not direction if self.inverted else direction
        wiringpi.digitalWrite(self.dir_pin, wiringpi.HIGH if actual_dir else wiringpi.LOW)
        
    def move_to_angle(self, target_angle, speed=800):
        """Move motor to specific angle from center position"""
        # Clamp the target angle to our limits
        target_angle = max(min(target_angle, self.max_angle), -self.max_angle)
        
        # Calculate change in angle
        delta_angle = target_angle - self.current_angle
        
        # Skip if the change is too small
        if abs(delta_angle) < 0.05:  # minimum angle change threshold (degrees)
            return
            
        # Convert angle to steps
        steps_to_move = int(delta_angle * self.steps_per_degree)
        
        # Skip if no steps needed
        if steps_to_move == 0:
            return
            
        # Set direction
        self.set_direction(steps_to_move > 0)
        steps_to_move = abs(steps_to_move)
        
        # Adjust speed for Y-axis if needed
        if self.is_y_axis:
            speed = speed * Y_SPEED_FACTOR  # Slow down Y-axis for smoother movement
            
        # Calculate delay based on speed (steps per second)
        # More consistent timing using wiringPi delay functions
        delay_micros = int(1000000 / (speed * 2))  # Convert to microseconds
        
        # Move the motor with more precise timing
        for _ in range(steps_to_move):
            wiringpi.digitalWrite(self.pul_pin, wiringpi.HIGH)
            wiringpi.delayMicroseconds(delay_micros)
            wiringpi.digitalWrite(self.pul_pin, wiringpi.LOW)
            wiringpi.delayMicroseconds(delay_micros)
            
        # Update current angle
        self.current_angle = target_angle
        self.current_steps = int(target_angle * self.steps_per_degree)
        
    def center(self, speed=600):
        """Return to center position (0 degrees) at moderate speed"""
        self.move_to_angle(0, speed)
        
    def disable(self):
        """Disable motor power"""
        wiringpi.digitalWrite(self.ena_pin, wiringpi.HIGH)
        
    def enable(self):
        """Enable motor power"""
        wiringpi.digitalWrite(self.ena_pin, wiringpi.LOW)




class TrapezoidalProfile:
    def __init__(self, max_speed, accel):
        self.max_speed = max_speed
        self.accel = accel

    def get_speed(self, distance, current_speed):
        if current_speed < self.max_speed:
            current_speed += self.accel
        if current_speed * current_speed / (2 * self.accel) > distance:
            current_speed -= self.accel  # start decelerating
        return min(current_speed, self.max_speed)
    



class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class TrackingSystem:
    """Main system to track faces and control motors"""
    
    def __init__(self):
        wiringpi.wiringPiSetup()
        
        # Initialize laser
        self.laser = LaserController(LASER_PIN)
        
        # Initialize angle calculator
        self.angle_calculator = AngleCalculator(
            FRAME_WIDTH, FRAME_HEIGHT, 
            CAMERA_HORIZONTAL_FOV, CAMERA_VERTICAL_FOV
        )
        
        # Initialize motors with proper angle constraints
        self.x_motor = MotorController(
            PUL1, DIR1, ENA1, 
            TOTAL_STEPS_PER_REV, MAX_ANGLE_X, 
            inverted=False
        )
        
        self.y_motor = MotorController(
            PUL2, DIR2, ENA2, 
            TOTAL_STEPS_PER_REV, MAX_ANGLE_Y, 
            inverted=True,  # Y axis is typically inverted
            is_y_axis=True  # Mark as Y-axis for special handling
        )
        
        # Control parameters
        self.target_angle_x = 0.0
        self.target_angle_y = 0.0
        self.last_update_time = time.time()
        self.lock = threading.Lock()
        
        # Filtering parameters - adjusted for better Y-axis stability
        self.angle_alpha_x = 0.3  # Filter factor (0-1) for X-axis
        self.angle_alpha_y = 0.2  # Filter factor for Y-axis (more filtering)
        self.min_movement_threshold_x = 0.2  # Threshold for X-axis
        self.min_movement_threshold_y = 0.3  # Higher threshold for Y-axis stability
        
        # State tracking
        self.last_face_time = time.time()
        self.face_tracking = True
        self.tracking_enabled = True
        self.debug_mode = False
        self.running = True
        
        # Start control thread
        threading.Thread(target=self.motor_control_loop, daemon=True).start()
    
    def set_target_position(self, x, y):
        """Update target position based on pixel coordinates"""
        # Calculate angles from pixel position
        angle_x, angle_y = self.angle_calculator.get_angle_from_pixel(x, y)
        
        with self.lock:
            # Apply exponential filter for smoother target changes
            # Different filtering factors for X and Y
            if hasattr(self, 'target_angle_x'):
                self.target_angle_x = self.angle_alpha_x * angle_x + (1 - self.angle_alpha_x) * self.target_angle_x
                self.target_angle_y = self.angle_alpha_y * angle_y + (1 - self.angle_alpha_y) * self.target_angle_y
            else:
                self.target_angle_x = angle_x
                self.target_angle_y = angle_y
                
            self.last_face_time = time.time()
            self.face_tracking = True
            
            if self.debug_mode:
                print(f"Target: X={x}, Y={y} | Angles: X={self.target_angle_x:.2f}째, Y={self.target_angle_y:.2f}째")
    
    def motor_control_loop(self):
        try:
            last_x_angle = self.x_motor.current_angle
            last_y_angle = self.y_motor.current_angle
            while self.running:
                if not self.tracking_enabled:
                    self.laser.off()
                    time.sleep(0.1)
                    continue

                current_time = time.time()

                # If no face detected for 2 seconds, center the camera and turn off laser
                if current_time - self.last_face_time > 2.0 and self.face_tracking:
                    self.face_tracking = False
                    self.laser.off()
                    self.center_position()
                    time.sleep(0.1)
                    continue

                if self.face_tracking:
                    self.laser.on()

                with self.lock:
                    target_x = self.target_angle_x
                    target_y = self.target_angle_y

                # X-axis
                x_error = target_x - self.x_motor.current_angle
                if abs(x_error) > self.min_movement_threshold_x:
                    # Optionally use PID here for speed or step calculation
                    # pid_speed_x = self.x_pid.compute(x_error, dt)
                    x_speed = min(MAX_SPEED, max(MIN_SPEED, MIN_SPEED + abs(x_error) * 40))
                    if abs(target_x - last_x_angle) > 0.05:  # Only send if target changed enough
                        self.x_motor.move_to_angle(target_x, speed=x_speed)
                        last_x_angle = target_x

                # Y-axis
                y_error = target_y - self.y_motor.current_angle
                if abs(y_error) > self.min_movement_threshold_y:
                    y_speed = min(MAX_SPEED * Y_SPEED_FACTOR, max(MIN_SPEED, MIN_SPEED + abs(y_error) * 30))
                    if abs(target_y - last_y_angle) > 0.05:
                        self.y_motor.move_to_angle(target_y, speed=y_speed)
                        last_y_angle = target_y

                time.sleep(0.03)  # Slightly faster loop for more responsive tracking
        except Exception as e:
            print(f"Control error: {e}")
        finally:
            self.cleanup()
            
    def center_position(self):
        """Center both motors"""
        with self.lock:
            self.target_angle_x = 0.0
            self.target_angle_y = 0.0
            
        # Center X first, then Y (slower speed for Y)
        self.x_motor.center(speed=800)
        self.y_motor.center(speed=500)  # Slower speed for Y-axis centering
        
    def toggle_laser(self):
        """Toggle laser on/off"""
        self.laser.toggle()
        return f"Laser {'OFF' if not self.laser.is_on else 'ON'}"
        
    def enable_tracking(self, enabled=True):
        """Enable or disable tracking"""
        self.tracking_enabled = enabled
        
    def enable_debug_mode(self, enabled=True):
        """Enable or disable debug output"""
        self.debug_mode = enabled
        
    def cleanup(self):
        """Disable motors and clean up resources"""
        self.laser.off()  # Turn off laser
        self.x_motor.disable()
        self.y_motor.disable()
        
    def calibrate(self):
        """Run a calibration routine"""
        print("\n=== Starting Calibration ===")
        self.enable_tracking(False)
        
        # Test X-axis direction
        print("Testing X-axis direction...")
        self.x_motor.move_to_angle(5)
        time.sleep(1)
        self.x_motor.move_to_angle(-5)
        time.sleep(1)
        self.x_motor.center()
        x_inverted = input("Is X-axis moving in the correct direction? (y/n): ").lower() == 'n'
        
        # Test Y-axis direction
        print("Testing Y-axis direction...")
        self.y_motor.move_to_angle(5)
        time.sleep(1)
        self.y_motor.move_to_angle(-5)
        time.sleep(1)
        self.y_motor.center()
        y_inverted = input("Is Y-axis moving in the correct direction? (y/n): ").lower() == 'n'
        
        # Update motor configuration
        self.x_motor.inverted = x_inverted
        self.y_motor.inverted = y_inverted
        
        print("Testing movement speed...")
        self.x_motor.move_to_angle(10, speed=800)
        self.x_motor.move_to_angle(-10, speed=800)
        self.x_motor.center()
        
        print("Calibration complete!")
        print(f"X-axis inverted: {self.x_motor.inverted}")
        print(f"Y-axis inverted: {self.y_motor.inverted}")
        
        self.enable_tracking(True)




class CameraThread(threading.Thread):
    """Thread for camera capture and face detection"""
    
    def __init__(self, tracking_system):
        threading.Thread.__init__(self)
        self.camera = cv2.VideoCapture(CAMERA_ID)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.camera.set(cv2.CAP_PROP_FPS, 30)  # Try to get 30fps if possible
        
        self.frame = None
        self.latest_faces = []
        self.running = True
        self.frame_counter = 0
        self.fps = 0
        self.last_time = time.time()
        self.lock = threading.Lock()
        self.tracking_system = tracking_system
        
        # Load face detector
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        # Face tracking state
        self.last_face_size = 0
        self.confidence_threshold = 0.5
        
    def run(self):
        while self.running:
            ret, frame = self.camera.read()
            if not ret:
                time.sleep(0.01)
                continue
                
            # Flip if needed
            if FLIP_CAMERA:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
                
            # Calculate FPS
            self.frame_counter += 1
            current_time = time.time()
            if current_time - self.last_time >= 1.0:
                self.fps = self.frame_counter / (current_time - self.last_time)
                self.frame_counter = 0
                self.last_time = current_time
                
            # Process face detection every N frames
            if self.frame_counter % FACE_DETECTION_INTERVAL == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                
                # Apply histogram equalization for better detection in varying light
                gray = cv2.equalizeHist(gray)
                
                faces = self.face_cascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=MIN_FACE_SIZE
                )
                
                with self.lock:
                    self.latest_faces = faces
                    
                    # Find the largest face
                    if len(faces) > 0:
                        largest_face = None
                        largest_area = 0
                        
                        for (x, y, w, h) in faces:
                            area = w * h
                            if area > largest_area:
                                largest_area = area
                                largest_face = (x, y, w, h)
                                
                        # Use largest face for tracking
                        if largest_face is not None:
                            x, y, w, h = largest_face
                            center_x = x + w // 2
                            center_y = y + h // 2
                            
                            # Update tracking system with face position
                            self.tracking_system.set_target_position(center_x, center_y)
                            
                            # Debug output
                            if self.tracking_system.debug_mode:
                                angle_x, angle_y = self.tracking_system.angle_calculator.get_angle_from_pixel(center_x, center_y)
                                print(f"Face: ({center_x}, {center_y}) | Size: {w}x{h} | Angle: ({angle_x:.2f}째, {angle_y:.2f}째)")
            
            # Store the latest frame
            with self.lock:
                self.frame = frame.copy()
                
    def get_frame_with_faces(self):
        with self.lock:
            if self.frame is None:
                return None, []
                
            frame = self.frame.copy()
            faces = list(self.latest_faces)
            
        # Draw face rectangles and tracking info
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            face_center = (x + w // 2, y + h // 2)
            cv2.circle(frame, face_center, 5, (0, 0, 255), -1)
            
            # Calculate angles for this face
            angle_x, angle_y = self.tracking_system.angle_calculator.get_angle_from_pixel(face_center[0], face_center[1])
            cv2.putText(
                frame, f"Angle: {angle_x:.1f}째, {angle_y:.1f}째", 
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1
            )
            
        # Draw center crosshair
        center_x, center_y = self.tracking_system.angle_calculator.center_x, self.tracking_system.angle_calculator.center_y
        cv2.circle(frame, (center_x, center_y), 8, (255, 0, 0), 2)
        cv2.line(frame, (center_x - 20, center_y), (center_x + 20, center_y), (255, 0, 0), 1)
        cv2.line(frame, (center_x, center_y - 20), (center_x, center_y + 20), (255, 0, 0), 1)
        
        # Display FPS and motor angles
        cv2.putText(
            frame, f"FPS: {self.fps:.1f}", (10, 25), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        
        # Display current motor angles
        cv2.putText(
            frame, f"Motor X: {self.tracking_system.x_motor.current_angle:.1f}째", (10, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.putText(
            frame, f"Motor Y: {self.tracking_system.y_motor.current_angle:.1f}째", (10, 75), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        
        # Display tracking status
        status = "TRACKING ON" if self.tracking_system.tracking_enabled else "TRACKING OFF"
        cv2.putText(
            frame, status, (FRAME_WIDTH - 150, 25), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
        )
        
        # Display laser status
        laser_status = "LASER ON" if self.tracking_system.laser.is_on else "LASER OFF"
        cv2.putText(
            frame, laser_status, (FRAME_WIDTH - 150, 50), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255) if self.tracking_system.laser.is_on else (100, 100, 100), 2
        )
            
        return frame, faces
        
    def stop(self):
        self.running = False
        if self.camera.isOpened():
            self.camera.release()
