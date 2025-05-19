#!/usr/bin/env python3
import cv2
import numpy as np
import time
import wiringpi
import threading
import signal
import sys

class PersonTracker:
    # Motor 1 pins (WiringPi numbers) - Pan
    PUL1 = 2     # w2 = GPIO12
    DIR1 = 3     # w3 = GPIO13
    ENA1 = 11    # w11 = GPIO19

    # Motor 2 pins (WiringPi numbers) - Tilt
    PUL2 = 4     # w4 = GPIO16
    DIR2 = 5     # w5 = GPIO1
    ENA2 = 6     # w6 = GPIO4

    # Motor control settings
    MOTOR_DELAY = 0.002  # Delay for stepper motors
    
    def __init__(self):
        # Control flags
        self.running = True
        
        # PID control parameters
        self.kp = 0.05  # Proportional gain
        self.ki = 0.02  # Integral gain
        self.kd = 0.01  # Derivative gain
        
        # PID state variables
        self.error_x = 0
        self.error_y = 0
        self.last_error_x = 0
        self.last_error_y = 0
        self.integral_x = 0
        self.integral_y = 0
        
        # Camera setup
        self.cap = None
        self.frame_width = 0
        self.frame_height = 0
        self.center_x = 0
        self.center_y = 0
        
        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Initialize WiringPi
        try:
            wiringpi.wiringPiSetup()
            self.setup_motors()
            print("Motors initialized successfully")
        except Exception as e:
            print(f"Error initializing motors: {e}")
            sys.exit(1)
    
    def setup_motors(self):
        """Configure motor pins and set initial state"""
        # Setup motor pins
        for pin in [self.PUL1, self.DIR1, self.ENA1, self.PUL2, self.DIR2, self.ENA2]:
            wiringpi.pinMode(pin, wiringpi.OUTPUT)
            wiringpi.digitalWrite(pin, wiringpi.LOW)
        
        # Enable motors
        wiringpi.digitalWrite(self.ENA1, wiringpi.HIGH)
        wiringpi.digitalWrite(self.ENA2, wiringpi.HIGH)
    
    def disable_motors(self):
        """Disable motors to prevent holding current"""
        try:
            wiringpi.digitalWrite(self.ENA1, wiringpi.LOW)
            wiringpi.digitalWrite(self.ENA2, wiringpi.LOW)
            print("Motors disabled")
        except:
            pass
    
    def setup_camera(self):
        """Initialize the camera and get frame dimensions"""
        try:
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                print("Error: Could not open camera.")
                self.running = False
                return False
            
            # Get frame dimensions
            ret, frame = self.cap.read()
            if not ret:
                print("Error: Could not read frame.")
                self.running = False
                return False
            
            self.frame_height, self.frame_width = frame.shape[:2]
            self.center_x = self.frame_width // 2
            self.center_y = self.frame_height // 2
            print(f"Camera initialized: {self.frame_width}x{self.frame_height}")
            return True
        except Exception as e:
            print(f"Camera setup error: {e}")
            self.running = False
            return False
    
    def move_motor(self, motor, direction, steps):
        """
        Move a motor in a specific direction for a certain number of steps
        motor: 1 for pan motor, 2 for tilt motor
        direction: 1 for clockwise, 0 for counter-clockwise
        steps: number of steps to move
        """
        if not self.running:
            return
            
        if motor == 1:
            pul_pin = self.PUL1
            dir_pin = self.DIR1
        else:
            pul_pin = self.PUL2
            dir_pin = self.DIR2
        
        # Set direction
        wiringpi.digitalWrite(dir_pin, direction)
        
        # Move the motor
        for _ in range(steps):
            if not self.running:
                break
            wiringpi.digitalWrite(pul_pin, wiringpi.HIGH)
            time.sleep(self.MOTOR_DELAY)
            wiringpi.digitalWrite(pul_pin, wiringpi.LOW)
            time.sleep(self.MOTOR_DELAY)
    
    def move_to_position(self, x_movement, y_movement):
        """
        Move motors based on required x and y adjustments
        x_movement: positive values move right, negative values move left
        y_movement: positive values move up, negative values move down
        """
        # Determine pan (horizontal) direction and steps
        pan_direction = 1 if x_movement > 0 else 0
        pan_steps = min(abs(int(x_movement)), 20)  # Limit steps for smooth movement
        
        # Determine tilt (vertical) direction and steps
        tilt_direction = 1 if y_movement > 0 else 0
        tilt_steps = min(abs(int(y_movement)), 20)  # Limit steps for smooth movement
        
        # Print movement info for debugging
        print(f"Movement - X: {x_movement:.2f} ({pan_steps} steps {'right' if pan_direction else 'left'}), " 
              f"Y: {y_movement:.2f} ({tilt_steps} steps {'up' if tilt_direction else 'down'})")
        
        # Create and start threads for parallel motor movement
        threads = []
        
        if pan_steps > 0:
            pan_thread = threading.Thread(target=self.move_motor, 
                                        args=(1, pan_direction, pan_steps))
            threads.append(pan_thread)
            pan_thread.start()
        
        if tilt_steps > 0:
            tilt_thread = threading.Thread(target=self.move_motor, 
                                         args=(2, tilt_direction, tilt_steps))
            threads.append(tilt_thread)
            tilt_thread.start()
        
        # Wait for threads to complete
        for thread in threads:
            thread.join()
    
    def pid_controller(self, error_x, error_y):
        """Calculate PID control values based on position errors"""
        # Update integral terms with limits to prevent windup
        self.integral_x = max(-100, min(100, self.integral_x + error_x))
        self.integral_y = max(-100, min(100, self.integral_y + error_y))
        
        # Calculate derivative terms
        derivative_x = error_x - self.last_error_x
        derivative_y = error_y - self.last_error_y
        
        # PID control calculation
        movement_x = self.kp * error_x + self.ki * self.integral_x + self.kd * derivative_x
        movement_y = self.kp * error_y + self.ki * self.integral_y + self.kd * derivative_y
        
        # Update last error values
        self.last_error_x = error_x
        self.last_error_y = error_y
        
        return movement_x, movement_y
    
    def detect_and_track(self):
        """Main loop for person detection and tracking"""
        # Load HOG detector for human detection
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # Initialize camera
        if not self.setup_camera():
            return
        
        print("Starting person detection and tracking...")
        
        frame_count = 0
        start_time = time.time()
        
        try:
            while self.running:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    print("Error: Could not read frame.")
                    break
                
                frame_count += 1
                
                # Process every other frame to reduce CPU load
                if frame_count % 2 != 0:
                    continue
                
                # Calculate FPS every 30 frames
                if frame_count % 30 == 0:
                    end_time = time.time()
                    fps = 30 / (end_time - start_time)
                    start_time = end_time
                    print(f"FPS: {fps:.1f}")
                
                # Detect people
                boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(4, 4), scale=1.05)
                
                # Find the largest person (assume it's the closest/most prominent)
                person_detected = False
                if len(boxes) > 0:
                    largest_box = max(boxes, key=lambda box: box[2] * box[3])
                    x, y, w, h = largest_box
                    
                    # Calculate person center
                    person_center_x = x + w // 2
                    person_center_y = y + h // 2
                    
                    # Calculate error (difference between frame center and person center)
                    error_x = self.center_x - person_center_x
                    error_y = self.center_y - person_center_y
                    
                    # Apply PID control
                    movement_x, movement_y = self.pid_controller(error_x, error_y)
                    
                    # Move motors to center the person
                    self.move_to_position(movement_x, movement_y)
                    
                    # Draw rectangle around the person
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (person_center_x, person_center_y), 5, (0, 0, 255), -1)
                    
                    # Show error values on screen
                    cv2.putText(frame, f"Error X: {error_x}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    cv2.putText(frame, f"Error Y: {error_y}", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    person_detected = True
                
                # Show tracking status
                status = "Tracking" if person_detected else "Searching"
                cv2.putText(frame, f"Status: {status}", (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                # Draw center of frame
                cv2.circle(frame, (self.center_x, self.center_y), 5, (255, 0, 0), -1)
                cv2.line(frame, (self.center_x - 20, self.center_y), 
                        (self.center_x + 20, self.center_y), (255, 0, 0), 1)
                cv2.line(frame, (self.center_x, self.center_y - 20), 
                        (self.center_x, self.center_y + 20), (255, 0, 0), 1)
                
                # Display the frame
                cv2.imshow('Person Tracking', frame)
                
                # Break if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("User requested exit")
                    self.running = False
                    break
                
                # Short delay to reduce CPU usage
                time.sleep(0.01)
        
        except Exception as e:
            print(f"Error in tracking loop: {e}")
        
        finally:
            # Clean up
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\nReceived interrupt signal. Shutting down...")
        self.running = False
    
    def cleanup(self):
        """Clean up resources before exit"""
        self.disable_motors()
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        print("Cleanup complete")
    
    def run(self):
        """Main entry point"""
        try:
            self.detect_and_track()
        except Exception as e:
            print(f"Error in main execution: {e}")
        finally:
            self.cleanup()

if __name__ == "__main__":
    # Create and run the tracker
    tracker = PersonTracker()
    tracker.run()