#!/usr/bin/env python3
import cv2
import numpy as np
import time
import wiringpi
import threading

# Motor 1 pins (WiringPi numbers) - Pan
PUL1 = 2     # w2 = GPIO12
DIR1 = 3     # w3 = GPIO13
ENA1 = 11    # w11 = GPIO19

# Motor 2 pins (WiringPi numbers) - Tilt
PUL2 = 4     # w4 = GPIO16
DIR2 = 5     # w5 = GPIO1
ENA2 = 6     # w6 = GPIO4

# Motor control settings
MOTOR_DELAY = 0.002  # Initial delay for stepper motors
MOTOR_DURATION = 3   # Duration for each motor step

# PID control parameters
KP = 0.05  # Proportional gain
KI = 0.02  # Integral gain
KD = 0.01  # Derivative gain

# Global variables
running = True
last_error_x = 0
last_error_y = 0
integral_x = 0
integral_y = 0

class MotorController:
    def __init__(self):
        # Initialize WiringPi
        wiringpi.wiringPiSetup()
        
        # Setup motor pins
        for pin in [PUL1, DIR1, ENA1, PUL2, DIR2, ENA2]:
            wiringpi.pinMode(pin, wiringpi.OUTPUT)
            wiringpi.digitalWrite(pin, wiringpi.LOW)
        
        # Enable motors
        wiringpi.digitalWrite(ENA1, wiringpi.HIGH)
        wiringpi.digitalWrite(ENA2, wiringpi.HIGH)
    
    def move_motor(self, motor, direction, steps, delay=MOTOR_DELAY):
        """
        Move a motor in a specific direction for a certain number of steps
        motor: 1 for pan motor, 2 for tilt motor
        direction: 1 for clockwise, 0 for counter-clockwise
        steps: number of steps to move
        delay: delay between pulses (controls speed)
        """
        if motor == 1:
            pul_pin = PUL1
            dir_pin = DIR1
        else:
            pul_pin = PUL2
            dir_pin = DIR2
        
        # Set direction
        wiringpi.digitalWrite(dir_pin, direction)
        
        # Move the motor
        for _ in range(steps):
            wiringpi.digitalWrite(pul_pin, wiringpi.HIGH)
            time.sleep(delay)
            wiringpi.digitalWrite(pul_pin, wiringpi.LOW)
            time.sleep(delay)
    
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
        
        # Create threads for parallel motor movement
        if pan_steps > 0:
            pan_thread = threading.Thread(target=self.move_motor, 
                                          args=(1, pan_direction, pan_steps))
            pan_thread.start()
        
        if tilt_steps > 0:
            tilt_thread = threading.Thread(target=self.move_motor, 
                                           args=(2, tilt_direction, tilt_steps))
            tilt_thread.start()
        
        # Wait for threads to complete
        if pan_steps > 0:
            pan_thread.join()
        if tilt_steps > 0:
            tilt_thread.join()

def person_detector():
    # Load HOG detector for human detection
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    
    # Initialize camera
    cap = cv2.VideoCapture(0)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return
    
    # Get frame dimensions
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        return
    
    frame_height, frame_width = frame.shape[:2]
    center_x = frame_width // 2
    center_y = frame_height // 2
    
    # Initialize motor controller
    motor_controller = MotorController()
    
    # Initialize PID variables
    global integral_x, integral_y, last_error_x, last_error_y
    
    try:
        while running:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break
            
            # Detect people
            boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8), padding=(4, 4), scale=1.05)
            
            # Find the largest person (assume it's the closest/most prominent)
            if len(boxes) > 0:
                largest_box = max(boxes, key=lambda box: box[2] * box[3])
                x, y, w, h = largest_box
                
                # Calculate person center
                person_center_x = x + w // 2
                person_center_y = y + h // 2
                
                # Calculate error (difference between frame center and person center)
                error_x = center_x - person_center_x
                error_y = center_y - person_center_y
                
                # Update integral terms
                integral_x += error_x
                integral_y += error_y
                
                # Calculate derivative terms
                derivative_x = error_x - last_error_x
                derivative_y = error_y - last_error_y
                
                # PID control calculation
                movement_x = KP * error_x + KI * integral_x + KD * derivative_x
                movement_y = KP * error_y + KI * integral_y + KD * derivative_y
                
                # Update last error values
                last_error_x = error_x
                last_error_y = error_y
                
                # Move motors to center the person
                motor_controller.move_to_position(movement_x, movement_y)
                
                # Draw rectangle around the person
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (person_center_x, person_center_y), 5, (0, 0, 255), -1)
            
            # Draw center of frame
            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
            
            # Display the frame
            cv2.imshow('Person Tracking', frame)
            
            # Break if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            time.sleep(0.01)  # Short delay to reduce CPU usage
    
    except KeyboardInterrupt:
        print("Stopping program...")
    
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        global running
        running = False

if __name__ == "__main__":
    try:
        # Start person detection and tracking
        person_detector()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Disable motors when the program exits
        wiringpi.pinMode(ENA1, wiringpi.OUTPUT)
        wiringpi.pinMode(ENA2, wiringpi.OUTPUT)
        wiringpi.digitalWrite(ENA1, wiringpi.LOW)
        wiringpi.digitalWrite(ENA2, wiringpi.LOW)
        print("Motors disabled and program terminated.")
