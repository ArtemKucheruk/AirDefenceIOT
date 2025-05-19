#!/usr/bin/env python
# Orange Pi Turret Control - Adapted from Raspberry Pi version

try:
    import cv2
except Exception as e:
    print("Warning: OpenCV not installed. To use motion detection, make sure you've properly configured OpenCV.")

import time
import threading
import atexit
import sys
import termios
import contextlib

import imutils
import wiringpi  # Using wiringpi instead of RPi.GPIO

### User Parameters ###

MOTOR_X_REVERSED = False
MOTOR_Y_REVERSED = False

MAX_STEPS_X = 30
MAX_STEPS_Y = 15

RELAY_PIN = 22  # Make sure this pin number is correct for Orange Pi

# Motor controller pins for Orange Pi
# You'll need to adjust these based on your specific Orange Pi model and connections
X_MOTOR_PIN1 = 1
X_MOTOR_PIN2 = 2
X_MOTOR_PIN3 = 3
X_MOTOR_PIN4 = 4

Y_MOTOR_PIN1 = 5
Y_MOTOR_PIN2 = 6
Y_MOTOR_PIN3 = 7
Y_MOTOR_PIN4 = 8

# Step sequence for stepper motors (full step)
STEP_SEQUENCE = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

#######################


@contextlib.contextmanager
def raw_mode(file):
    """
    Magic function that allows key presses.
    :param file:
    :return:
    """
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)


class StepperMotor:
    """
    Class to control a stepper motor using WiringPi on Orange Pi
    """
    def __init__(self, pin1, pin2, pin3, pin4, steps_per_rev=200):
        self.pins = [pin1, pin2, pin3, pin4]
        self.steps_per_rev = steps_per_rev
        self.current_step = 0
        self.rpm = 5
        
        # Setup pins as outputs
        for pin in self.pins:
            wiringpi.pinMode(pin, wiringpi.OUTPUT)
            wiringpi.digitalWrite(pin, wiringpi.LOW)
    
    def setSpeed(self, rpm):
        """
        Set the motor speed in RPM
        """
        self.rpm = rpm
    
    def step(self, steps, direction):
        """
        Move the motor a specific number of steps in given direction
        """
        delay = 60.0 / (self.steps_per_rev * self.rpm)  # seconds per step
        delay_ms = int(delay * 1000)  # milliseconds
        
        for _ in range(steps):
            if direction == 'forward':
                self.current_step = (self.current_step + 1) % 4
            else:  # backward
                self.current_step = (self.current_step - 1) % 4
            
            # Set the pins according to the step sequence
            for i in range(4):
                wiringpi.digitalWrite(self.pins[i], STEP_SEQUENCE[self.current_step][i])
            
            # Delay between steps
            wiringpi.delay(delay_ms)
        
        # Turn off all pins to save power
        for pin in self.pins:
            wiringpi.digitalWrite(pin, wiringpi.LOW)


class VideoUtils(object):
    """
    Helper functions for video utilities.
    """
    @staticmethod
    def live_video(camera_port=0):
        """
        Opens a window with live video.
        :param camera:
        :return:
        """
        video_capture = cv2.VideoCapture(camera_port)

        while True:
            # Capture frame-by-frame
            ret, frame = video_capture.read()

            # Display the resulting frame
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything is done, release the capture
        video_capture.release()
        cv2.destroyAllWindows()

    @staticmethod
    def find_motion(callback, camera_port=0, show_video=False):
        """
        Detect motion from camera feed and call the callback function
        """
        camera = cv2.VideoCapture(camera_port)
        time.sleep(0.25)

        # initialize the first frame in the video stream
        firstFrame = None
        tempFrame = None
        count = 0

        # loop over the frames of the video
        while True:
            # grab the current frame and initialize the occupied/unoccupied text
            (grabbed, frame) = camera.read()

            # if the frame could not be grabbed, then we have reached the end
            # of the video
            if not grabbed:
                break

            # resize the frame, convert it to grayscale, and blur it
            frame = imutils.resize(frame, width=500)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)

            # if the first frame is None, initialize it
            if firstFrame is None:
                print("Waiting for video to adjust...")
                if tempFrame is None:
                    tempFrame = gray
                    continue
                else:
                    delta = cv2.absdiff(tempFrame, gray)
                    tempFrame = gray
                    tst = cv2.threshold(delta, 5, 255, cv2.THRESH_BINARY)[1]
                    tst = cv2.dilate(tst, None, iterations=2)
                    if count > 30:
                        print("Done.\nWaiting for motion.")
                        if not cv2.countNonZero(tst) > 0:
                            firstFrame = gray
                        else:
                            continue
                    else:
                        count += 1
                        continue

            # compute the absolute difference between the current frame and
            # first frame
            frameDelta = cv2.absdiff(firstFrame, gray)
            thresh = cv2.threshold(frameDelta, 25, 255, cv2.THRESH_BINARY)[1]

            # dilate the thresholded image to fill in holes, then find contours
            # on thresholded image
            thresh = cv2.dilate(thresh, None, iterations=2)
            c = VideoUtils.get_best_contour(thresh.copy(), 5000)

            if c is not None:
                # compute the bounding box for the contour, draw it on the frame,
                # and update the text
                (x, y, w, h) = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                callback(c, frame)

            # show the frame and record if the user presses a key
            if show_video:
                cv2.imshow("Security Feed", frame)
                key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the lop
                if key == ord("q"):
                    break

        # cleanup the camera and close any open windows
        camera.release()
        cv2.destroyAllWindows()

    @staticmethod
    def get_best_contour(imgmask, threshold):
        """
        Find the largest contour in the image that meets the threshold
        """
        contours, _ = cv2.findContours(imgmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_area = threshold
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best_cnt = cnt
        return best_cnt


class Turret(object):
    """
    Class used for turret control.
    """
    def __init__(self, friendly_mode=True):
        self.friendly_mode = friendly_mode

        # Initialize WiringPi
        wiringpi.wiringPiSetup()

        # Create stepper motor objects
        self.sm_x = StepperMotor(X_MOTOR_PIN1, X_MOTOR_PIN2, X_MOTOR_PIN3, X_MOTOR_PIN4)
        self.sm_x.setSpeed(5)  # 5 RPM
        self.current_x_steps = 0

        self.sm_y = StepperMotor(Y_MOTOR_PIN1, Y_MOTOR_PIN2, Y_MOTOR_PIN3, Y_MOTOR_PIN4)
        self.sm_y.setSpeed(5)  # 5 RPM
        self.current_y_steps = 0

        # Setup Relay
        wiringpi.pinMode(RELAY_PIN, wiringpi.OUTPUT)
        wiringpi.digitalWrite(RELAY_PIN, wiringpi.LOW)
        
        # Register cleanup function
        atexit.register(self.__turn_off_motors)

    def __turn_off_motors(self):
        """
        Turn off all motor pins to prevent overheating
        """
        # Turn off X motor pins
        for pin in [X_MOTOR_PIN1, X_MOTOR_PIN2, X_MOTOR_PIN3, X_MOTOR_PIN4]:
            wiringpi.digitalWrite(pin, wiringpi.LOW)
        
        # Turn off Y motor pins
        for pin in [Y_MOTOR_PIN1, Y_MOTOR_PIN2, Y_MOTOR_PIN3, Y_MOTOR_PIN4]:
            wiringpi.digitalWrite(pin, wiringpi.LOW)
        
        # Turn off relay
        wiringpi.digitalWrite(RELAY_PIN, wiringpi.LOW)

    def fire(self, duration=0.5):
        """
        Activate the relay to fire
        """
        if not self.friendly_mode:
            wiringpi.digitalWrite(RELAY_PIN, wiringpi.HIGH)
            time.sleep(duration)
            wiringpi.digitalWrite(RELAY_PIN, wiringpi.LOW)

    @staticmethod
    def move_forward(stepper, steps):
        """
        Move stepper motor forward
        """
        stepper.step(steps, 'forward')

    @staticmethod
    def move_backward(stepper, steps):
        """
        Move stepper motor backward
        """
        stepper.step(steps, 'backward')

    def calibrate(self):
        """
        Waits for input to calibrate the turret's axis
        :return:
        """
        print("Please calibrate the tilt of the gun so that it is level. Commands: (w) moves up, "
              "(s) moves down. Press (enter) to finish.\n")
        self.__calibrate_y_axis()

        print("Please calibrate the yaw of the gun so that it aligns with the camera. Commands: (a) moves left, "
              "(d) moves right. Press (enter) to finish.\n")
        self.__calibrate_x_axis()

        print("Calibration finished.")

    def __calibrate_x_axis(self):
        """
        Waits for input to calibrate the x axis
        :return:
        """
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    elif ch == "a":
                        if MOTOR_X_REVERSED:
                            Turret.move_backward(self.sm_x, 5)
                        else:
                            Turret.move_forward(self.sm_x, 5)
                    elif ch == "d":
                        if MOTOR_X_REVERSED:
                            Turret.move_forward(self.sm_x, 5)
                        else:
                            Turret.move_backward(self.sm_x, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def __calibrate_y_axis(self):
        """
        Waits for input to calibrate the y axis.
        :return:
        """
        with raw_mode(sys.stdin):
            try:
                while True:
                    ch = sys.stdin.read(1)
                    if not ch:
                        break

                    if ch == "w":
                        if MOTOR_Y_REVERSED:
                            Turret.move_forward(self.sm_y, 5)
                        else:
                            Turret.move_backward(self.sm_y, 5)
                    elif ch == "s":
                        if MOTOR_Y_REVERSED:
                            Turret.move_backward(self.sm_y, 5)
                        else:
                            Turret.move_forward(self.sm_y, 5)
                    elif ch == "\n":
                        break

            except (KeyboardInterrupt, EOFError):
                print("Error: Unable to calibrate turret. Exiting...")
                sys.exit(1)

    def motion_detection(self, show_video=False):
        """
        Uses the camera to move the turret. OpenCV must be configured to use this.
        :return:
        """
        VideoUtils.find_motion(self.__move_axis, show_video=show_video)

    def __move_axis(self, contour, frame):
        """
        Move the turret based on detected motion
        """
        (v_h, v_w) = frame.shape[:2]
        (x, y, w, h) = cv2.boundingRect(contour)

        # find height
        target_steps_x = (2*MAX_STEPS_X * (x + w / 2) / v_w) - MAX_STEPS_X
        target_steps_y = (2*MAX_STEPS_Y*(y+h/2) / v_h) - MAX_STEPS_Y

        print("x: %s, y: %s" % (str(target_steps_x), str(target_steps_y)))
        print("current x: %s, current y: %s" % (str(self.current_x_steps), str(self.current_y_steps)))

        # Move X axis
        if (target_steps_x - self.current_x_steps) > 0:
            self.current_x_steps += 1
            if MOTOR_X_REVERSED:
                Turret.move_forward(self.sm_x, 2)
            else:
                Turret.move_backward(self.sm_x, 2)
        elif (target_steps_x - self.current_x_steps) < 0:
            self.current_x_steps -= 1
            if MOTOR_X_REVERSED:
                Turret.move_backward(self.sm_x, 2)
            else:
                Turret.move_forward(self.sm_x, 2)

        # Move Y axis
        if (target_steps_y - self.current_y_steps) > 0:
            self.current_y_steps += 1
            if MOTOR_Y_REVERSED:
                Turret.move_forward(self.sm_y, 2)
            else:
                Turret.move_backward(self.sm_y, 2)
        elif (target_steps_y - self.current_y_steps) < 0:
            self.current_y_steps -= 1
            if MOTOR_Y_REVERSED:
                Turret.move_backward(self.sm_y, 2)
            else:
                Turret.move_forward(self.sm_y, 2)

        # Center of target acquired - fire!
        if abs(target_steps_x - self.current_x_steps) <= 2 and abs(target_steps_y - self.current_y_steps) <= 2:
            self.fire()


def main():
    """
    Main function to run the turret
    """
    turret = Turret(friendly_mode=True)  # Set to False to enable firing
    
    print("Calibrating turret...")
    turret.calibrate()
    
    print("Starting motion detection...")
    turret.motion_detection(show_video=True)


if __name__ == "__main__":
    main()

