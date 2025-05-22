from configs.configAirDefence import MIN_ANGLE_THRESHOLD, axis_config
import math

def pixel_to_angle(pixel_x, pixel_y, image_width, image_height, fov_x, fov_y):
    """
    Convert pixel coordinates in the image to angular offsets (degrees)
    relative to the center of the image, using the camera's field of view.
    """
    center_x = image_width / 2
    center_y = image_height / 2
    angle_x = (pixel_x - center_x) * (fov_x / image_width)
    angle_y = (pixel_y - center_y) * (fov_y / image_height)
    return angle_x, angle_y

def compute_speed(error, axis='X'):
    """
    Compute the motor speed for a given axis based on the error (distance from target).
    Uses a proportional-derivative-like formula with tanh for smooth limiting.
    """
    conf = axis_config[axis]
    speed = conf['kp'] * math.tanh(error * 0.1) * conf['speed_limit']
    return speed

def apply_angle_threshold(angle_delta):
    """
    Apply a minimum angle threshold to avoid micro-movements.
    Returns 0 if the angle change is below the threshold.
    """
    return 0 if abs(angle_delta) < MIN_ANGLE_THRESHOLD else angle_delta

def compensate_backlash(target, current, backlash=0.1):
    """
    Compensate for mechanical backlash when changing direction.
    Adjusts the target position slightly to ensure the mechanism moves as intended.
    """
    if target < current:
        return target - backlash
    elif target > current:
        return target + backlash
    return target


