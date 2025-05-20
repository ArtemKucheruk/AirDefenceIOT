from configs.configAirDefence import MIN_ANGLE_THRESHOLD, axis_config
import math

def pixel_to_angle(pixel_x, pixel_y, image_width, image_height, fov_x, fov_y):
    center_x = image_width / 2
    center_y = image_height / 2
    angle_x = (pixel_x - center_x) * (fov_x / image_width)
    angle_y = (pixel_y - center_y) * (fov_y / image_height)
    return angle_x, angle_y



def compute_speed(error, axis='X'):
    conf = axis_config[axis]
    speed = conf['kp'] * math.tanh(error * 0.1) * conf['speed_limit']
    return speed

def apply_angle_threshold(angle_delta):
    return 0 if abs(angle_delta) < MIN_ANGLE_THRESHOLD else angle_delta



def compensate_backlash(target, current, backlash=0.1):
    if target < current:
        return target - backlash
    elif target > current:
        return target + backlash
    return target


