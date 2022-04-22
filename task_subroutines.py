import time

from ..Command_Hub import *
from .planner import *
from .camera_helpers import *

# goes for left, middle, or right breaker
def flip_breaker_down(curr_x):
    # # get into position (assuming we're starting ~0.2m away)
    # retract_pair_retract_solo()
    # time.sleep(0.5)
    # moveRelDistXSLOW(0.3)
    # moveRelDistXSLOW(-0.1)

    # extend_pair_retract_solo()
    # send_SKR_command(y_pos=90, z_pos=5)
    # time.sleep(3)

    # get into starting position
    retract_pair_retract_solo()
    time.sleep(0.5)

    send_SKR_command(x_pos=curr_x)
    time.sleep(10)
    
    moveRelDistX(0.3)
    moveRelDistXSLOW(-0.15)
    extend_pair_retract_solo()
    send_SKR_command(y_pos=30, z_pos=(100-44.45)) # 44.45 is the distance from the camera to the fingertip
    time.sleep(5)

    moveRelDistXSLOW(0.1)
    send_SKR_command(z_pos=0)
    time.sleep(1)
    send_SKR_command(y_pos=65)
    time.sleep(2)

    # align camera with middle of target breaker
    for _ in range(3):
        pos = get_breaker_x_in_center_of_frame()
        # print(pos)

        # Horizontal FOV is 69.4 degrees
        angle = np.deg2rad(pos / (FRAME_WIDTH / 2) * (69.4 / 2))
        # print(angle)

        dist = np.tan(angle) * 100 # mm
        curr_x -= dist

        send_SKR_command(x_pos=curr_x)
        time.sleep(1)

    # Distance from end effector to camera is 38.1 mm (left)
    # Move finger to final position
    curr_x += 43 # 38.1
    send_SKR_command(x_pos=curr_x, y_pos=90)
    time.sleep(3)

    send_SKR_command(z_pos=15)
    moveRelDistX(0.1)
    send_SKR_command(z_pos=0)

    send_SKR_command(y_pos=50)
    time.sleep(3)
    extend_pair_extend_solo()
    time.sleep(.25)
    send_SKR_command(y_pos=0)
    time.sleep(3)

    # reset all
    moveRelDistX(-0.1)
    extend_pair_retract_solo()
    send_SKR_command(y_pos=100, z_pos=5)

    moveRelDistX(-0.2)
    # wahoo?

def flip_breaker_up():
    time.sleep(1)

def open_upwards_stopcock():
    time.sleep(1)

def close_upwards_stopcock():
    time.sleep(1)

def open_towards_stopcock():
    time.sleep(1)

def close_towards_stopcock():
    time.sleep(1)