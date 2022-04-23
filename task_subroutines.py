import time

from ..Command_Hub import *
from ..CV_Module import *
from .planner import *
from .camera_helpers import *

# goes for left, middle, or right breaker
def flip_breaker_down(curr_x):
    print("Flip breaker down!")
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

    send_SKR_command(x_pos=131.5, z_pos=0)
    
    # Move forward to align, then move back
    moveRelDistXSLOW(0.3)
    moveRelDistXSLOW(-0.12)

    # Extend finger
    extend_pair_retract_solo()
    send_SKR_command(y_pos=10, z_pos=(15)) # 44.45 is the distance from the camera to the fingertip

    # Align with panel again
    moveRelDistXVERYSLOW(0.1)
    moveRelDistXSLOW(-0.05)

    # Move finger back
    send_SKR_command(z_pos=0)
    send_SKR_command(x_pos=curr_x, y_pos=65)

    # align camera with middle of target breaker
    for _ in range(3):
        pos, state = get_breaker_x_in_center_of_frame()
        if state == BreakerState.DOWN or state == BreakerState.DOWN_UPSIDE_DOWN:
            # We're done. Stop here.
            moveRelDistXSLOW(-0.1)
            extend_pair_retract_solo()
            send_SKR_command(x_pos=131.5, y_pos=100, z_pos=5)
            moveRelDistX(-0.1)
            return

        # print(pos)

        # Horizontal FOV is 69.4 degrees
        angle = np.deg2rad(pos / (FRAME_WIDTH / 2) * (69.4 / 2))
        # print(angle)

        dist = np.tan(angle) * 80 # mm
        curr_x -= dist

        send_SKR_command(x_pos=curr_x)

    # Distance from end effector to camera is 38.1 mm (left)
    # Move finger to final position
    curr_x += 38.1 + 2
    send_SKR_command(x_pos=curr_x, y_pos=90)

    # Flip breaker
    send_SKR_command(z_pos=22)
    # moveRelDistXVERYSLOW(0.1)
    # send_SKR_command(z_pos=0)

    send_SKR_command(y_pos=50)
    extend_pair_extend_solo()
    time.sleep(.25)
    send_SKR_command(y_pos=0)

    # reset all
    moveRelDistXSLOW(-0.1)
    extend_pair_retract_solo()
    send_SKR_command(y_pos=100, z_pos=5)

    moveRelDistX(-0.1)
    # wahoo?

def flip_breaker_up(curr_x):
    print("Flip breaker up!")
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

    send_SKR_command(x_pos=131.5, z_pos=0)
    
    # Move forward to align, then move back
    moveRelDistXSLOW(0.3)
    moveRelDistXSLOW(-0.12)

    # Extend finger
    extend_pair_retract_solo()
    send_SKR_command(y_pos=10, z_pos=(15)) # 44.45 is the distance from the camera to the fingertip

    # Align with panel again
    moveRelDistXVERYSLOW(0.1)
    moveRelDistXSLOW(-0.05)

    # Move finger back
    send_SKR_command(z_pos=0)
    send_SKR_command(x_pos=curr_x, y_pos=65)

    # align camera with middle of target breaker
    for _ in range(3):
        pos, state = get_breaker_x_in_center_of_frame()
        if state == BreakerState.UP or state == BreakerState.UP_UPSIDE_DOWN:
            # We're done. Stop here.
            moveRelDistXSLOW(-0.1)
            extend_pair_retract_solo()
            send_SKR_command(x_pos=131.5, y_pos=100, z_pos=5)
            moveRelDistX(-0.1)
            return
        # print(pos)

        # Horizontal FOV is 69.4 degrees
        angle = np.deg2rad(pos / (FRAME_WIDTH / 2) * (69.4 / 2))
        # print(angle)

        dist = np.tan(angle) * 80 # mm
        curr_x -= dist

        send_SKR_command(x_pos=curr_x)

    # Distance from end effector to camera is 38.1 mm (left)
    # Move finger to final position
    curr_x += 38.1 + 2
    send_SKR_command(x_pos=curr_x, y_pos=40)

    # Flip breaker
    send_SKR_command(z_pos=22)
    # moveRelDistXVERYSLOW(0.1)
    # send_SKR_command(z_pos=0)

    send_SKR_command(y_pos=110)
    time.sleep(.25)

    # reset all
    moveRelDistXSLOW(-0.1)
    extend_pair_retract_solo()
    send_SKR_command(y_pos=100, z_pos=5)

    moveRelDistX(-0.1)
    # wahoo?

def open_upwards_stopcock():
    print("Open upwards stopcock!")
    time.sleep(1)

def close_upwards_stopcock():
    print("Close upwards stopcock!")
    time.sleep(1)

def open_towards_stopcock():
    print("Open towards stopcock!")
    time.sleep(1)

def close_towards_stopcock():
    print("Close towards stopcock!")
    time.sleep(1)

def turn_upwards_spigot(angle):
    print("Turn upwards spigot!")
    time.sleep(1)

def turn_towards_spigot(angle):
    print("Turn towards spigot!")
    time.sleep(1)

def turn_rotary_valve(angle):
    print("Turn rotary valve!")
    time.sleep(1)