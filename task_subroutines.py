import time
import numpy as np

from ..Command_Hub import *
from ..CV_Module import *
from .planner import *
from .camera_helpers import *

# goes for left, middle, or right breaker
def flip_breaker_down(curr_x):
    print("Flip breaker down!")

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
    
    # Align with wall
    retract_pair_retract_solo()
    time.sleep(0.5)
    moveRelDistXSLOW(0.4)
    moveRelDistXSLOW(-0.15)

    # Extend finger to start pos (wait at least 1.5s so that the actuator is fully down)
    extend_pair_extend_solo()
    startTime = time.time()
    curr_x, curr_y = 131.5, 20
    send_SKR_command(x_pos=curr_x, y_pos=215, z_pos=curr_y)
    time.sleep(max(0, 1.5 - (time.time() - startTime)))

    # Center with valve
    max_refines = 3
    for _ in range(5):
        if max_refines <= 0:
            break

        x, y, rot = get_stopcock_pos()
        print(f"Detected stopcock rotation: {rot}")
        if x is None or y is None:
            curr_x -= 10
            curr_y += 10
            send_SKR_command(x_pos=curr_x, z_pos=curr_y)
            continue
        if rot is not None and abs(rot - np.pi / 2) > np.pi / 4:
            # We're done! Valve already in desired pose. Stop here
            moveRelDistXSLOW(-0.25)

            extend_pair_retract_solo()
            send_SKR_command(x_pos=131.5, y_pos=100, z_pos=0)
            return
        
        max_refines -= 1

        # Horizontal FOV is 69.4 degrees, VFOV is 42.5 degrees
        angle_x = np.deg2rad(x / (FRAME_WIDTH / 2) * (69.4 / 2))
        angle_y = np.deg2rad(y / (FRAME_HEIGHT / 2) * (42.5 / 2))
        # print(angle)

        dist_x = np.tan(angle_x) * 110 # mm
        dist_y = np.tan(angle_y) * 110 # mm
        curr_x -= dist_x
        curr_y -= dist_y

        send_SKR_command(x_pos=curr_x, z_pos=curr_y)

    # Move to start position
    curr_x += 90
    curr_y -= 30
    send_SKR_command(x_pos=curr_x, z_pos=curr_y, y_pos=150)
    moveRelDistXSLOW(0.1)

    # Move to goal position
    curr_x -= 85
    curr_y += 30
    send_SKR_command(x_pos=curr_x, z_pos=curr_y)

    # Move back a bit
    curr_y -= 20
    send_SKR_command(z_pos=curr_y)

    # Back out
    send_SKR_command(y_pos=215)
    moveRelDistXSLOW(-0.3)

    extend_pair_retract_solo()
    send_SKR_command(x_pos=131.5, y_pos=100, z_pos=0)

def close_upwards_stopcock():
    print("Close upwards stopcock!")
    
    # Align with wall
    retract_pair_retract_solo()
    time.sleep(0.5)
    moveRelDistXSLOW(0.4)

    # Extend finger to start pos (wait at least 1.5s so that the actuator is fully down)
    extend_pair_extend_solo()
    startTime = time.time()
    curr_x, curr_y = 70, 15
    send_SKR_command(x_pos=curr_x, y_pos=215, z_pos=curr_y)
    time.sleep(max(0, 1.5 - (time.time() - startTime)))

    # Center with valve
    max_refines = 3
    for _ in range(5):
        if max_refines <= 0:
            break

        x, y, rot = get_stopcock_pos()
        print(f"Detected stopcock rotation: {rot}")
        if x is None or y is None:
            curr_x -= 10
            curr_y += 10
            send_SKR_command(x_pos=curr_x, z_pos=curr_y)
            continue
        if rot is not None and abs(rot - np.pi / 2) < np.pi / 4:
            # We're done! Valve already in desired pose. Stop here
            moveRelDistXSLOW(-0.25)

            extend_pair_retract_solo()
            send_SKR_command(x_pos=131.5, y_pos=100, z_pos=0)
            return
        
        max_refines -= 1

        # Horizontal FOV is 69.4 degrees, VFOV is 42.5 degrees
        angle_x = np.deg2rad(x / (FRAME_WIDTH / 2) * (69.4 / 2))
        angle_y = np.deg2rad(y / (FRAME_HEIGHT / 2) * (42.5 / 2))
        # print(angle)

        dist_x = np.tan(angle_x) * 110 # mm
        dist_y = np.tan(angle_y) * 110 # mm
        curr_x -= dist_x
        curr_y -= dist_y

        send_SKR_command(x_pos=curr_x, z_pos=curr_y)

    # Move to start position
    curr_x += 70
    curr_y -= 10
    send_SKR_command(x_pos=curr_x, z_pos=curr_y)
    extend_pair_retract_solo() # Move finger forwards a bit so it doesn't hit the valve
    send_SKR_command(y_pos=150)
    extend_pair_extend_solo()
    curr_y += 60
    send_SKR_command(z_pos=curr_y)

    # Move to goal position
    curr_x += 40
    curr_y -= 60
    send_SKR_command(x_pos=curr_x, z_pos=curr_y)

    # Move back a bit
    curr_x -= 80
    curr_y -= 50
    send_SKR_command(x_pos=curr_x, z_pos=curr_y)

    # Back out
    send_SKR_command(y_pos=215)
    moveRelDistXSLOW(-0.3)

    extend_pair_retract_solo()
    send_SKR_command(x_pos=131.5, y_pos=100, z_pos=0)

def open_towards_stopcock():
    print("Open towards stopcock!")

    # Align with wall
    retract_pair_retract_solo()
    time.sleep(0.5)
    moveRelDistXSLOW(0.4)
    moveRelDistXSLOW(-0.25)

    # Extend finger to start pos (wait at least 1.5s so that the actuator is fully down)
    extend_pair_retract_solo()
    startTime = time.time()
    curr_x, curr_y = 131.5, 90
    send_SKR_command(x_pos=curr_x, y_pos=curr_y, z_pos=0)
    time.sleep(max(0, 1.5 - (time.time() - startTime)))

    # Center with valve
    max_refines = 3
    for _ in range(5):
        if max_refines <= 0:
            break

        x, y, rot = get_stopcock_pos()
        print(f"Detected stopcock rotation: {rot}")
        if x is None or y is None:
            curr_x -= 25
            curr_y -= 15
            send_SKR_command(x_pos=curr_x, y_pos=curr_y)
            continue
        if rot is not None and abs(rot - np.pi / 2) < np.pi / 4:
            # We're done! Valve already in desired pose. Stop here
            moveRelDistXSLOW(-0.15)
            return
        
        max_refines -= 1

        # Horizontal FOV is 69.4 degrees, VFOV is 42.5 degrees
        angle_x = np.deg2rad(x / (FRAME_WIDTH / 2) * (69.4 / 2))
        angle_y = np.deg2rad(y / (FRAME_HEIGHT / 2) * (42.5 / 2))
        # print(angle)

        dist_x = np.tan(angle_x) * 110 # mm
        dist_y = np.tan(angle_y) * 110 # mm
        curr_x -= dist_x
        curr_y -= dist_y

        send_SKR_command(x_pos=curr_x, y_pos=curr_y)

    # Move to start position
    moveRelDistXSLOW(0.07)
    curr_x += 10
    curr_y -= 20
    send_SKR_command(x_pos=curr_x, y_pos=curr_y, z_pos=65)

    # Move to goal position
    curr_x += 105
    curr_y += 125
    send_SKR_command(x_pos=curr_x, y_pos=curr_y)

    # Move right a bit
    curr_x -= 20
    send_SKR_command(x_pos=curr_x)

    # Back out
    send_SKR_command(z_pos=0)
    moveRelDistXSLOW(-0.2)


def close_towards_stopcock():
    print("Close towards stopcock!")

    # Align with wall
    retract_pair_retract_solo()
    time.sleep(0.5)
    moveRelDistXSLOW(0.4)
    moveRelDistXSLOW(-0.3)

    # Extend finger to start pos (wait at least 1.5s so that the actuator is fully down)
    extend_pair_retract_solo()
    startTime = time.time()
    curr_x, curr_y = 131.5, 90
    send_SKR_command(x_pos=curr_x, y_pos=curr_y, z_pos=0)
    time.sleep(max(0, 1.5 - (time.time() - startTime)))

    # Center with valve
    max_refines = 3
    for _ in range(5):
        if max_refines <= 0:
            break

        x, y, rot = get_stopcock_pos()
        print(f"Detected stopcock rotation: {rot} (x: {x}, y: {y})")
        if x is None or y is None:
            curr_x -= 5
            curr_y -= 20
            send_SKR_command(x_pos=curr_x, y_pos=curr_y)
            continue
        if rot is not None and abs((rot % np.pi) - np.pi / 2) >= np.pi / 4:
            # We're done! Valve already in desired pose. Stop here
            moveRelDistXSLOW(-0.15)
            return
        
        max_refines -= 1

        # Horizontal FOV is 69.4 degrees, VFOV is 42.5 degrees
        angle_x = np.deg2rad(x / (FRAME_WIDTH / 2) * (69.4 / 2))
        angle_y = np.deg2rad(y / (FRAME_HEIGHT / 2) * (42.5 / 2))
        # print(angle)

        dist_x = np.tan(angle_x) * 110 # mm
        dist_y = np.tan(angle_y) * 110 # mm

        # For some reason we think the valve is pointing down, which is not
        # possible. So, manually flip the adjustment here.
        print(dist_x, dist_y)

        curr_x -= dist_x
        curr_y -= dist_y

        send_SKR_command(x_pos=curr_x, y_pos=curr_y)

    # Move to start position
    moveRelDistXSLOW(0.125)
    curr_x += 70
    send_SKR_command(x_pos=curr_x)
    curr_y += 10
    send_SKR_command(y_pos=curr_y, z_pos=65)

    # Move to goal position
    curr_x -= 125
    curr_y -= 85
    send_SKR_command(x_pos=curr_x, y_pos=curr_y)

    # Move up a bit
    curr_y += 20
    send_SKR_command(y_pos=curr_y)

    # Back out
    send_SKR_command(z_pos=0)
    moveRelDistXSLOW(-0.2)

def turn_upwards_spigot(angle):
    print("Turn upwards spigot!")
    time.sleep(1)

def turn_towards_spigot(angle):
    print("Turn towards spigot!")
    time.sleep(1)

def turn_rotary_valve(angle):
    print("Turn rotary valve!")
    time.sleep(1)