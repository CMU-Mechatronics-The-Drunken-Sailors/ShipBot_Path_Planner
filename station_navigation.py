import time

from . import *
from ..Command_Hub import *
from .planner import *
from .camera_helpers import *

LEFT_BREAKER = 240
MID_BREAKER = 131.5
RIGHT_BREAKER = 60

def choose_task_subroutine(station):
    for task in station.task_list:
        if station.task_type == 'V': # valves
            if task[0] == 1: # spigot task
                # evaluate orientation of spigot
                angle = task[1]
                frame, spigot_orientation = analyze_frame(task[0])
                cv2.imwrite("/home/mechatronics/Desktop/Mechatronics/Command_Hub/debug_photos/station_" + station.name + ".jpg", frame)
                
                # turn spigot
                if spigot_orientation == NN_Labels.SPIGOTTOPVIEW:
                    turn_towards_spigot(angle)
                else: # just assume that head-on detection is better
                    turn_upwards_spigot(angle)

            if task[0] == 2: # rotary task
                angle = task[1]
                turn_rotary_valve(angle)

            if task[0] == 3: # stopcock task
                # evaluate orientation of stopcock
                position = task[1]
                frame, stopcock_orientation = analyze_frame(task[0])
                cv2.imwrite("/home/mechatronics/Desktop/Mechatronics/Command_Hub/debug_photos/station_" + station.name + ".jpg", frame)
                
                if position: # close stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        close_towards_stopcock()
                    else: # just assume that head-on detection is better
                        close_upwards_stopcock()
                else: # open stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        open_towards_stopcock()
                    else: # just assume that head-on detection is better
                        open_upwards_stopcock()
        if station.task_type == 'B': # breakers
            breaker_num = task[1]
            breaker_pos = task[2]

            # choose which breaker to flip to what position
            if breaker_num == 1:
                if breaker_pos == 'U':
                    flip_breaker_up(LEFT_BREAKER)
                elif breaker_pos == 'D':
                    flip_breaker_down(LEFT_BREAKER)
            elif breaker_num == 2:
                if breaker_pos == 'U':
                    flip_breaker_up(MID_BREAKER)
                elif breaker_pos == 'D':
                    flip_breaker_down(MID_BREAKER)
            elif breaker_num == 3:
                if breaker_pos == 'U':
                    flip_breaker_up(RIGHT_BREAKER)
                elif breaker_pos == 'D':
                    flip_breaker_down(RIGHT_BREAKER)

 # get the camera in position for task
def orient_camera():
    print("Something to do here!")
    send_SKR_command(x_pos=80, y_pos=80, z_pos=0)
    extend_pair_retract_solo()
    time.sleep(1)

# assume that we are at the starting position, facing .58 m away from the wall
def navigate_stations():
    # # Reverse
    # moveRelDistXSLOW(-0.2)
    # extend_pair_retract_solo()

    # get close to station A and perform task
    station = station_list[0]
    print("Currently on: " + station.name)
    if len(station.task_list):
        # get up close, perform task, then move back
        orient_camera()
        moveRelDistXSLOW(0.55)
        time.sleep(1)
        choose_task_subroutine(station)
        moveRelDistXSLOW(-0.4)
        time.sleep(1)
    moveRelDistYSLOW(-0.315) # move to station B

    # get close to station B and perform task
    station = station_list[1]
    print("Currently on: " + station.name)
    if len(station.task_list):
        # get up close, perform task, then move back
        orient_camera()
        moveRelDistXSLOW(0.55)
        time.sleep(1)
        choose_task_subroutine(station)
        moveRelDistXSLOW(-0.4)
        time.sleep(1)
    moveRelDistYSLOW(-0.315) # move to station C

    # get close to station C and perform task
    station = station_list[2]
    print("Currently on: " + station.name)
    if len(station.task_list):
        # get up close, perform task, then move back
        orient_camera()
        moveRelDistXSLOW(0.55)
        time.sleep(1)
        choose_task_subroutine(station)
        moveRelDistXSLOW(-0.4)
        time.sleep(1)
    moveRelDistYSLOW(-0.315) # move to station D  

    # get close to station D and perform task
    station = station_list[3]
    print("Currently on: " + station.name)
    retract_pair_retract_solo()
    moveRelDistX(0.65) # Slam against wall to align
    moveRelDistXSLOW(-0.25)
    extend_pair_retract_solo()
    time.sleep(2)
    # moveRelDistXSLOW(0.3) # move forward slightly
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine(station)
        # moveRelDistXSLOW(-0.2) # move back slightly?
    moveRelDistYSLOW(-0.315) # move to station E

    # perform task for station E, then rotate to face station F
    station = station_list[4]
    print("Currently on: " + station.name)
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine(station)
    retract_pair_retract_solo() # rotate to face station F
    moveRelDistYSLOW(0.1)
    turnRelAngle(-1.57079633)
    time.sleep(1)
    moveRelDistY(0.4) # move left toward wall and back slightly
    moveRelDistYSLOW(-0.1)
    moveRelDistXSLOW(-0.225)
    moveRelDistY(0.2)
    moveRelDistY(-0.025)
    time.sleep(1)
    resetFF(0,0,0)

    # perform task for station F
    station = station_list[5]
    print("Currently on: " + station.name)
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine(station)
    moveRelDistYSLOW(-0.3) # move over to station G
    time.sleep(1)

    # perform task for station G
    station = station_list[6]
    print("Currently on: " + station.name)
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine(station)

    # check if we need to move over to station H, in case there is a pipe at the end
    station = station_list[7]
    if not len(station.task_list): return
    moveRelDistYSLOW(-0.3) # move over to station H
    time.sleep(1)

    # perform task for station G
    print("Currently on: " + station.name)
    orient_camera()
    choose_task_subroutine(station)
    
    # wahoo!
    return