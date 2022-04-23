from ast import Num
import time

from . import *
from ..Command_Hub import *
from .planner import *
from .camera_helpers import *

LEFT_BREAKER = 262
MID_BREAKER = 131.5
RIGHT_BREAKER = 75

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
    send_SKR_command(x_pos=131.5, y_pos=120, z_pos=0)
    extend_pair_retract_solo()

# assume that we are at the starting position, facing .58 m away from the wall
def navigate_stations():
    # go thru stations A to D
    for station in station_list[0:3]: 
        # check if we have something to do at this station
        print("Currently on: " + station.name)

        if len(station.task_list):
            # get up close and perform task
            moveRelDistXSLOW(0.4)
            time.sleep(1)
            choose_task_subroutine(station)

        if not (station == station_list[3]): # we've reached station D
            moveRelDistXSLOW(-0.4) # step back
            moveRelDistY(-0.3) # move to next station

    # get close to station D and perform task
    station = station[3]
    print("Currently on: " + station.name)
    moveRelDistXSLOW(0.2) # move forward slightly
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine()
    moveRelDistXSLOW(-0.2) # move back slightly
    moveRelDistY(-0.3) # move to station E

    # perform task for station E, then rotate to evaluate stations F/G/H
    station = station[4]
    print("Currently on: " + station.name)
    if len(station.task_list):
        orient_camera()
        choose_task_subroutine()
    turnRelAngle(-1.57079633) # rotate to face station F
    moveRelDistXSLOW(-0.2) # move back slightly


    # evaluate each station in order of F -> G -> H
    for station in station_list[5:8]:
        # if len(station.task_list):
            # moveRelDistX(0.4) # already up close
            # choose_task_subroutine(station) # decide what task to perform
            # moveRelDistX(-0.4) # step back

        # if not (station == station_list[5]): # last station
            # moveRelDistY(0.305) # move to next station
    
    # job is done!
    print("Job's done!")