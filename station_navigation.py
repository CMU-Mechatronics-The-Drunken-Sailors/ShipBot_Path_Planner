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
        if station.task_type == 'V':
            if task[0] == 1: # spigot task
                # evaluate orientation of spigot
                angle = task[1]
                frame, spigot_orientation = analyze_frame()
                cv2.imwrite("/home/mechatronics/Desktop/Mechatronics/Command_Hub/debug_photos/station_" + station.name + ".jpg", frame)
                
                # turn spigot
                if spigot_orientation == NN_Labels.SPIGOTTOPVIEW:
                    turn_towards_spigot(angle)
                else:
                    turn_upwards_spigot(angle)

            if task[0] == 2: # rotary task
                angle = task[1]
                turn_rotary_valve(angle)

            if task[0] == 3: # stopcock task
                # evaluate orientation of stopcock
                position = task[1]
                frame, stopcock_orientation = analyze_frame()
                cv2.imwrite("/home/mechatronics/Desktop/Mechatronics/Command_Hub/debug_photos/station_" + station.name + ".jpg", frame)
                
                if position: # close stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        close_towards_stopcock()
                    else:
                        close_upwards_stopcock()
                else: # open stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        open_towards_stopcock()
                    else:
                        open_upwards_stopcock()
        if station.task_type == 'B':
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

# assume that we are at the starting position, facing .58 m away from the wall
def navigate_stations():
    # evaluate each station in order of A thru E
    for station in station_list[0:5]: 
        # check if we have something to do at this station
        print("Currently on: " + station.name)

        if len(station.task_list):
            print("Something to do here!") # get the camera in position
            send_SKR_command(x_pos=131.5, y_pos=120, z_pos=0)
            extend_pair_retract_solo()

            moveRelDistXSLOW(0.4) # get up close
            sleep(1)
            choose_task_subroutine(station) # decide what task to perform
            moveRelDistXSLOW(-0.4) # step back

        if not (station == station_list[4]):
            moveRelDistY(-0.3) # move to next station

    # rotate for remaining 3 stations
    turnRelAngle(-1.57079633) # start facing station E, then turn towards station H

    # evaluate each station in order of H -> G -> F
    for station in station_list[-1:-4:-1]:
        if len(station.task_list):
            # moveRelDistX(0.4) # already up close
            choose_task_subroutine(station) # decide what task to perform
            # moveRelDistX(-0.4) # step back

        if not (station == station_list[5]):
            moveRelDistY(0.305) # move to next station
    
    # job is done!
    print("Job's done!")