from ast import Num
import time

from ShipBot_Path_Planner.task_subroutines import close_towards_stopcock, close_upwards_stopcock, flip_breaker_down, flip_breaker_up, open_towards_stopcock, open_upwards_stopcock, turn_rotary_valve, turn_towards_spigot, turn_upwards_spigot

from ..Command_Hub import *
from .planner import *
from .camera_helpers import *

LEFT_BREAKER = 262
MID_BREAKER = 131.5
RIGHT_BREAKER = 75

def choose_task_subroutine(station):
    for task in station.task_list:
        if task.task_type == 'V':
            if task[0] == 1: # spigot task
                # evaluate orientation of spigot
                angle = task[1]
                spigot_orientation = analyze_frame()
                
                # turn spigot
                if stopcock_orientation == NN_Labels.SPIGOTTOPVIEW:
                    turn_upwards_spigot(angle)
                elif stopcock_orientation == NN_Labels.SPIGOTSIDEVIEW:
                    turn_towards_spigot(angle)

            if task[0] == 2: # rotary task
                angle = task[1]
                turn_rotary_valve(angle)

            if task[0] == 3: # stopcock task
                # evaluate orientation of stopcock
                position = task[1]
                stopcock_orientation = analyze_frame()
                
                if position: # close stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKSIDEVIEW:
                        close_towards_stopcock()
                    elif stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        close_upwards_stopcock()
                else: # open stopcock
                    if stopcock_orientation == NN_Labels.STOPCOCKSIDEVIEW:
                        open_towards_stopcock()
                    elif stopcock_orientation == NN_Labels.STOPCOCKTOPVIEW:
                        open_upwards_stopcock()
        if task.task_type == 'B':
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
    for station in station_list: 
        # check if we have something to do at this station
        if len(station.task_list):
            moveRelDistX(0.4) # get up close
            choose_task_subroutine(station) # decide what task to perform
            moveRelDistX(-0.4) # step back

        moveRelDistY(0.305) # move to next station

    # rotate for remaining 3 stations

    # evaluate each station in order of F thru H
    for station in station_list:
        if len(station.task_list):
            moveRelDistX(0.4) # get up close
            choose_task_subroutine(station) # decide what task to perform
            moveRelDistX(-0.4) # step back

        moveRelDistY(0.305) # move to next station
    # job is done!