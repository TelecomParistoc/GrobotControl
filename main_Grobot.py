#!/usr/bin/env python

import motion                           #from libmotors
from starting_block import add_jack_and_delay, time_elapsed, manage_time_elapsed
# the robot is "constructed" in structure_Grobot.py
from structure_Grobot import *

from time import sleep


STARTING_POINT = None #set in init_color()
STARTING_HEADING = None

def init_color(robot):
    global STARTING_POINT, STARTING_HEADING
    robot.color = get_team_color()
    if(robot.color == "orange"):
        print "robot is orange: ", STARTING_POINT
        STARTING_POINT = [50, 155]
        STARTING_HEADING = 0
    else:
        STARTING_POINT = [2950, 155]
        STARTING_HEADING = 180


########################  JACK MANAGEMENT ####################

# delay = 10 = maximum time the robot waits before aborting
manage_jack = add_jack_and_delay(robot, 666)

gpio.assign_callback_on_gpio_down(jack_pin_bcm, lambda: manage_jack(False))
gpio.assign_callback_on_gpio_up(jack_pin_bcm, lambda: manage_jack(True))

robot.wait_sequence() # We wait for jack beeing pushed/pulled


############################ TOP LEVEL ACTION DEFINITION #######################
# remainder: definition != execution



def log():
    print "look at me mom", robot.color


init_color(robot)

robot.add_sequence("main_sequence")

# IMPORTANT : programs a global stop on the raspi, ie no more actions will be done
# NOTE : this is not sufficient !!! a stop command must be send to the STM
# some cleaning must also be done: stop AX12, ...

robot.add_parallel(log, [], False)
robot.add_parallel(time_elapsed, [100, grobot_time_elapsed], False)

#warning: arguments must been known when we add_parallel a function!
#so we don't write: robot.add_parallel(robot.setPosition, STARTING_POINT, False)
#but
robot.add_parallel(lambda: robot.setPosition(*STARTING_POINT), [], False)
robot.add_parallel(lambda: robot.set_heading(STARTING_HEADING), [], False)
#robot.add_parallel(robot.start_collision_detection, [is_obstacle_forwards, is_obstacle_backwards], False)
robot.wait()

robot.load_add_path("paths/starting_point_to_bee.json")


#robot.add_parallel(deploy_left_ball_collector, [], False)
#robot.wait()

#robot.add_parallel(robot.moveTo, [-210, 0, 0])
#robot.wait()

#robot.wait(max_delay=2.0, n_callbacks=1)

#robot.add_parallel(log, [], False)
"""robot.add_parallel_thread(process_balls, ["right", True])
robot.add_parallel_thread(shake, [10], False)
robot.wait()
"""
"""
robot.add_parallel(log, [], False)
robot.add_parallel(robot.move, [-100])
robot.wait()
robot.add_parallel(robot.turn, [270])
robot.wait()
"""

robot.add_parallel(launch_ball, [4], False) #TODO rajouter un callback au launch_ball
robot.wait(max_delay=15, n_callbacks=1)
robot.sequence_done()




########################### MAIN SEQUENCE EXECUTION ############################

robot.start_sequence("main_sequence")
robot.wait_sequence()
robot.stop()

#TODO is the following line at the right place?
gpio.join() # I don't exactly know when gpio.join() should be called
