import motion, motordriver                           #from libmotors
from I2C_bus import I2C_bus             #from libAX12/pythonBinding
from starting_block import add_jack_and_delay, time_elapsed, manage_time_elapsed
# the robot is "constructed" in structure_Grobot.py
from structure_Grobot import *
import paths

from time import sleep

robot.setPosition(0, 0)
robot.set_heading(0)
#robot.start_collision_detection(is_obstacle_forwards, is_obstacle_backwards)

#motion.turn(180)
#robot.moveTo(100, 0, -1)
while 1:
    sleep(.5)
    print "x = ", motordriver.get_pos_X()
    print "y = ", motordriver.get_pos_Y()
