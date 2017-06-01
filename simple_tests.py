import motion, motordriver                           #from libmotors
from I2C_bus import I2C_bus             #from libAX12/pythonBinding
from starting_block import add_jack_and_delay, time_elapsed, manage_time_elapsed
# the robot is "constructed" in structure_Grobot.py
from structure_Grobot import *

from time import sleep

#robot.setPosition(0, 0)
robot.set_heading(0)
#robot.start_collision_detection(is_obstacle_forwards, is_obstacle_backwards)
sleep(.1)
robot.set_direction_to_wall(motion.DIR_FORWARD)
sleep(.1)
robot.set_orientation_after_wall(90)
sleep(.1)
robot.move_to_wall()

sleep(.5)

print "initial heading = ", motordriver.get_heading()
#motion.turn(90)
#robot.moveTo(400, 0, -1)

while 1:
    print "x = ", motordriver.get_pos_X()
    print "y = ", motordriver.get_pos_Y()
    print "heading =", motordriver.get_heading()
    sleep(.25)
