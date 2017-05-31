#!/usr/bin/env python

import motion                           #from libmotors
from I2C_bus import I2C_bus             #from libAX12/pythonBinding
from starting_block import add_jack_and_delay, time_elapsed, manage_time_elapsed
# the robot is "constructed" in structure_Grobot.py
from structure_Grobot import *

from time import sleep




############################ TOP LEVEL ACTION DEFINITION #######################
# remainder: definition != execution

robot.add_sequence("main_sequence")

# IMPORTANT : programs a global stop on the raspi, ie no more actions will be done
# NOTE : this is not sufficient !!! a stop command must be send to the STM
# some cleaning must also be done: stop AX12, ...
robot.add_parallel(time_elapsed, [60, lambda: manage_time_elapsed(robot)], False)
robot.wait()


robot.add_parallel(deploy_left_ball_collector, [], False)
robot.wait()

robot.add_parallel(motion.move, [-210])
robot.wait()

robot.wait(max_delay=2.0, n_callbacks=1)
robot.add_parallel(process_balls, ["left", True])
robot.add_parallel(shake, [10], False)
robot.wait()

robot.add_parallel(motion.turn, [270])
robot.wait()
"""robot.add_parallel(robot.AX12_catapult.turn, [-100], False)
robot.wait(max_delay=10, n_callbacks=1)

robot.add_parallel(robot.AX12_catapult.turn, [0], False)
robot.wait()
"""
robot.add_parallel(launch_ball, [4], False) #TODO rajouter un callback au launch_ball
robot.wait(max_delay=15, n_callbacks=1)
robot.sequence_done()





######################## INITIALISATION AND JACK MANAGEMENT ####################

jack_pin_wpi = 10

try:
    I2C_bus.init(115200)
except Exception as e:
    print "[-] Unable to start I2C communication ("+str(e)+"), exiting"
    exit()

gpio.init()

#converts the wPi pin number to the BCM pin number
#see the output of "gpio readall" on a raspi
jack_pin_bcm = gpio.gpio_index_of_wpi_pin(jack_pin_wpi)
print "Jack pin corresponds to BCM index "+str(jack_pin_bcm)
gpio.set_pin_mode(jack_pin_bcm, gpio.INPUT) #easier to test with hand (gpio write 5 0 ou 1)
# ie we don't test with the real jack, we just simulate it
#with the real jack, it must be gpio.INPUT )

# delay = 10 = maximum time the robot waits before aborting
manage_jack = add_jack_and_delay(robot, 30)

gpio.assign_callback_on_gpio_down(jack_pin_bcm, lambda: manage_jack(False))
gpio.assign_callback_on_gpio_up(jack_pin_bcm, lambda: manage_jack(True))

robot.wait_sequence() # We wait for jack beeing pushed/pulled




########################### MAIN SEQUENCE EXECUTION ############################

robot.start_sequence("main_sequence")
robot.wait_sequence()
robot.stop()

#TODO is the following line at the right place?
gpio.join() # I don't exactly know when gpio.join() should be called
