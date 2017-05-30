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
robot.add_parallel(time_elapsed, [10, lambda: manage_time_elapsed(robot)], False)
robot.wait()

robot.add_parallel(lambda a: sleep(20), []) #bidon juste pour etre bloquant
robot.wait()

robot.sequence_done()





######################## INITIALISATION AND JACK MANAGEMENT ####################
try:
    I2C_bus.init(115200)
except Exception as e:
    print "[-] Unable to start I2C communication ("+str(e)+"), exiting"
    exit()

gpio.init()

#converts the wPi pin number to the BCM pin number
#see the output of "gpio readall" on a raspi
jack_pin = gpio.gpio_index_of_wpi_pin(5)
print "Jack pin corresponds to BCM index "+str(jack_pin)
gpio.set_pin_mode(jack_pin, gpio.OUTPUT) #easier to test with hand (gpio write 5 0 ou 1)
# ie we don't test with the real jack, we just simulate it
#with the real jack, it must be gpio.INPUT )

# delay = 10 = maximum time the robot waits before aborting
manage_jack = add_jack_and_delay(robot, 30)

gpio.assign_callback_on_gpio_down(24, lambda: manage_jack(False))
gpio.assign_callback_on_gpio_up(24, lambda: manage_jack(True))

robot.wait_sequence() # We wait for jack beeing pushed/pulled




########################### MAIN SEQUENCE EXECUTION ############################

robot.start_sequence("main_sequence")
robot.wait_sequence()
robot.stop()

#TODO is the following line at the right place?
gpio.join() # I don't exactly know when gpio.join() should be called
