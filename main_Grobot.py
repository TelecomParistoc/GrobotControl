#!/usr/bin/env python

import motion
import I2C_bus
from starting_block import add_jack_and_delay
# the robot is "constructed" in structure_Grobot.py
from structure_Grobot import *


############################ TOP LEVEL ACTION DEFINITION #######################
# remainder: definition != execution

robot.add_sequence("main_sequence")


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
