# -*- coding: utf-8 -*-
from robot import Robot		#robot is an installed package
import motordriver

from AX12 import AX12
import gpio

from time import sleep

############################# PARAMETERS #######################################

catapult_button_pin = 0



########################## CONSTRUCTION OF THE ROBOT ###########################

robot = Robot()
robot.add_object(AX12(143), "AX12_left_ball_collector")
robot.add_object(AX12(162), "AX12_catapult")
robot.add_object(AX12(144), "AX12_sorter")
robot.add_object(AX12(142), "AX12_ball_release")


######################### ACTION FUNCTIONS #####################################

def init_catapult_button():
    catapult_button_pin = gpio.gpio_index_of_wpi_pin(26) #TODO check pin value (the selected value correponds to the physical pin 30)
    gpio.set_pin_mode(catapult_button_pin, gpio.INPUT)

def catapult_button():
    read_value = digital_read(catapult_button_pin)
    return read_value != 0


def deploy_left_ball_collector():
    robot.AX12_left_ball_collector.move(30)

def close_left_ball_collector():
    robot.AX12_ball_collector.move(-41)

def move_sorter_to_input_position(side):
    if side == "left":
        robot.AX12_sorter.move(73)
    elif side == "right":
        robot.AX12_sorter.move(666) #TODO find a good value!!
    else:
        print "[ERROR]"

def move_sorter_to_clean_position():
    robot.AX12_sorter.move(0) #this is really the real value =D

def move_sorter_to_drop(side):
    if side == "left":
        robot.AX12_sorter.move(126)

def process_balls(side, all_clean, callback):
    """
        side: must be "left" or "right"
        all_clean: boolean, true if all balls are clean and thus must be launched
        callback: function to call when the process is done
    """

    #un while on a une balle serait mieux
    for i in range(7):
        move_sorter_to_input_position(side)

        #wait for a ball
        #must be improved
        sleep(1.)
        if all_clean:
            move_sorter_to_clean_position()

        else:
            #check color and make a decision
            pass

        sleep(1.)

    callback()


def eject_ball():

    #Start the rotation of the AX12
    robot.AX12_catapult.turn(0) #TODO vitesse à préciser

    #Turn the AX12 until the button is pushed and released
    #TODO use callbacks
    while(not catapult_button()):
        #add delay between checks?
        pass

    while(catapult_button()):
        #add delay between checks?
        pass

    #Stop the rotation of the AX12
    robot.AX12_catapult.turn(0)
