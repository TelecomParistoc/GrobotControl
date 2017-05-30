# -*- coding: utf-8 -*-
from robot import Robot		#robot is an installed package
import motordriver

from AX12 import AX12
import gpio


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


# TODO : la couleur importe ici ! a prendre en compte
def deploy_ball_collector():
    robot.AX12_ball_collector.move_to((0, 0)) #TODO valeurs à préciser

def close_ball_collector():
    robot.AX12_ball_collector.move_to((0, 0)) #TODO valeurs à préciser

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
