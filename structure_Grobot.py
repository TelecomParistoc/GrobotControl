from robot import Robot		#robot is an installed package

#from utils import AX12, moving_interface  #This part is temporary : AX12 and Moving_Interface will be packages in a near future

#this file should import moving_interface, but moving_interface is not yet implemented !!
#from utils import moving_interface 	#This line is temporary

import motordriver

from AX12 import AX12
import gpio

catapult_button_pin = 0

def init_catapult_button():
    catapult_button_pin = gpio.gpio_index_of_wpi_pin(26) #TODO check pin value (the selected value correponds to the physical pin 30)
    gpio.set_pin_mode(catapult_button_pin, gpio.INPUT)

def catapult_button():
    read_value = digital_read(catapult_button_pin)
    return read_value != 0

def deploy_ball_collector(robot):
    robot.AX12_ball_collector.move_to((0, 0)) #TODO valeurs à préciser

def close_ball_collector(robot):
    robot.AX12_ball_collector.move_to((0, 0)) #TODO valeurs à préciser

def eject_ball(robot):

    #Start the rotation of the AX12
    robot.AX12_catapult.turn(0) #TODO vitesse à préciser

    #Turn the AX12 until the button is pushed and released
    #TODO use callbacks
    while(not catapult_button()):
        #add delay between checks?
    while(catapult_button()):
        #add delay between checks?

    #Stop the rotation of the AX12
    robot.AX12_catapult.turn(0)
