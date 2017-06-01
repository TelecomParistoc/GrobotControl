# -*- coding: utf-8 -*-
from robot import Robot		#robot is an installed package
import motordriver
from I2C_bus import I2C_bus             #from libAX12/pythonBinding
from AX12 import AX12
import gpio

from time import sleep
from threading import Thread

from starting_block import manage_time_elapsed

try:
    I2C_bus.init(115200)
except Exception as e:
    print "[-] Unable to start I2C communication ("+str(e)+"), exiting"
    exit()

gpio.init()

############################# PARAMETERS #######################################

CATAPULT_MEASUREMENT_PERIOD = 0.01 #seconds
CATAPULT_SPEED = 60 #must be in range [0, 100]

# IMPORTANT: functions in gpio bcm numbers (except gpio_index_of_wpi_pin!)

#converts the wPi pin number to the BCM pin number
#see the output of "gpio readall" on a raspi
catapult_button_pin_bcm = gpio.gpio_index_of_wpi_pin(25)
shaker_pin_bcm          = gpio.gpio_index_of_wpi_pin(1)

color_button_pin_bcm    = gpio.gpio_index_of_wpi_pin(22)

front_sensors_pin_list_bcm = list(map(gpio.gpio_index_of_wpi_pin, [3, 4]))
rear_sensors_pin_list_bcm = list(map(gpio.gpio_index_of_wpi_pin, [2, 0]))

jack_pin_bcm = gpio.gpio_index_of_wpi_pin(10)
gpio.set_pin_mode(jack_pin_bcm, gpio.INPUT)
#gpio.OUTPUT -> easier to test with hand (gpio write 5 0 ou 1)
# ie we don't test with the real jack, we just simulate it
#with the real jack, it must be gpio.INPUT )

#####################    PIN INITIALISATION       ###############################
gpio.set_pull_up_down(catapult_button_pin_bcm, gpio.PULL_UP)
gpio.set_pin_mode(catapult_button_pin_bcm, gpio.INPUT)

gpio.set_pull_up_down(shaker_pin_bcm, gpio.PULL_UP)
gpio.set_pin_mode(shaker_pin_bcm, gpio.OUTPUT)

gpio.set_pull_up_down(color_button_pin_bcm, gpio.PULL_UP)
gpio.set_pin_mode(color_button_pin_bcm, gpio.INPUT)

for pin in front_sensors_pin_list_bcm + rear_sensors_pin_list_bcm:
    gpio.set_pull_up_down(pin, gpio.PULL_DOWN)
    gpio.set_pin_mode(pin, gpio.INPUT)

def get_team_color():
    if gpio.digital_read(color_button_pin_bcm) == 1:
        return "orange"
    else:
        return "green"


##################     CONSTRUCTION OF THE ROBOT    ############################

#contains the (name, id) of the used AX12
AX12_list = [("AX12_left_ball_collector", 143),
            ("AX12_catapult", 162),
            ("AX12_sorter", 144),
            ("AX12_ball_release", 142),
            ("AX12_bee_arm", 161)]

robot = Robot()
for name, id in AX12_list:
    robot.add_object(AX12(id), name)

###################     ACTION FUNCTIONS    ####################################

def is_obstacle_forwards():
    for sensor in front_sensors_pin_list_bcm:
        if gpio.digital_read(sensor) == 1:
            return True
    return False

def is_obstacle_backwards():
    for sensor in rear_sensors_pin_list_bcm:
        if gpio.digital_read(sensor) == 1:
            return True
    return False

def deploy_left_ball_collector():
    robot.AX12_left_ball_collector.move(30)

def close_left_ball_collector():
    robot.AX12_ball_collector.move(-41)

def move_sorter_to_input_position(side):
    if side == "left":
        robot.AX12_sorter.move(73)
    elif side == "right":
        robot.AX12_sorter.move(-73) #TODO find a good value!!
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
    robot.AX12_sorter.set_speed(10)
    for i in range(7):
        move_sorter_to_input_position(side)

        #wait for a ball
        #must be improved
        sleep(3.)
        if all_clean:
            move_sorter_to_clean_position()

        else:
            #check color and make a decision
            #to check color, read gpio pins 18 and 22 (physical numbers)
            pass

        sleep(1.)

    callback()


def wait_for_catapult_down():
    while gpio.digital_read(catapult_button_pin_bcm) == 1:
        sleep(CATAPULT_MEASUREMENT_PERIOD)

def wait_for_catapult_up():
    while gpio.digital_read(catapult_button_pin_bcm) == 0:
        sleep(CATAPULT_MEASUREMENT_PERIOD)


def launch_ball(number_of_balls=1):

    #Start the rotation of the AX12
    robot.AX12_catapult.turn(-CATAPULT_SPEED)

    #we wait for the catapult to be in the down position
    wait_for_catapult_down()

    for i in range(number_of_balls):

        wait_for_catapult_up()
        wait_for_catapult_down()

    robot.AX12_catapult.turn(0)

def shake(duration=2.):
    #duration is in seconds

    #we don't use pwm
    gpio.digital_write(shaker_pin_bcm, 1)
    sleep(duration)
    gpio.digital_write(shaker_pin_bcm, 0)

def grobot_time_elapsed():
    '''
    called when the time is over to stop Grobot from moving
    '''
    for name, _ in AX12_list:
        robot.getattr(name).turn(0)
    robot.emergency_stop()
    manage_time_elapsed(robot)

def init_bee_arm(robot):
    #make sure the robot has enough space to move its arm
    if robot.color == "green":
        robot.AX12_bee_arm.move(90)
    elif robot.color == "orange":
        robot.AX12_bee_arm.move(-90)
    else:
        print "ERROR in init_bee_arm - unknown color"

def push_bee(robot):
    if robot.color == "green":
        robot.AX12_bee_arm.move(-90)
    elif robot.color == "orange":
        robot.AX12_bee_arm.move(90)
    else:
        print "ERROR in push_bee - unknown color"
        return
    sleep(1.)

robot.add_method(init_bee_arm)
robot.add_method(push_bee)
