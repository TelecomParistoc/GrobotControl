# -*- coding: utf-8 -*-
from robot import Robot		#robot is an installed package
import motordriver
from AX12 import AX12
import gpio

from time import sleep
from threading import Thread
import math

from starting_block import manage_time_elapsed



############################# PARAMETERS #######################################

ROBOT_WIDTH                 = 305       # mm

LOW_TORQUE                  = 10       # in range [0, 100]
POSITION_ARM_LENGTH         = 160       # mm
TIME_TO_READ_POS_ARM        = .5        # second

CATAPULT_MEASUREMENT_PERIOD = 0.01      # seconds
CATAPULT_SPEED              = 60        # must be in range [0, 100]

# IMPORTANT: functions in gpio bcm numbers (except gpio_index_of_wpi_pin!)

#converts the wPi pin number to the BCM pin number
#see the output of "gpio readall" on a raspi
catapult_button_pin_bcm = gpio.gpio_index_of_wpi_pin    (25)
shaker_pin_bcm          = gpio.gpio_index_of_wpi_pin    (1)

color_button_pin_bcm    = gpio.gpio_index_of_wpi_pin    (22)

front_sensors_pin_list_bcm = list(map(gpio.gpio_index_of_wpi_pin,   [3, 4]))
rear_sensors_pin_list_bcm = list(map(gpio.gpio_index_of_wpi_pin,    [2, 0]))

jack_pin_bcm = gpio.gpio_index_of_wpi_pin               (10)
gpio.set_pin_mode(jack_pin_bcm, gpio.INPUT)
#gpio.OUTPUT -> easier to test with hand (gpio write 5 0 ou 1)
# ie we don't test with the real jack, we just simulate it
#with the real jack, it must be gpio.INPUT )

#####################    PIN INITIALISATION       ###############################
gpio.set_pull_up_down(  catapult_button_pin_bcm,  gpio.PULL_UP)
gpio.set_pin_mode(      catapult_button_pin_bcm,  gpio.INPUT)

gpio.set_pull_up_down(  shaker_pin_bcm,           gpio.PULL_UP)
gpio.set_pin_mode(      shaker_pin_bcm,           gpio.OUTPUT)

gpio.set_pull_up_down(  color_button_pin_bcm,     gpio.PULL_UP)
gpio.set_pin_mode(      color_button_pin_bcm,     gpio.INPUT)

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
AX12_list = [("AX12_left_ball_collector", 133),
            ("AX12_right_ball_collector", 143),
            ("AX12_catapult", 162),
            #("AX12_sorter", 144),
            #("AX12_ball_release", 142),
            ("AX12_bee_arm", 161),
            ("AX12_pos_read_left", 144),
            ("AX12_pos_read_right", 129)
            ]

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
    robot.AX12_left_ball_collector.move(50)

def deploy_right_ball_collector():
    robot.AX12_right_ball_collector.move(30)

def close_left_ball_collector():
    robot.AX12_left_ball_collector.move(130)

def close_right_ball_collector():
    robot.AX12_right_ball_collector.move(-41)

def wiggle_right_ear(nb_wiggles, period=0.5):
    for i in range(nb_wiggles):
        close_right_ball_collector()
        sleep(period)
        deploy_right_ball_collector()
        sleep(period)

def wiggle_left_ear(nb_wiggles, period=0.5):
    for i in range(nb_wiggles):
        close_left_ball_collector()
        sleep(period)
        deploy_left_ball_collector()
        sleep(period)

def wiggle_ears(nb_wiggles):
    for i in range(nb_wiggles):
        close_left_ball_collector()
        close_right_ball_collector()
        sleep(0.5)
        deploy_left_ball_collector()
        deploy_right_ball_collector()
        sleep(0.5)


def move_sorter_to_input_position(side):
    if side == "right":
        robot.AX12_sorter.move(73)
    elif side == "left":
        robot.AX12_sorter.move(-73) #TODO find a good value!!
    else:
        print "[ERROR]"

def move_sorter_to_clean_position():
    robot.AX12_sorter.move(0) #this is really the real value =D

def move_sorter_to_drop(side):
    if side == "left":
        robot.AX12_sorter.move(-70)
    else:
        robot.AX12_sorter.move(80)

def process_balls(side, all_clean, callback):
    """
        side: must be "left" or "right"
        all_clean: boolean, true if all balls are clean and thus must be launched
        callback: function to call when the process is done
    """

    #un while on a une balle serait mieux
    robot.AX12_sorter.set_speed(10)
    for i in range(3):
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

        sleep(3.)

    callback()


def wait_for_catapult_down():
    while gpio.digital_read(catapult_button_pin_bcm) == 1:
        sleep(CATAPULT_MEASUREMENT_PERIOD)

def wait_for_catapult_up():
    while gpio.digital_read(catapult_button_pin_bcm) == 0:
        sleep(CATAPULT_MEASUREMENT_PERIOD)


def launch_ball(number_of_balls, callback):

    #Start the rotation of the AX12
    robot.AX12_catapult.turn(-CATAPULT_SPEED)

    #we wait for the catapult to be in the down position
    wait_for_catapult_down()

    for i in range(number_of_balls):

        wiggle_ears(1)

        wait_for_catapult_up()
        wait_for_catapult_down()
        robot.AX12_catapult.turn(-20)
        sleep(.6)
        robot.AX12_catapult.turn(-CATAPULT_SPEED)


    robot.AX12_catapult.turn(0)
    callback()

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
    print "switching AX12 off..."
    for name, _ in AX12_list:
        getattr(robot, name).turn(0)
	getattr(robot, name).set_torque(0)
    gpio.digital_write(shaker_pin_bcm, 0)
    robot.emergency_stop()

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


def get_distance_to_left_edge(robot, angle):
    return private_get_distance_with_arm(robot.AX12_pos_read_left, -36, -43.2, angle, 1)

def get_distance_to_right_edge(robot, angle):
    return private_get_distance_with_arm(robot.AX12_pos_read_right, 128, 137.5, angle, -1)

def private_get_distance_with_arm(ax12, start_pos, straight_line, angle, sign):
    padding = 4 # take into account wood padding
    angle_offset = sign * (angle - 90)
    ax12.set_torque(LOW_TORQUE)
    ax12.move(straight_line + sign * 90)
    last_pos = -666
    while abs(last_pos - ax12.get_position()) > 1:
        last_pos = ax12.get_position()
        sleep(.2)
    last_pos = ax12.get_position()
    ax12.move(start_pos + sign * 1) # add offet to starting position to make sure it doesn't force
    ret = POSITION_ARM_LENGTH * math.sin((abs(straight_line - last_pos) - padding - angle_offset)*math.pi/180)
    print "arm distance with angle_offset = ", angle_offset, " : ", ret
    if (angle_offset != 0):
        print "arm distance  with angle_offset = 0 : ", POSITION_ARM_LENGTH*math.sin((abs(straight_line-last_pos)-padding)*math.pi/180)
    return ret

robot.add_method(get_distance_to_left_edge)
robot.add_method(get_distance_to_right_edge)
