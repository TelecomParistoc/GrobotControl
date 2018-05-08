from I2C_bus import I2C_bus             #from libAX12/pythonBinding
import gpio
from threading import Thread
import sys, traceback

pin_red_led_wpi     = 11
pin_orange_led_wpi  = 30
pin_green_led_wpi   = 31

try:
    I2C_bus.init(115200)
except Exception as e:
    print "[-] Unable to start I2C communication ("+str(e)+"), exiting"
    exit()

gpio.init()

for ind in [pin_green_led_wpi, pin_red_led_wpi, pin_orange_led_wpi]:
    gpio.set_pin_mode(gpio.gpio_index_of_wpi_pin(ind), gpio.OUTPUT)


def set_led(wpi_index, value):
    #value must be 0 or 1
    gpio.digital_write(gpio.gpio_index_of_wpi_pin(wpi_index), value)

def turn_green_on():
    set_led(pin_green_led_wpi, 1)

def turn_orange_on():
    set_led(pin_orange_led_wpi, 1)

set_led(pin_red_led_wpi, 0)
set_led(pin_green_led_wpi, 0)

def red_led_manager(exception_type, exception_value, exception_traceback):
    print "ERROR detected! Switching the red LED on..."
    set_led(pin_red_led_wpi, 1)
    set_led(pin_green_led_wpi, 0)
    set_led(pin_orange_led_wpi, 0)
    traceback.print_exception(exception_type, exception_value, exception_traceback)
    print "Exiting exception's thread..."
    exit(-1)

sys.excepthook = red_led_manager
