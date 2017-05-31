import RPi.GPIO as GPIO
pin = 26

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN)

print GPIO.input(pin)

GPIO.cleanup()
