import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(8, GPIO.IN)

print GPIO.input(8)

GPIO.cleanup()
