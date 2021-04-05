import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)

GPIO.setup(38, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW) 
GPIO.setup(32, GPIO.OUT, initial=GPIO.LOW) 


def main():

	while True:
		red(2)
		green(2)
		white(2)

def red(waitTime):
	GPIO.output(38, GPIO.HIGH)
	GPIO.output(36, GPIO.LOW)
	GPIO.output(32, GPIO.LOW)
	#
	time.sleep(waitTime)
	#

def green(waitTime):
	GPIO.output(38, GPIO.LOW)
	GPIO.output(36, GPIO.HIGH)
	GPIO.output(32, GPIO.LOW)
	#
	time.sleep(waitTime)
	#

def white(waitTime):
	GPIO.output(38, GPIO.HIGH)
	GPIO.output(36, GPIO.HIGH)
	GPIO.output(32, GPIO.HIGH)
	#
	time.sleep(waitTime)
	#

if __name__ == "__main__":
	main()