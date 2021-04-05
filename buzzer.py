import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)

GPIO.setup(37, GPIO.OUT, initial=GPIO.LOW) 

def main():

	while True:
		buzzerOn(2)
		buzzerOff(2)

def buzzerOff(waitTime):
	GPIO.output(37, GPIO.LOW)
	#
	time.sleep(waitTime)
	#

def buzzerOn(waitTime):
	GPIO.output(37, GPIO.HIGH)
	#
	time.sleep(waitTime)
	#

if __name__ == "__main__":
	main()