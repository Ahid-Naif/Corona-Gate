import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
pwm11=GPIO.PWM(11, 50)
pwm11.start(0)

def main():

    while True:
        closeGate(5)
        openGate(5)

def closeGate(waitTime):
    #
    angle = 100
    duty = angle / 18 + 2
    GPIO.output(11, True)
    pwm11.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(11, False)
    pwm11.ChangeDutyCycle(0)
    #
    time.sleep(waitTime)
    #

def openGate(waitTime):
    angle = 180
    duty = angle / 18 + 2
    GPIO.output(11, True)
    pwm11.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(11, False)
    pwm11.ChangeDutyCycle(0)
    #
    time.sleep(waitTime)
    #

if __name__ == "__main__":
	main()