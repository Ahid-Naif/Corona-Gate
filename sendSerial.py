import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

ser.write('r')
time.sleep(2)

ser.write('g')
time.sleep(2)

ser.write('w')
time.sleep(2)