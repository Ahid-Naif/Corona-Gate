import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

ser.write('r'.encode())
time.sleep(2)

ser.write('g'.encode())
time.sleep(2)

ser.write('w'.encode())
time.sleep(2)