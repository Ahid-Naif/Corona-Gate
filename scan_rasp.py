# import the necessary packages
import time
from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import time
import cv2
import re
from keras.models import load_model
import numpy as np
import serial
from smbus2 import SMBus
from mlx90614 import MLX90614
import RPi.GPIO as GPIO

bus = SMBus(1)

#
#servo
GPIO.setwarnings(False) 
GPIO.setmode(GPIO.BOARD)

GPIO.setup(11, GPIO.OUT)
pwm11=GPIO.PWM(11, 50)
pwm11.start(0)

model=load_model("./model2-005.model")

labels_dict={0:'without mask',1:'mask'}
color_dict={0:(0,0,255),1:(0,255,0)}

# We load the xml file
classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

checkForMask = False
timeIn = time.time()
waitMaskDuration = 15
num_people = 0

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()

def main():
	global checkForMask
	global timeIn
	global vs
	global ser
	global num_people

	ser.write(b"closeGate\n")
	time.sleep(2.0)

	# loop over the frames from the video stream
	while True:
		ser.write(b"buzzerOFF\n")
		ser.write(b"white\n")
		# grab the frame from the threaded video stream and resize it to
		# have a maximum width of 400 pixels
		frame = vs.read()
		frame = imutils.resize(frame, width=400)

		height, width,_ = frame.shape
		
		hsv = cv2.cvtColor(frame.copy(), cv2.COLOR_BGR2HSV)
		(h, s, v) = cv2.split(hsv)

		blurreds = cv2.GaussianBlur(s, (11, 11), 0)
		blurredv = cv2.GaussianBlur(v, (11, 11), 0)

		masks = cv2.inRange(blurreds, 230, 255)
		maskv = cv2.inRange(blurredv, 110, 150)
		maskvWhite = cv2.inRange(blurredv, 150, 255)

		bitwiseAnd = cv2.bitwise_and(masks, maskv)
		bitwiseOr = cv2.bitwise_or(bitwiseAnd, maskvWhite)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
		closing = cv2.morphologyEx(bitwiseOr, cv2.MORPH_CLOSE, kernel)
		opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
		closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

		green = cv2.bitwise_or(frame, frame, mask=closing)
		gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
		(T, qrCode) = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)
		barcodes = pyzbar.decode(qrCode)
		# loop over the detected barcodes
		for barcode in barcodes:
			# extract the bounding box location of the barcode and draw the
			# bounding box surrounding the barcode on the image
			(x, y, w, h) = barcode.rect
			cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
			# the barcode data is a bytes object so if we want to draw it on
			# our output image we need to convert it to a string first
			barcodeData = barcode.data.decode("utf-8")
			barcodeType = barcode.type
			# draw the barcode data and barcode type on the image
			text = "{}".format(barcodeData)
			cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 0, 255), 2)

			regex_result = re.findall("[\w\d\.\*\\=*?*+*-*/*]", barcodeData)

			if len(regex_result) == 44:
				print("[INFO] QR Code Pass")
				checkForMask = True
		
		if checkForMask:
			checkForMask = False
			cv2.destroyAllWindows()

			ser.write(b"buzzerON\n")
			time.sleep(2)
			timeIn = time.time()
			maskCode()

		cv2.putText(frame, "QR Scan", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 255, 255), 2)
		cv2.putText(frame, "#{}".format(num_people), (width - 60, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 255, 255), 2)
		cv2.imshow('QR Code Scanner', frame)
		key = cv2.waitKey(1) & 0xFF

 
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			print("[INFO] cleaning up...")
			cv2.destroyAllWindows()
			vs.stop()
			break

def maskCode():
	global timeIn
	global waitMaskDuration
	global vs
	global ser
	global num_people

	size = 4
	masktime = 0
	isMaskOn = False
	isBreak = False

	sensor = MLX90614(bus, address=0x5A)

	cv2.destroyAllWindows()

	while True:
		ser.write(b"buzzerOFF\n")
		ser.write(b"white\n")
		if time.time() - timeIn > waitMaskDuration:
			cv2.destroyAllWindows()
			# red & buzzer
			ser.write(b"buzzerON_red\n")
			time.sleep(2)
			break

		frame = vs.read()
		frame = imutils.resize(frame, width=400)

		height, width,_ = frame.shape

		frame = cv2.flip(frame,1,1) #Flip to act as a mirror

		# Resize the image to speed up detection
		mini = cv2.resize(frame, (frame.shape[1] // size, frame.shape[0] // size))

		# detect MultiScale / faces 
		faces = classifier.detectMultiScale(mini)

		# Draw rectangles around each face
		for f in faces:
			(x, y, w, h) = [v * size for v in f] #Scale the shapesize backup
			#Save just the rectangle faces in SubRecFaces

			face_img = frame[y:y+h, x:x+w]
			resized = cv2.resize(face_img,(150,150))
			normalized = resized / 255.0
			reshaped = np.reshape(normalized,(1,150,150,3))
			reshaped = np.vstack([reshaped])
			result = model.predict(reshaped)
			
			label = np.argmax(result,axis=1)[0]

			print(labels_dict[label])
			
			if isMaskOn == False:
				if(labels_dict[label] == 'mask'):
					isMaskOn = True
					maskTime = time.time()
			else:
				if time.time() - maskTime > 1:
					if(labels_dict[label] == 'mask' and (sensor.get_object_1() > 27 or sensor.get_object_1() < 40) ):
						isMaskOn = False
						isBreak = True
						num_people = num_people + 1
						ser.write(b"openGate\n")
						ser.write(b"buzzerON\n")
						ser.write(b"green\n")
						time.sleep(10)
						ser.write(b"closeGate\n")
						break
					else:
						isMaskOn = False
					
					
			cv2.rectangle(frame,(x,y),(x+w,y+h),color_dict[label],2)
			cv2.rectangle(frame,(x,y-40),(x+w,y),color_dict[label],-1)
			cv2.putText(frame, labels_dict[label], (x, y-10),cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)
		
		# Show the image
		cv2.putText(frame, "Mask Scan", (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 255, 255), 2)
		cv2.putText(frame, "#{}".format(num_people), (width - 60, height - 10), cv2.FONT_HERSHEY_SIMPLEX,
			1, (0, 255, 255), 2)
		cv2.imshow('Mask Detection', frame)
		key = cv2.waitKey(1) & 0xFF
		# if the `q` key was pressed, break from the loop
		if key == ord("q"):
			print("[INFO] cleaning up...")
			cv2.destroyAllWindows()
			vs.stop()
			break
			
		if isBreak == True:
			print("[INFO] cleaning up...")
			cv2.destroyAllWindows()
			break
		

if __name__ == "__main__":
	main()