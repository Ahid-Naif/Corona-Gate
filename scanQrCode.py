# import the necessary packages
from imutils.video import VideoStream
from pyzbar import pyzbar
import imutils
import time
import cv2

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it to
	# have a maximum width of 400 pixels
	frame = vs.read()
	frame = imutils.resize(frame, width=600)
	
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
		# print the barcode type and data to the terminal
		print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))

	cv2.imshow('QR Code Scanner', frame)
	key = cv2.waitKey(1) & 0xFF
 
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

print("[INFO] cleaning up...")
cv2.destroyAllWindows()
vs.stop()