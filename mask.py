# import the necessary packages
import imutils
import cv2

image = cv2.imread('appImage.jpg')
image = imutils.resize(image, width=600)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
(h, s, v) = cv2.split(hsv)

blurreds = cv2.GaussianBlur(s, (11, 11), 0)
blurredv = cv2.GaussianBlur(v, (11, 11), 0)

masks = cv2.inRange(blurreds, 230, 255)
maskv = cv2.inRange(blurredv, 110, 150)
maskvWhite = cv2.inRange(blurredv, 150, 255)

# cv2.imshow('masks', masks)
# cv2.imshow('maskv', maskv)
cv2.imshow('maskvWhite', maskvWhite)

cv2.waitKey(0)
cv2.destroyAllWindows()