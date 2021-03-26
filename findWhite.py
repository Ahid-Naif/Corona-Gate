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

bitwiseAnd = cv2.bitwise_and(masks, maskv)
bitwiseOr = cv2.bitwise_or(bitwiseAnd, maskvWhite)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11,11))
closing = cv2.morphologyEx(bitwiseOr, cv2.MORPH_CLOSE, kernel)
opening = cv2.morphologyEx(closing, cv2.MORPH_OPEN, kernel)
closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

green = cv2.bitwise_or(image, image, mask=closing)
cv2.imshow('green', green)

gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
(T, qrCode) = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY_INV)
cv2.imshow('qrCode', qrCode)

# cv2.imshow('bitwiseAnd', bitwiseAnd)
# cv2.imshow('maskvWhite', maskvWhite)
# cv2.imshow('bitwiseOr', bitwiseOr)
# cv2.imshow('closing', closing)

cv2.waitKey(0)
cv2.destroyAllWindows()