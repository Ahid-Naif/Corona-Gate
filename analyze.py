# import the necessary packages
import imutils
import cv2
from matplotlib import pyplot as plt

image = cv2.imread('appImage.jpg')
image = imutils.resize(image, width=600)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
(h, s, v) = cv2.split(hsv)

blurreds = cv2.GaussianBlur(s, (11, 11), 0)
blurredv = cv2.GaussianBlur(v, (11, 11), 0)

# cv2.imshow('blurreds', blurreds)
cv2.imshow('blurredv', blurredv)

# cv2.imshow('blurred', blurred)
	
# construct a grayscale histogram
hist = cv2.calcHist([blurredv], [0], None, [256], [0, 256])
# normalize the histogram
hist /= hist.sum()

# plot
plt.figure()
plt.title("Grayscale Histogram")
plt.xlabel("Bins")
plt.ylabel("# of Pixels")
plt.plot(hist)
plt.xlim([0, 256])
plt.show()

cv2.waitKey(0)
print("[INFO] cleaning up...")
cv2.destroyAllWindows()