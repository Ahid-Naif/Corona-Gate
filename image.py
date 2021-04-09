import cv2
import imutils
import numpy as np

done_image = np.zeros((400,400,3), np.uint8)
done_image[:,0:400] = (0,255,0)

cv2.putText(done_image, "Pass!", (100, 200), cv2.FONT_HERSHEY_SIMPLEX,
			2, (255, 255, 255), 2)

cv2.imshow('Pass', done_image)
cv2.waitKey(0   )