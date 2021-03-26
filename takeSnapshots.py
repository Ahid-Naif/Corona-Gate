import cv2

video = cv2.VideoCapture(0)
cameraName = "Back_Camera"
chessboardDimension = (6,9)
counter = 0 # to keep track of number of snapshots taken
while True:
    # read frames from the camera
    isGrabbed, frame = video.read() # grab frames one be one.. isGrabbed is bool that states if there's a frame grabbed or not
    if not isGrabbed:
        break

    cv2.imshow("Video", frame)
    
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    elif key == ord("s"): # snapshot will be taken when "s" is pressed
        cv2.imwrite("appImage.jpg", frame) # save original frame