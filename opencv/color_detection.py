import numpy as np
import cv2 as cv
  
  
# Capturing video through webcam
obs_camera = cv.VideoCapture(2)
obs_camera.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
obs_camera.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Set range for teams
robot_low  = (80, 50, 80)
robot_high = (140, 255, 255)

if not obs_camera.isOpened():
        print("Cannot open camera")
        exit()
    
# Start a while loop
while True:
      
    # Reading the video from the
    # webcam in image frames
    ret, imageFrame = obs_camera.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Convert To HSV
    hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
    # Create Robot Mask
    robot_mask = cv.inRange(hsvFrame, robot_low, robot_high) 
    # Fill holes
    contours, hierarchy = cv.findContours(robot_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        cv.fillPoly(robot_mask, pts=[contour], color=(255, 255, 255))
    # Apply mask   
    robots = cv.bitwise_and(imageFrame, imageFrame, mask = robot_mask)
    
    # Program Termination
    cv.imshow("Multiple Color Detection in Real-TIme", robots)
    if cv.waitKey(10) & 0xFF == ord('q'):
        obs_camera.release()
        cv.destroyAllWindows()
        break