import numpy as np
import cv2 as cv
  
  
# Capturing video through webcam
obs_camera = cv.VideoCapture(2)
obs_camera.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
obs_camera.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

# Set range for teams
robot_low  = (80, 50, 80)
robot_high = (140, 255, 255)
# Limits for teams
t1_color = [(130, 50, 50), (160, 255, 255)]     # Purple ish
t2_color = [(80, 50, 60), (100, 255, 255)]      # Yellow-ish
teams_colors = [t1_color, t2_color]

def robot_present(img, robot, threshold=0.8):
    total = np.float(np.sum(img))
    if total/img.size  > (1-threshold):
        return True
    else:
        return False


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
    robots_img = cv.bitwise_and(imageFrame, imageFrame, mask = robot_mask)
    
    # Find individual robots
    robots_contours, _      = cv.findContours(robot_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    robots_contours_poly    = [None]*len(robots_contours)
    robots_boundRect        = [None]*len(robots_contours)
    robots_centers          = [None]*len(robots_contours)
    robots_radius           = [None]*len(robots_contours)
    for i, c in enumerate(robots_contours):
        robots_contours_poly[i] = cv.approxPolyDP(c, 3, True)
        robots_boundRect[i] = cv.boundingRect(robots_contours_poly[i])
        robots_centers[i], robots_radius[i] = cv.minEnclosingCircle(robots_contours_poly[i])

    robots_cropped = [None]*len(robots_contours)
    
    # RoI of individual robots
    for i, c in enumerate(robots_contours):
        robots_cropped[i] = robots_img[robots_boundRect[i][1]:robots_boundRect[i][1] + robots_boundRect[i][3], robots_boundRect[i][0]:robots_boundRect[i][0] + robots_boundRect[i][2]].copy()
    
    t1_robots = []
    t2_robots = []

    for i, c in enumerate(robots_cropped):
        img_hsv = cv.cvtColor(robots_cropped[i], cv.COLOR_RGB2HSV)
        # Apply color filter of teams to determine robot team
        for j in range(len(teams_colors)):
            mask = cv.inRange(img_hsv, teams_colors[j][0], teams_colors[j][1]) 
            result = cv.bitwise_and(robots_cropped[i], robots_cropped[i], mask=mask)
            # Classify by teams
            if robot_present(result, i):
                if j == 0:
                    t1_robots.append(i)
                    cv.putText(robots_img, "T1", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (248, 65, 209), 2)
                else:
                    t2_robots.append(i)
                    cv.putText(robots_img, "T2", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (57, 242, 248), 2) # BGR

    print(" {} {}".format(t1_robots, t2_robots))
    
    

    # Program Termination
    cv.imshow("Robots", robots_img)

    # print("T1 -  R1 {} R2 {} R3 {}    T2 -  R1 {} R2 {} R3 {}".format(
    #     robots_centers[t1_robots[0]], robots_centers[t1_robots[1]], robots_centers[t1_robots[2]],
    #     robots_centers[t2_robots[0]], robots_centers[t2_robots[1]], robots_centers[t2_robots[2]]
    #     ))
    if cv.waitKey(10) & 0xFF == ord('q'):
        obs_camera.release()
        cv.destroyAllWindows()
        break