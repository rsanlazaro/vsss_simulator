import numpy as np
import cv2 as cv

########            COLORS            ########
# Limit for robot color
robot_color     = [(80, 50, 80), (140, 255, 255)]
# Limits for team colors
t1_color        = [(130, 50, 50), (160, 255, 255)]     # Purple ish
t2_color        = [(80, 50, 60), (100, 255, 255)]      # Yellow-ish
teams_colors    = [t1_color, t2_color]
# Limits for role colors
goal_color      = [(0, 0, 0), (255, 255, 255)]       # Red-ish
mid_color       = goal_color     # Green-ish
for_color       = goal_color    # Blue-ish

# goal_color      = [(156, 0, 0), (200, 255, 255)]       # Red-ish
# mid_color       = [(50, 150, 200), (70, 255, 255)]     # Green-ish
# for_color       = [(110, 100, 60), (125, 255, 255)]    # Blue-ish

role_colors     = [goal_color, mid_color, for_color]

########            DETECTION INFORMATION            ########
t1_info = {
    "G": [(0, 0), 0],
    "F": [(0, 0), 0],
    "M": [(0, 0), 0],
    "B": (0, 0)
}
t2_info = {
    "G": [(0, 0), 0],
    "F": [(0, 0), 0],
    "M": [(0, 0), 0],
    "B": (0, 0)
}

########            STREAM PARAMETERS            ########
obs_camera = cv.VideoCapture(2)
obs_camera.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
obs_camera.set(cv.CAP_PROP_FRAME_HEIGHT, 720)



def robot_present(img, threshold=0.8):
    total = np.float(np.sum(img))
    if total/img.size  > (1-threshold):
        return True
    else:
        return False

def get_role(img_hsv, robot_cropped, robot):
    # counter = 1
    # plt.figure(figsize=(20, 20))
    for j in range(len(role_colors)):
        mask = cv.inRange(img_hsv, role_colors[j][0], role_colors[j][1]) 
        result = cv.bitwise_and(robot_cropped, robot_cropped, mask=mask)

        # plt.subplot(1, 18, counter) # this can be deleted
        # plt.imshow(result)
        # counter = counter + 1

        # Classify by teams
        if robot_present(result):
            cv.imshow(str(robot), result)
            if j == 0:
                return 'G'
            if j == 1:
                return 'M'
            if j == 2:
                return "F"


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
    robot_mask = cv.inRange(hsvFrame, robot_color[0], robot_color[1]) 
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
    robots_cropped          = [None]*len(robots_contours)   # Save images of robot region
    for i, c in enumerate(robots_contours):
        robots_contours_poly[i] = cv.approxPolyDP(c, 3, True)
        robots_boundRect[i] = cv.boundingRect(robots_contours_poly[i])
        robots_centers[i], robots_radius[i] = cv.minEnclosingCircle(robots_contours_poly[i])
        robots_cropped[i] = robots_img[robots_boundRect[i][1]:robots_boundRect[i][1] + robots_boundRect[i][3], robots_boundRect[i][0]:robots_boundRect[i][0] + robots_boundRect[i][2]].copy()
    
    t1_robots = []
    t2_robots = []

    t1_roles = []
    t2_roles = []

    for i, c in enumerate(robots_cropped):
        img_hsv = cv.cvtColor(robots_cropped[i], cv.COLOR_RGB2HSV)
        # Apply color filter of teams to determine robot team
        for j in range(len(teams_colors)):
            mask = cv.inRange(img_hsv, teams_colors[j][0], teams_colors[j][1]) 
            result = cv.bitwise_and(robots_cropped[i], robots_cropped[i], mask=mask)
            # Classify by teams
            if robot_present(result):
                if j == 0:
                    t1_robots.append(i)
                    t1_roles.append(get_role(img_hsv, robots_cropped[i], i))
                    # Draw identifiers on screen
                    cv.putText(robots_img, "T1:", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (248, 65, 209), 2)
                    cv.putText(robots_img, t1_roles[-1], (int(robots_centers[i][0] + 30), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (248, 65, 209), 2)
                else:
                    t2_robots.append(i)
                    t2_roles.append(get_role(img_hsv, robots_cropped[i], i))
                    # Draw identifiers on screen
                    cv.putText(robots_img, "T2", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (57, 242, 248), 2) # BGR
                    cv.putText(robots_img, t2_roles[-1], (int(robots_centers[i][0] + 30), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (57, 242, 248), 2) # BGR

    print("t1 {} {} t2 {} {}".format(t1_robots, t1_roles, t2_robots, t2_roles))
    
    

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
