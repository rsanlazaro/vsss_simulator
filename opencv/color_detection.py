import numpy as np
import cv2 as cv
import math

########            COLORS            ########
# Limits for robot and ball colors
robot_color     = [(80, 50, 80), (140, 255, 255)]
ball_color      = [(5, 50, 80), (20, 255, 255)]
# Limits for team colors
t1_color        = [(130, 50, 50), (160, 255, 255)]     # Purple ish
t2_color        = [(80, 50, 60), (100, 255, 255)]      # Yellow-ish
teams_colors    = [t1_color, t2_color]
# Limits for role colors
goal_color      = [(100, 0, 0), (150, 255, 255)]        # Red-ish
mid_color       = [(50, 100, 0), (70, 255, 255)]       # Green-ish
for_color       = [(0, 100, 0), (20, 255, 255)]     # Blue-ish

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

def get_role(img_hsv, robot_cropped):
    for j in range(len(role_colors)):
        mask = cv.inRange(img_hsv, role_colors[j][0], role_colors[j][1]) 
        result = cv.bitwise_and(robot_cropped, robot_cropped, mask=mask)

        # Classify by teams
        if robot_present(result):
            # cv.imshow(str(robot), result)
            if j == 0:
                return 'G'
            if j == 1:
                return 'M'
            if j == 2:
                return "F"

def get_orientation(result):
    x2 = result.shape[0]/2
    y2 = result.shape[1]/2
    # This were left as they might be useful, although only robot_radius is being used right now
    robot_contours, _     = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    robot_contours_poly   = [None]*len(robot_contours)
    robot_boundRect       = [None]*len(robot_contours)
    robot_centers         = [None]*len(robot_contours)
    robot_radius          = [None]*len(robot_contours)
    
    for i, c in enumerate(robot_contours):
        robot_contours_poly[i] = cv.approxPolyDP(c, 3, True)
        robot_boundRect[i] = cv.boundingRect(robot_contours_poly[i])
        robot_centers[i], robot_radius[i] = cv.minEnclosingCircle(robot_contours_poly[i])

    radians = math.atan2(robot_centers[0][1]-y2, robot_centers[0][0]-x2)
    degrees = math.degrees(radians)
    return degrees + 45 # Offset for orientation to be centered

def draw_orientation(robot_position, orientation, robots_img):
    a = orientation
    l = 25
    p1 = (int(robot_position[0]), int(robot_position[1]))
    p2 = (int(p1[0] + l * math.cos(a * math.pi / 180.0)), int(p1[1] + l * math.sin(a * math.pi / 180.0)))
    image = cv.arrowedLine(robots_img, p1, p2, (0,0,255), 3)
    return image

if not obs_camera.isOpened():
        print("Cannot open camera")
        exit()
    
# Start a while loop
while True:
    # Read new frame from OBS
    ret, imageFrame = obs_camera.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Convert To HSV
    hsvFrame = cv.cvtColor(imageFrame, cv.COLOR_BGR2HSV)
    # Create Robot and ball Masks
    robot_mask = cv.inRange(hsvFrame, robot_color[0], robot_color[1]) 
    ball_mask = cv.inRange(hsvFrame, ball_color[0], ball_color[1]) 
    # Fill holes
    contours, hierarchy = cv.findContours(robot_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        cv.fillPoly(robot_mask, pts=[contour], color=(255, 255, 255))
    # Apply mask   
    robots_img  = cv.bitwise_and(imageFrame, imageFrame, mask = robot_mask)
    ball_img    = cv.bitwise_and(imageFrame, imageFrame, mask = ball_mask)
    
    # Find individual robots properties
    ball_contour, _             = cv.findContours(ball_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    ball_contours_poly          = [None]*len(ball_contour)
    ball_boundRect              = [None]*len(ball_contour)
    ball_centers                = [None]*len(ball_contour)
    ball_radius                 = [None]*len(ball_contour)
    ball_contours_poly          = cv.approxPolyDP(ball_contour[0], 3, True)
    ball_boundRect              = cv.boundingRect(ball_contours_poly)
    ball_centers, ball_radius   = cv.minEnclosingCircle(ball_contours_poly)
    t1_info["B"] = ball_centers
    t2_info["B"] = ball_centers

    # Find individual robots properties
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
        # Generate RoI of each robot found, later filters are only applied to this images
        robots_cropped[i] = robots_img[robots_boundRect[i][1]:robots_boundRect[i][1] + robots_boundRect[i][3], robots_boundRect[i][0]:robots_boundRect[i][0] + robots_boundRect[i][2]].copy()
    
    # Iterate over each robot found, determine team, role, and save position and orientation
    for i, robot_cropped in enumerate(robots_cropped):
        img_hsv = cv.cvtColor(robot_cropped, cv.COLOR_RGB2HSV)
        # Determine obot team
        for j in range(len(teams_colors)):
            mask = cv.inRange(img_hsv, teams_colors[j][0], teams_colors[j][1]) 
            result = cv.bitwise_and(robot_cropped, robot_cropped, mask=mask)
            # Classify by teams and save robot properties
            if robot_present(result):
                if j == 0:
                    role = get_role(img_hsv, robot_cropped)
                    # Set robots position and orientation
                    t1_info[role][0] = robots_centers[i]
                    t1_info[role][1] = get_orientation(result)
                    
                    # Draw identifiers on screen
                    cv.putText(robots_img, "T1:", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (248, 65, 209), 2)
                    cv.putText(robots_img, role, (int(robots_centers[i][0] + 30), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (248, 65, 209), 2)
                    robots_img = draw_orientation(t1_info[role][0], t1_info[role][1],robots_img)
                else:
                    role = get_role(img_hsv, robot_cropped)
                    # Set robots position and orientation
                    t2_info[role][0] = robots_centers[i]
                    t2_info[role][1] = get_orientation(result)

                    # Draw identifiers on screen
                    cv.putText(robots_img, "T2", (int(robots_centers[i][0]), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (57, 242, 248), 2) # BGR
                    cv.putText(robots_img, role, (int(robots_centers[i][0] + 30), int(robots_centers[i][1])),
                        cv.FONT_HERSHEY_SIMPLEX, 0.5, (57, 242, 248), 2) # BGR
                    robots_img = draw_orientation(t2_info[role][0], t2_info[role][1],robots_img)
                    
                
    cv.circle(robots_img, (int(t2_info["B"][0]), int(t2_info["B"][1])), 5, (0, 128, 255), 2, 8)
    print(t2_info["B"])

    # Program Termination
    #cv.imshow("Robots", ball_img)
    cv.imshow("Robots", robots_img)

    if cv.waitKey(10) & 0xFF == ord('q'):
        obs_camera.release()
        cv.destroyAllWindows()
        break
