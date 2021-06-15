import socket  
import numpy as np
import math
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time
import random 
import keyboard ### NOTE: Run first: !pip install keyboard    in the terminal to install keyboard module
from IPython.display import clear_output

start = False
n = 0 # To show the first READY! message
index = 0 # To select the coordinates from the array

########################################################### SECTION FUNCTIONS
################################################################################################# STARTS COORD VISION TO NORM
def coordinatesfromvisiontonormal (XR1OPP,YR1OPP,XR2OPP,YR2OPP,XR3OPP,YR3OPP,XR1,YR1,XR2,YR2,XR3,YR3,Xball,Yball):
    XR1OPP = ((XR1OPP*10)/75)-25
    YR1OPP = abs((((YR1OPP*10)/75)-(5/3))-130)
    XR2OPP = ((XR2OPP*10)/75)-25
    YR2OPP = abs((((YR2OPP*10)/75)-(5/3))-130)
    XR3OPP = ((XR3OPP*10)/75)-25
    YR3OPP = abs((((YR3OPP*10)/75)-(5/3))-130)
    XR1 = ((XR1*10)/75)-25 #SET AS S
    YR1 = abs((((YR1*10)/75)-(5/3))-130)
    XR2 = ((XR2*10)/75)-25 #SET AS D
    YR2 = abs((((YR2*10)/75)-(5/3))-130)
    XR3 = ((XR3*10)/75)-25 #SET AS G
    YR3 = abs((((YR3*10)/75)-(5/3))-130)
    Xball = ((Xball*10)/75)-25
    Yball = abs((((Yball*10)/75)-(5/3))-130)
    return print(XR1OPP,YR1OPP,XR2OPP,YR2OPP,XR3OPP,YR3OPP,XR1,YR1,XR2,YR2,XR3,YR3,Xball,Yball)
################################################################################################# ENDS COORD VISION TO NORM
################################################################################################# STARTS DISTANCES
def distances (XR1OPP,YR1OPP,XR2OPP,YR2OPP,XR3OPP,YR3OPP,XR1,YR1,XR2,YR2,XR3,YR3,Xball,Yball,side):

# DISTANCES RELEVANT TO S
    DS_GOP = 0 #DISTANCE FROM S TO THE OPPOSITE G
    DS_OP1 = 0 #DISTANCE FROM S TO THE OPPOSITE OP1
    DS_OP2 = 0 #DISTANCE FROM S TO THE OPPOSITE OP2

    DSB = 0 #DISTANCE FROM S TO THE BALL
    DC = 0 #DISTANCE FROM S TO THE GOAL LINE

    DSD = 0 #DISTANCES FROM S TO D = D TO S
    DSG = 0 #DISTANCES FROM S TO G = G TO S

# DISTANCES RELEVANT TO D
    DD_GOP = 0 #DISTANCE FROM D TO THE OPPOSITE G
    DD_OP1 = 0 #DISTANCE FROM D TO THE OPPOSITE OP1
    DD_OP2 = 0 #DISTANCE FROM D TO THE OPPOSITE OP2

    DDB = 0 #DISTANCE FROM D TO THE BALL
    DC2 = 0 #DISTANCE FROM D TO THE GOAL LINE

    DDS = 0 #DISTANCES FROM D TO S = S TO D
    DDG = 0 #DISTANCES FROM D TO G = G TO D

# DISTANCES RELEVANT TO G
    DG_GOP = 0 #DISTANCE FROM G TO THE OPPOSITE G
    DG_OP1 = 0 #DISTANCE FROM G TO THE OPPOSITE OP1
    DG_OP2 = 0 #DISTANCE FROM G TO THE OPPOSITE OP2

    DGB = 0 #DISTANCE FROM G TO THE BALL
    DC3 = 0 #DISTANCE FROM G TO THE GOAL LINE

    DGS = 0 #DISTANCES FROM G TO S = S TO G
    DGD = 0 #DISTANCES FROM G TO D = D TO G

# RELEVANT TO DETERMINE THE GOALIE 
    DGLR1OPP = 0  
    DGLR2OPP = 0
    DGLR3OPP = 0 
    OPGX = 0 
    OPGY = 0
# FOR COORDINATES OF THE OPP GOAL-LINE
    COPGX = 0
    COPGY = 0
# FOR COORDINATES OUR GOAL-LINE
    GLX = 0 
    GLY = 0


    if (side == 0): # LEFT
        COPGX = 150;
        COPGY = 65;
        GLX = 0;
        GLY = 65;
    
    if (side == 1): # RIGHT
        COPGX = 0;
        COPGY = 65
        GLX = 150;
        GLY = 65;

    DGLR1OPP = np.sqrt(pow((COPGX - XR1OPP),2) + pow((COPGY - YR1OPP),2))
    DGLR2OPP = np.sqrt(pow((COPGX - XR2OPP),2) + pow((COPGY - YR2OPP),2))
    DGLR3OPP = np.sqrt(pow((COPGX - XR3OPP),2) + pow((COPGY - YR3OPP),2))

# DETERMINE THE GOALIE
    if (DGLR1OPP < DGLR2OPP and DGLR1OPP < DGLR3OPP):
        OPGX = XR1OPP # SET R1 AS THE GOALIE
        OPGY = YR1OPP # SET R1 AS THE GOALIE
    if (DGLR2OPP < DGLR1OPP and DGLR2OPP < DGLR3OPP):
        OPGX = XR2OPP # SET R2 AS THE GOALIE
        OPGY = YR2OPP # SET R2 AS THE GOALIE
    if (DGLR3OPP < DGLR1OPP and DGLR3OPP < DGLR2OPP):
        OPGX = XR3OPP # SET R2 AS THE GOALIE
        OPGY = YR3OPP # SET R2 AS THE GOALIE
# GOALIE HAS BEEN DETERMINED 
  
# COMPUTE ALL THE EUCLIDEAN DISTANCES RELEVANT TO S
    DS_GOP = np.sqrt(pow((OPGX - XR1),2) + pow((OPGY - YR1),2))
    DS_OP1 = np.sqrt(pow((XR2OPP - XR1),2) + pow((YR2OPP - YR1),2))
    DS_OP2 = np.sqrt(pow((XR3OPP - XR1),2) + pow((YR3OPP - YR1),2))
  
    DSB = np.sqrt(pow((Xball - XR1),2) + pow((Yball - YR1),2))
    DC = np.sqrt(pow((GLX - XR1),2) + pow((GLY - YR1),2))
  
    DSD = np.sqrt(pow((XR2 - XR1),2) + pow((YR2 - YR1),2))
    DSG = np.sqrt(pow((XR3 - XR1),2) + pow((YR3 - YR1),2))

#COMPUTE ALL THE EUCLIDEAN DISTANCES RELEVANT TO D
    DD_GOP = np.sqrt(pow((OPGX - XR2),2) + pow((OPGY - YR2),2))
    DD_OP1 = np.sqrt(pow((XR2OPP - XR2),2) + pow((YR2OPP - YR2),2))
    DD_OP2 = np.sqrt(pow((XR3OPP - XR2),2) + pow((YR3OPP - YR2),2))
  
    DDB = np.sqrt(pow((Xball - XR2),2) + pow((Yball - YR2),2))
    DC2 = np.sqrt(pow((GLX - XR2),2) + pow((GLY - YR2),2))
  
    DDS = DSD
    DDG = np.sqrt(pow((XR3 - XR1),2) + pow((YR3 - YR1),2))

#COMPUTE ALL THE EUCLIDEAN DISTANCES RELEVANT TO G
    DG_GOP = np.sqrt(pow((OPGX - XR3),2) + pow((OPGY - YR3),2))
    DG_OP1 = np.sqrt(pow((XR2OPP - XR3),2) + pow((YR2OPP - YR3),2))
    DG_OP2 = np.sqrt(pow((XR3OPP - XR3),2) + pow((YR3OPP - YR3),2))
  
    DGB = np.sqrt(pow((Xball - XR3),2) + pow((Yball - YR3),2))
    DC3 = np.sqrt(pow((GLX - XR3),2) + pow((GLY - YR3),2))

    DGS = DSG
    DGD = DDG

    return print(DS_GOP, DS_OP1, DS_OP2, DSB, DC, DSD, DSG, DD_GOP, DD_OP1, DD_OP2, DDB, DC2, DDS, DDG, DG_GOP, DG_OP1,DG_OP2, DGB, DC3)
################################################################################################# ENDS DISTANCES
################################################################################################# STARTS KICKOFF
def kickoff (KICKOFF,BALLPOSSE,SIDE):
    
####################### Positions of our robots (only for internal use)
    # GOALIE R3 - DEF R2 - STR R1
    Xpr1 = 0
    Xpr2 = 0
    Xpr3 = 0
    Ypr1 = 0
    Ypr2 = 0
    Ypr3 = 0
    Ar1 = 0
    Ar2 = 0
    Ar3 = 0
######################

###################################################################################################### KICKOFF POS
    if (KICKOFF == 1):
############################################################ ATTACKING POSITIONING FOR KICKOFF
        if (BALLPOSSE == 1):
            if (SIDE == 0):
                ####### SETS THE ROBOT (THE GOALIE) INSIDE THE GOAL ZONE (CENTER)
                Xpr3 = 300 #X value center of the goal zone left side
                Ypr3 = 500 #Y value center of the goal zone left side
                Ar3 = 0 # ANGLE
                ####### PICK A RANDOM POSITION BETWEEN DOT 1 AND DOT 2
                rnd = random.random()
                if (rnd < 0.5):
                    Xpr2 = 468.75 #Xvalue DOT 1 (UP)
                    Ypr2 = 200 #Yvalue DOT 1 (UP)
                    Ar2 = -45 #ANGLE
                if (rnd > 0.5):
                    Xpr2 = 468.75 #Xvalue DOT 2 (DOWN)
                    Ypr2 = 800 #Yvalue DOT 2 (DOWN)
                    Ar2 = 45 #ANGLE
                ####### FOR R3 SET IT IN THE CENTER OVER THE LINE OF THE MIDDLE CIRCLE RIGHT IN FRONT OF THE BALL
                Xpr1 = 550 #Xcenl
                Ypr1 = 500 #Ycenl
                Ar1 = 0 #ANGLE
###############################################################################################################FIRST IF BREAKS
            if (SIDE == 1):
                ####### SETS THE ROBOT (THE GOALIE) INSIDE THE GOAL ZONE (CENTER)
                Xpr3 = 1256.25 #X value center of the goal zone R side
                Ypr3 = 500 #Y value center of the goal zone R side
                Ar3 = 180 #ANGLE
                ####### PICK A RANDOM POSITION BETWEEN DOT 3 AND DOT 4
                rnd = random.random()
                if (rnd < 0.5):
                    Xpr2 = 843.75 #Xvalue DOT 3 (UP)
                    Ypr2 = 200 #Yvalue DOT 3 (UP)
                    Ar2 = 225 #ANGLE
                if (rnd > 0.5):
                    Xpr2 = 843.75 #Xvalue DOT 4 (DOWN)
                    Ypr2 = 800 #Yvalue DOT 2 (DOWN)
                    Ar2 =135 #ANGLE
            ####### FOR R3 SET IT IN THE CENTER OVER THE LINE OF THE MIDDLE CIRCLE RIGHT IN FRONT OF THE BALL
                Xpr1 = 812.5 #XcenR
                Ypr1 = 500 #YcenR
                Ar1 = 180 #ANGLE
##############################################################################################################SECOND IF BREAKS
############################################################ ENDS ATTACKING POSITIONING FOR KICKOFF
############################################################ DEFENDING POSITIONING FOR KICKOFF       
        if (BALLPOSSE == 0): 
            if (SIDE == 0):
                Xpr3 = 187.5 #XGZL
                Ypr3 = 500 #YGZL
                Ar3 = 0
                Xpr2 = 468.75 #XFBLUP 
                Ypr2 = 200 #YFBLUP
                Ar2 = 0
                Xpr1 = 618.75 #XFBLDOWN
                Ypr1 = 800 #YFBLDOWN
                Ar1 = 45
            if (SIDE == 1):
                Xpr3 = 1256.25 #XGZR
                Ypr3 = 500 #YGZR
                Ar3 = 180
                Xpr2 = 693.75 #XFBRUP 
                Ypr2 = 200 #YFBRUP
                Ar2 = 225
                Xpr1 = 843.75 #XFBRDOWN
                Ypr1 = 800 #YFBRDOWN
                Ar1 = 180
############################################################ DEFENDING POSITIONING FOR KICKOFF  
    return Xpr1, Xpr2, Xpr3, Ypr1, Ypr2, Ypr3, Ar1, Ar2, Ar3
###################################################################################################### ENDS KICKOFF POS
###################################################################################################### STARTS GOES FOR THE BALL
def GOESFORTHEBALL (DS_GOP, DS_OP1, DS_OP2, DSB, DC, DSD, DSG, DD_GOP, DD_OP1, DD_OP2, DDB, DC2, DDS, DDG, DG_GOP, DG_OP1,DG_OP2, DGB, DC3):
#### TO RECOVER THE BALL
    if DGB >= DSB and DDB >= DSB and DC > 20:
        if DS_OP1 >= DSB and DS_GOP >= DSB and DS_OP2 >= DSB:
            print ("S GOES FOR THE BALL") #Here Path Planning Should be Called

    if DGB < 20:
        if DG_OP1 >= 20 and DG_GOP >= 20 and DG_GOP2 >= 20:
            print ("G GOES FOR THE BALL") #Here Path Planning Should be Called

    if DGB >= DDB and DSB > DDB and DC2 > 20:
        if DS_OP1 >= DSB and DS_GOP >= DSB and DS_OP2 >= DSB:
            print ("D GOES FOR THE BALL") #Here Path Planning SHould be called
###################################################################################################### ENDS GOES FOR THE BALL
########################################################### ENDS SECTION FUNCTIONS


#################################################### Needed
side = 0 # 0 = LEFT / 1 = RIGHT

## THESE COME FROM THE REFEREE
SIDE = 0  # 0 = LEFT / 1 = RIGHT
BALLPOSSE = 0 # DECIDED BY TOSSING A COIN
KICKOFF = 0 # IF KICKOFF = 1 THERE IS A KICKOFF

####################################################


#### STARTS "MAIN"

HOST = 'localhost'
PORT = 12345
s = socket.socket()
s.connect((HOST, PORT))
print("Data from Python:")

########################################################################################################## STARTS REC DATA 
while True: 
    data = s.recv(1000)
    data2 = data.decode()
    arr = data2.split()
    if len(arr)>20:
        arr = arr[arr.index('s')+1:]
        arr = arr[:arr.index('\x00')]
        data = ""
        # BALL REAL TIME
        Xball = float (arr[0])
        Yball = float (arr[1])
        # GOALIE (OURS) REAL TIME
        XR3 = float(arr[2])
        YR3 = float(arr[3])
        Ar3 = float(arr[4])
        # MID - DEF (OURS) REAL TIME
        XR2 = float(arr[5])
        YR2 = float(arr[6])
        Ar2 = float(arr[7])
        # STRIKER (OURS) REAL TIME
        XR1 = float(arr[8])
        YR1 = float(arr[9])
        Ar1 = float(arr[10])
        # GOALIE (OPP) REAL TIME
        XR1OPP = float(arr[11])
        YR1OPP = float(arr[12])
        # MID - DEF (OPP) REAL TIME
        XR2OPP = float(arr[14])
        YR2OPP = float(arr[15])
        # STRIKER (OPP) REAL TIME
        XR3OPP = float(arr[17])
        YR3OPP = float(arr[18])
        
        if n == 0:
            print("READY!")
        n = n+1
        
        ############################# SEND DATA TO PROCESSING SERVER ##################################################
        if keyboard.is_pressed('ENTER'):
            start = True
            print("Key pressed")

        if start == True:
            
            #coordinates = [[640,400],[680,390],[690,360],[700,340],[720,300],[740,300],[760,300],[780,300],[800,300],[850,300]]
            #coordinates = [[640,450],[680,460],[690,490],[700,510],[720,550],[740,550],[760,550],[780,550],[800,550],[850,550]]
            #coordinates = [[630,450],[620,460],[610,490],[600,510],[580,550],[560,550],[540,550],[520,550],[500,550],[450,550]]
            coordinates = [[629, 424], [638,425], [741, 490], [752, 497], [753, 498], [752, 498],[751,498],[751,498],[751,498],[749,499]]
            #coordinates = [[630,575],[500,550],[200,500]]
            ############################# TARGET ANGLE #######################
            ####### NOTA! Para cada punto de la trayectoria del path planning, x1 y y1 son las posiciones actuales del robot, 
            # es decir: XR3 y YR3 para el primer robot. Mientras que x2 y y2 son las siguientes coordenadas a seguir de la trayectoria
            # Mientras el robot avanza por la trayectoria que le digamos, x1 y y1 se actualizan a lo que envíe el sistema de visión y
            # x2 y y2 se actualizan al siguiente punto de la trayectoria
            
            # x1 and y1 are the robot points
            x1 = XR1
            y1 = YR1
            
            # x2 and y2 are the goal points  ###### HERE THE TARGET COORDINATES ARE MANUALLY SET #########
            x2 = coordinates[index][0]
            y2 = coordinates[index][1]
            if index < (len(coordinates)-1):
                index = index+1
            
            if x2 != x1:
                slope = (y1-y2)/(x2-x1)
                theta = math.degrees(math.atan(slope))
                if theta>0:
                    if (x2-x1)<0:
                        theta = theta + 180.0
                elif theta<0:
                    if (x2-x1)<0:
                        theta = 180.0 + theta
                    else:
                        theta = 360.0 + theta
                else:
                    if (x2-x1)<0:
                        theta = 180.0
            else:
                if y1 > y2:
                    theta = 90.0
                else:
                    theta = 270.0

            if Ar1<180:
                limit = Ar1 + 180
                if theta > limit:
                    angle_error = -(360-(theta-Ar1))
                else:
                    angle_error = (theta-Ar1)
            elif (Ar1)>180:
                limit = Ar1 - 180
                if theta < limit:
                    angle_error = 360+(theta-Ar1)
                else:
                    angle_error = (theta-Ar1)
            else:
                angle_error = (theta-Ar1)

            ka = 0.6 # Gain for the angular velocity controller (Adjust if the robot spins too fast or too slow)
            angle = ka * angle_error
            kv = 0.1 # Gain for the linear velocity controller (Adjust if the robot goes too fast or too slow)
            LV1 = kv*math.sqrt((x2-x1)**2 + (y2-y1)**2)
            if (math.sqrt((x2-x1)**2 + (y2-y1)**2))<10: # This is to avoid that the robot spins when it achieves the goal point
                angle = 0.0
                LV1 = 0.0
                
            if abs(angle)>10:
                if angle<0:
                    angle = -10
                if angle>0:
                    angle = 10
                
            clear_output()
            print(math.sqrt((x2-x1)**2 + (y2-y1)**2))
            print("Linear Velocity 1:", LV1)
            print("Angular Velocity 1:", angle)
            print(arr[2],arr[3],arr[4])
            data = str(2)+ " " + str(LV1)+ " " + str(angle) + "\n\0"
            data = str(data).encode()
            s.send(data)
            data = str(1)+ " " + str(0.0)+ " " + str(0.0) + "\n\0"
            data = str(data).encode()
            s.send(data)
            data = str(0)+ " " + str(0.0)+ " " + str(0.0) + "\n\0"
            data = str(data).encode()
            s.send(data)
            ## ID = 0 -> GREEN -> XR3, YR3, Ar3
            ## ID = 1 -> CYAN -> XR2, YR2, Ar2
            ## ID = 2 -> PURPLE -> XR1, YR1, Ar1
            
        ###################################### AQUÍ TERMINA EL FOR PARA EL PATH PLANNING ##################################
    ########################################################################################################## ENDS REC DATA 