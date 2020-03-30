#source: https://github.com/arimb/PurePursuit/blob/master/PathGenerator.py

import cv2
import configparser
import math
import sys

# INITIALIZE VALUES
waypoints = []
start_pos = (0,0)
mouse_down = False

# READ CONFIG FILE
config = configparser.ConfigParser()
config.read("config.ini")
scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])

# READ & SHOW IMAGE, SET OPENCV PROPERTIES
img = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
cv2.imshow("Field", img)
cv2.setMouseCallback("Field", click)
cv2.waitKey()

# MAKE SURE AT LEAST 2 POINTS SELECTED
if len(waypoints) < 2:
    sys.exit(0)

# CONVERT PIXELS TO INCHES, CALCULATE WAYPOINT DISTANCE, INJECT MID-WAYPOINTS
total_waypoints = []
waypoints[0] = tuple(x/scaler for x in waypoints[0])
for i in range(len(waypoints) - 1):
    waypoints[i+1] = tuple(x/scaler for x in waypoints[i+1])
    dist = math.sqrt((waypoints[i+1][0]-waypoints[i][0])**2 + (waypoints[i+1][1]-waypoints[i][1])**2))
    j = 0
    while j < dist:
        total_waypoints.append(tuple(a+j*(b-a)/dist for a,b in zip(waypoints[i], waypoints[i+1])))
        j += float(config["POINT_INJECTION"]["POINT_DIST"])
total_waypoints.append(waypoints[-1])

# SMOOTH WAYPOINTS - W[0]=X, W[1]=Y
smooth_waypoints = [[w[0], w[1]] for w in total_waypoints]
weight_data = float(config["POINT_INJECTION"]["WEIGHT_DATA"])
weight_smooth = float(config["POINT_INJECTION"]["WEIGHT_SMOOTH"])
tolerance = float(config["POINT_INJECTION"]["TOLERANCE"])
change = tolerance
while change >= tolerance:
    change = 0
    for i in range(1, len(total_waypoints)-1):
        for j in range(len(total_waypoints[i])):
            aux = smooth_waypoints[i][j]
            smooth_waypoints[i][j] += weight_data*(total_waypoints[i][j]-smooth_waypoints[i][j]) + \
                weight_smooth*(smooth_waypoints[i-1][j]+smooth_waypoints[i+1][j]-2*smooth_waypoints[i][j])
            change += abs(aux - smooth_waypoints[i][j])

# CALCULATE PATH DISTANCE - W[2]
smooth_waypoints[0].append(0)
for i,w in enumerate(smooth_waypoints[1,:],start=1):
    w.append(smooth_waypoints[i-1][2] + math.sqrt((w[0]-smooth_waypoints[i-1][0])**2 + (w[1]-smooth_waypoints[i-1][1])**2))

# CALCULATE CURVATURE - W[3]
smooth_waypoints[0].append(0.0001)
smooth_waypoints[-1].append(0.0001)
# ADD CURVATURE = 0 TO THE START AND END INDICES AS c in (x,y,d,c) WHERE C is CURVATURE
for i,w in enumerate(smooth_waypoints[1:-1],start = 1):
    w[0]+= 0.0001 #INCREMENT x1 and y1 TO AVOID DIVIDE BY ZERO ERROR
    w[1]+= 0.0001
    k1 = .5*(w[0]**2 + w[1]**2 - smooth_waypoints[i-1][0]**2 - smooth_waypoints[i-1][1]**2) / (w[0] - smooth_waypoints[i-1][0])
    k2 = (w[1] - smooth_waypoints[i-1][1]) / (w[0] - smooth_waypoints[i-1][0])
    b = .5*(smooth_waypoints[i-1][0]**2 - 2*smooth_waypoints[i-1][0]*k1 + smooth_waypoints[i-1][1]**2 - smooth_waypoints[i+1][0]**2 + 2*smooth_waypoints[i+1][0]*k1 - smooth_waypoints[i+1][1]**2) / (smooth_waypoints[i+1][0]*k2 - smooth_waypoints[i+1][1] + smooth_waypoints[i-1][1] - smooth_waypoints[i-1][0]*k2)
    a = k1 - k2*b
    r = math.sqrt((w[0]-a)**2 + (w[1]-b)**2)
    w.append(1/r)