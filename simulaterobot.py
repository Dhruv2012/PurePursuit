import configparser
import math
import numpy as np
import cv2
from matplotlib import pyplot as plt 

def closest():
    global path, pos
    min_distance = (0, math.sqrt((path[0][0] - pos[0]) ** 2 + (path[0][1] - pos[1]) ** 2))
    for i, p in enumerate(path):
        distance = math.sqrt((p[0]-pos[0])**2 + (p[1]-pos[1])**2)
        if distance < min_distance[1]:
            min_distance = (i, distance)

    return min_distance[0]

def curvature(lookahead):
    global path, pos, angle
    side = np.sign(math.sin(3.1415/2 - angle)*(lookahead[0]-pos[0]) - math.cos(3.1415/2 - angle)*(lookahead[1]-pos[1]))
    a = -math.tan(3.145/2 - angle)
    c = math.tan(3.145/2 - angle)*pos[0] - pos[1]
    x = abs(a*lookahead[0] + lookahead[1] + c)/math.sqrt(a**2 + 1)
    return side * (2*x/(float(lookahead_distance)**2))

def turn(curvature,velocity,trackwidth):
    return  [velocity*(2+curvature*trackwidth)/2, velocity*(2-curvature*trackwidth)/2]


def draw_path(img):
    global path, start_pos
    cv2.circle(img, (int(start_pos[0]+path[0][0]*scaler), int(start_pos[1]-path[0][1]*scaler)), 2,
               (255*(1-path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[0][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    for i in range(1, len(path)):
        cv2.circle(img, (int(start_pos[0]+path[i][0]*scaler), int(start_pos[1]-path[i][1]*scaler)), 2,
                   (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
        cv2.line(img, (int(start_pos[0]+path[i][0]*scaler), int(start_pos[1]-path[i][1]*scaler)),
                 (int(start_pos[0]+path[i-1][0]*scaler), int(start_pos[1]-path[i-1][1]*scaler)),
                 (255*(1-path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[i-1][2]/float(config["VELOCITY"]["MAX_VEL"])), 1)

def lookahead():
    global path, t, t_i, pos

    for i,p in enumerate(reversed(path[:-1])):
        index = len(path)-2 - i
        d = (path[index+1][0] - p[0], path[index+1][1] - p[1])
        f = (p[0]-pos[0], p[1]-pos[1])
        a = sum(j**2 for j in d)
        b = 2*sum(j*k for j,k in zip(d,f))
        c = sum(j**2 for j in f) - float(lookahead_distance)**2

        Discriminant = b**2 - 4*a*c
        if Discriminant >= 0:
            Discriminant = math.sqrt(Discriminant)
            t1 = (-b - Discriminant)/(2*a)
            t2 = (-b + Discriminant)/(2*a)

            if 0 <= t1 <= 1:
                t = t1
                t_i = index
                return p[0]+t*d[0], p[1]+t*d[1]
            
            if 0 <= t2 <= 1:
                t = t2
                t_i = index
                return p[0]+t*d[0], p[1]+t*d[1]
    t = 0
    t_i = 0
    return path[closest()][0:2]

def draw_robot(img):
    tmp = img.copy()
    cv2.drawContours(tmp, [np.array([(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)+width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler),
                                     (start_pos[0] + (pos[0]-length/2*math.sin(angle)-width/2*math.cos(angle))*scaler,
                                      start_pos[1] + (pos[1]-length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler)])
                     .reshape((-1,1,2)).astype(np.int32)], 0, (0, 255, 255), 2)
    cv2.circle(tmp, (int(start_pos[0]+pos[0]*scaler), int(start_pos[1]-pos[1]*scaler)), int(lookahead_distance), (0, 255, 0), 1)
    cv2.circle(tmp, (int(start_pos[0]+path[close][0]*scaler), int(start_pos[1]-path[close][1]*scaler)), 4,
               (255*(1-path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), 0, 255*path[close][2]/float(config["VELOCITY"]["MAX_VEL"])), -1)
    cv2.circle(tmp, (int(start_pos[0]+look[0]*scaler), int(start_pos[1]-look[1]*scaler)), 4, (0,255,0), -1)

    try:
        x3 = (pos[0]+look[0])/2
        y3 = -(pos[1]+look[1])/2
        q = math.sqrt((pos[0]-look[0])**2 + (pos[1]-look[1])**2)
        x = x3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[1]-look[1])/q * np.sign(curv)
        y = y3 - math.sqrt(1/curv**2 - (q/2)**2) * (pos[0]-look[0])/q * np.sign(curv)
        cv2.circle(tmp, (int(x*scaler+start_pos[0]), int(y*scaler+start_pos[1])), int(abs(1/curv*scaler)), (0,255,255), 1)
    except:
        pass

    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)-width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)+width/2*math.sin(angle))*-scaler)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[0]/5)*math.sin(angle)-width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+(length/2+wheels[0]/5)*math.cos(angle)+width/2*math.sin(angle))*-scaler)),
             (0,0,255), 2)
    cv2.line(tmp,
             (int(start_pos[0] + (pos[0]+length/2*math.sin(angle)+width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+length/2*math.cos(angle)-width/2*math.sin(angle))*-scaler)),
             (int(start_pos[0] + (pos[0]+(length/2+wheels[1]/5)*math.sin(angle)+width/2*math.cos(angle))*scaler),
              int(start_pos[1] + (pos[1]+(length/2+wheels[1]/5)*math.cos(angle)-width/2*math.sin(angle))*-scaler)),
             (0,0,255), 2)

    cv2.imshow("img", tmp)
    cv2.waitKey(5)

def click(event, x, y, flags, param):
    global pos, angle, t, t_i, wheels
    if event == cv2.EVENT_LBUTTONDOWN:
        pos = ((x-start_pos[0])/scaler,(start_pos[0]-y)/scaler)
        angle = 0
        t = 0
        t_i = 0
        wheels = [0, 0]
        print("click triggered")






config = configparser.ConfigParser()
config.read("config.ini")

with open(config["PATH"]["FILE_LOCATION"]) as file:
    path = [([float(x) for x in line.split(",")]) for line in file.readlines()]

scaler = float(config["FIELD_IMAGE"]["PIXELS_PER_UNIT"])/1.5
width = float(config["ROBOT"]["TRACKWIDTH"])
length = float(config["ROBOT"]["LENGTH"])
lookahead_distance = float(config["PATH"]["LOOKAHEAD"])
pos = (0,0)
angle = math.atan2(path[1][0], path[1][1])                  
t = 0
t_i = 0
wheels = [0,0]
measured_wheels = [0,0]
lastmeasured_wheels = [0,0]
error = [0,0]
integral_error = [0,0]
Ka = 0.001
Kp = 0.90
Kv = 1/(float(config["VELOCITY"]["MAX_VEL"]))
Ki = 0.001
dt = 0.005

field = cv2.imread(config["FIELD_IMAGE"]["FILE_LOCATION"])
img = np.zeros((field.shape[0], field.shape[1], 3), np.uint8)
start_pos = (field.shape[0]/2, field.shape[1]/2)
draw_path(img)
cv2.imshow("img", img)
cv2.setMouseCallback('img', click)      #FOR CUSTOMIZING AND CHANGING STARTING POINT BY CLICK
cv2.waitKey(2000)
# PATH, START_POS INITIALIZATION, CALCLULATING WHEEL VELOCITY(LINE 159)
#WORK LEFT: CALCULATING WHEEL VELOCITY(LINE 159), START_POS INITIALIZATION, MOUSE CLICK EVENT
print(pos)
itr = 0
while closest() != len(path) - 1:
    look = lookahead()
    close = closest()
    curv = curvature(look) if t_i > close else 0.0001
    #lookahead_distance = (1 - abs(curv))*lookahead_distance
    vel = path[close][2]
    last_wheels =  wheels
    lastmeasured_wheels = measured_wheels
    wheels = turn(curv, vel, width)
    
    for i in range(len(wheels)):
        error[i] = wheels[i] - lastmeasured_wheels[i]
        integral_error[i] += error[i]
    
    for i,w in enumerate(wheels):
        measured_wheels[i] = Kp*(wheels[i] - lastmeasured_wheels[i]) + Kv*(wheels[i]) + Ka*(wheels[i] - last_wheels[i])/dt + Ki*integral_error[i]
        #wheels[i] = last_wheels[i] + min(float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, max(-float(config["ROBOT"]["MAX_VEL_CHANGE"])*dt, w-last_wheels[i]))
        if(measured_wheels[i] > float(config["VELOCITY"]["MAX_VEL"])):
            measured_wheels[i] = config["VELOCITY"]["MAX_VEL"]
    #pos = (pos[0] + (wheels[0]+wheels[1])/2*dt * math.sin(angle), pos[1] + (wheels[0]+wheels[1])/2*dt * math.cos(angle))
    #angle += math.atan((wheels[0]-wheels[1])/width*dt)
    print(measured_wheels)
    print("Calculated velocity:" + str(wheels))
    pos = (pos[0] + (measured_wheels[0]+measured_wheels[1])/2*dt * math.sin(angle), pos[1] + (measured_wheels[0]+measured_wheels[1])/2*dt * math.cos(angle))
    angle += math.atan((measured_wheels[0]-measured_wheels[1])/width*dt)
    draw_robot(img)
    itr += 1
cv2.waitKey()

#wheels,last_wheels => target

