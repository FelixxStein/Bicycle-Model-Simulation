import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
import csv
import pandas as pd

import matplotlib.pyplot as plt



df = pd.read_csv("/Users/felixarbeit/Documents/Strategiewechsel/Bicycle-Model-Simulation/Hockenheim.csv")
x=df.iloc[:, 0].tolist()
y=df.iloc[:, 1].tolist()
w_tr_right_m=df.iloc[:, 2].tolist()
w_tr_left_m=df.iloc[:, 3].tolist()
x=np.array(x)
y=np.array(y)
points = np.column_stack((x, y))
points = points.squeeze()
borderright=[]
borderleft=[]
s=np.sin(np.radians(90))
c=np.cos(np.radians(90))
RR = np.array([[c, -s], [s, c]])
RL = np.array([[c, s], [-s, c]])
for i in range(len(points)-1):
    diff=points[i+1]-points[i]
    normalvector=diff/np.linalg.norm(diff)
    borderright.append(points[i]+w_tr_right_m[i]*(RR@normalvector))
    borderleft.append(points[i]+w_tr_left_m[i]*(RL@normalvector))

diff=points[0]-points[len(points)-1]
normalvector=diff/np.linalg.norm(diff)
borderright.append(points[0]+w_tr_right_m[len(points)-1]*(RR@normalvector))
borderleft.append(points[0]+w_tr_left_m[len(points)-1]*(RL@normalvector))

borderleft = np.array(borderleft)
borderright = np.array(borderright)

##pure persuite 
vector=points[1]-points[0]
vehicle_angle=np.atan2(vector[1],vector[0])*(180/np.pi)
car=vehicle.vehicle(starting_x=points[0][0], starting_y=points[0][1], starting_vehicle_angle=vehicle_angle, velocity=20)
next_index=0
lookahead=5
cartrace=[]
##get current pos
##fallback pfadende setzten
##nächster punkt suchen+nach vorne schauen
closest_index=0
last_index=0
cartrace.append(car.get_position())
while True:
    for i in range(last_index, min(last_index+40, len(points)-1)):
        if np.sqrt(np.sum((car.get_position() - points[i])**2))<np.sqrt(np.sum((car.get_position() - points[closest_index])**2)):
            closest_index=i
    last_index=closest_index
    next_index=len(points)-1
    distance=0
    for i in range(closest_index, len(points)-1):
        distance=np.sqrt(np.sum((car.get_position() - points[i])**2))
        dxy=points[i]-car.get_position()
        if distance>lookahead and dxy[0] * np.cos(np.radians(car.get_vehicle_angle())) + dxy[1]*np.sin(np.radians(car.get_vehicle_angle()))>0:
            next_index=i
            break
    dxy=points[next_index]-car.get_position()
    distance=np.sqrt(np.sum((car.get_position() - points[next_index])**2))
    ylokal=-dxy[0]*np.sin(np.radians(car.get_vehicle_angle()))+dxy[1]*np.cos(np.radians(car.get_vehicle_angle()))
    path_curvature=(2*ylokal/distance**2)
    steering_angle=np.arctan(path_curvature*car.wheel_base)
    car.update_steering_angle(steering_angle*(180/np.pi))
    car.time_step()
    cartrace.append(car.get_position())
    if np.sqrt(np.sum((car.get_position() - points[next_index])**2))<1 and next_index==len(points)-1:
        break

##Plot
cartrace=np.array(cartrace)


plt.plot(points[:, 0], points[:, 1], color="RED")
plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")
plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")
plt.show()
