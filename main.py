import matplotlib.pyplot as plt
import numpy as np
import vehicle
from csv_to_coordinates import csv_to_coordinates
import matplotlib.pyplot as plt
from basic_drive_controller import basic_drive_controller
import pandas as pd

#convert csv to coordinates
spielberg=csv_to_coordinates("Spielberg.csv")
path=spielberg.get_midline()
borderleft=spielberg.get_border_left()
borderright=spielberg.get_border_right()
#create car
car=vehicle.dynamic_vehicle(starting_x=path[0][0], starting_y=path[0][1], max_acceleration=6.5, max_brake=13)
car.set_starting_angle(path)

##controller 
BDC=basic_drive_controller.basic_drive_controller(path, car.get_max_brake(), car.get_wheelbase(),car.get_max_speed())


r=0

cartrace=[]
cartrace.append(car.get_position())
finished= False
velocity_list=[car.get_velocity()]

while not finished:
    
    velocity=car.get_velocity()
    position=car.get_position()
    acceleration, steering_angle, finished = BDC.compute_control(position, car.get_velocity(), car.get_yaw())
    control_inputs=np.array([steering_angle,acceleration])
    car.time_step(control_inputs,0.01)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())
    r+=1
    if r==20000: break
cartrace=np.array(cartrace)
#plt.plot(path[:, 0], path[:, 1], color="RED")
plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")
#plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")

sc = plt.scatter(
    cartrace[:, 0],
    cartrace[:, 1],
    c=velocity_list,
    cmap="coolwarm",  
    s=3,  # Punktgröße
)


cbar = plt.colorbar(sc)
print("Time:", len(cartrace)*0.01)
cbar.set_label("Geschwindigkeit [m/s]")
plt.show()
