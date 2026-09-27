import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
from csv_to_coordinates import csv_to_coordinates
from pure_pursuit_controller import pure_pursuit_controller
import matplotlib.pyplot as plt

#convert csv to coordinates
spielberg=csv_to_coordinates("Spielberg.csv")
path=spielberg.get_midline()
borderleft=spielberg.get_border_left()
borderright=spielberg.get_border_right()
#create car
car=vehicle.vehicle(starting_x=path[0][0], starting_y=path[0][1], velocity=20)
car.set_starting_angle(path)

##pure persuite 
controller=pure_pursuit_controller(path, car.get_wheelbase())
next_index=0
cartrace=[]
closest_index=0
last_closest_index=0
cartrace.append(car.get_position())
finished= False
while not finished:
    steering_angle, finished=controller.compute_control(car.get_position(), car.get_vehicle_angle(), car.get_velocity())
    car.update_steering_angle(steering_angle*(180/np.pi))
    car.time_step()
    cartrace.append(car.get_position())

##Plot
cartrace=np.array(cartrace)


plt.plot(path[:, 0], path[:, 1], color="RED")
plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")
plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")
plt.show()
