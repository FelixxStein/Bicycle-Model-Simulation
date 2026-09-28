import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
from csv_to_coordinates import csv_to_coordinates
from pure_pursuit_controller import pure_pursuit_controller
import matplotlib.pyplot as plt
from PID_speed_controller import PID_speed_controller


#convert csv to coordinates
spielberg=csv_to_coordinates("Spielberg.csv")
path=spielberg.get_midline()
borderleft=spielberg.get_border_left()
borderright=spielberg.get_border_right()
#create car
car=vehicle.vehicle(starting_x=path[0][0], starting_y=path[0][1], starting_velocity=0)
car.set_starting_angle(path)

##controller 
steering_controller=pure_pursuit_controller(path, car.get_wheelbase())
speed_controller=PID_speed_controller(path,car.get_max_brake(), car.get_max_speed())




cartrace=[]
cartrace.append(car.get_position())
finished= False
velocity_list=[car.get_velocity()]
while not finished:
    steering_angle, finished=steering_controller.compute_control(car.get_position(), car.get_vehicle_angle(), car.get_velocity())
    
    velocity=car.get_velocity()
    position=car.get_position()
    acceleration=speed_controller.compute_control(position,velocity)
    car.update_acceleration(acceleration)
    car.update_steering_angle(steering_angle*(180/np.pi))
    car.time_step()
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())

##Plot
cartrace=np.array(cartrace)


#plt.plot(path[:, 0], path[:, 1], color="RED")
plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")
#plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")

sc = plt.scatter(
    cartrace[:, 0],
    cartrace[:, 1],
    c=velocity_list,
    cmap="coolwarm",  # Blau = langsam, Rot = schnell
    s=3,  # Punktgröße
)

cbar = plt.colorbar(sc)
cbar.set_label("Geschwindigkeit [m/s]")
plt.show()

plt.show()
