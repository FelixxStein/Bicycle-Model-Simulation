import matplotlib.pyplot as plt
import numpy as np
import vehicle
from csv_to_coordinates import csv_to_coordinates
import matplotlib.pyplot as plt
from basic_drive_controller import basic_drive_controller
import pandas as pd
from plot import plot

#convert csv to coordinates
spielberg=csv_to_coordinates("Hockenheim.csv")
path=spielberg.get_midline()
borderleft=spielberg.get_border_left()
borderright=spielberg.get_border_right()
#create car
car=vehicle.dynamic_vehicle(starting_x=path[0][0], starting_y=path[0][1], max_acceleration=8)
car.set_starting_angle(path)

##controller 
BDC=basic_drive_controller.basic_drive_controller(path, car.get_max_brake(), car.get_max_acceleration(), car.get_wheelbase(),car.get_max_speed(), car.get_friction_coefficient_front(), car.get_friction_coefficient_rear(),car.get_downforce_factor(), car.get_mass())


r=0

cartrace=[]
cartrace.append(car.get_position())
finished= False
velocity_list=[car.get_velocity()]

while not finished:
    
    velocity=car.get_velocity()
    position=car.get_position()
    acceleration, steering_angle, finished = BDC.compute_control(position, car.get_velocity(), car.get_vx(), car.get_yaw(), car.get_yaw_rate(), 0.01)
    control_inputs=np.array([steering_angle,acceleration])
    car.time_step(control_inputs,0.01)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())
    r+=1
    if r==50000: break



cartrace=np.array(cartrace)

Plot=plot(borderleft, borderright)
Plot.plot_speed(cartrace,velocity_list)
Plot.show()

print("Time:", len(cartrace)*0.01)

