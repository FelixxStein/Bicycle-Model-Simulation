import matplotlib.pyplot as plt
import numpy as np
import vehicle
from track import track
import matplotlib.pyplot as plt
from basic_drive_controller import basic_drive_controller
import pandas as pd
from plot import plot

#convert csv to coordinates
Track=track("Hockenheim.csv")
path=Track.get_midline()
borderleft=Track.get_border_left()
borderright=Track.get_border_right()
#create car
car=vehicle.dynamic_vehicle(starting_x=path[0][0], starting_y=path[0][1], max_acceleration=8)
car.set_starting_angle(path)

##controller 
BDC=basic_drive_controller.basic_drive_controller(Track, car.get_max_brake(), car.get_max_acceleration(), car.get_wheelbase(),car.get_max_speed(), car.get_friction_coefficient_front(), car.get_friction_coefficient_rear(),car.get_downforce_factor(), car.get_mass())


r=0

cartrace=[]
cartrace.append(car.get_position())
velocity_list=[car.get_velocity()]
index=0
total_progress=0
progress_old=0
r=0
while True:
    
    velocity=car.get_velocity()
    position=car.get_position()
    index, progress,lateral,ontrack=Track.locate(position,index)
    total_progress+=Track.progress_difference(progress_old, progress)
    progress_old=progress
    if total_progress>Track.get_track_length():break
    if r>50000 or ontrack==False: break
    r+=1
    acceleration, steering_angle = BDC.compute_control(position,index, car.get_velocity(), car.get_vx(), car.get_yaw(), car.get_yaw_rate(), 0.01)
    control_inputs=np.array([steering_angle,acceleration])
    car.time_step(control_inputs,0.01)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())



cartrace=np.array(cartrace)

Plot=plot(borderleft, borderright)
Plot.plot_speed(cartrace,velocity_list)
Plot.show()

print("Time:", len(cartrace)*0.01)

