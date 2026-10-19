import matplotlib.pyplot as plt
import numpy as np
import vehicle
from track import track
import matplotlib.pyplot as plt
from basic_drive_controller import basic_drive_controller
import pandas as pd
from plot import plot
from mppi_controller.mppi_controller import mppi_controller
#create car
car=vehicle.dynamic_vehicle()
friction_coefficient=car.get_min_friction_coefficient()
downforce_factor=car.get_downforce_factor()
mass=car.get_mass()
v_max=car.get_max_speed()
#convert csv to coordinates
Track=track("Hockenheim.csv",friction_coefficient, downforce_factor, mass, v_max,car.get_max_brake(), profile_grip_factor=0.974)
path=Track.get_midline()
borderleft=Track.get_border_left()
borderright=Track.get_border_right()
car.set_starting_angle(path)
car.set_starting_position(path)

##controller 
BDC=basic_drive_controller.basic_drive_controller(Track, car.get_max_brake(), car.get_max_acceleration(), car.get_wheelbase(),car.get_max_speed(), car.get_friction_coefficient_front(), car.get_friction_coefficient_rear(),car.get_downforce_factor(), car.get_mass())


r=0

cartrace=[]
cartrace.append(car.get_position())
velocity_list=[car.get_velocity()]
'''
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
    steering_angle, acceleration = BDC.compute_control(position,index, car.get_velocity(), car.get_vx(), car.get_yaw(), car.get_yaw_rate(), 0.01)
    control_inputs=np.array([steering_angle,acceleration])
    car.time_step(control_inputs,0.01)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())



cartrace=np.array(cartrace)
'''

#Mppi controller test:
Mppi=mppi_controller(car,Track,1000,74,0.01,5,1.08,3.77,None,42,83,0.63,1.4)
index=0
total_progress=0
progress_old=0
r=0
s=Mppi.get_substep()
stuck=0
while True:
    velocity=car.get_velocity()
    index, progress,lateral,ontrack=Track.locate(car.get_position(),index)
    total_progress+=Track.progress_difference(progress_old, progress)
    progress_old=progress
    if total_progress>Track.get_track_length():break
    if r>20000 or ontrack==False: break
    r+=1
    if s==Mppi.get_substep():
        control_inputs= Mppi.compute_control(car.get_state(),progress,index)
        s=0
    if velocity<1: stuck+=1
    else: stuck=0
    if stuck>200: break
    s+=1
    car.time_step(control_inputs,0.01)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())



cartrace=np.array(cartrace)


Plot=plot(borderleft, borderright)

Plot.plot_speed(cartrace,velocity_list)
Plot.show()

print("Time:", len(cartrace)*0.01)

