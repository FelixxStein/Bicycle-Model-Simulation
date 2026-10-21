import matplotlib.pyplot as plt
import numpy as np
import vehicle
from track import track
import matplotlib.pyplot as plt
from basic_drive_controller.basic_drive_controller import BasicDriveController
import pandas as pd
from plot import plot
from mppi_controller.mppi_controller import MPPIController
from pathlib import Path
#create car
car=vehicle.dynamic_vehicle()
friction_coefficient=car.get_min_friction_coefficient()
downforce_factor=car.get_downforce_factor()
mass=car.get_mass()
v_max=car.get_max_speed()
dt=0.01
#convert csv to coordinates
Track=track("Spielberg.csv",friction_coefficient=friction_coefficient, downforce_factor=downforce_factor, mass=mass, v_max=v_max,max_break=car.get_max_brake(), profile_grip_factor=0.9144784340353015)
path=Track.get_midline()
borderleft=Track.get_border_left()
borderright=Track.get_border_right()
car.set_starting_angle(path)
car.set_starting_position(path)

##controller 
BDC=BasicDriveController.basic_drive_controller(Track, car.get_max_brake(), car.get_max_acceleration(), car.get_wheelbase(),car.get_max_speed(), car.get_friction_coefficient_front(), car.get_friction_coefficient_rear(),car.get_downforce_factor(), car.get_mass())


r=0

cartrace=[]
cartrace.append(car.get_position())
velocity_list=[car.get_velocity()]
heading_angle=[car.get_yaw()]
controls=[[0,0]]
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
Mppi=MPPIController(car,Track,number_plans=1000,number_control_steps=67,physics_time_step=0.01,substeps=5,sd_acceleration=2.3544670063559345,temperature_factor=0.18059570323893012,starting_plan=None,seed=1899,ao=3.255868523905106,c_under=1.401774358294363,c_over=1.014917219517641,c_close=0.7200081587015296,c_steer=2.5184027645699096, log_rollouts=True)
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
    car.time_step(control_inputs,dt)
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())
    heading_angle.append(car.get_yaw())
    controls.append(control_inputs)

path_trajectory_array, weights_array, off_track_array,current_trajectory=Mppi.get_logs()

cartrace=np.stack(cartrace)
velocity_list=np.stack(velocity_list)
heading_angle=np.stack(heading_angle)
controls=np.stack(controls)


run_dir = Path("runs")
run_dir.mkdir(exist_ok=True)

np.savez(run_dir / f"seed_{Mppi.get_seed()}",midline=Track.get_midline(),border_left=Track.get_border_left(),border_right=Track.get_border_right(),cartrace=cartrace,velocity_list=velocity_list,heading_angle=heading_angle,controls=controls,path_trajectory_array=path_trajectory_array,weights_array=weights_array,off_track_array=off_track_array,current_trajectory=current_trajectory,seed=Mppi.get_seed(),dt=dt,lap_time=len(cartrace)*0.01, substeps=Mppi.get_substep(),wheelbase=car.get_wheelbase())



Plot=plot(borderleft, borderright)

Plot.plot_speed(cartrace,velocity_list)
Plot.show()

print("Time:", len(cartrace)*0.01)

