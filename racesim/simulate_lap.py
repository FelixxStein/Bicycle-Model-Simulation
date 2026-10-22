import numpy as np
def simulate_lap(car, track, controller, max_steps=20000,dt=0.01):
    cartrace=[car.get_position()]
    velocity_list=[car.get_velocity()]
    heading_angle=[car.get_yaw()]
    controls=[[0,0]]
    index=0
    stuck=0
    progress_stuck=0
    total_progress=0
    progress_old=0
    r=0
    while True:
        velocity=car.get_velocity()
        index, progress,_,ontrack=track.locate(car.get_position(),index)
        if velocity<1:
            stuck+=1
        else: 
            stuck=0
            progress_stuck=progress
        if (stuck>200) & (np.isclose(progress,progress_stuck,atol=0.1)):
            reason=3
            break
        total_progress+=track.progress_difference(progress_old, progress)
        progress_old=progress
        if total_progress>track.get_track_length():
            reason=1
            break
        if r>max_steps:
            reason=4
            break
        if ontrack==False:
            reason=2
            break
        r+=1
        control_inputs=controller.compute_control(car.get_state(),progress,index)
        car.time_step(control_inputs,dt)
        cartrace.append(car.get_position())
        velocity_list.append(car.get_velocity())
        heading_angle.append(car.get_yaw())
        controls.append(control_inputs)
    cartrace=np.stack(cartrace)
    velocity_list=np.stack(velocity_list)
    heading_angle=np.stack(heading_angle)
    controls=np.stack(controls)
    time=r*dt
    dictionary={"reason":reason,"lap_time":time,"progress":total_progress,"cartrace":cartrace,"velocity_list":velocity_list,"heading_angle":heading_angle,"controls":controls, "steps":r}
    return dictionary