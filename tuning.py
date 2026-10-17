import vehicle
from track import track 
from mppi_controller.mppi_controller import mppi_controller
import numpy as np
class tuning:
    def __init__(self,
                 number_plans=1000,
                 substeps=5,
                 physics_time_step=0.01,
                 csv_racetrack="Hockenheim.csv",
                 seeds=[42,69,1900]):
        self.number_plans=number_plans
        self.substeps=substeps
        self.physics_time_step=physics_time_step
        self.csv_racetrack=csv_racetrack
        self.seeds=seeds
        self.log_array=np.array(True,True,True,True,True,False,False)
        self.min_values=np.array(0.3,5,0.5,0.3,1,0.8,30)
        self.max_values=np.array(20,100,5,3,20,1.05,80)
        self.z_min=np.where(self.log_array,np.log10(self.min_values), self.min_values)
        self.z_max=np.where(self.log_array,np.log10(self.max_values), self.max_values)

    def run_lap(self, params, seed):
        temperature=params[0]
        ao=params[1]
        sd_acceleration=params[2]
        c_under=params[3]
        c_over=params[4]
        profile_grip_factor=params[5]
        number_control_steps=int(round(params[6]))
        car=vehicle.dynamic_vehicle()
        Track=track(self.csv_racetrack,car.get_min_friction_coefficient(),car.get_downforce_factor(),car.get_mass(),car.get_max_speed(),profile_grip_factor)
        mppi=mppi_controller(car,Track,self.number_plans,number_control_steps,self.physics_time_step,self.substeps,sd_acceleration,temperature,None,seed,ao,c_under,c_over)
        index=0
        total_progress=0
        progress_old=0
        r=0
        s=mppi.get_substep()
        stuck=0
        reason=0 #1=lap complete 2=off track 3=stuck 4=time limit
        time=0
        while True:
            velocity=car.get_velocity()
            index, progress,lateral,ontrack=Track.locate(car.get_position(),index)
            total_progress+=Track.progress_difference(progress_old, progress)
            progress_old=progress
            if total_progress>Track.get_track_length():
                reason=1
                break
            if r>20000:
                reason=4
                break
            if ontrack==False:
                reason=2
                break
            r+=1
            if s==mppi.get_substep():
                control_inputs=mppi.compute_control(car.get_state(),progress,index)
                s=0
            if velocity<1:
                stuck+=1
            else: stuck=0
            if stuck>200:
                reason=3
                break
            s+=1
            car.time_step(control_inputs,0.01)
            time+=0.01
        if reason==1:
            score=time
        else:
            score=time+((Track.get_track_length()-total_progress)/33)+200
        return reason, total_progress, time, score

        