import racesim.vehicle as vehicle
from racesim.track import Track 
from racesim.controllers.mppi_controller import MPPIController
import numpy as np
import csv
from pathlib import Path
from racesim.simulate_lap import simulate_lap
from datetime import datetime
from multiprocessing import Pool
class Tuning:
    def __init__(self,
                 number_plans=1000,
                 substeps=5,
                 physics_time_step=0.01,
                 csv_racetrack="Hockenheim.csv",
                 seed=420,
                 seeds=[25,1,2005],
                 K=20,
                 k=5,
                 max_steps=20000,
                 processes=5):
        self.processes=processes
        self.max_steps=max_steps
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M")
        timestamp=timestamp+".csv"
        REPO_ROOT = Path(__file__).resolve().parent.parent.parent
        self.file_path = REPO_ROOT /"results" / timestamp
        table_row=["generation", "set", "seed","temperature_factor", "ao", "sd_acceleration", "c_under", "c_over","profile_grip_factor", "number_control_steps", "c_close", "c_steer","reason", "progress", "lap_time", "score"]
        with open(self.file_path, 'w', newline='') as f: csv.writer(f).writerow([*table_row])
        self.K=K
        self.k=k
        self.rng=np.random.default_rng(seed)
        self.number_plans=number_plans
        self.substeps=substeps
        self.physics_time_step=physics_time_step
        self.csv_racetrack=csv_racetrack
        self.seeds=seeds
        self.log_array=np.array([True,True,True,True,True,False,False,True,True])
        self.min_values=np.array([0.05,0.5,0.5,0.3,0.1,0.8,30,0.1,0.1])
        self.max_values=np.array([2,100,5,3,20,1.05,80,2,50])
        self.z_min=np.where(self.log_array,np.log10(self.min_values), self.min_values)
        self.z_max=np.where(self.log_array,np.log10(self.max_values), self.max_values)
        self.mean_value_array=(self.z_max+self.z_min)/2
        self.o=(self.z_max-self.z_min)/4
        self.o_min=(self.z_max-self.z_min)/10
        self.starting_values=self.rng.uniform(self.z_min,self.z_max)
        self.best_score_so_far=100000
        self.variables_best_score_so_far=[]
        self.generation=0

    

    def one_gen(self):
        score_set=[]
        draw=self.rng.normal(self.mean_value_array,self.o, size=(self.K,len(self.mean_value_array)))
        draw=np.clip(draw,self.z_min,self.z_max)
        draw_real=np.where(self.log_array,10**draw,draw)
        jobs=[]
        for i in range(self.K):
            for j in self.seeds:
                jobs.append((draw_real[i,...],j,self.csv_racetrack,self.number_plans,self.physics_time_step,self.substeps,self.max_steps))
        with Pool(processes=self.processes) as pool: solution=pool.starmap(run_lap,jobs,chunksize=1)
        n=0
        for element in solution:
            set_index=n//len(self.seeds)
            seed=self.seeds[n%len(self.seeds)]
            with open(self.file_path, 'a', newline='') as f: csv.writer(f).writerow([self.generation, set_index, seed, *draw_real[set_index], element[0], element[1], element[2], element[3]])
            n+=1
        score_set=np.array(solution)
        score_set=score_set[...,3]
        score_set=score_set.reshape(self.K, len(self.seeds))
        score_set=score_set.mean(axis=1)
        scores_sorted=np.argsort(score_set)
        elite=draw[scores_sorted,...]
        elite=elite[0:self.k,...]
        self.mean_value_array=np.mean(elite,axis=0)
        self.o=np.std(elite,axis=0)
        self.o=np.maximum(self.o,self.o_min)
        if self.best_score_so_far>score_set[scores_sorted[0]]:
            self.best_score_so_far=score_set[scores_sorted[0]]
            variables_best_score_so_far=elite[0].copy()
            self.variables_best_score_so_far=np.where(self.log_array,10**variables_best_score_so_far,variables_best_score_so_far)
        self.generation+=1
        return np.mean(score_set)




    def run(self,generations):
        for i in range(generations):
            mean=self.one_gen()
            current_mean=np.where(self.log_array,10**self.mean_value_array,self.mean_value_array)
            print("score:",mean,"Mean:",current_mean)

    def set_start_median(self):
        array=[0.18059570323893012,3.255868523905106,2.3544670063559345,1.401774358294363,1.014917219517641,0.9144784340353015,67,0.7200081587015296,2.5184027645699096]
        self.mean_value_array=np.where(self.log_array,np.log10(array), array)
        self.o=(self.z_max-self.z_min)/8
        self.o=np.maximum(self.o,self.o_min)


def run_lap(params, seed,csv_racetrack, number_plans,physics_time_step, substeps,max_steps):
        temperature_factor=params[0]
        ao=params[1]
        sd_acceleration=params[2]
        c_under=params[3]
        c_over=params[4]
        profile_grip_factor=params[5]
        number_control_steps=int(round(params[6]))
        c_close=params[7]
        c_steer=params[8]
        car=vehicle.DynamicVehicle()
        track=Track(csv_racetrack=csv_racetrack,friction_coefficient=car.get_min_friction_coefficient(),downforce_factor=car.get_downforce_factor(),mass=car.get_mass(),v_max=car.get_max_speed(),max_break=car.get_max_brake(),profile_grip_factor=profile_grip_factor)
        mppi=MPPIController(car=car,track=track,number_plans=number_plans,number_control_steps=number_control_steps,physics_time_step=physics_time_step,substeps=substeps,sd_acceleration=sd_acceleration,temperature_factor=temperature_factor,starting_plan=None,seed=seed,ao=ao,c_under=c_under, c_over=c_over, c_close=c_close,c_steer=c_steer)
        car.set_starting_position(track.get_midline())
        car.set_starting_angle(track.get_midline())
        dictionary=simulate_lap(car=car,track=track,controller=mppi,max_steps=max_steps,dt=physics_time_step)
        reason=dictionary["reason"]
        total_progress=dictionary["progress"]
        time=dictionary["lap_time"]
        if reason==1:
            score=time
        else:
            score=((track.get_track_length()-total_progress)/33)+200
        return reason, total_progress, time, score

        
        


        