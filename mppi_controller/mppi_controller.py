import numpy as np
class mppi_controller():
    def __init__(self,
                car,
                track,
                number_plans=1000,
                number_control_steps=60,
                physics_time_step=0.01,
                substeps=5,
                sd_steering=0.15,
                sd_acceleration=2,
                temperature=0.5,
                starting_plan=None, 
                seed=42,
                ao=9,
                min_sd_steering=0.003,
                max_sd_steering=0.2):
        self.rng=np.random.default_rng(seed)
        self.car=car
        self.track=track
        self.number_plans=number_plans
        self.number_control_steps=number_control_steps
        self.physics_time_step=physics_time_step
        self.substeps=substeps
        self.sd_steering=sd_steering
        self.sd_acceleration=sd_acceleration
        if starting_plan is None:
            self.best_current_plan=np.zeros((number_control_steps,2))
            self.best_current_plan[...,1]+=5
        else:
            self.best_current_plan=starting_plan.copy()
        self.temperature=temperature
        self.crashrate=[]
        self.effective_number_of_plans=[]
        self.ao=ao
        self.min_sd_steering=min_sd_steering
        self.max_sd_steering=max_sd_steering

    def rollout(self,state,V_shape_array):
        n, t, inputs=np.shape(V_shape_array)
        results=np.zeros((n,t,len(state)))
        results[...,-1,:]=state
        for i in range(t):
            current_timestep=results[...,i-1,:]
            for j in range(self.substeps):
                current_timestep=self.car.calculate_new_state(current_timestep, V_shape_array[...,i,:],self.physics_time_step)
            results[...,i,:]=current_timestep

        return results

    def cost_function(self, result, start_progress, start_index):
        n,t,shape=np.shape(result)
        index=start_index
        progress_arr=np.zeros(n)
        off_track_arr=np.full(n,False)
        last_progress=start_progress
        for i in range(t):
            index, progress, lateral,ontrack=self.track.locate(result[...,i,0:2], index)
            progress_arr+=self.track.progress_difference(last_progress,progress)
            last_progress=progress
            off_track_arr=np.where((ontrack==False)|(off_track_arr==True),True,False)
        cost=np.where(off_track_arr==True,-progress_arr+1000,-progress_arr)
        self.crashrate.append(np.sum(off_track_arr)/len(off_track_arr))
        return cost

    def compute_weights(self,costs):
        min_cost=np.min(costs) 
        weights=np.exp(-(costs-min_cost)/self.temperature)
        weights=weights/np.sum(weights)
        return weights
        
    def compute_control(self,state, progress, index):
        noise=np.zeros((self.number_plans,self.number_control_steps,2))
        sd_steering=(self.ao*self.car.get_wheelbase())/np.maximum(1e-5,(self.car.get_velocity()**2))
        sd_steering=np.clip(sd_steering,self.min_sd_steering,self.max_sd_steering)
        steering_noise=self.rng.normal(0, sd_steering,(self.number_plans,self.number_control_steps))
        acceleration_noise=self.rng.normal(0,self.sd_acceleration,(self.number_plans,self.number_control_steps))
        noise[...,0]+=steering_noise
        noise[...,1]+=acceleration_noise
        v_shape_array=self.best_current_plan+noise
        v_shape_array[..., 0]=np.clip(v_shape_array[..., 0],-self.car.get_max_steering_angle(),self.car.get_max_steering_angle())
        v_shape_array[..., 1]=np.clip(v_shape_array[..., 1],-self.car.get_max_brake(),self.car.get_max_acceleration())
        result=self.rollout(state,v_shape_array)
        cost=self.cost_function(result,progress,index)
        weights=self.compute_weights(cost)
        self.effective_number_of_plans.append(1/np.sum(weights**2))
        weights=weights[...,None,None]
        self.best_current_plan=np.sum(v_shape_array*weights,axis=0)
        car_controls=self.best_current_plan[0]
        self.best_current_plan=np.roll(self.best_current_plan,-1,0)
        self.best_current_plan[-1]=self.best_current_plan[-2]
        return car_controls

    def get_substep(self):
        return self.substeps
