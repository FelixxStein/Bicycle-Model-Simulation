import numpy as np
class MPPIController():
    def __init__(self,
                car,
                track,*,
                number_plans=1000,
                number_control_steps=60,
                physics_time_step=0.01,
                substeps=5,
                sd_acceleration=2,
                temperature_factor=0.5,
                starting_plan=None, 
                seed=42,
                ao=70,
                c_under=1,
                c_over=5,
                min_sd_steering=0.003,
                max_sd_steering=0.2,
                crash_penalty=1000,
                d_safe_min=0.3,
                scale_factor_width_speed=0.02,
                c_close=1,
                noise_factor=0.9,
                temperature_epsilon=0.1, 
                c_steer=10, 
                log_rollouts=False):
        self.temperature_epsilon=temperature_epsilon
        self.log_rollouts=log_rollouts
        self.off_track_array=[]
        self.path_trajectory_array=[]
        self.weights_array=[]
        self.current_trajectory=[]
        self.c_steer=c_steer
        self.noise_factor=noise_factor
        self.scale_factor_width_speed=scale_factor_width_speed
        self.c_close=c_close
        self.d_safe_min=d_safe_min
        self.c_under=c_under
        self.c_over=c_over
        self.rng=np.random.default_rng(seed)
        self.seed=seed
        self.car=car
        self.track=track
        self.number_plans=number_plans
        self.number_control_steps=number_control_steps
        self.physics_time_step=physics_time_step
        self.substeps=substeps
        self.sd_acceleration=sd_acceleration
        if starting_plan is None:
            self.best_current_plan=np.zeros((number_control_steps,2))
            self.best_current_plan[...,1]+=5
        else:
            self.best_current_plan=starting_plan.copy()
        self.temperature_factor=temperature_factor
        self.crashrate=[]
        self.effective_number_of_plans=[]
        self.ao=ao
        self.min_sd_steering=min_sd_steering
        self.max_sd_steering=max_sd_steering
        self.v_allowed=self.track.get_v_allowed()
        self.crash_penalty=crash_penalty
        self.width_left=self.track.get_distance_left()
        self.width_right=self.track.get_distance_right()
        self.last_steering=0

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

    def cost_function(self, result, start_progress, start_index, v_shape_arr):
        n,t,shape=np.shape(result)
        index=start_index
        progress_arr=np.zeros(n)
        off_track_arr=np.full(n,False)
        last_progress=start_progress
        distance_array=np.zeros(n)
        for i in range(t):
            index, progress, lateral,ontrack=self.track.locate(result[...,i,0:2], index)
            distance_array+=np.maximum(0,(self.d_safe_min+self.scale_factor_width_speed*np.linalg.norm(result[...,i,3:5],axis=-1))-np.minimum(self.width_left[index]-lateral,self.width_right[index]+lateral))**2
            off_track_arr=np.where((ontrack==False)|(off_track_arr==True),True,False)
            progress_arr=progress_arr+np.where(~ off_track_arr,self.track.progress_difference(last_progress,progress),0)
            last_progress=progress
        v_end=np.linalg.norm(result[...,-1,3:5],axis=-1)
        v_allowed=self.v_allowed[index]
        steering_diff=np.sum(((np.diff(v_shape_arr[...,0],axis=1,prepend=self.last_steering))/self.car.get_max_steering_angle())**2,axis=1)
        cost=np.where(off_track_arr==True,-progress_arr+self.crash_penalty,-progress_arr)-self.c_under*np.minimum(v_end, v_allowed)+self.c_over*np.maximum(0,v_end-v_allowed)+self.c_close*distance_array+self.c_steer*steering_diff
        self.crashrate.append(np.sum(off_track_arr)/len(off_track_arr))
        return cost, off_track_arr

    def compute_weights(self,costs, off_track_arr):
        if np.any(~ off_track_arr):
            spread=np.std(costs[~ off_track_arr])
        else:
            spread=np.std(costs)
        temperature=self.temperature_factor*spread+self.temperature_epsilon
        min_cost=np.min(costs) 
        weights=np.exp(-(costs-min_cost)/temperature)
        weights=weights/np.sum(weights)
        return weights
        
    def compute_control(self,state, progress, index):
        noise=np.zeros((self.number_plans,self.number_control_steps,2))
        sd_steering=(self.ao*self.car.get_wheelbase())/np.maximum(5,(self.car.get_velocity()**2))
        sd_steering=np.clip(sd_steering,self.min_sd_steering,self.max_sd_steering)
        steering_noise=self.rng.normal(0, sd_steering,(self.number_plans,self.number_control_steps))
        acceleration_noise=self.rng.normal(0,self.sd_acceleration,(self.number_plans,self.number_control_steps))
        for i in range(1,self.number_control_steps):
            steering_noise[...,i]=self.noise_factor*steering_noise[...,i-1]+np.sqrt(1-(self.noise_factor)**2)*steering_noise[...,i]
            acceleration_noise[...,i]=self.noise_factor*acceleration_noise[...,i-1]+np.sqrt(1-(self.noise_factor)**2)*acceleration_noise[...,i]
        noise[...,0]+=steering_noise
        noise[...,1]+=acceleration_noise
        v_shape_array=self.best_current_plan+noise
        v_shape_array[..., 0]=np.clip(v_shape_array[..., 0],-self.car.get_max_steering_angle(),self.car.get_max_steering_angle())
        v_shape_array[..., 1]=np.clip(v_shape_array[..., 1],-self.car.get_max_brake(),self.car.get_max_acceleration())
        given_sets = np.array([
            self.best_current_plan,
            np.stack([
                self.best_current_plan[..., 0], 
                np.full((self.number_control_steps), self.car.get_max_acceleration())
            ], axis=-1),
            np.stack([
                self.best_current_plan[..., 0], 
                np.full((self.number_control_steps), -self.car.get_max_brake())
            ], axis=-1)
        ])
        v_shape_array=np.concatenate((v_shape_array,given_sets),axis=0)
        result=self.rollout(state,v_shape_array)
        cost, off_track_arr=self.cost_function(result,progress,index, v_shape_arr=v_shape_array)
        weights=self.compute_weights(cost, off_track_arr)
        self.effective_number_of_plans.append(1/np.sum(weights**2))
        weights_appended=weights[...,None,None]
        self.best_current_plan=np.sum(v_shape_array*weights_appended,axis=0)
        car_controls=self.best_current_plan[0]
        self.last_steering=car_controls[0]
        if self.log_rollouts==True:
            #best 30 arrays
            weights_sorted=np.argsort(-weights)
            best_thirty=weights_sorted[0:30]
            #random arrays of rest
            all_indices=np.arange(len(weights))
            valid_indices=np.setdiff1d(all_indices,best_thirty)
            selected_indices=np.concatenate((best_thirty,np.random.choice(valid_indices, size=20, replace=False)))
            self.off_track_array.append(off_track_arr[selected_indices])
            self.path_trajectory_array.append(result[selected_indices,...,0:2])
            selected_weights=weights[selected_indices]/np.max(weights[selected_indices])
            self.weights_array.append(selected_weights)
            self.current_trajectory.append(self.rollout(state,self.best_current_plan[None,...])[0,...,0:2])
        self.best_current_plan=np.roll(self.best_current_plan,-1,0)
        self.best_current_plan[-1]=self.best_current_plan[-2]
        return car_controls

    def get_substep(self):
        return self.substeps

    def get_logs(self):
        return np.stack(self.path_trajectory_array), np.stack(self.weights_array), np.stack(self.off_track_array), np.stack(self.current_trajectory)

    def get_seed(self):
        return self.seed