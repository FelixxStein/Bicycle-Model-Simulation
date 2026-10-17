import numpy as np
class mppi_controller():
    def __init__(self,
                car,
                track,
                number_plans=200,
                number_control_steps=30,
                physics_time_step=0.01,
                substeps=5,
                sd_steering=0.05,
                sd_acceleration=2):
        self.car=car
        self.track=track
        self.number_plans=number_plans
        self.number_control_steps=number_control_steps
        self.physics_time_step=physics_time_step
        self.substeps=substeps

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
        return cost

    def compute_weights(self,costs):
        pass
    def compute_control(self,state, progress, index):
        noise=np.zeros((self.number_plans,self.number_control_steps,2))
        acceleration_mean=(self.car.get_max_acceleration()+self.car.get_max_brake())/2
        steering_noise=np.random.normal(0,)
        acceleration_noise=np.random.normal()
        