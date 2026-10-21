import numpy as np

class PurePursuitController:
    def __init__(self,track, wheel_base, lookahead_distance_minimum=3.0, lookahead_distance_maximum=20, lookahead_scale_factor=1.5, lookahead_distance_base=1):
        self.lookahead_min=lookahead_distance_minimum
        self.lookahead_max=lookahead_distance_maximum
        self.lookahead_gain=lookahead_scale_factor
        self.lookahead_base=lookahead_distance_base
        self.path=track.get_midline()
        self.wheel_base=wheel_base

    def update_lookahead_distance(self,velocity):
        ld=self.lookahead_gain*abs(velocity)+self.lookahead_base
        return min(max(ld,self.lookahead_min),self.lookahead_max)


    def find_next_waypoint(self, position,index,vehicle_angle,lookahead):
        distance=0
        break_index=0
        i=index
        while True:
            distance=np.sqrt(np.sum((position - self.path[i])**2))
            dxy=self.path[i%len(self.path)]-position
            if distance>lookahead and dxy[0] * np.cos(vehicle_angle) + dxy[1]*np.sin(vehicle_angle)>0:
                return i%len(self.path)
            i+=1
            i=i%len(self.path)
            break_index+=1
            if break_index==len(self.path): 
                raise RuntimeError("No next index found")
            
        
                
    def compute_control(self,position, index, vehicle_angle, velocity):
        lookahead_distance=self.update_lookahead_distance(velocity)
        next_index=self.find_next_waypoint(position, index, vehicle_angle, lookahead_distance)
        dxy=self.path[next_index]-position
        distance=np.sqrt(np.sum((position - self.path[next_index])**2))
        ylokal=-dxy[0]*np.sin(vehicle_angle)+dxy[1]*np.cos(vehicle_angle)
        path_curvature=(2*ylokal/distance**2)
        steering_angle=np.arctan(path_curvature*self.wheel_base)
        return steering_angle
        