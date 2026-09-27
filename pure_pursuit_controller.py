import numpy as np

class pure_pursuit_controller:
    def __init__(self,path, wheel_base, lookahead_distance_minimum=3.5, lookahead_distance_maximum=20, lookahead_scale_factor=1.5, lookahead_distance_base=1):
        self.lookahead_min=lookahead_distance_minimum
        self.lookahead_max=lookahead_distance_maximum
        self.lookahead_gain=lookahead_scale_factor
        self.lookahead_base=lookahead_distance_base
        self.last_closest_index=0
        self.path=path
        self.wheel_base=wheel_base

    def update_lookahead_distance(self,velocity):
        ld=self.lookahead_gain*abs(velocity)+self.lookahead_base
        return min(max(ld,self.lookahead_min),self.lookahead_max)

    def find_closest_index(self, car_position):
        closest_index=0
        for i in range(self.last_closest_index, min(self.last_closest_index+40, len(self.path)-1)):
            if np.sqrt(np.sum((car_position - self.path[i])**2))<np.sqrt(np.sum((car_position - self.path[closest_index])**2)):
                closest_index=i
        self.last_closest_index=closest_index

    def find_next_waypoint(self, position,vehicle_angle,lookahead):
        distance=0
        for i in range(self.last_closest_index, len(self.path)-1):
            distance=np.sqrt(np.sum((position - self.path[i])**2))
            dxy=self.path[i]-position
            if distance>lookahead and dxy[0] * np.cos(np.radians(vehicle_angle)) + dxy[1]*np.sin(np.radians(vehicle_angle))>0:
                return i
        return len(self.path)-1
        
                
    def compute_control(self,position, vehicle_angle, velocity):
        self.find_closest_index(position)
        lookahead_distance=self.update_lookahead_distance(velocity)
        next_index=self.find_next_waypoint(position, vehicle_angle, lookahead_distance)
        if np.sqrt(np.sum((position - self.path[next_index])**2))<1 and next_index==len(self.path)-1:
            return 0.0, True
        dxy=self.path[next_index]-position
        distance=np.sqrt(np.sum((position - self.path[next_index])**2))
        ylokal=-dxy[0]*np.sin(np.radians(vehicle_angle))+dxy[1]*np.cos(np.radians(vehicle_angle))
        path_curvature=(2*ylokal/distance**2)
        steering_angle=np.arctan(path_curvature*self.wheel_base)
        return steering_angle, False
        