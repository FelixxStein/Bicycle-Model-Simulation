import numpy as np
import constants
class PID_speed_controller:
    def __init__(self,path ,max_break,max_acceleration, max_speed,lookahead_distance, max_lateral_acceleration, Kp,Ki,Kd, grip_usage, friction_coefficient, downforce_factor, car_mass):
        self.max_break=max_break
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.max_speed=max_speed
        self.max_lateral_acceleration=max_lateral_acceleration
        self.ek_prev=0
        self.Ik=0
        self.last_closest_index=0
        self.path=path
        self.lookahead_distance=lookahead_distance
        self.cumulative_point_distance=[0]
        self.corner_speed_per_point=[]
        self.curvature_per_pathpoint=[]
        self.grip_usage=grip_usage
        self.friction_coefficient=friction_coefficient
        self.downforce_factor=downforce_factor
        self.max_acceleration=max_acceleration
        self.car_mass=car_mass
        self.planner_break=0.6*grip_usage*friction_coefficient*constants.GRAVITY


        f=self.calculate_distance(self.path[-1],self.path[0])
        g=self.calculate_distance(self.path[0],self.path[1])
        h=self.calculate_distance(self.path[-1],self.path[1])
        s=0.5*(f+g+h)
        A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
        denom=(f*g*h)
        if denom>1e-6:
            curve=(4*A)/denom
            self.corner_speed_per_point.append(np.sqrt(self.max_lateral_acceleration/curve))
            self.curvature_per_pathpoint.append(curve)
        else:
            self.corner_speed_per_point.append(self.max_speed)
            self.curvature_per_pathpoint.append(0)
        
        dist_accumulated=0
        for i in range(0,len(self.path)-2):
            f=self.calculate_distance(self.path[i],self.path[i+1])
            g=self.calculate_distance(self.path[i+1],self.path[i+2])
            h=self.calculate_distance(self.path[i],self.path[i+2])
            s=0.5*(f+g+h)
            A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
            denom=(f*g*h)
            if denom>1e-6:
                curve=(4*A)/denom
                self.corner_speed_per_point.append(np.sqrt(self.max_lateral_acceleration/curve))
                self.curvature_per_pathpoint.append(curve)
            else:
                self.corner_speed_per_point.append(self.max_speed)
                self.curvature_per_pathpoint.append(0)

        dist_accumulated=dist_accumulated+self.calculate_distance(self.path[len(self.path)-2],self.path[len(self.path)-1])
        self.cumulative_point_distance.append(dist_accumulated)
        f=self.calculate_distance(self.path[len(self.path)-2],self.path[len(self.path)-1])
        g=self.calculate_distance(self.path[len(self.path)-1],self.path[0])
        h=self.calculate_distance(self.path[len(self.path)-2],self.path[0])
        s=0.5*(f+g+h)
        A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
        denom=(f*g*h)
        if denom>1e-6:
            curve=(4*A)/denom
            self.corner_speed_per_point.append(np.sqrt(self.max_lateral_acceleration/curve))
            self.curvature_per_pathpoint.append(curve)
        else:
            self.corner_speed_per_point.append(self.max_speed)
            self.curvature_per_pathpoint.append(0)

       

    def calculate_distance(self,vek2, vek1):
        return np.sqrt(np.sum((vek2 - vek1)**2))


    def calculate_min_v_allowed(self):
        v_allowed_min=self.max_speed
        for i in range(self.last_closest_index, min(self.last_closest_index+self.lookahead_distance, len(self.path)-3)):
            v_allowed=np.sqrt(self.corner_speed_per_point[i]**2+2*self.planner_break*(self.cumulative_point_distance[i]-self.cumulative_point_distance[self.last_closest_index]))
            if v_allowed<v_allowed_min:
                v_allowed_min=v_allowed
        return v_allowed_min
    def calculate_driver_limits(self, vx, yaw_rate):
        available = self.grip_usage*self.friction_coefficient*(constants.GRAVITY+self.downforce_factor*(vx**2)/self.car_mass)
        lateral_needed_pathbased= (vx**2)*abs(self.curvature_per_pathpoint[self.last_closest_index])
        lateral_needed_measured=vx*abs(yaw_rate)
        lateral_needed=np.maximum(lateral_needed_pathbased, lateral_needed_measured)
        allowed_longitudinal =np.sqrt(max(0, (available**2)-(lateral_needed**2)))
        
        return allowed_longitudinal

        
    def compute_control(self, position, velocity, vx, yaw_rate, delta_t):
        self.get_closest_index(position)
        v_allowed_min=self.calculate_min_v_allowed()
        v_target=min(self.max_speed, v_allowed_min)
        ek=v_target-velocity
        self.Ik = self.Ik + ek * delta_t
        self.Ik=np.clip(self.Ik,-5.0,+5.0)
    
        Dk=(ek-self.ek_prev)/delta_t
        acceleration=self.Kp*ek+self.Ki*self.Ik+self.Kd*Dk
        self.ek_prev=ek
        allowed_longitudinal=self.calculate_driver_limits(vx, yaw_rate)
        acceleration=np.clip(acceleration,-allowed_longitudinal, min(allowed_longitudinal, self.max_acceleration))
        return acceleration
    

    
