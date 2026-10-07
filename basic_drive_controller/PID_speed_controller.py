import numpy as np

class PID_speed_controller:
    def __init__(self,path ,max_break, max_speed,lookahead_distance=100,delta_t=0.01, max_lateral_acceleration=2, Kp=1.5,Ki=0.1,Kd=0.01):
        self.max_break=max_break
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.max_speed=max_speed
        self.max_lateral_acceleration=max_lateral_acceleration
        self.delta_t=delta_t
        self.ek_prev=0
        self.Ik=0
        self.last_closest_index=0
        self.path=path
        self.lookahead_distance=lookahead_distance
        self.cumulative_point_distance=[0]
        self.corner_speed_per_point=[]

        f=self.calculate_distance(self.path[-1],self.path[0])
        g=self.calculate_distance(self.path[0],self.path[1])
        h=self.calculate_distance(self.path[-1],self.path[1])
        s=0.5*(f+g+h)
        A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
        denom=(f*g*h)
        if denom>1e-6:
            curve=(4*A)/denom
            self.corner_speed_per_point.append(np.sqrt(self.max_lateral_acceleration/curve))
        else:
            self.corner_speed_per_point.append(self.max_speed)
        
        dist_accumulated=0
        for i in range(0,len(self.path)-2):
            dist_accumulated=dist_accumulated+self.calculate_distance(self.path[i],self.path[i+1])
            self.cumulative_point_distance.append(dist_accumulated)
            f=self.calculate_distance(self.path[i],self.path[i+1])
            g=self.calculate_distance(self.path[i+1],self.path[i+2])
            h=self.calculate_distance(self.path[i],self.path[i+2])
            s=0.5*(f+g+h)
            A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
            denom=(f*g*h)
            if denom>1e-6:
                curve=(4*A)/denom
                self.corner_speed_per_point.append(np.sqrt(self.max_lateral_acceleration/curve))
            else:
                self.corner_speed_per_point.append(self.max_speed)

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
        else:
            self.corner_speed_per_point.append(self.max_speed)

       

    def calculate_distance(self,vek2, vek1):
        return np.sqrt(np.sum((vek2 - vek1)**2))

    def get_closest_index(self, position):
        closest_index=self.last_closest_index
        for i in range(self.last_closest_index, min(self.last_closest_index+40, len(self.path))):
            if self.calculate_distance(position, self.path[i])<self.calculate_distance(position, self.path[closest_index]):
                closest_index=i
        self.last_closest_index=closest_index

    def calculate_min_v_allowed(self):
        v_allowed_min=self.max_speed
        for i in range(self.last_closest_index, min(self.last_closest_index+self.lookahead_distance, len(self.path)-3)):
            v_allowed=np.sqrt(self.corner_speed_per_point[i]**2+2*self.max_break*(self.cumulative_point_distance[i]-self.cumulative_point_distance[self.last_closest_index]))
            if v_allowed<v_allowed_min:
                v_allowed_min=v_allowed
        return v_allowed_min

    def compute_control(self, position, velocity):
        self.get_closest_index(position)
        v_allowed_min=self.calculate_min_v_allowed()
        v_target=min(self.max_speed, v_allowed_min)
        ek=v_target-velocity
        self.Ik = self.Ik + ek * self.delta_t
        self.Ik=np.clip(self.Ik,-5.0,+5.0)
    
        Dk=(ek-self.ek_prev)/self.delta_t
        acceleration=self.Kp*ek+self.Ki*self.Ik+self.Kd*Dk
        self.ek_prev=ek
        return acceleration

    
