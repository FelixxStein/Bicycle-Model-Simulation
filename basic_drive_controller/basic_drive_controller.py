from basic_drive_controller.PID_speed_controller import PID_speed_controller
from basic_drive_controller.pure_pursuit_controller import pure_pursuit_controller

class basic_drive_controller:
    def __init__(self,
                path,
                max_break,
                wheel_base,
                max_speed,
                lookahead_distance_PID=10,
                delta_t=0.01,
                max_lateral_acceleration=6,
                Kp=1.5,
                Ki=0.1,
                Kd=0.01,
                lookahead_distance_minimum_PP=3.0,
                lookahead_distance_maximum_PP=20,
                lookahead_scale_factor_PP=1.5,
                lookahead_distance_base_PP=1):
        self.PID=PID_speed_controller(path,max_break,max_speed,lookahead_distance_PID,delta_t,max_lateral_acceleration, Kp,Ki,Kd)
        self.PP=pure_pursuit_controller(path, wheel_base, lookahead_distance_minimum_PP, lookahead_distance_maximum_PP, lookahead_scale_factor_PP, lookahead_distance_base_PP)

    def compute_control(self, position, velocity, vehicle_angle):
        steering_angle, finished=self.PP.compute_control(position, vehicle_angle, velocity)
        acceleration=self.PID.compute_control(position,velocity)
        return acceleration, steering_angle, finished