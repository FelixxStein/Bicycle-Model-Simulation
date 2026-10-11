from basic_drive_controller.PID_speed_controller import PID_speed_controller
from basic_drive_controller.pure_pursuit_controller import pure_pursuit_controller

class basic_drive_controller:
    def __init__(self,
                path,
                max_break,
                max_acceleration,
                wheel_base,
                max_speed,
                friction_coefficient_front,
                friction_coefficient_rear,
                downforce_factor,
                car_mass,
                lookahead_distance_PID=200,
                max_lateral_acceleration=8,
                Kp=5,
                Ki=0.1,
                Kd=0.01,
                lookahead_distance_minimum_PP=3.0,
                lookahead_distance_maximum_PP=20,
                lookahead_scale_factor_PP=1.5,
                lookahead_distance_base_PP=1,
                grip_usage=0.85):
        friction_coefficient=min(friction_coefficient_rear, friction_coefficient_front)
        self.PID=PID_speed_controller(path,max_break, max_acceleration,max_speed,lookahead_distance_PID,max_lateral_acceleration, Kp,Ki,Kd, grip_usage, friction_coefficient, downforce_factor,car_mass)
        self.PP=pure_pursuit_controller(path, wheel_base, lookahead_distance_minimum_PP, lookahead_distance_maximum_PP, lookahead_scale_factor_PP, lookahead_distance_base_PP)

    def compute_control(self, position, velocity, vx, vehicle_angle,yaw_rate, delta_t):
        steering_angle, finished=self.PP.compute_control(position, vehicle_angle, velocity)
        acceleration=self.PID.compute_control(position,velocity, vx, yaw_rate, delta_t)
        return acceleration, steering_angle, finished