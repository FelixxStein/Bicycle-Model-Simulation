from basic_drive_controller.PID_speed_controller import PID_speed_controller
from basic_drive_controller.pure_pursuit_controller import pure_pursuit_controller

class basic_drive_controller:
    def __init__(self,
                track,
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
        self.track=track
        friction_coefficient=min(friction_coefficient_rear, friction_coefficient_front)
        self.PID=PID_speed_controller(track, max_break, max_acceleration,max_speed,lookahead_distance_PID,max_lateral_acceleration, Kp,Ki,Kd, grip_usage, friction_coefficient, downforce_factor,car_mass)
        self.PP=pure_pursuit_controller(track, wheel_base, lookahead_distance_minimum_PP, lookahead_distance_maximum_PP, lookahead_scale_factor_PP, lookahead_distance_base_PP)

    def compute_control(self, position, index, velocity, vx, vehicle_angle,yaw_rate, delta_t):

        steering_angle=self.PP.compute_control(position,index, vehicle_angle, velocity)
        acceleration=self.PID.compute_control(index,velocity, vx, yaw_rate, delta_t)
        return acceleration, steering_angle