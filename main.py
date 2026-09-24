import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
car=vehicle.vehicle(velocity=15,starting_steering_angle=30)
x_coordinate_list=[]
y_coordinate_list=[]
for i in range(10):
    for i in range(300):
        car.update_position()
        y_coordinate_list.append(car.y)
        x_coordinate_list.append(car.x)
    car.update_steering_angle(-60)
    for i in range(300):
        car.update_position()
        y_coordinate_list.append(car.y)
        x_coordinate_list.append(car.x)
    car.update_steering_angle(60)


animation=animate.animate()
animation.create(x_coordinate_list,y_coordinate_list)


