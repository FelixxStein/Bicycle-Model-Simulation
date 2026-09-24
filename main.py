import matplotlib.pyplot as plt
import numpy as np
import vehicle
car=vehicle.vehicle(velocity=100)
x_coordinate_list=[]
y_coordinate_list=[]

for i in range(4):
    car.update()
    y_coordinate_list.append(car.y)
    x_coordinate_list.append(car.x)

plt.plot(x_coordinate_list,y_coordinate_list)
plt.show()


