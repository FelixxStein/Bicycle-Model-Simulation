import matplotlib.pyplot as plt
import numpy as np
import vehicle
import math
import animate
from csv_to_coordinates import csv_to_coordinates
from pure_pursuit_controller import pure_pursuit_controller
import matplotlib.pyplot as plt
###calculate distance...
def calculate_distance(vek2, vek1):
    return np.sqrt(np.sum((vek2 - vek1)**2))

#convert csv to coordinates
spielberg=csv_to_coordinates("Spielberg.csv")
path=spielberg.get_midline()
borderleft=spielberg.get_border_left()
borderright=spielberg.get_border_right()
#create car
car=vehicle.vehicle(starting_x=path[0][0], starting_y=path[0][1], velocity=20)
car.set_starting_angle(path)

##pure persuite 
controller=pure_pursuit_controller(path, car.get_wheelbase())
cartrace=[]
last_closest_index=0
cartrace.append(car.get_position())
finished= False
max_lateral_acceleration=6 #in m/s^2
max_speed=33 #in m/s^2
Kp=1.5
Ki=0.1
Kd=0.01
delta_t=0.01
ek_prev=0
Ik=0
velocity_list=[car.get_velocity()]
max_break=car.max_break
while not finished:
    steering_angle, finished=controller.compute_control(car.get_position(), car.get_vehicle_angle(), car.get_velocity())
    
    ### Speed control
    velocity=car.get_velocity()
    position=car.get_position()
    ###get closest index
    closest_index=0
    for i in range(last_closest_index, min(last_closest_index+40, len(path)-1)):
        if calculate_distance(position, path[i])<calculate_distance(position, path[closest_index]):
            closest_index=i
    last_closest_index=closest_index
    ###look ahead n points and calculate curve, take max curve
    max_curve=0
    lookahead_distance=10
    v_allowed_min=max_speed
    for i in range(closest_index,min(closest_index+lookahead_distance,len(path)-3)):
        curve=0
        dist_accumulated=0
        f=calculate_distance(path[i],path[i+1])
        g=calculate_distance(path[i+1],path[i+2])
        h=calculate_distance(path[i],path[i+2])
        s=0.5*(f+g+h)
        A=np.sqrt(max(0.0,s*(s-f)*(s-g)*(s-h)))
        denom=(f*g*h)

        if denom>1e-6:
            curve=(4*A)/denom
            v_curve=np.sqrt(max_lateral_acceleration/curve)
            for j in range(closest_index, i-1):
                dist_accumulated=dist_accumulated+calculate_distance(path[j],path[j+1])
            v_allowed=np.sqrt(v_curve**2+2*max_break*dist_accumulated)
        else: 
            v_allowed=max_speed
        if v_allowed<v_allowed_min:
            v_allowed_min=v_allowed
    ### calculate target speed 
    v_target=min(max_speed, v_allowed_min)
    ### adapt speed (PID-controller)
    ek=v_target-velocity
    Ik = Ik + ek * delta_t
    Ik=np.clip(Ik,-5.0,+5.0)

    Dk=(ek-ek_prev)/delta_t
    a=Kp*ek+Ki*Ik+Kd*Dk
    ek_prev=ek
    

    
    
    ### end
    car.update_acceleration(a)
    car.update_steering_angle(steering_angle*(180/np.pi))
    car.time_step()
    cartrace.append(car.get_position())
    velocity_list.append(car.get_velocity())

##Plot
cartrace=np.array(cartrace)


#plt.plot(path[:, 0], path[:, 1], color="RED")
plt.plot(borderleft[:, 0], borderleft[:, 1], color="GRAY")
plt.plot(borderright[:, 0], borderright[:, 1], color="GRAY")
#plt.plot(cartrace[:,0],cartrace[:,1], color="BLUE")

sc = plt.scatter(
    cartrace[:, 0],
    cartrace[:, 1],
    c=velocity_list,
    cmap="coolwarm",  # Blau = langsam, Rot = schnell
    s=3,  # Punktgröße
)

cbar = plt.colorbar(sc)
cbar.set_label("Geschwindigkeit [m/s]")
plt.show()

plt.show()
