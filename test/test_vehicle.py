import numpy as np
import racesim.vehicle as vehicle
import racesim.constants as constants
dt=0.01
def test_sideways_sliding():
    car=vehicle.DynamicVehicle(starting_vy=40,starting_vx=0)
    v_start=car.get_velocity()
    s=0
    for i in range(5):
        car.time_step(np.array([0,0]),dt)
        s+=1
    v_end=car.get_velocity()
    deceleration=(v_start-v_end)/(dt*s)
    assert np.isclose(deceleration, car.get_min_friction_coefficient()*constants.GRAVITY,rtol=0.2), "sideways slide decelerates faster than tire grip allows"

def test_batch_compatible():
    state=np.array([[0.0,0.0,0.0,1,0,0],[0.0,0.0,0,5,0,0],[0.0,0.0,0,0,40,0],[0.0,0.0,0.0,40,0,0]])
    control_inputs=np.array([[0.1,7.0],[0.1,0.0],[0.0,0.0],[0.0,-30.0]])
    car_1=vehicle.DynamicVehicle(starting_x=state[0][0],starting_y=state[0][1],starting_vx=state[0][3],starting_vy=state[0][4],starting_yaw=state[0][2],starting_yaw_rate=state[0][5])
    car_2=vehicle.DynamicVehicle(starting_x=state[1][0],starting_y=state[1][1],starting_vx=state[1][3],starting_vy=state[1][4],starting_yaw=state[1][2],starting_yaw_rate=state[1][5])
    car_3=vehicle.DynamicVehicle(starting_x=state[2][0],starting_y=state[2][1],starting_vx=state[2][3],starting_vy=state[2][4],starting_yaw=state[2][2],starting_yaw_rate=state[2][5])
    car_4=vehicle.DynamicVehicle(starting_x=state[3][0],starting_y=state[3][1],starting_vx=state[3][3],starting_vy=state[3][4],starting_yaw=state[3][2],starting_yaw_rate=state[3][5])
    for i in range(10):
        car_1.time_step(control_inputs=control_inputs[0],delta_t=dt)
        car_2.time_step(control_inputs=control_inputs[1],delta_t=dt)
        car_3.time_step(control_inputs=control_inputs[2],delta_t=dt)
        car_4.time_step(control_inputs=control_inputs[3],delta_t=dt)
        state=car_1.calculate_new_state(state,control_inputs,dt)
    assert np.allclose(car_1.state, state[0],rtol=0.01), "error at batch compatibility: check slow speed and acceleration from there"
    assert np.allclose(car_2.state, state[1],rtol=0.01), "error at batch compatibility: check steering and transition between dynamic and kinematic model"
    assert np.allclose(car_3.state, state[2],rtol=0.01), "error at batch compatibility: check high vy scenarios"
    assert np.allclose(car_4.state, state[3],rtol=0.01), "error at batch compatibility: check high speed and breaking"

def test_standstill_and_brake():
    car=vehicle.DynamicVehicle(starting_vy=0,starting_vx=0)
    for i in range(10):
        car.time_step(control_inputs=np.array([0,-10]),delta_t=dt)
    assert np.isclose(car.get_vx(),0,atol=0.000001), "car is getting negative speed from braking"

def test_standstill_and_brake():
    car=vehicle.DynamicVehicle(starting_vy=0,starting_vx=-5)
    for i in range(100):
        car.time_step(control_inputs=np.array([0,-10]),delta_t=dt)
    assert np.isclose(car.get_vx(),0,atol=0.000001), "car is getting positiv speed from braking"
    
def test_normal_braking():
    car=vehicle.DynamicVehicle(starting_vy=0,starting_vx=50)
    car.time_step(control_inputs=np.array([0,-14]),delta_t=dt)
    assert (np.isclose(car.get_vx(),50-14*dt,atol=0.1)), "car is instantly stopping"
    for i in range(500):
        car.time_step(control_inputs=np.array([0,-14]),delta_t=dt)
    assert np.isclose(car.get_vx(),0,atol=0.000001), "car is getting negative speed from braking"

def test_spinning_braking():
    car=vehicle.DynamicVehicle(starting_vy=0,starting_vx=5, starting_yaw_rate=5)
    for i in range(500):
        car.time_step(control_inputs=np.array([0,-14]),delta_t=dt)
    assert np.isclose(car.get_vx(),0,atol=0.000001), "spinning car is a problem"

