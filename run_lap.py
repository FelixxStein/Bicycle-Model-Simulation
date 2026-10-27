from pathlib import Path
from racesim.controllers.mppi_controller import MPPIController
from racesim.controllers.basic_drive import BasicDriveController
from racesim import vehicle
from racesim.track import Track
import argparse
import json
import numpy as np
from racesim.visualization.plot import Plot
from racesim.simulate_lap import simulate_lap
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set lap settings.")
    parser.add_argument("--controller", choices=["basic_drive", "mppi"], default="mppi",help="What controller do you want?")
    parser.add_argument("--track", type=str, default="Hockenheim", help="What track do you want?")
    parser.add_argument("--configs", type=str, default="best_configs", help="What configuration of the controller do you want?")
    parser.add_argument("--seed", type=int, default=1900, help="What seed do you want?")
    parser.add_argument("--log", action="store_true", help="rollout logging on/off")
    parser.add_argument("--plot", action="store_true", help="Should we plot the lap?")
    args = parser.parse_args()
    trackname=args.track+".csv"
    with open(f"configs/{args.configs}.json", "r", encoding="utf-8") as f:
        config = json.load(f)   
    number_plans=config["number_plans"]
    number_control_steps=config["number_control_steps"]
    physics_time_step=config["physics_time_step"]
    substeps=config["substeps"]
    sd_acceleration=config["sd_acceleration"]
    temperature_factor=config["temperature_factor"]
    starting_plan=config["starting_plan"]
    ao=config["ao"]
    c_under=config["c_under"]
    c_over=config["c_over"]
    c_close=config["c_close"]
    c_steer=config["c_steer"]
    profile_grip_factor=config["profile_grip_factor"]
    run_dir = Path("runs")
    run_dir.mkdir(exist_ok=True)
    car=vehicle.DynamicVehicle()
    track=Track(trackname,friction_coefficient=car.get_min_friction_coefficient(), downforce_factor=car.get_downforce_factor(), mass=car.get_mass(), v_max=car.get_max_speed(),max_break=car.get_max_brake(), profile_grip_factor=profile_grip_factor)
    path=track.get_midline()
    car.set_starting_angle(path)
    car.set_starting_position(path)

    if args.controller=="basic_drive":
        controller=BasicDriveController(track=track,max_break=car.get_max_brake(),max_acceleration=car.get_max_acceleration(),wheel_base=car.get_wheelbase(),max_speed=car.get_max_speed(),friction_coefficient_front=car.get_friction_coefficient_front(),friction_coefficient_rear=car.get_friction_coefficient_rear(),downforce_factor=car.get_downforce_factor(),car_mass=car.get_mass())
    else:
        controller=MPPIController(car=car,track=track,number_plans=number_plans,number_control_steps=number_control_steps,physics_time_step=physics_time_step,substeps=substeps,sd_acceleration=sd_acceleration,temperature_factor=temperature_factor,starting_plan=starting_plan,seed=args.seed,ao=ao,c_under=c_under,c_over=c_over,c_close=c_close,c_steer=c_steer, log_rollouts=args.log)
    results=simulate_lap(car=car,track=track,controller=controller,)
    cartrace=results["cartrace"]
    velocity_list=results["velocity_list"]
    heading_angle=results["heading_angle"]
    controls=results["controls"]
    lap_time=results["lap_time"]
    reason=results["reason"]
    d={"midline":track.get_midline(),
       "border_left":track.get_border_left(),
       "border_right":track.get_border_right(),
       "cartrace":cartrace,
       "velocity_list":velocity_list,
       "heading_angle":heading_angle,
       "controls":controls,
       "seed":args.seed,
       "dt":physics_time_step,
       "lap_time":lap_time,
       "wheelbase":car.get_wheelbase(),
       "controller":args.controller}
    if args.log and args.controller=="mppi":
        path_trajectory_array, weights_array, off_track_array,current_trajectory=controller.get_logs() 
        d["path_trajectory_array"]=path_trajectory_array
        d["weights_array"]=weights_array
        d["off_track_array"]=off_track_array
        d["current_trajectory"]=current_trajectory
    if args.controller=="mppi":
        d["substeps"]=controller.get_substep()
    np.savez(run_dir / f"{args.controller}_{args.seed}_{args.track}", **d)
    print("Reason:", reason," and Lap_time:", lap_time)
    if args.plot:
        borderleft=track.get_border_left()
        borderright=track.get_border_right()
        plot=Plot(borderleft, borderright)
        plot.plot_speed(cartrace,velocity_list)
        plot.show()