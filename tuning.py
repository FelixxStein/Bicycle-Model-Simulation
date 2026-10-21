from racesim.tuning.cem import Tuning
import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tune controller parameters on a track.")
    parser.add_argument("--population_size", type=int, default=18, help="Population size")
    parser.add_argument("--elites", type=int, default=5, help="Number of elites per population")
    parser.add_argument("--tuning_run", type=str, default="1", help="Which tuning run is this?")
    parser.add_argument("--seed", type=int, nargs="+", default=42+1900, help="Array of training seeds")
    parser.add_argument("--processes", type=int, default=6, help="Number of parallel processes")
    parser.add_argument("--generations", type=int, default=50, help="How many generations do you want?")
    args = parser.parse_args()
    tuning=Tuning(K=args.population_size,k=args.elites,max_steps=args.elites,csv_name="tuning_log_" + args.tuning_run + ".csv", seeds=args.seed, processes=args.processes)
    tuning.run(args.generations)