import argparse
from racesim.visualization.animate import Animate
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tune controller parameters on a track.")
    parser.add_argument("--path", type=str, default="mppi",help="What run do you want to animate?")
    parser.add_argument("--window_height", type=int, default=785, help="What window-height do you want?")
    parser.add_argument("--window_width", type=int, default=1200, help="What window-width do you want?")
    args = parser.parse_args()
    animate=Animate(path=args.path, window_height=args.window_height, window_width=args.window_width)
    animate.main_loop()