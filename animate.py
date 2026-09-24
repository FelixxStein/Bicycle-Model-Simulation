import vehicle
import math
import matplotlib.animation as animation
class animate:
    def __init__(self):
        pass
        
    def create(x_coordinate_list, y_coordinate_list,target_fps=30):
        vehicle=vehicle.vehicle()
        data_hz=vehicle.delta_t
        points_per_frame = data_hz / target_fps
        total_frames = math.ceil(len(x_coordinate_list) / points_per_frame)

        max_x = max(x_coordinate_list) + 1
        max_y = max(y_coordinate_list) + 1
        min_x = min(x_coordinate_list) - 1
        min_y = min(y_coordinate_list) - 1


        fig, ax = plt.subplots()
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_aspect('equal', adjustable='box')

        graph, = ax.plot([], [], '-', color='blue', linewidth=2)

        def animate(i):
            current_index = int(i * points_per_frame)
            
            current_index = min(current_index, len(x_coordinate_list))
            
            graph.set_data(x_coordinate_list[:current_index], y_coordinate_list[:current_index])
            return graph,

        anim = animation.FuncAnimation(
            fig, 
            animate, 
            frames=total_frames, 
            interval=1000/target_fps, # ~33.3 ms
            blit=True,
            repeat=False
        )

        anim.save('Carride.mp4', writer='ffmpeg', fps=target_fps)

