import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os


class MultiSphereLogger:
    def __init__(self, ball_names):
        # Initialize dictionaries for each ball’s logs
        self.ball_names = ball_names
        self.logs = {ball: {'time': [], 'z': [], 'x': [], 'y': []}
                     for ball in ball_names}

    def record(self, ball_name, time, pos):
        """ Record trajectory data for a specific ball at a timestep. """
        self.logs[ball_name]['time'].append(time)
        self.logs[ball_name]['z'].append(pos[2])
        self.logs[ball_name]['x'].append(pos[0])
        self.logs[ball_name]['y'].append(pos[1])

    def save_all_plots(self, output_dir="src/plots"):
        """ Save height vs time, 2D trajectories, and combined 3D plots for all balls. """
        os.makedirs(output_dir, exist_ok=True)

        # 1. Individual height vs time & XY trajectory plots
        for ball, data in self.logs.items():
            time, z, x, y = data['time'], data['z'], data['x'], data['y']

            # Height vs time
            plt.figure()
            plt.plot(time, z, label=f"{ball} height")
            plt.xlabel("Time (s)")
            plt.ylabel("Height (z)")
            plt.title(f"{ball} Height vs Time")
            plt.grid()
            plt.savefig(f"{output_dir}/{ball}_height_vs_time.png")
            plt.close()

            # XY trajectory
            plt.figure()
            plt.plot(x, y, label=f"{ball} XY trajectory")
            plt.xlabel("X position")
            plt.ylabel("Y position")
            plt.title(f"{ball} 2D Trajectory (X-Y plane)")
            plt.grid()
            plt.savefig(f"{output_dir}/{ball}_trajectory_xy.png")
            plt.close()

        # 2. Combined 3D trajectories
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for ball, data in self.logs.items():
            ax.plot(data['x'], data['y'], data['z'], label=ball)
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_zlabel("Height (z)")
        ax.set_title("3D Trajectories of All Balls")
        ax.legend()
        plt.savefig(f"{output_dir}/combined_3d_trajectories.png")
        plt.close()

        # 3. Combined height vs time for all balls
        plt.figure()
        for ball, data in self.logs.items():
            plt.plot(data['time'], data['z'], label=ball)
        plt.xlabel("Time (s)")
        plt.ylabel("Height (z)")
        plt.title("Combined Height vs Time for All Balls")
        plt.legend()
        plt.grid()
        plt.savefig(f"{output_dir}/combined_height_vs_time.png")
        plt.close()

        print(f"All multi-sphere plots saved in {output_dir}")
