# src/visualization/multi_sphere_logger.py

import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.visualization.logger_base import LoggerBase


class MultiSphereLogger:
    """
    Logger for multiple objects, maintaining separate trajectories and times.
    """

    def __init__(self, ball_names):
        self.ball_names = ball_names
        self.loggers = {ball: LoggerBase() for ball in ball_names}

    def record(self, ball_name, time, pos):
        """
        Record position for a specific ball at a timestep.
        """
        self.loggers[ball_name].record(time, pos)

    def save_all_plots(self, output_dir="data/multi_sphere/plots"):
        """
        Save individual plots and combined plots for all balls.
        """
        os.makedirs(output_dir, exist_ok=True)

        # 1. Save individual plots
        for ball, logger in self.loggers.items():
            logger.save_height_vs_time(
                os.path.join(output_dir, f"{ball}_height_vs_time.png"))
            logger.save_3d_trajectory(
                os.path.join(output_dir, f"{ball}_trajectory_3d.png"))

            # Optional: Save XY trajectory
            plt.figure()
            plt.plot(logger.x_positions, logger.y_positions, marker="o")
            plt.xlabel("X")
            plt.ylabel("Y")
            plt.title(f"{ball} XY Trajectory")
            plt.grid(True)
            plt.savefig(os.path.join(output_dir, f"{ball}_trajectory_xy.png"))
            plt.close()

        # 2. Save combined 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for ball, logger in self.loggers.items():
            ax.plot(logger.x_positions, logger.y_positions,
                    logger.z_positions, label=ball)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("Combined 3D Trajectories")
        ax.legend()
        plt.savefig(os.path.join(output_dir, "combined_3d_trajectories.png"))
        plt.close()

        # 3. Save combined height vs time plot
        plt.figure()
        for ball, logger in self.loggers.items():
            plt.plot(logger.times, logger.z_positions, label=ball)
        plt.xlabel("Time (s)")
        plt.ylabel("Height (z)")
        plt.title("Combined Height vs Time")
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(output_dir, "combined_height_vs_time.png"))
        plt.close()

        print(f"All multi-sphere plots saved in {output_dir}")
