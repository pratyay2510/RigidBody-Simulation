# src/visualization/multi_sphere_logger.py

import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from src.visualization.logger_base import LoggerBase


class MultiSphereLogger:
    """
    Logger for multiple objects (e.g., multiple spheres).
    Stores individual LoggerBase instances for each ball.
    """

    def __init__(self, ball_names):
        self.ball_names = ball_names
        self.loggers = {ball: LoggerBase() for ball in ball_names}

    def record(self, ball_name, time, pos):
        """
        Records data for a specific ball.
        """
        if ball_name in self.loggers:
            self.loggers[ball_name].record(time, pos)

    def save_all_plots(self, output_dir="src/plots"):
        """
        Save plots for each ball and combined plots.
        """
        os.makedirs(output_dir, exist_ok=True)

        # Individual plots
        for ball, logger in self.loggers.items():
            logger.save_height_vs_time(os.path.join(
                output_dir, f"{ball}_height_vs_time.png"))
            logger.save_trajectory_3d(os.path.join(
                output_dir, f"{ball}_trajectory_3d.png"))

        # Combined 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        for ball, logger in self.loggers.items():
            ax.plot(logger.x_positions, logger.y_positions,
                    logger.z_positions, label=ball)
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_zlabel("Height (z)")
        ax.set_title("3D Trajectories of All Balls")
        ax.legend()
        plt.savefig(os.path.join(output_dir, "combined_3d_trajectories.png"))
        plt.close()

        # Combined height vs time plot
        plt.figure()
        for ball, logger in self.loggers.items():
            plt.plot(logger.times, logger.z_positions, label=ball)
        plt.xlabel("Time (s)")
        plt.ylabel("Height (z)")
        plt.title("Combined Height vs Time for All Balls")
        plt.legend()
        plt.grid(True)
        plt.savefig(os.path.join(output_dir, "combined_height_vs_time.png"))
        plt.close()

        print(f"All plots saved in {output_dir}")
