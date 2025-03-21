# src/visualization/logger_base.py

import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LoggerBase:
    """
    Base class for logging simulation trajectory and visualizing plots.
    """

    def __init__(self):
        """
        Initializes common lists for time and positions.
        """
        self.times = []
        self.x_positions = []
        self.y_positions = []
        self.z_positions = []

    def record(self, time, pos):
        """
        Records time and position values.
        Args:
            time (float): Current simulation time.
            pos (np.ndarray or list): Position [x, y, z].
        """
        self.times.append(time)
        self.x_positions.append(pos[0])
        self.y_positions.append(pos[1])
        self.z_positions.append(pos[2])

    def save_height_vs_time(self, save_path):
        """
        Saves height (z-axis) vs time plot.
        """
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.figure(figsize=(10, 6))
        plt.plot(self.times, self.z_positions, marker="o", linestyle="-")
        plt.xlabel("Time (s)")
        plt.ylabel("Height (z)")
        plt.title("Height vs Time")
        plt.grid(True)
        plt.savefig(save_path)
        plt.close()
        print(f"Saved: {save_path}")

    def save_trajectory_3d(self, save_path):
        """
        Saves 3D trajectory plot.
        """
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.x_positions, self.y_positions,
                self.z_positions, marker="o")
        ax.set_xlabel("X position")
        ax.set_ylabel("Y position")
        ax.set_zlabel("Height (z)")
        ax.set_title("3D Trajectory")
        plt.savefig(save_path)
        plt.close()
        print(f"Saved: {save_path}")
