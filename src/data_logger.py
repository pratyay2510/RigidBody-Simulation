import matplotlib.pyplot as plt
import os
from mpl_toolkits.mplot3d import Axes3D


class DataLogger:
    def __init__(self):
        self.times = []
        self.z_positions = []
        self.x_positions = []  # New: record x positions
        self.y_positions = []  # New: record y positions

    def record(self, time_point, z_position, x_position=None, y_position=None):
        """Record a single timestamp, height, and optionally full 3D position."""
        self.times.append(time_point)
        self.z_positions.append(z_position)
        if x_position is not None and y_position is not None:
            self.x_positions.append(x_position)
            self.y_positions.append(y_position)

    def save_plot(self, save_path="plots/height_vs_time.png"):
        """Save the height vs time plot."""
        if not os.path.exists(os.path.dirname(save_path)):
            os.makedirs(os.path.dirname(save_path))

        plt.figure(figsize=(10, 6))
        plt.plot(self.times, self.z_positions, marker="o", linestyle="-")
        plt.xlabel("Time (s)")
        plt.ylabel("Height (z-axis)")
        plt.title("Ball Height vs. Time")
        plt.grid(True)
        plt.savefig(save_path)
        print(f"Height vs. Time graph saved at: {save_path}")

    def save_trajectory_plot_3d(self, save_path="plots/3d_trajectory.png"):
        """Save a 3D trajectory plot of the sphere's movement in space."""
        if not os.path.exists(os.path.dirname(save_path)):
            os.makedirs(os.path.dirname(save_path))

        fig = plt.figure(figsize=(10, 7))
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(self.x_positions, self.y_positions,
                self.z_positions, marker="o", linestyle="-")

        ax.set_xlabel("X Position")
        ax.set_ylabel("Y Position")
        ax.set_zlabel("Z Position (Height)")
        ax.set_title("3D Trajectory of Sphere")

        plt.savefig(save_path)
        print(f"3D trajectory graph saved at: {save_path}")
