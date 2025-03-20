import matplotlib.pyplot as plt
import os


class DataLogger:
    def __init__(self):
        self.times = []
        self.z_positions = []

    def record(self, time_point, z_position):
        """Record a single timestamp and height."""
        self.times.append(time_point)
        self.z_positions.append(z_position)

    def save_plot(self, save_path="plots/height_vs_time.png"):
        """Save the height vs time plot to the specified path."""
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
