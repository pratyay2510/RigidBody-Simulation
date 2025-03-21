# src/visualization/data_logger.py

from src.visualization.logger_base import LoggerBase


class DataLogger(LoggerBase):
    """
    Logger for single object (ball/cube).
    Inherits record and plotting methods from LoggerBase.
    """

    def __init__(self):
        super().__init__()

    def record(self, time_point, z_position, x_position=None, y_position=None):
        """
        Backward-compatible record method to allow z-position first.
        """
        pos = [
            x_position if x_position is not None else 0.0,
            y_position if y_position is not None else 0.0,
            z_position
        ]
        super().record(time_point, pos)

    def save_plot(self, save_path):
        """
        Save height vs time plot (legacy compatibility).
        """
        self.save_height_vs_time(save_path)

    def save_trajectory_plot_3d(self, save_path):
        """
        Save 3D trajectory plot (legacy compatibility).
        """
        self.save_3d_trajectory(save_path)
