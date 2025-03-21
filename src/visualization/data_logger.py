# src/visualization/data_logger.py

from src.visualization.logger_base import LoggerBase


class DataLogger(LoggerBase):
    """
    Logger for a single object (e.g., a ball or cube).
    Inherits plotting and recording logic from LoggerBase.
    """

    def __init__(self):
        super().__init__()

    def record(self, time_point, z_position, x_position=None, y_position=None):
        """
        Overrides record to allow legacy positional arguments.
        """
        pos = [x_position if x_position is not None else 0.0,
               y_position if y_position is not None else 0.0,
               z_position]
        super().record(time_point, pos)
