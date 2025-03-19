"""
viewer.py

This module sets up a custom visualization window for the simulation.
For demonstration purposes, it uses matplotlib to render the sphere in red
and the static floor (at y = 0) as a horizontal line.
"""

import matplotlib.pyplot as plt
import numpy as np


class Viewer:
    def __init__(self, window_title="Rigid Body Simulation", xlim=(-2, 2), ylim=(0, 3)):
        """
        Initializes the simulation viewer with a matplotlib figure and axis.

        Parameters:
            window_title (str): Title of the simulation window.
            xlim (tuple): Limits for the x-axis.
            ylim (tuple): Limits for the y-axis.
        """
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title(window_title)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(xlim)
        self.ax.set_ylim(ylim)
        self.ax.set_xlabel("X Position [m]")
        self.ax.set_ylabel("Y Position [m]")
        self.ax.set_title("Rigid Body Simulation (Phase 1)")
        plt.ion()
        plt.show()

    def render(self, sphere):
        """
        Renders the current state of the simulation.
        Draws the sphere as a red circle and the floor as a black line at y = 0.

        Parameters:
            sphere: An object with attributes 'position' and 'radius'.
        """
        self.ax.clear()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-2, 2)
        self.ax.set_ylim(0, 3)
        self.ax.set_xlabel("X Position [m]")
        self.ax.set_ylabel("Y Position [m]")
        self.ax.set_title("Rigid Body Simulation (Phase 1)")

        # Draw the floor as a thick black line at y = 0
        self.ax.plot([-10, 10], [0, 0], 'k-', linewidth=2)

        # Draw the sphere as a red circle (using its x-y projection)
        circle = plt.Circle(
            (sphere.position[0], sphere.position[1]), sphere.radius, color='red')
        self.ax.add_patch(circle)

        # Optionally, draw a line indicating the sphere's local x-axis
        orientation_line_length = sphere.radius
        start_point = sphere.position[:2]
        end_point = sphere.position[:2] + \
            np.array([orientation_line_length, 0])
        self.ax.plot([start_point[0], end_point[0]], [
                     start_point[1], end_point[1]], 'b-')

        plt.pause(0.001)
