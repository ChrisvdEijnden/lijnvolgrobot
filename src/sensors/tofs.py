import math
from leaphymicropython.sensors.tof import TimeOfFlight

class TOF:
    """
    Handles Time-of-Flight (TOF) distance sensors for obstacle detection.

    Provides access to the raw distances, as well as higher-level
    calculations such as angular deviation and ball detection heuristics.
    """

    def __init__(self):
        """Initialize four TOF sensors, one for each channel."""
        self.tofs = [TimeOfFlight(channel=ch) for ch in range(4)]

    def tof_distances(self):
        """Return a list of measured distances from all TOF sensors."""
        return [tof.get_distance() for tof in self.tofs]

    def delta_left_right(self):
        """Return the difference between the right-side and left-side sensor distances."""
        dist = self.tof_distances()
        return dist[3] - dist[1]

    def angle(self):
        """Return the angle of the robot"""
        dist = self.tof_distances()
        return math.degrees(atan(self.delta_left_right()/(43/3)))

    def is_ball(self):
        """Heuristically detect whether a ball is present in front of the robot.

        The logic uses specific offsets and thresholds based on sensor readings.
        """
        dist = self.tof_distances()
        return self.delta_left_right() / 2 - 70 - dist[2] > 60


tof = TOF()




