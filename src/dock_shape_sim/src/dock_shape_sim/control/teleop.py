"""Manual control: keyboard mixing and acceleration limits, independent of Qt/ROS."""

from dataclasses import dataclass, field
from math import isfinite


def approach(current, target, maximum_change):
    return current + max(-maximum_change, min(maximum_change, target - current))


@dataclass
class TeleopControl:
    max_linear: float = 0.15
    max_angular: float = 0.50
    linear_acceleration: float = 0.30
    angular_acceleration: float = 1.00
    keys: set = field(default_factory=set)
    linear: float = 0.0
    angular: float = 0.0

    @property
    def target(self):
        return ((int('W' in self.keys) - int('S' in self.keys)) * self.max_linear,
                (int('A' in self.keys) - int('D' in self.keys)) * self.max_angular)

    def step(self, dt):
        # Never continue an old input after the UI event loop stalls.
        if not isfinite(dt) or dt < 0.0 or dt > 0.25:
            self.stop()
            return self.linear, self.angular
        target_linear, target_angular = self.target
        self.linear = approach(
            self.linear, target_linear, self.linear_acceleration * dt)
        self.angular = approach(
            self.angular, target_angular, self.angular_acceleration * dt)
        return self.linear, self.angular

    def stop(self):
        self.keys.clear()
        self.linear = self.angular = 0.0
