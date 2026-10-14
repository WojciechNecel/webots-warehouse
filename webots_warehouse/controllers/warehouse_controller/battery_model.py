"""Accelerated demonstration battery model (percentage per simulation second)."""
from dataclasses import dataclass


@dataclass
class Battery:
    level: float = 65.0
    low: float = 35.0
    target: float = 95.0

    def update(self, seconds, wheel_speed=0.0, load_kg=0.0,
               manipulating=False, docked=False):
        if docked:
            rate = 1.8
        else:
            drive = min(abs(wheel_speed) / 5.0, 1.5)
            load = max(0.0, min(load_kg, 50.0)) / 50.0
            rate = -(0.012 + 0.14 * drive * (1.0 + 0.35 * load)
                     + (0.07 if manipulating else 0.0))
        self.level = max(0.0, min(100.0, self.level + rate * max(seconds, 0.0)))
        return self.level

    def needs_charge(self, route_cells):
        # Includes turns, handling and a reserve for obstacle detours.
        required = max(self.low, 20.0 + 0.8 * max(route_cells, 0))
        return self.level < min(required, self.target - 2.0)


class BatteryEmpty(RuntimeError):
    """Interrupt motion immediately when no energy remains."""
