"""
bicycle_model.py - Kinematic bicycle model for the NXP Cup car.

State:
    x       - world X position (metres)
    y       - world Y position (metres)
    theta   - heading angle (radians, 0 = east / +X direction)
    v       - current speed (metres per second)

Control inputs (matching C code conventions):
    steer_normalized  - steering command in range [-100, +100]
                        Mirrors `current_steer` in main.c / Steer() in servo.c.
                        +100 = full right, -100 = full left.
    speed_normalized  - speed command in range [-100, +100]
                        Mirrors `HbridgeSpeed` duty-cycle argument.
                        +100 = full forward, -100 = full reverse.

Conversion from servo.c Steer() function:
    hw_angle = STEERING_OFFSET + angle * (100 - STEERING_OFFSET) / 100   (angle >= 0)
    hw_angle = STEERING_OFFSET + angle * (100 + STEERING_OFFSET) / 100   (angle < 0)
    The hw_angle then maps to a servo pulse but physically represents steering
    degrees.  For simulation we map directly to a wheel angle using
    STEERING_LIMIT_RIGHT / STEERING_LIMIT_LEFT from Config.h.
"""

from __future__ import annotations

import math

from config import (
    MAX_SPEED_MS,
    STEERING_LIMIT_RIGHT,
    STEERING_LIMIT_LEFT,
    STEERING_OFFSET,
    WHEELBASE,
)


class BicycleModel:
    """
    Simple kinematic bicycle (front-wheel steering, rear-wheel drive).

    The steering angle delta is the angle of the front wheel relative to
    the car longitudinal axis (positive = left turn, i.e. counter-clockwise
    when viewed from above - standard math convention).

    Note: The servo.c Steer() function uses the convention that positive
    current_steer = right turn.  We mirror that here: positive
    steer_normalized turns RIGHT.
    """

    def __init__(self, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0) -> None:
        self.x: float = x0
        self.y: float = y0
        self.theta: float = theta0  # radians
        self.v: float = 0.0         # m/s

    # ------------------------------------------------------------------
    # Coordinate conversion helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _steer_to_delta(steer_normalized: float) -> float:
        """
        Convert a logical steer command in [-100, +100] to a front-wheel
        steering angle delta in radians.

        Mirrors the mapping in servo.c Steer():
            hw_angle = SERVO_CENTER_OFFSET + angle * (100 ∓ SERVO_CENTER_OFFSET) / 100

        Then we map the resulting [-100, +100] hw_angle linearly to the
        physical steering limits [STEERING_LIMIT_LEFT, STEERING_LIMIT_RIGHT].

        Convention: positive delta = turn LEFT (counter-clockwise) in the
        bicycle model, but positive steer_normalized = turn RIGHT in C code,
        so we negate.
        """
        angle = float(steer_normalized)
        # Clamp (mirrors servo.c clamp)
        angle = max(-100.0, min(100.0, angle))

        # Apply hardware offset mapping (servo.c lines 19-23)
        offset = float(STEERING_OFFSET)
        if angle >= 0.0:
            hw_angle = offset + angle * (100.0 - offset) / 100.0
        else:
            hw_angle = offset + angle * (100.0 + offset) / 100.0

        # hw_angle is now in the range [-100, +100] with 0-centre corrected.
        # Map linearly to physical degrees.
        # Positive hw_angle -> right -> negative delta in math convention.
        if hw_angle >= 0.0:
            delta_deg = hw_angle * STEERING_LIMIT_RIGHT / 100.0
        else:
            delta_deg = hw_angle * abs(STEERING_LIMIT_LEFT) / 100.0

        # Negate: positive steer_normalized = right = negative delta (bicycle model)
        return -math.radians(delta_deg)

    @staticmethod
    def _speed_to_ms(speed_normalized: float) -> float:
        """
        Convert HbridgeSpeed-style duty cycle [-100, +100] to m/s.
        Mirrors HbridgeSpeed clamp in hbridge.c.
        """
        speed = max(-100.0, min(100.0, float(speed_normalized)))
        return speed / 100.0 * MAX_SPEED_MS

    # ------------------------------------------------------------------
    # Integration step
    # ------------------------------------------------------------------

    def step(
        self,
        steer_normalized: float,
        speed_normalized: float,
        dt: float,
    ) -> None:
        """
        Advance the car state by one timestep dt (seconds).

        Args:
            steer_normalized : steering command, -100..+100 (mirrors current_steer)
            speed_normalized : speed command, -100..+100 (mirrors HbridgeSpeed arg)
            dt               : timestep in seconds
        """
        delta = self._steer_to_delta(steer_normalized)
        self.v = self._speed_to_ms(speed_normalized)

        # Kinematic bicycle model equations:
        #   dx     = v * cos(theta) * dt
        #   dy     = v * sin(theta) * dt
        #   dtheta = v * tan(delta) / L * dt
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        if abs(WHEELBASE) > 1e-9:
            self.theta += self.v * math.tan(delta) / WHEELBASE * dt

        # Keep theta in [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def state(self) -> tuple:
        """Return (x, y, theta, v)."""
        return self.x, self.y, self.theta, self.v

    def reset(self, x0: float = 0.0, y0: float = 0.0, theta0: float = 0.0) -> None:
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.v = 0.0
