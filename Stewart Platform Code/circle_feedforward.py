"""Ramp circle speed and compensate for centripetal acceleration and target velocity."""

import math


class CircleFeedforward:
    RAMP_SECONDS = 0.5
    PLATFORM_RADIUS_M = 0.14  # Match the controller's nominal distance scale.
    ROLLING_ACCEL_M_S2 = 5.89  # Small-angle acceleration per radian for a rolling sphere.
    CENTRIPETAL_GAIN = 6.4  # Factor to increase centripetal acceleration for stronger circular motion
    MAX_TARGET_OFFSET = 0.20  # Fraction of the platform radius.
    MAX_VELOCITY_OFFSET = 0.25  # Bound the tangential offset when Kp is small.

    def __init__(self):
        self.speed = 0.0

    def stop(self):
        """Clear the speed when tracking measurements are interrupted or the circle ends."""
        self.speed = 0.0

    def advance(self, dt, requested_speed, max_speed, remaining):
        """Return distance to advance, in platform radii, for this frame."""
        acceleration = max_speed / self.RAMP_SECONDS
        stopping_speed = math.sqrt(2 * acceleration * remaining)
        # Limit acceleration and deceleration so the inward bias fades with speed.
        step = acceleration * dt
        change = max(-step, min(step, requested_speed - self.speed))
        self.speed = min(self.speed + change, stopping_speed)
        return self.speed * dt

    def command_target(self, target, kp, kd):
        """Use the P term to supply inward tilt and subtract Kd times target velocity."""
        radius = math.hypot(*target)
        if self.speed <= 0 or radius <= 0 or kp <= 0:
            return target

        inward_accel = self.speed**2 * self.PLATFORM_RADIUS_M / radius
        inward_accel *= self.CENTRIPETAL_GAIN
        tilt_deg = math.degrees(inward_accel / self.ROLLING_ACCEL_M_S2)
        offset = min(tilt_deg / kp, self.MAX_TARGET_OFFSET, radius)
        x, y = target
        inward_scale = 1 - offset / radius
        # The circle advances clockwise after image Y is flipped during normalization.
        tangent_scale = min(kd * self.speed / kp, self.MAX_VELOCITY_OFFSET) / radius
        return x * inward_scale + y * tangent_scale, y * inward_scale - x * tangent_scale
