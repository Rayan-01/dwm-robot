"""Approximate straight travel from existing FG wheel odometry; brush command is controlled by Runtime."""

from dwm_core.vision_approach import SmoothApproach


class ForwardPass:
    def __init__(self, left, right, now, distance):
        self.origin = (left, right)
        self.started = now
        self.distance = distance
        self.progress = 0.0
        self.last_progress = 0.0
        self.progress_at = now
        self.smoother = SmoothApproach()

    def step(self, left, right, now):
        l, r = left - self.origin[0], right - self.origin[1]
        self.progress = max(0, (l + r) / 2)
        if min(l, r) < -0.03:
            return (0, 0), "Forward pass reported reverse travel"
        if abs(l - r) > 0.15:
            return (0, 0), "Forward pass wheel drift exceeded 15 cm"
        if self.progress >= self.distance:
            return (0, 0), "Forward pass completed; brush off"
        if self.progress > self.last_progress + 0.005:
            self.last_progress = self.progress
            self.progress_at = now
        if now - self.started > 30:
            return (0, 0), "Forward pass timed out"
        if now - self.progress_at > 5:
            return (0, 0), "Forward pass has no measured progress"
        # Slow the leading track gently; never command a track reversal.
        pair = self.smoother.step(
            -(l - r) * 3, self.distance - self.progress + 0.45, 0.45, now
        )
        return pair, None

    def status(self):
        return {
            "estimated_travel_m": round(self.progress, 3),
            "target_m": self.distance,
            "calibration_verified": False,
        }
