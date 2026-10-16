"""Visual steering with forward arcs, filtered observations and PWM slew limits.

PWM is not a calibrated velocity command. Firmware supplies its own ramp too.
"""

import math


class SmoothApproach:
    def __init__(self):
        self.reset()

    def reset(self):
        self.at = None
        self.offset = None
        self.depth = None
        self.pair = (30.0, 30.0)

    def step(self, offset, depth, stop_distance, now):
        if self.at is None:
            self.at = now
            self.offset = offset
            self.depth = depth
            return (30, 30)
        dt = min(0.1, max(0, now - self.at))
        self.at = now
        self.offset += (1 - math.exp(-dt / 0.18)) * (offset - self.offset)
        self.depth += (1 - math.exp(-dt / 0.25)) * (depth - self.depth)
        gap = max(0, self.depth - stop_distance)
        base = 30 + 10 * min(1, gap / 0.4)
        error = 0 if abs(self.offset) < 0.025 else self.offset
        if abs(error) > 0.06:
            base = max(base, 34)
        turn = max(-(base - 30), min(base - 30, error * 40))
        desired = (base + turn, base - turn)
        max_step = 30 * dt
        self.pair = tuple(
            old + max(-max_step, min(max_step, new - old))
            for old, new in zip(self.pair, desired)
        )
        return tuple(int(round(v)) for v in self.pair)


def overlap(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1])
    )
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0
