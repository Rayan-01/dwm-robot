"""Appearance fallback, not a learned poop classifier."""

import cv2
import numpy as np
from dwm_core.control import Detection


def candidates(frame):
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, sat, value = cv2.split(hsv)
    b, g, r = [channel.astype(float) for channel in cv2.split(frame)]
    green = (
        (hue >= 35)
        & (hue <= 95)
        & (sat >= 45)
        & (value >= 45)
        & (g > r * 1.15)
        & (g > b * 1.05)
    )
    dark = ((value < 65) | ((value < 85) & (sat < 100))) & ~green
    dark[: int(h * 0.22)] = False
    mask = cv2.morphologyEx(
        dark.astype(np.uint8) * 255, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    boxes = []
    for i in range(1, n):
        x, y, bw, bh, area = stats[i]
        if not (
            max(80, h * w * 0.00035) <= area <= h * w * 0.035 and bw >= 8 and bh >= 5
        ):
            continue
        if (
            not 0.3 <= bw / bh <= 4
            or area / (bw * bh) < 0.35
            or bw > w * 0.25
            or bh > h * 0.25
        ):
            continue
        if x < 3 or x + bw > w - 3 or y + bh > h - 3:
            continue
        pad = max(8, int(max(bw, bh) * 0.3))
        xa, ya, xb, yb = (
            max(0, x - pad),
            max(0, y - pad),
            min(w, x + bw + pad),
            min(h, y + bh + pad),
        )
        ring = np.ones((yb - ya, xb - xa), dtype=bool)
        ring[y - ya : y + bh - ya, x - xa : x + bw - xa] = False
        surrounding = green[ya:yb, xa:xb] & ring
        if surrounding.sum() / ring.sum() < 0.6:
            continue
        # Grass must occur on every side, not just above a dark mat border.
        sides = (
            green[ya:y, x : x + bw],
            green[y + bh : yb, x : x + bw],
            green[y : y + bh, xa:x],
            green[y : y + bh, x + bw : xb],
        )
        if any(side.size == 0 or side.mean() < 0.5 for side in sides):
            continue
        inside = labels[y : y + bh, x : x + bw] == i
        if (
            np.median(value[ya:yb, xa:xb][surrounding])
            - np.median(value[y : y + bh, x : x + bw][inside])
            < 25
        ):
            continue
        boxes.append((int(x), int(y), int(x + bw), int(y + bh)))
    return boxes


class DarkGrassFallback:
    def __init__(self):
        self.tracks = {}
        self.next_id = -1

    def detect(self, frame, at, model_present=False):
        if model_present:
            self.tracks.clear()
            return []
        new = {i: (b, t) for i, (b, t) in self.tracks.items() if at - t <= 2}
        old = {i: b for i, (b, t) in new.items()}
        detections = []
        for box in candidates(frame):

            def distance(other):
                return ((box[0] + box[2] - other[0] - other[2]) / 2) ** 2 + (
                    (box[1] + box[3] - other[1] - other[3]) / 2
                ) ** 2

            ident = min(old, key=lambda i: distance(old[i])) if old else None
            if (
                ident is None
                or distance(old[ident]) > max(25, (box[2] - box[0]) * 0.5) ** 2
            ):
                ident = self.next_id
                self.next_id -= 1
            else:
                old.pop(ident)
            new[ident] = (box, at)
            # A rule score for confirmation, never displayed as model confidence.
            detections.append(Detection(ident, 0.5, box, source="dark_grass"))
        self.tracks = new
        return detections
