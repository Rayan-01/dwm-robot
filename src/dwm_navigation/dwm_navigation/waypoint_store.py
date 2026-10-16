"""Map-bound waypoint drafts. Saving a draft never commands a motor."""

import json
import math
import re
from pathlib import Path


def validate_route(data):
    if not isinstance(data, dict):
        raise ValueError("Route must be an object")
    name = data.get("map_name")
    if not isinstance(name, str) or not re.fullmatch(
        r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name
    ):
        raise ValueError("Choose a saved map first")
    points = data.get("waypoints")
    if not isinstance(points, list) or not 1 <= len(points) <= 100:
        raise ValueError("Add 1–100 waypoints")
    result = []
    for point in points:
        if not isinstance(point, dict):
            raise ValueError("Invalid waypoint")
        values = [point.get("x"), point.get("y"), point.get("yaw", 0)]
        if any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            or abs(v) > 10000
            for v in values
        ):
            raise ValueError("Waypoint coordinates must be finite metres")
        result.append(dict(zip(("x", "y", "yaw"), values)))
    mode = data.get("heading_mode", "path")
    if mode not in ("path", "fixed"):
        raise ValueError("Invalid waypoint heading mode")
    return {"map_name": name, "waypoints": result, "heading_mode": mode, "version": 2}


class RouteStore:
    def __init__(self, directory):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)

    def save(self, data):
        route = validate_route(data)
        target = self.directory / (route["map_name"] + ".json")
        temp = target.with_suffix(".tmp")
        temp.write_text(json.dumps(route, indent=2) + "\n")
        temp.replace(target)
        return route

    def read(self, name):
        validate_route({"map_name": name, "waypoints": [{"x": 0, "y": 0}]})
        target = self.directory / (name + ".json")
        return (
            validate_route(json.loads(target.read_text())) if target.exists() else None
        )
