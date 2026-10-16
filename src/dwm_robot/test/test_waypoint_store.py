import math
import tempfile
import unittest
from pathlib import Path
from dwm_navigation.waypoint_store import RouteStore, validate_route


class WaypointStoreTests(unittest.TestCase):
    def test_map_bound_round_trip_preserves_order(self):
        with tempfile.TemporaryDirectory() as folder:
            store = RouteStore(folder)
            route = {
                "map_name": "room_map",
                "waypoints": [{"x": 1, "y": 2}, {"x": -3.5, "y": 0.2, "yaw": 1.4}],
            }
            store.save(route)
            self.assertEqual(store.read("room_map")["waypoints"][1]["x"], -3.5)
            self.assertIsNone(store.read("another_map"))
            self.assertFalse((Path(folder) / "room_map.tmp").exists())

    def test_invalid_coordinates_names_and_counts_rejected(self):
        valid = {"map_name": "room_map", "waypoints": [{"x": 1, "y": 2}]}
        invalid = [
            dict(valid, map_name="../outside"),
            dict(valid, waypoints=[]),
            dict(valid, waypoints=[{"x": math.nan, "y": 0}]),
            dict(valid, waypoints=[{"x": True, "y": 0}]),
            dict(valid, waypoints=[{"y": 0}]),
            dict(valid, waypoints=valid["waypoints"] * 101),
        ]
        for route in invalid:
            with self.subTest(route=route), self.assertRaises(ValueError):
                validate_route(route)
