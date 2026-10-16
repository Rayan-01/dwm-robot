import unittest
import cv2
import numpy as np
from dwm_perception.dark_grass import candidates, DarkGrassFallback


class GrassTests(unittest.TestCase):
    def frame(self):
        return np.full((360, 640, 3), (30, 125, 35), dtype=np.uint8)

    def test_dark_object_on_grass_is_candidate(self):
        for color in ((15, 15, 15), (20, 30, 45)):
            frame = self.frame()
            cv2.rectangle(frame, (290, 250), (330, 272), color, -1)
            self.assertEqual(len(candidates(frame)), 1)

    def test_plain_grass_and_black_outside_grass_are_rejected(self):
        self.assertEqual(candidates(self.frame()), [])
        frame = np.full((360, 640, 3), 180, dtype=np.uint8)
        cv2.rectangle(frame, (290, 250), (330, 272), (15, 15, 15), -1)
        self.assertEqual(candidates(frame), [])

    def test_farther_object_above_image_midpoint_is_detected(self):
        frame = self.frame()
        cv2.rectangle(frame, (290, 145), (315, 157), (15, 15, 15), -1)
        self.assertEqual(len(candidates(frame)), 1)

    def test_grass_border_with_no_grass_below_is_rejected(self):
        frame = self.frame()
        frame[300:] = (180, 180, 180)
        cv2.rectangle(frame, (290, 284), (330, 299), (15, 15, 15), -1)
        self.assertEqual(candidates(frame), [])

    def test_huge_shadow_thin_line_and_upper_scene_are_rejected(self):
        for a, b in [
            ((50, 210), (500, 320)),
            ((50, 250), (450, 253)),
            ((290, 50), (330, 72)),
        ]:
            frame = self.frame()
            cv2.rectangle(frame, a, b, (15, 15, 15), -1)
            self.assertEqual(candidates(frame), [])

    def test_model_detections_take_priority(self):
        frame = self.frame()
        cv2.rectangle(frame, (290, 250), (330, 272), (15, 15, 15), -1)
        fallback = DarkGrassFallback()
        self.assertEqual(fallback.detect(frame, 1, model_present=True), [])

    def test_id_survives_small_motion_and_brief_occlusion(self):
        frame = self.frame()
        cv2.rectangle(frame, (290, 250), (330, 272), (15, 15, 15), -1)
        fallback = DarkGrassFallback()
        d = fallback.detect(frame, 1)[0]
        self.assertEqual(d.source, "dark_grass")
        self.assertEqual(fallback.detect(self.frame(), 1.3), [])
        self.assertEqual(fallback.detect(frame, 1.6)[0].id, d.id)
        moved = self.frame()
        cv2.rectangle(moved, (295, 250), (335, 272), (15, 15, 15), -1)
        self.assertEqual(fallback.detect(moved, 1.7)[0].id, d.id)

    def test_long_loss_gets_new_id(self):
        frame = self.frame()
        cv2.rectangle(frame, (290, 250), (330, 272), (15, 15, 15), -1)
        fallback = DarkGrassFallback()
        d = fallback.detect(frame, 1)[0]
        self.assertNotEqual(fallback.detect(frame, 4)[0].id, d.id)
