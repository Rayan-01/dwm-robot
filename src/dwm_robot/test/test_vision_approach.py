import unittest
from dwm_core.vision_approach import SmoothApproach


class SmoothingTests(unittest.TestCase):
    def run_steps(self, offset, depth=1):
        c = SmoothApproach()
        return [c.step(offset, depth, 0.45, i * 0.05) for i in range(60)]

    def test_turns_use_forward_arcs_not_track_reversals(self):
        for error in [-0.5, -0.2, 0, 0.2, 0.5]:
            commands = self.run_steps(error)
            self.assertTrue(all(30 <= l <= 50 and 30 <= r <= 50 for l, r in commands))
            if error > 0:
                self.assertGreater(commands[-1][0], commands[-1][1])
            if error < 0:
                self.assertLess(commands[-1][0], commands[-1][1])

    def test_command_changes_are_bounded_even_on_direction_change(self):
        c = SmoothApproach()
        commands = []
        for i in range(100):
            commands.append(c.step(0.3 if i < 50 else -0.3, 1, 0.45, i * 0.05))
        self.assertTrue(
            all(
                max(abs(l - a), abs(r - b)) <= 2
                for (l, r), (a, b) in zip(commands, commands[1:])
            )
        )

    def test_image_jitter_inside_deadband_does_not_alternate_turns(self):
        c = SmoothApproach()
        commands = [
            c.step(0.01 if i % 2 else -0.01, 1, 0.45, i * 0.05) for i in range(100)
        ]
        self.assertTrue(all(l == r for l, r in commands))

    def test_speed_reduces_near_target(self):
        far = self.run_steps(0, 1)[-1]
        near = self.run_steps(0, 0.49)[-1]
        self.assertLess(near[0], far[0])
        self.assertGreaterEqual(near[0], 30)

    def test_restart_after_pause_starts_from_low_command(self):
        c = SmoothApproach()
        for i in range(50):
            c.step(0.3, 1, 0.45, i * 0.05)
        c.reset()
        self.assertEqual(c.step(0.3, 1, 0.45, 10), (30, 30))
