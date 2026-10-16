import unittest
import test_forward_pass


class CleaningTests(unittest.TestCase):
    setUp = test_forward_pass.PassTransitionTests.setUp
    tearDown = test_forward_pass.PassTransitionTests.tearDown
    target = test_forward_pass.PassTransitionTests.target
    begin_pass = test_forward_pass.PassTransitionTests.begin_pass

    def tick(self, offset, rpm=0):
        at = self.now + offset
        e = self.rt.engine
        e.browser_at = e.capture_at = e.vision_at = e.telemetry_at = at
        e.left_rpm = e.right_rpm = rpm
        self.rt.follow_tick(at)
        return e.tick(at)

    def complete_pass(self):
        self.begin_pass()
        self.rt.follow["clean_after_pass"] = True
        self.rt.engine.left_m += 1
        self.rt.engine.right_m += 1
        self.tick(0.05, 10)
        self.assertEqual(self.rt.engine.phase, "WHEELS_SETTLING")

    def test_wheels_settle_before_cleaning_and_twenty_seconds_turns_off(self):
        self.complete_pass()
        self.assertEqual(self.tick(0.2, 5), (0, 0, False))
        self.assertEqual(self.tick(0.5), (0, 0, False))
        self.assertEqual(self.tick(2.49), (0, 0, False))
        self.assertEqual(self.tick(2.5), (0, 0, True))
        self.assertEqual(self.tick(12.5), (0, 0, True))
        self.assertEqual(self.tick(22.49), (0, 0, True))
        self.assertEqual(self.tick(22.5), (0, 0, False))
        self.assertTrue(self.rt.engine.latched)
        self.assertIn("20-second cleaning completed", self.rt.engine.reason)

    def test_stop_immediately_turns_off_cleaning(self):
        self.complete_pass()
        self.tick(0.1)
        self.tick(2.15)
        self.assertTrue(self.rt.cleaning_status["active"])
        self.rt.stop()
        self.assertFalse(self.rt.cleaning_status["active"])
        self.assertEqual(self.rt.engine.output, (0, 0, False))

    def test_wheel_movement_during_cleaning_cancels_cycle(self):
        self.complete_pass()
        self.tick(0.1)
        self.tick(2.15)
        self.assertEqual(self.tick(2.2, 2), (0, 0, False))
        self.assertTrue(self.rt.engine.latched)

    def test_failed_pass_does_not_start_brush(self):
        self.begin_pass()
        self.rt.follow["clean_after_pass"] = True
        self.rt.engine.left_m += 0.2
        self.assertEqual(self.tick(0.1), (0, 0, False))
        self.assertTrue(self.rt.engine.latched)
        self.assertIsNone(self.rt.cleaning_status)

    def test_link_loss_during_cleaning_cancels_brush(self):
        self.complete_pass()
        self.tick(0.1)
        self.tick(2.15)
        e = self.rt.engine
        e.browser_at = self.now - 10
        self.rt.follow_tick(self.now + 2.2)
        self.assertEqual(e.output, (0, 0, False))
        self.assertFalse(self.rt.cleaning_status["active"])
