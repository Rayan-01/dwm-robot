import argparse
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer
from dwm_robot import robot
from dwm_core.control import DEFAULTS


class APITests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.oldroot = robot.ROOT
        robot.ROOT = Path(self.temp.name)
        (robot.ROOT / "config.json").write_text(json.dumps(DEFAULTS))
        args = argparse.Namespace(
            config="config.json",
            camera=None,
            device=None,
            serial=None,
            demo=True,
            hardware=False,
        )
        self.rt = robot.Runtime(args)
        now = time.monotonic()
        e = self.rt.engine
        e.vision_ok = True
        e.capture_at = e.vision_at = now
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), robot.Handler)
        self.server.daemon_threads = True
        self.server.runtime = self.rt
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = "http://127.0.0.1:" + str(self.server.server_port)
        self.operator = ""

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.rt.close()
        robot.ROOT = self.oldroot
        self.temp.cleanup()

    def request(
        self,
        path,
        data=None,
        origin=None,
        content_type="application/json",
        fetch_site=None,
    ):
        headers = {"Content-Type": content_type}
        if fetch_site:
            headers["Sec-Fetch-Site"] = fetch_site
        if self.operator:
            headers["X-Operator"] = self.operator
        if origin:
            headers["Origin"] = origin
        req = Request(
            self.url + path,
            headers=headers,
            data=None if data is None else json.dumps(data).encode(),
        )
        try:
            with urlopen(req, timeout=2) as r:
                return r.status, json.load(r)
        except HTTPError as e:
            return e.code, json.load(e)

    def claim(self):
        status, res = self.request("/api/claim", {})
        self.assertEqual(status, 200)
        self.operator = res["operator"]

    def test_state_and_claim_need_no_login_token(self):
        self.assertEqual(self.request("/api/state")[0], 200)
        self.claim()
        self.assertFalse((robot.ROOT / ".control_token").exists())

    def test_camera_endpoints_need_no_login_token(self):
        self.rt.jpeg = b"camera-frame"
        self.rt.jpeg_seq = 1
        with urlopen(self.url + "/snapshot.jpg", timeout=2) as response:
            self.assertEqual(response.read(), b"camera-frame")
        with urlopen(self.url + "/stream", timeout=2) as response:
            self.assertEqual(response.status, 200)
            self.assertEqual(response.readline(), b"--frame\r\n")

    def test_stop_works_without_operator_claim(self):
        self.assertEqual(self.request("/api/stop", {})[0], 200)
        self.assertTrue(self.rt.engine.latched)

    def test_automatic_stop_reports_its_actual_trigger(self):
        for cause, text in [
            ("visibility", "Dashboard hidden"),
            ("connection", "heartbeat failed"),
            ("button", "Stop button"),
            ("page_closed", "closed or reloaded"),
        ]:
            self.assertEqual(self.request("/api/stop", {"cause": cause})[0], 200)
            self.assertIn(text, self.rt.engine.reason)

    def test_invalid_stop_cause_still_stops_without_server_fault(self):
        self.assertEqual(self.request("/api/stop", {"cause": {}})[0], 200)
        self.assertTrue(self.rt.engine.latched)

    def test_claim_does_not_start_motion(self):
        self.claim()
        self.assertTrue(self.rt.engine.latched)

    def test_second_operator_cannot_claim_active_browser(self):
        self.claim()
        self.operator = ""
        self.assertEqual(self.request("/api/claim", {})[0], 400)

    def test_current_operator_can_reconnect_and_stays_stopped(self):
        self.claim()
        old_operator = self.operator
        self.claim()
        self.assertNotEqual(self.operator, old_operator)
        self.assertTrue(self.rt.engine.latched)

    def test_enable_drive_and_stop(self):
        self.claim()
        self.assertEqual(self.request("/api/manual", {})[0], 200)
        self.assertEqual(
            self.request("/api/drive", {"direction": "forward", "seq": 1})[0], 200
        )
        self.assertEqual(self.rt.engine.tick(time.monotonic()), (55, 55, False))
        self.request("/api/brush", {"on": True})
        self.request("/api/stop", {})
        self.assertEqual(self.rt.engine.tick(time.monotonic()), (0, 0, False))
        self.assertEqual(
            self.request("/api/drive", {"direction": "forward", "seq": 2})[0], 400
        )

    def test_late_direction_packet_cannot_override_release(self):
        self.claim()
        self.request("/api/manual", {})
        self.request("/api/drive", {"direction": "stop", "seq": 2})
        self.request("/api/drive", {"direction": "forward", "seq": 1})
        self.assertEqual(self.rt.engine.tick(time.monotonic()), (0, 0, False))

    def test_mutation_from_another_origin_denied(self):
        self.assertEqual(
            self.request("/api/claim", {}, origin="http://evil.example")[0], 403
        )

    def test_cross_site_request_without_origin_denied(self):
        self.assertEqual(
            self.request("/api/claim", {}, fetch_site="cross-site")[0], 403
        )

    def test_plain_form_submission_denied(self):
        self.assertEqual(
            self.request("/api/stop", {}, content_type="text/plain")[0], 415
        )

    def test_same_origin_dashboard_can_connect(self):
        self.assertEqual(self.request("/api/claim", {}, origin=self.url)[0], 200)

    def test_invalid_numeric_settings_rejected(self):
        self.claim()
        self.assertEqual(self.request("/api/settings", {"manual_pwm": 999})[0], 400)

    def test_only_owner_can_drive(self):
        self.claim()
        self.operator = "wrong"
        self.assertEqual(self.request("/api/manual", {})[0], 400)

    def test_expired_heartbeat_cannot_resume_motion(self):
        self.claim()
        self.rt.operator_seen -= 3
        self.assertEqual(self.request("/api/heartbeat", {})[0], 400)
        self.assertTrue(self.rt.engine.latched)
        self.assertIsNone(self.rt.operator)

    def test_settings_blocked_while_armed(self):
        self.claim()
        self.request("/api/manual", {})
        self.assertEqual(self.request("/api/settings", {"manual_pwm": 60})[0], 400)

    def test_json_array_rejected(self):
        self.claim()
        self.assertEqual(self.request("/api/manual", [])[0], 400)


if __name__ == "__main__":
    unittest.main()
