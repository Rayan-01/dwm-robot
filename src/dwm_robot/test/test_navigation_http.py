import io
import json
import unittest
from dwm_robot.robot import Handler


class NavigationHTTPTests(unittest.TestCase):
    def test_unsaved_route_is_valid_json_null_with_content_length(self):
        handler = Handler.__new__(Handler)
        headers = {}
        handler.send_response = lambda status: self.assertEqual(status, 200)
        handler.send_header = lambda key, value: headers.update({key: value})
        handler.end_headers = lambda: None
        handler.wfile = io.BytesIO()
        handler.response(200, None)
        body = handler.wfile.getvalue()
        self.assertIsNone(json.loads(body))
        self.assertEqual(int(headers["Content-Length"]), len(body))

    def test_binary_camera_response_is_preserved(self):
        handler = Handler.__new__(Handler)
        handler.send_response = lambda _: None
        handler.send_header = lambda *_: None
        handler.end_headers = lambda: None
        handler.wfile = io.BytesIO()
        body = b"\xff\xd8camera\xff\xd9"
        handler.response(200, body, "image/jpeg")
        self.assertEqual(handler.wfile.getvalue(), body)
