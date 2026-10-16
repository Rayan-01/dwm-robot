import unittest
from types import SimpleNamespace
from dwm_robot.robot import Handler


class TransportTests(unittest.TestCase):
    def test_disconnected_browser_is_not_an_api_failure(self):
        for error in (BrokenPipeError, ConnectionResetError):
            for stage in ("headers", "body"):

                def broken(*args):
                    raise error("browser closed connection")

                handler = SimpleNamespace(
                    send_response=lambda *args: None,
                    send_header=lambda *args: None,
                    close_connection=False,
                    end_headers=broken if stage == "headers" else lambda: None,
                    wfile=SimpleNamespace(
                        write=broken if stage == "body" else lambda *args: None
                    ),
                )
                Handler.response(handler, 200, {"ok": True})
                self.assertTrue(handler.close_connection)

    def test_unrelated_write_failure_still_reports_error(self):
        def failed(*args):
            raise RuntimeError("unexpected write failure")

        handler = SimpleNamespace(
            send_response=lambda *args: None,
            send_header=lambda *args: None,
            end_headers=lambda: None,
            wfile=SimpleNamespace(write=failed),
        )
        with self.assertRaises(RuntimeError):
            Handler.response(handler, 200, {"ok": True})
