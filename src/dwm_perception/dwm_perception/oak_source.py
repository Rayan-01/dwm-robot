"""Synchronized RGB and aligned metric depth from one OAK-D device."""

import time


class OakSource:
    def __init__(self):
        import depthai as dai

        self.dai = dai
        p = dai.Pipeline()
        rgb = p.create(dai.node.ColorCamera)
        rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setIspScale(1, 3)
        rgb.setFps(15)
        left = p.create(dai.node.MonoCamera)
        right = p.create(dai.node.MonoCamera)
        for cam, socket in (
            (left, dai.CameraBoardSocket.CAM_B),
            (right, dai.CameraBoardSocket.CAM_C),
        ):
            cam.setBoardSocket(socket)
            cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            cam.setFps(15)
        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(False)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(640, 360)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        sync = p.create(dai.node.Sync)
        from datetime import timedelta

        sync.setSyncThreshold(timedelta(milliseconds=35))
        sync.setSyncAttempts(-1)
        rgb.isp.link(sync.inputs["rgb"])
        stereo.depth.link(sync.inputs["depth"])
        out = p.create(dai.node.XLinkOut)
        out.setStreamName("frames")
        sync.out.link(out.input)
        self.device = dai.Device(p)
        self.info = {
            "name": self.device.getDeviceName(),
            "mxid": self.device.getMxId(),
            "usb_speed": str(self.device.getUsbSpeed()),
        }
        self.queue = self.device.getOutputQueue("frames", maxSize=1, blocking=False)

    def read(self, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            group = self.queue.tryGet()
            if group is not None:
                rgb, depth = group["rgb"], group["depth"]
                age = max(
                    0, (self.dai.Clock.now() - rgb.getTimestamp()).total_seconds()
                )
                skew = abs((rgb.getTimestamp() - depth.getTimestamp()).total_seconds())
                return rgb.getCvFrame(), depth.getFrame(), time.monotonic() - age, skew
            time.sleep(0.005)
        raise RuntimeError("OAK colour/depth frames stopped")

    def close(self):
        self.device.close()


def box_depth(depth, box):
    """Median valid depth in the middle of a detection; never invent missing depth."""
    import numpy as np

    h, w = depth.shape
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    rx, ry = max(2, (x2 - x1) * 0.2), max(2, (y2 - y1) * 0.2)
    roi = depth[
        max(0, int(cy - ry)) : min(h, int(cy + ry) + 1),
        max(0, int(cx - rx)) : min(w, int(cx + rx) + 1),
    ]
    valid = roi[(roi >= 200) & (roi <= 10000)]
    if roi.size == 0 or valid.size < max(8, roi.size * 0.3):
        return None
    return {
        "z_m": round(float(np.median(valid)) / 1000, 3),
        "valid_fraction": round(valid.size / roi.size, 2),
    }
