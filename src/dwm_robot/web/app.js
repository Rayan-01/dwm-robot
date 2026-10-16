"use strict";
const $ = (id) => document.getElementById(id);
const cameraOnly = location.pathname === "/camera";
if (cameraOnly) {
    document
        .querySelectorAll(".setup,.footnote")
        .forEach((el) => (el.hidden = true));
    document.body.classList.add("camera-view");
    $("connection").textContent = "Camera and robot controls";
}
let operator = "",
    connected = false,
    state = null;
let heartbeatBusy = false,
    pollBusy = false,
    holding = null,
    driveTimer = null,
    driveSeq = 0,
    stopBusy = false;
let loadedSettings = false,
    points = [],
    calibrationImage = null;
let lastPollError = "";
let actionEpoch = 0;
let driveBusy = false,
    lastDepthPreview = 0;
let previewBusy = false,
    previewTimer = null,
    previewUrl = null,
    depthUrl = null;
async function refreshPreview() {
    if (previewBusy || document.hidden || stopBusy) return;
    previewBusy = true;
    const started = performance.now(),
        controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 3000);
    try {
        // One request and decode at a time: never queue old video frames.
        const response = await fetch("/snapshot.jpg", {
            cache: "no-store",
            signal: controller.signal,
        });
        if (!response.ok) throw new Error("Camera is starting");
        const blob = await response.blob(),
            next = URL.createObjectURL(blob),
            image = new Image();
        try {
            image.src = next;
            await image.decode();
            const old = previewUrl;
            previewUrl = next;
            $("feed").src = next;
            $("no-feed").style.display = "none";
            if (old) URL.revokeObjectURL(old);
            if (
                state?.camera?.source === "OAK-D Pro" &&
                performance.now() - lastDepthPreview > 1000 &&
                !holding &&
                !heartbeatBusy
            ) {
                lastDepthPreview = performance.now();
                const depthResponse = await fetch("/depth.jpg", {
                    cache: "no-store",
                    signal: controller.signal,
                });
                if (depthResponse.ok) {
                    const nextDepth = URL.createObjectURL(
                            await depthResponse.blob(),
                        ),
                        depthImage = new Image();
                    try {
                        depthImage.src = nextDepth;
                        await depthImage.decode();
                        const oldDepth = depthUrl;
                        depthUrl = nextDepth;
                        $("depth-feed").src = nextDepth;
                        if (oldDepth) URL.revokeObjectURL(oldDepth);
                    } catch (error) {
                        URL.revokeObjectURL(nextDepth);
                        throw error;
                    }
                }
            }
        } catch (error) {
            URL.revokeObjectURL(next);
            throw error;
        }
    } catch (error) {
        if (!previewUrl) $("no-feed").textContent = "Waiting for camera…";
    } finally {
        clearTimeout(timeout);
        previewBusy = false;
        previewTimer = setTimeout(
            refreshPreview,
            Math.max(
                100,
                (connected ? 350 : 170) - (performance.now() - started),
            ),
        );
    }
}
document.addEventListener("visibilitychange", () => {
    if (!document.hidden) {
        clearTimeout(previewTimer);
        refreshPreview();
    }
});
const metreKeys = [
    "wheel_diameter_m",
    "axle_to_brush_m",
    "brush_width_m",
    "brush_margin_m",
    "pass_trigger_m",
];
const settingKeys = [
    ...metreKeys,
    "wheel_scale",
    "manual_pwm",
    "approach_pwm",
    "clean_pwm",
    "turn_pwm",
];
function notice(message, error = false) {
    $("notice").textContent = message;
    $("notice").classList.toggle("error", error);
}
async function api(action, data = {}, method = "POST") {
    const controller = new AbortController();
    const timer = setTimeout(
        () => controller.abort(),
        action === "state"
            ? 5000
            : ["claim", "settings", "calibrate"].includes(action)
              ? 3000
              : 1500,
    );
    try {
        const res = await fetch("/api/" + action, {
            method,
            signal: controller.signal,
            headers: {
                "X-Operator": operator,
                "Content-Type": "application/json",
            },
            ...(method === "POST" ? { body: JSON.stringify(data) } : {}),
        });
        const body = await res.json();
        if (!res.ok) throw new Error(body.error || "Request failed");
        return body;
    } catch (error) {
        if (controller.signal.aborted)
            throw new Error(
                "Robot connection timed out. Check the Wi-Fi connection.",
            );
        throw error;
    } finally {
        clearTimeout(timer);
    }
}
async function action(name, data = {}) {
    try {
        await api(name, data);
        notice("Command accepted");
        await poll();
    } catch (e) {
        notice(e.message, true);
    }
}
async function emergencyStop(cause = "button") {
    if (stopBusy) return;
    stopBusy = true;
    actionEpoch++;
    disconnect("Stopping robot.");
    document
        .querySelectorAll("#auto,#brush-on,#brush-off")
        .forEach((b) => (b.disabled = true));
    $("connect").disabled = true;
    clearTimeout(previewTimer);
    try {
        let confirmed = false;
        for (let attempt = 0; attempt < 3 && !confirmed; attempt++) {
            try {
                await api("stop", { cause });
                confirmed = true;
            } catch (error) {}
        }
        notice(
            confirmed
                ? "Robot and brush stopped. Choose an action to continue."
                : "Stop could not be confirmed. Use the physical emergency switch.",
            !confirmed,
        );
    } finally {
        stopBusy = false;
        $("connect").disabled = false;
        document
            .querySelectorAll("#auto,#brush-on,#brush-off")
            .forEach((b) => (b.disabled = false));
        refreshPreview();
    }
}
function disconnect(message) {
    connected = false;
    endDrive(false);
    $("connection").textContent = "Disconnected — stopped";
    $("connect").textContent = "Reconnect controls";
    document
        .querySelectorAll("[data-direction],#manual")
        .forEach((b) => (b.disabled = true));
    notice(message + " Choose an action to reconnect automatically.", true);
}
async function ensureControls() {
    if (stopBusy) throw new Error("Wait until Stop finishes");
    if (connected) {
        await api("heartbeat");
        return;
    }
    endDrive(false);
    const result = await api("claim");
    operator = result.operator;
    connected = true;
    driveSeq = 0;
    loadedSettings = false;
    $("connection").textContent = "Controls connected";
    $("connect").textContent = "Reconnect controls";
    $("no-feed").style.display = "none";
    notice("Connected. Robot stays stopped until an action is selected.");
}
$("connect").onclick = () =>
    ensureControls().catch((e) => notice(e.message, true));
let quickBusy = false;
async function quickAction(name, data = {}) {
    if (quickBusy || stopBusy) return;
    quickBusy = true;
    const epoch = actionEpoch;
    try {
        await ensureControls();
        if (stopBusy || !connected || epoch !== actionEpoch) return;
        if (name === "brush") {
            await api("stop", { cause: "brush_change" });
            if (stopBusy || epoch !== actionEpoch) return;
            if (data.on) {
                await api("manual");
                if (stopBusy || epoch !== actionEpoch) return;
                await api("brush", data);
            }
        } else await api(name, data);
        notice(
            name === "detect"
                ? "Auto detect started. Approaching the selected camera distance; brush stays off."
                : name === "manual"
                  ? "Manual ready. Hold a direction to move; release to stop."
                  : name === "clear_fault"
                    ? "Motor fault-clear requested. Waiting for Arduino acknowledgement."
                    : data.on
                      ? "Brush on; wheels stopped."
                      : "Brush off; robot stopped.",
        );
        await poll();
    } catch (e) {
        notice(e.message, true);
    } finally {
        if (epoch !== actionEpoch) {
            connected = false;
            await api("stop", { cause: "cancelled_action" }).catch(() => {});
        }
        quickBusy = false;
    }
}
$("stop").onclick = () => emergencyStop("button");
$("manual").onclick = () => quickAction("manual");
$("auto").onclick = () =>
    quickAction("detect", {
        distance_cm: Number($("stop-distance").value),
        forward_pass_m: Number($("forward-pass-distance").value),
        clean_after_pass: $("clean-after-pass").checked,
    });
$("brush-on").onclick = () => quickAction("brush", { on: true });
$("brush-off").onclick = () => quickAction("brush", { on: false });
$("clear-fault").onclick = () => quickAction("clear_fault");
function endDrive(send = true) {
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = null;
    holding = null;
    document
        .querySelectorAll("[data-direction]")
        .forEach((b) => b.classList.remove("active"));
    if (send && connected)
        api("drive", { direction: "stop", seq: ++driveSeq }).catch((e) =>
            notice(e.message, true),
        );
}
document.querySelectorAll("[data-direction]").forEach((b) => {
    b.addEventListener("pointerdown", (e) => {
        e.preventDefault();
        if (!connected) return;
        endDrive(false);
        b.setPointerCapture(e.pointerId);
        holding = b.dataset.direction;
        b.classList.add("active");
        const drive = async () => {
            if (!holding || driveBusy) return;
            driveBusy = true;
            try {
                await api("drive", { direction: holding, seq: ++driveSeq });
            } catch (e) {
                notice(e.message, true);
                endDrive(false);
            } finally {
                driveBusy = false;
            }
        };
        drive();
        driveTimer = setInterval(drive, 120);
    });
    b.addEventListener("pointerup", () => endDrive());
    b.addEventListener("pointercancel", () => endDrive());
    b.addEventListener("lostpointercapture", () => {
        if (holding) endDrive();
    });
});
document.addEventListener("keydown", (e) => {
    if (
        e.code === "Space" &&
        !["INPUT", "TEXTAREA"].includes(document.activeElement.tagName)
    ) {
        e.preventDefault();
        emergencyStop("keyboard");
    }
});
window.addEventListener("blur", () => {
    if (connected && holding) endDrive();
});
document.addEventListener("visibilitychange", () => {
    if (document.hidden && connected) {
        emergencyStop("visibility");
        disconnect("Dashboard hidden; controls stopped.");
    }
});
window.addEventListener("pagehide", () => {
    if (connected)
        fetch("/api/stop", {
            method: "POST",
            keepalive: true,
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ cause: "page_closed" }),
        }).catch(() => {});
});
async function poll() {
    if (pollBusy || document.hidden) return;
    pollBusy = true;
    try {
        state = await api("state", {}, "GET");
        const oak = state.camera?.source === "OAK-D Pro";
        $("depth-panel").hidden = !oak;
        document.body.classList.toggle("oak-view", oak);
        $("depth-status").textContent =
            state.camera?.depth_age_s == null
                ? "No depth"
                : state.camera.depth_age_s > 0.65
                  ? "Depth stale"
                  : Math.round(state.camera.depth_age_s * 1000) + " ms old";
        $("sensor-status").textContent =
            (state.camera?.source || "Camera") +
            " · detection threshold " +
            Math.round(state.detection_threshold * 100) +
            "%";
        $("target-distances").textContent =
            (state.detections || [])
                .map(
                    (d) =>
                        "#" +
                        d.id +
                        " · " +
                        (d.source === "dark_grass"
                            ? "Possible poop · dark object on grass"
                            : Math.round(d.confidence * 100) + "%") +
                        " · " +
                        (d.depth
                            ? d.depth.z_m.toFixed(2) + " m from camera"
                            : "depth unavailable"),
                )
                .join(" | ") || "No current poop detection";
        if (lastPollError && $("notice").textContent === lastPollError)
            notice(
                connected
                    ? "Connected. Robot stays stopped until enabled."
                    : "Connect, then enable manual control when ready.",
            );
        lastPollError = "";
        $("phase").textContent = state.phase;
        $("reason").textContent = state.error || state.reason;
        $("fps").textContent =
            (state.camera_fps || state.fps) +
            " camera FPS · " +
            state.fps +
            " detection FPS";
        $("runmode").textContent = state.demo
            ? "SIMULATION"
            : state.dry_run
              ? "DRY RUN • no motor writes"
              : "HARDWARE";
        $("target").textContent = state.target
            ? "#" +
              state.target.id +
              " · " +
              (state.target.source === "dark_grass"
                  ? "Possible poop"
                  : Math.round(state.target.confidence * 100) + "%")
            : "—";
        $("wheels").textContent =
            state.left_rpm + " / " + state.right_rpm + " rpm";
        const rpm = state.drive_motor_rpm;
        $("motor-rpm").textContent =
            rpm?.left != null && rpm?.right != null
                ? rpm.left.toFixed(1) + " / " + rpm.right.toFixed(1) + " rpm"
                : "Waiting for motor RPM";
        $("rpm-note").textContent =
            "Left / right · gearbox " +
            (rpm?.gear_ratio || 20) +
            ":1 · Motor RPM uses an assumed " +
            (rpm?.fg_pulses_per_motor_rev_assumed || 4) +
            " FG pulses per motor revolution; absolute RPM calibration is not verified. Direction follows the commanded motor direction.";
        $("wheels").closest(".card").hidden = !state.wheel_rpm_enabled;
        $("travel").closest(".card").hidden = !state.wheel_rpm_enabled;
        const imu = state.imu,
            data = imu && imu.connected ? imu.data : null;
        $("imu-status").textContent =
            imu && imu.error
                ? "Invalid sensor reading"
                : data
                  ? "Live · " + Math.round(imu.age_s * 1000) + " ms ago"
                  : imu && imu.data
                    ? "Sensor stale"
                    : "No IMU telemetry";
        $("imu-orientation").textContent = data
            ? [data.roll_deg, data.pitch_deg, data.yaw_deg]
                  .map((v) => v.toFixed(1))
                  .join(" / ")
            : "—";
        $("imu-gyro").textContent = data
            ? data.gyro_rad_s.map((v) => v.toFixed(3)).join(" / ")
            : "—";
        $("imu-acceleration").textContent = data
            ? data.acceleration_m_s2.map((v) => v.toFixed(3)).join(" / ")
            : "—";
        $("imu-note").textContent =
            imu && imu.error
                ? imu.error
                : "Orientation follows the sensor mounting axes.";
        $("brush").textContent = state.output[2] ? "ON" : "OFF";
        $("cleaning-status").textContent = state.cleaning?.active
            ? "Cleaning motor on: " +
              state.cleaning.remaining_s.toFixed(1) +
              " seconds remaining"
            : state.phase === "BEFORE_PASS_PAUSE"
              ? "Drive stopped for 1 second; brush starts with the forward pass"
              : state.phase === "FORWARD_PASS"
                ? "Brush on during the forward pass"
                : state.phase === "WHEELS_SETTLING"
                  ? "Drive wheels stopped; waiting 2 seconds before cleaning"
                  : state.cleaning
                    ? "Cleaning motor off"
                    : "";
        $("travel").textContent = state.forward_pass
            ? state.forward_pass.estimated_travel_m.toFixed(2) +
              " / " +
              state.forward_pass.target_m.toFixed(2) +
              " m estimated"
            : state.phase === "CLEAN_PASS"
              ? state.travel_m + " / " + state.pass_distance_m + " m"
              : ((state.odometry_left_m + state.odometry_right_m) / 2).toFixed(
                    3,
                ) + " m total";
        $("battery").textContent = state.battery
            ? state.battery.voltage.toFixed(1) +
              " V · " +
              Math.round(state.battery.percent) +
              "%"
            : "No telemetry";
        $("ready").textContent = !state.wheel_rpm_enabled
            ? "Manual only · RPM feedback bypassed"
            : state.auto_ready
              ? "Auto clean ready"
              : state.calibrated
                ? "Verify geometry and travel"
                : "Ground calibration required";
        $("manual").disabled = stopBusy || quickBusy;
        $("auto").disabled =
            stopBusy || quickBusy || state.camera?.source !== "OAK-D Pro";
        $("brush-on").disabled = stopBusy || quickBusy;
        $("brush-off").disabled = stopBusy || quickBusy;
        const manual =
            connected &&
            state.mode === "MANUAL" &&
            state.phase === "MANUAL" &&
            !state.stopped;
        document
            .querySelectorAll("[data-direction]")
            .forEach((b) => (b.disabled = !manual));
        const c = state.connection;
        const driver = state.motor_driver;
        const updatedDriver = driver?.protocol === "motor_safety_v2";
        $("clear-fault").hidden = !updatedDriver || !driver.fault.code;
        $("clear-fault").disabled =
            stopBusy || quickBusy || !state.stopped || driver?.clear_pending;
        $("motor-status").textContent = updatedDriver
            ? "Motor driver: " +
              (driver.status
                  ? "left " +
                    driver.status.left.phase +
                    " / right " +
                    driver.status.right.phase +
                    " · actual PWM " +
                    driver.status.left.actual_pwm +
                    " / " +
                    driver.status.right.actual_pwm
                  : "waiting for status") +
              (driver.fault.code
                  ? " · " +
                    driver.fault.name +
                    " (wheel " +
                    driver.fault.wheel +
                    ")"
                  : " · no motor fault")
            : "";
        if (c) {
            const age =
                c.wheel_age_s === null
                    ? "no packets"
                    : Math.round(c.wheel_age_s * 1000) + " ms ago";
            $("link-health").textContent = state.dry_run
                ? "Arduino: no hardware connection in this mode"
                : "Arduino: " +
                  (c.serial_reader_alive
                      ? "reader running"
                      : "reader stopped") +
                  " · wheels " +
                  age +
                  " · " +
                  c.wheel_packets +
                  " valid packets · " +
                  c.bad_serial_lines +
                  " invalid" +
                  (c.serial_error ? " · " + c.serial_error : "");
            if (c.feedback_noise)
                $("link-health").textContent +=
                    " · FG noise filtered (left/right " +
                    c.fg_rejected.join("/") +
                    "); auto clean blocked";
            if (!state.wheel_rpm_enabled)
                $("link-health").textContent =
                    "Arduino: " +
                    (c.serial_reader_alive ? "connected" : "disconnected") +
                    " · manual control · RPM feedback disabled";
        }
        if (!loadedSettings) {
            for (const key of settingKeys)
                $(key).value = metreKeys.includes(key)
                    ? Number((state.setup[key] * 100).toFixed(2))
                    : state.setup[key];
            $("geometry_verified").checked = state.setup.geometry_verified;
            loadedSettings = true;
        }
    } catch (e) {
        lastPollError = e.message;
        notice(e.message, true);
    } finally {
        pollBusy = false;
    }
}
setInterval(async () => {
    if (!connected || heartbeatBusy || document.hidden) return;
    heartbeatBusy = true;
    try {
        await api("heartbeat");
    } catch (e) {
        emergencyStop("connection");
        disconnect(e.message);
    } finally {
        heartbeatBusy = false;
    }
}, 300);
setInterval(poll, 800);
refreshPreview();
poll();
$("save-settings").onclick = () => {
    const data = { geometry_verified: $("geometry_verified").checked };
    for (const key of settingKeys)
        data[key] = Number($(key).value) / (metreKeys.includes(key) ? 100 : 1);
    action("settings", data);
};
function drawCalibration() {
    const c = $("calibration"),
        ctx = c.getContext("2d");
    ctx.clearRect(0, 0, c.width, c.height);
    if (calibrationImage)
        ctx.drawImage(calibrationImage, 0, 0, c.width, c.height);
    points.forEach((p, i) => {
        ctx.fillStyle = "#ffce55";
        ctx.beginPath();
        ctx.arc(p[0], p[1], 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = "bold 20px sans-serif";
        ctx.fillText(String(i + 1), p[0] + 10, p[1] - 10);
    });
    const labels = ["far-left", "far-right", "near-right", "near-left"];
    $("cal-instruction").textContent =
        points.length < 4
            ? "Next click: " + (points.length + 1) + " " + labels[points.length]
            : "Four corners selected. Save ground calibration.";
}
$("freeze").onclick = async () => {
    if (!connected) return notice("Connect first", true);
    if (!state?.stopped) return notice("Stop robot before calibration", true);
    try {
        const r = await fetch("/snapshot.jpg");
        if (!r.ok) throw new Error("No camera frame yet");
        const url = URL.createObjectURL(await r.blob()),
            im = new Image();
        im.onload = () => {
            calibrationImage = im;
            $("calibration").width = state.image_size[0];
            $("calibration").height = state.image_size[1];
            points = [];
            drawCalibration();
            URL.revokeObjectURL(url);
        };
        im.src = url;
    } catch (e) {
        notice(e.message, true);
    }
};
$("calibration").addEventListener("pointerdown", (e) => {
    if (!calibrationImage || points.length >= 4) return;
    const c = e.currentTarget,
        r = c.getBoundingClientRect();
    points.push([
        ((e.clientX - r.left) * c.width) / r.width,
        ((e.clientY - r.top) * c.height) / r.height,
    ]);
    drawCalibration();
});
$("clear-points").onclick = () => {
    points = [];
    drawCalibration();
};
$("save-cal").onclick = async () => {
    if (points.length !== 4)
        return notice("Click all four corners first", true);
    try {
        await api("calibrate", {
            points,
            width_m: Number($("cal-width").value) / 100,
            near_m: Number($("cal-near").value) / 100,
            far_m: Number($("cal-far").value) / 100,
        });
        $("geometry_verified").checked = false;
        notice(
            "Ground calibration saved. Verify distances and travel before enabling auto.",
        );
        await poll();
    } catch (e) {
        notice(e.message, true);
    }
};
