"use strict";
const $ = (id) => document.getElementById(id);
let grid = null,
    raster = null,
    points = [],
    state = null,
    scale = 1,
    ox = 0,
    oy = 0,
    selected = "",
    dirty = false;
let down = null,
    dragged = false,
    loadEpoch = 0;
let operator = "",
    connected = false,
    poseMode = false,
    holding = null,
    driveTimer = null,
    alignmentLoaded = false;
let statusUntil = 0;
function showStatus(message) {
    $("status").textContent = message;
    statusUntil = performance.now() + 5000;
}
async function api(path, data) {
    const r = await fetch("/api/navigation/" + path, {
        signal: AbortSignal.timeout(4000),
        ...(data
            ? {
                  method: "POST",
                  headers: {
                      "Content-Type": "application/json",
                      "X-Operator": operator,
                  },
                  body: JSON.stringify(data),
              }
            : {}),
    });
    const value = await r.json();
    if (!r.ok) throw Error(value.error || "Navigation request failed");
    return value;
}
function size(canvas) {
    const r = canvas.getBoundingClientRect();
    if (
        canvas.width !== Math.round(r.width) ||
        canvas.height !== Math.round(r.height)
    ) {
        canvas.width = Math.round(r.width);
        canvas.height = Math.round(r.height);
    }
    return [canvas.width, canvas.height];
}
function worldPixel(p) {
    const [x, y, a] = grid.metadata.origin,
        dx = p.x - x,
        dy = p.y - y,
        r = grid.metadata.resolution;
    return [
        (Math.cos(a) * dx + Math.sin(a) * dy) / r,
        grid.height - (-Math.sin(a) * dx + Math.cos(a) * dy) / r,
    ];
}
function pixelWorld(px, py) {
    const [x, y, a] = grid.metadata.origin,
        r = grid.metadata.resolution,
        lx = px * r,
        ly = (grid.height - py) * r;
    return {
        x: x + Math.cos(a) * lx - Math.sin(a) * ly,
        y: y + Math.sin(a) * lx + Math.cos(a) * ly,
        yaw: 0,
    };
}
function screen(p) {
    const [x, y] = worldPixel(p);
    return [ox + x * scale, oy + y * scale];
}
function redraw() {
    const c = $("map"),
        [w, h] = size(c),
        ctx = c.getContext("2d");
    ctx.clearRect(0, 0, w, h);
    if (!grid) {
        ctx.fillStyle = "#234";
        ctx.font = "16px sans-serif";
        ctx.fillText("Choose a saved map to place waypoints", 20, 40);
        return;
    }
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(raster, ox, oy, grid.width * scale, grid.height * scale);
    function line(list, dash) {
        if (!list.length) return;
        ctx.beginPath();
        ctx.setLineDash(dash);
        list.forEach((p, i) => {
            const [x, y] = screen(p);
            i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
        });
        ctx.strokeStyle = "#ff8c32";
        ctx.lineWidth = 3;
        ctx.stroke();
        ctx.setLineDash([]);
    }
    line(points, [6, 6]);
    // Only a real, matching active map may carry a live plan; saved drafts have no localization.
    if (state?.active_map_name === selected)
        line(
            (state.plan || []).map((p) => ({ x: p[0], y: p[1] })),
            [],
        );
    if (
        state?.robot &&
        (state.mode === "mapping" || state.active_map_name === selected)
    ) {
        const p = state.robot,
            [x, y] = screen(p);
        ctx.fillStyle = "#e87518";
        ctx.beginPath();
        for (const [bx, by] of state.geometry.navigation_footprint_m) {
            const [sx, sy] = screen({
                x: p.x + Math.cos(p.yaw) * bx - Math.sin(p.yaw) * by,
                y: p.y + Math.sin(p.yaw) * bx + Math.cos(p.yaw) * by,
            });
            ctx.lineTo(sx, sy);
        }
        ctx.closePath();
        ctx.fill();
        const [hx, hy] = screen({
            x: p.x + 0.35 * Math.cos(p.yaw),
            y: p.y + 0.35 * Math.sin(p.yaw),
        });
        ctx.strokeStyle = "#ffde68";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(hx, hy);
        ctx.stroke();
        ctx.fillStyle = "#14b295";
        for (const [bx, by] of state.body_points || []) {
            const [sx, sy] = screen({
                x: p.x + Math.cos(p.yaw) * bx - Math.sin(p.yaw) * by,
                y: p.y + Math.sin(p.yaw) * bx + Math.cos(p.yaw) * by,
            });
            ctx.fillRect(sx, sy, 2, 2);
        }
    }
    points.forEach((p, i) => {
        const [x, y] = screen(p);
        ctx.beginPath();
        ctx.arc(x, y, 8, 0, 2 * Math.PI);
        ctx.fillStyle = "#111";
        ctx.fill();
        ctx.strokeStyle = "#fff";
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.fillStyle = "#111";
        ctx.font = "bold 16px sans-serif";
        ctx.fillText(String(i + 1), x + 11, y - 8);
    });
}
function list() {
    $("points").replaceChildren();
    points.forEach((p, i) => {
        const li = document.createElement("li");
        li.textContent = `${p.x.toFixed(2)}, ${p.y.toFixed(2)} m`;
        const b = document.createElement("button");
        b.textContent = "Remove";
        b.onclick = () => {
            points.splice(i, 1);
            dirty = true;
            list();
            redraw();
        };
        li.append(b);
        $("points").append(li);
    });
}
function setGrid(g) {
    grid = g;
    const raw = Uint8Array.from(atob(g.pixels), (c) => c.charCodeAt(0));
    raster = document.createElement("canvas");
    raster.width = g.width;
    raster.height = g.height;
    const ctx = raster.getContext("2d"),
        image = ctx.createImageData(g.width, g.height);
    for (let i = 0; i < raw.length; i++) {
        const v = g.metadata.negate ? 255 - raw[i] : raw[i];
        image.data.set([v, v, v, 255], 4 * i);
    }
    ctx.putImageData(image, 0, 0);
    const [w, h] = size($("map"));
    scale = Math.min((w - 20) / g.width, (h - 20) / g.height);
    ox = (w - g.width * scale) / 2;
    oy = (h - g.height * scale) / 2;
    redraw();
}
$("maps").onchange = async () => {
    const name = $("maps").value,
        epoch = ++loadEpoch;
    if (dirty && !confirm("Discard unsaved waypoint changes?")) {
        $("maps").value = selected;
        return;
    }
    selected = name;
    grid = null;
    points = [];
    dirty = false;
    list();
    redraw();
    if (!name) return;
    try {
        const [g, route] = await Promise.all([
            api("map?name=" + encodeURIComponent(name)),
            api("route?name=" + encodeURIComponent(name)),
        ]);
        if (epoch !== loadEpoch) return;
        setGrid(g);
        points = route?.waypoints || [];
        list();
        redraw();
    } catch (e) {
        if (epoch === loadEpoch) $("status").textContent = e.message;
    }
};
$("save").onclick = async () => {
    try {
        await api("route", { map_name: selected, waypoints: points });
        dirty = false;
        $("status").textContent =
            "Waypoint draft saved. Robot has not been commanded to move.";
    } catch (e) {
        $("status").textContent = e.message;
    }
};
$("undo").onclick = () => {
    points.pop();
    dirty = true;
    list();
    redraw();
};
$("clear").onclick = () => {
    points = [];
    dirty = true;
    list();
    redraw();
};
const canvas = $("map");
canvas.onpointerdown = (e) => {
    canvas.setPointerCapture(e.pointerId);
    down = { x: e.offsetX, y: e.offsetY, ox, oy };
    dragged = false;
};
canvas.onpointermove = (e) => {
    if (!down) return;
    const dx = e.offsetX - down.x,
        dy = e.offsetY - down.y;
    if (Math.hypot(dx, dy) > 5) dragged = true;
    if (dragged) {
        ox = down.ox + dx;
        oy = down.oy + dy;
        redraw();
    }
};
canvas.onpointercancel = () => {
    down = null;
};
canvas.onpointerup = (e) => {
    if (down && !dragged && grid) {
        const px = (e.offsetX - ox) / scale,
            py = (e.offsetY - oy) / scale,
            ix = Math.floor(px),
            iy = Math.floor(py);
        if (ix < 0 || iy < 0 || ix >= grid.width || iy >= grid.height) {
            $("status").textContent = "Choose a point inside the map.";
        } else {
            const v = raster.getContext("2d").getImageData(ix, iy, 1, 1)
                    .data[0],
                occupied = (255 - v) / 255;
            if (occupied >= grid.metadata.free_thresh) {
                $("status").textContent =
                    "Choose known free space, away from the wall.";
            } else if (points.length >= 100) {
                $("status").textContent = "Maximum 100 waypoints.";
            } else if (poseMode) {
                const pose = pixelWorld(px, py);
                pose.yaw = (Number($("pose-yaw").value) * Math.PI) / 180;
                command("initial", pose);
                poseMode = false;
                $("pose-mode").textContent = "Set robot position on map";
            } else {
                points.push(pixelWorld(px, py));
                dirty = true;
                list();
                redraw();
            }
        }
    }
    down = null;
};
canvas.onwheel = (e) => {
    e.preventDefault();
    if (!grid) return;
    const next = Math.max(
        0.15,
        Math.min(40, scale * Math.exp(-e.deltaY * 0.001)),
    );
    ox = e.offsetX - ((e.offsetX - ox) * next) / scale;
    oy = e.offsetY - ((e.offsetY - oy) * next) / scale;
    scale = next;
    redraw();
};
function laser(scan) {
    const c = $("laser"),
        [w, h] = size(c),
        ctx = c.getContext("2d");
    ctx.clearRect(0, 0, w, h);
    const s = Math.min(w, h) / 12;
    ctx.strokeStyle = "#345";
    ctx.fillStyle = "#abc";
    ctx.font = "12px sans-serif";
    for (let r = 1; r <= 5; r++) {
        ctx.beginPath();
        ctx.arc(w / 2, h / 2, r * s, 0, 2 * Math.PI);
        ctx.stroke();
        if (r % 2 === 0) ctx.fillText(r + " m", w / 2 + r * s, h / 2);
    }
    ctx.fillStyle = "#70e6ca";
    for (const [x, y] of state?.body_points || scan.points || []) {
        if (Math.hypot(x, y) <= 6)
            ctx.fillRect(w / 2 - y * s, h / 2 - x * s, 2, 2);
    }
    ctx.fillStyle = "#ff9a32";
    ctx.beginPath();
    ctx.moveTo(w / 2, h / 2 - 7);
    ctx.lineTo(w / 2 - 5, h / 2 + 4);
    ctx.lineTo(w / 2 + 5, h / 2 + 4);
    ctx.fill();
    $("laser-status").textContent = scan.fresh
        ? `Live · ${scan.points.length} displayed points · ${(scan.age_s * 1000).toFixed(0)} ms old`
        : "Lidar scan unavailable or stale";
}
async function poll() {
    try {
        state = await api("state");
        laser(state.scan);
        const names = [...$("maps").options].slice(1).map((o) => o.value);
        $("fusion-status").textContent = state.odometry.imu_used
            ? "IMU gyro + wheel odometry active"
            : "Wheel odometry only — IMU is warming up, stale or outside its mounting checks";
        const bearing = state.goal_bearing_deg,
            w = state.command_velocity?.angular || 0;
        $("steering-status").textContent = state.running
            ? `Waypoint relative to robot: ${bearing === null ? "unknown" : Math.abs(bearing).toFixed(0) + "° " + (bearing > 5 ? "left" : bearing < -5 ? "right" : "ahead")} · Nav2 turn command: ${w > 0.015 ? "left" : w < -0.015 ? "right" : "straight"}`
            : "Robot is stopped. Waypoint headings follow the route; check the orange robot arrow before running.";
        if (JSON.stringify(names) !== JSON.stringify(state.maps)) {
            const initial = document.createElement("option");
            initial.value = "";
            initial.textContent = "Choose a saved map";
            $("maps").replaceChildren(initial);
            for (const name of state.maps) {
                const o = document.createElement("option");
                o.value = o.textContent = name;
                $("maps").append(o);
            }
            $("maps").value = selected;
        }
        if (performance.now() > statusUntil) {
            const missing = [];
            if (state.mode === "navigation" && !state.running) {
                if (!state.geometry.lidar_alignment_confirmed)
                    missing.push("verify lidar forward direction");
                if (!state.robot) missing.push("wait for robot localization");
                if (!state.localization_confirmed)
                    missing.push("confirm robot position on map");
                if (!state.collision_monitor_active)
                    missing.push("wait for collision monitor");
            }
            $("status").textContent =
                state.message +
                (missing.length ? " Next: " + missing.join("; ") + "." : "");
        }
        if (state.active_map_name && !selected) {
            $("maps").value = state.active_map_name;
            $("maps").onchange();
        }
        if (!alignmentLoaded) {
            $("lidar-yaw").value = String(
                Math.round(
                    ((state.geometry.lidar.yaw_rad || 0) * 180) / Math.PI,
                ),
            );
            $("alignment-confirm").checked =
                !!state.geometry.lidar_alignment_confirmed;
            alignmentLoaded = true;
        }
        $("run").disabled =
            !state.navigation_ready ||
            state.running ||
            selected !== state.active_map_name ||
            !points.length;
        const blockers = [];
        if (state.mode !== "navigation")
            blockers.push("load a map for navigation");
        else {
            if (selected !== state.active_map_name)
                blockers.push("select " + state.active_map_name);
            if (!state.robot || !state.initial_set)
                blockers.push(
                    "set the current robot position and wait for localization",
                );
            if (!state.localization_confirmed)
                blockers.push("press “Confirm robot & scan match map”");
            if (!state.geometry.lidar_alignment_confirmed)
                blockers.push("confirm lidar alignment");
            if (!state.collision_monitor_active)
                blockers.push("wait for the collision monitor");
            if (!state.scan.fresh) blockers.push("wait for fresh lidar scans");
            if (state.map_match < 0.6)
                blockers.push("correct robot position or update the map");
        }
        if (!points.length) blockers.push("add a waypoint");
        if (state.running)
            blockers.push("wait for the current route or press Stop robot");
        if (!state.navigation_ready && !blockers.length)
            blockers.push("wait for fresh odometry and Nav2 readiness");
        $("run-reason").textContent = blockers.length
            ? "Run route unavailable: " + blockers.join("; ") + "."
            : "Ready to run your selected waypoints.";
        $("confirm-pose").style.outline =
            state.mode === "navigation" && !state.localization_confirmed
                ? "2px solid #f99c48"
                : "";
        if (state.mode === "mapping" && state.live_map) {
            const oldScale = scale,
                oldOx = ox,
                oldOy = oy,
                had = !!grid;
            setGrid(state.live_map);
            if (had) {
                scale = oldScale;
                ox = oldOx;
                oy = oldOy;
            }
        }
        redraw();
    } catch (e) {
        $("status").textContent = e.message;
        laser({ points: [], fresh: false });
    }
    setTimeout(poll, 500);
}
function camera() {
    if (!document.hidden) $("camera").src = "/snapshot.jpg?t=" + Date.now();
    setTimeout(camera, 700);
}
window.addEventListener("resize", redraw);
redraw();
poll();
camera();

async function robotApi(action, data = {}) {
    const r = await fetch("/api/" + action, {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Operator": operator },
        body: JSON.stringify(data),
        signal: AbortSignal.timeout(3000),
    });
    const value = await r.json();
    if (!r.ok) throw Error(value.error);
    return value;
}
async function claim() {
    if (connected) return;
    const result = await robotApi("claim");
    operator = result.operator;
    connected = true;
}
async function command(action, data = {}) {
    try {
        await api(action, data);
        return true;
    } catch (e) {
        showStatus(e.message);
        return false;
    }
}
setInterval(async () => {
    if (!connected || document.hidden) return;
    try {
        await robotApi("heartbeat");
    } catch (e) {
        connected = false;
        operator = "";
        $("status").textContent = e.message;
    }
}, 500);
$("alignment").onclick = () =>
    command("alignment", {
        yaw_degrees: Number($("lidar-yaw").value),
        confirmed: $("alignment-confirm").checked,
    });
$("mapping").onclick = async () => {
    if (await command("mapping")) {
        selected = "";
        $("maps").value = "";
        points = [];
        grid = null;
        dirty = false;
        list();
    }
};
$("save-map").onclick = () =>
    command("save_map", { name: $("map-name").value });
$("load-nav").onclick = () => command("navigation", { name: selected });
$("end-session").onclick = () => command("stop_session");
$("pose-mode").onclick = () => {
    poseMode = !poseMode;
    $("pose-mode").textContent = poseMode
        ? "Click current robot track centre"
        : "Set robot position on map";
};
$("confirm-pose").onclick = () => command("confirm_pose");
$("run").onclick = async () => {
    try {
        await claim();
        await api("run", { map_name: selected, waypoints: points });
    } catch (e) {
        showStatus(e.message);
    }
};
async function release() {
    clearInterval(driveTimer);
    driveTimer = null;
    holding = null;
    if (connected) await command("mapping_drive", { direction: "stop" });
}
document.querySelectorAll("[data-drive]").forEach((button) => {
    button.onpointerdown = async (e) => {
        e.preventDefault();
        button.setPointerCapture(e.pointerId);
        holding = button.dataset.drive;
        try {
            await claim();
            if (!holding) return;
            await api("mapping_drive", { direction: holding });
            driveTimer = setInterval(() => {
                if (holding) command("mapping_drive", { direction: holding });
            }, 120);
        } catch (error) {
            $("status").textContent = error.message;
            holding = null;
        }
    };
    button.onpointerup = release;
    button.onpointercancel = release;
});
$("cancel").onclick = async () => {
    try {
        await robotApi("stop", { cause: "button" });
    } catch (e) {
        showStatus(e.message);
    } finally {
        clearInterval(driveTimer);
        holding = null;
        await command("cancel");
    }
};
document.addEventListener("visibilitychange", () => {
    if (document.hidden && connected) {
        release();
        robotApi("stop", { cause: "visibility" }).catch(() => {});
        connected = false;
    }
});
window.addEventListener("pagehide", () => {
    if (connected)
        fetch("/api/stop", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ cause: "page_closed" }),
            keepalive: true,
        }).catch(() => {});
});
