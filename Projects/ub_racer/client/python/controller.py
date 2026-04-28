#!/usr/bin/env python3
"""
controller.py — UB Racer student controller template.

This is YOUR file.  Implement your AI/control logic in the sections below.
The racerlib backend handles all communication with server.py and the car.

─── Quick start ──────────────────────────────────────────────────────────────

Normal mode (requires a running host):
    python server.py --host https://HOST_IP:8086
    python controller.py --port CLIENT_PORT

Dev mode (no host required — use your own camera URL):
    python server.py --dev
    python controller.py --dev --port CLIENT_PORT

─── How it works ─────────────────────────────────────────────────────────────

1. conn.run() connects to server.py and blocks until stopped.
2. The system calls your callbacks as events arrive:
     on_session_start  → a car has been assigned to you
     on_session_end    → the session is over
     on_telemetry      → fresh car data arrived (~10 Hz); call conn.drive() here
     on_system_status  → queue / availability update (~1 Hz)
     on_confirm_required → you are next; confirm within timeoutSec or lose your spot
     on_estop            → toggle whether the controller is in state of emergency stop
3. Call conn.join() when you are ready to enter the queue.
4. Call conn.drive(<steering>, <throttle>) to drive the car.
5. Call conn.stop() when you are done.

Publishing notices to your client webpage:
conn.notice(<severity level>, "<some message>"
    Valid Severity Levels:
    ub_utils.SEVERITY_EMERGENCY       ub_utils.SEVERITY_ALERT       ub_utils.SEVERITY_CRITICAL
    ub_utils.SEVERITY_ERROR           ub_utils.SEVERITY_WARNING     ub_utils.SEVERITY_NOTICE
    ub_utils.SEVERITY_INFO            ub_utils.SEVERITY_DEBUG
Ex:  conn.notice(ub_utils.SEVERITY_INFO, "You are connected")
"""
import argparse
import os
import sys

from lib.racerlib import Racer

CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
UB_CODE_PATH = os.path.abspath(
    os.path.join(CURRENT_DIR, "..", "..", "..", "..", "ub_code")
)

if UB_CODE_PATH not in sys.path:
    sys.path.append(UB_CODE_PATH)

import ub_camera
import ub_utils
import cv2
import numpy as np
import time

# Check version and get update notification:
ub_camera.checkVersion()

# ── CLI args ──────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="UB Racer controller")
parser.add_argument("--dev",    action="store_true",
                                help="Dev mode — no host or car required")
parser.add_argument("--port",   default=8443,
                                help="Port used by client server")
parser.add_argument("--server", default=None,
                                help="Override server.py URL (auto-detected if omitted)")
args = parser.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
#  YOUR CODE — implement your control logic below
# ══════════════════════════════════════════════════════════════════════════════

# ── Algo params ───────────────────────────────────────────────────────────────
# The browser (index.html Algo Params panel) is the canonical source of truth.
# These values are automatically pushed to controller.py at the start of every
# session (dev or real), overwriting whatever is here.
#
# Edit these only as a fallback for headless/autonomous operation (no browser).
# For normal use, set your defaults in the browser — they persist via localStorage.
#
# All color values below are in cv2 ranges (pre-converted by the browser):
#   hue:        [0, 179]   (half of the UI's [0, 360])
#   saturation: [0, 255]   (scaled from the UI's [0, 100])
#   value:      [0, 255]   (scaled from the UI's [0, 100])

_params = {
    "cropTop":          0,
    "cropBottom":       0,
    "color":            {"h": 90, "s": 255, "v": 255},
    "hueTolerance":     {"min": 5,  "max": 5},
    "satTolerance":     {"min": 40, "max": 40},
    "valTolerance":     {"min": 40, "max": 40},
    "maxThrottle":      30,
    "steeringPerPixel": 0.5,
    "deadZonePixels":   10,
}

isDriving = False   # set by E-Stop button; True = driving enabled
cam = {}

STEERING_MIN = -100  # full left
STEERING_MAX =  100  # full right


def my_pipeline(frame):
    global isDriving, _params

    # Get frame dimensions
    h, w, d = frame.shape

    # Convert to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Build bounds around the TARGET color using tolerances
    target_h = _params["color"]["h"]
    target_s = _params["color"]["s"]
    target_v = _params["color"]["v"]

    lower_color = np.array([
        max(0, target_h - _params["hueTolerance"]["min"]),
        max(0, target_s - _params["satTolerance"]["min"]),
        max(0, target_v - _params["valTolerance"]["min"])
    ], dtype=np.uint8)

    upper_color = np.array([
        min(179, target_h + _params["hueTolerance"]["max"]),
        min(255, target_s + _params["satTolerance"]["max"]),
        min(255, target_v + _params["valTolerance"]["max"])
    ], dtype=np.uint8)

    # Binary mask for selected color
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Mask out top / bottom crop areas
    crop_top = min(_params["cropTop"], h)
    crop_bottom = min(_params["cropBottom"], h)

    mask[0:crop_top, 0:w] = 0
    if crop_bottom > 0:
        mask[h-crop_bottom:h, 0:w] = 0

    # Optional denoising
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # Visualization
    display = cv2.bitwise_and(frame, frame, mask=mask)
    display[0:crop_top, 0:w] = 100
    if crop_bottom > 0:
        display[h-crop_bottom:h, 0:w] = 100

    # Find centroid of detected region
    M = cv2.moments(mask)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        cv2.circle(display, (cx, cy), 20, (0, 0, 255), -1)

        # Draw center line
        center_x = w // 2
        cv2.line(display, (center_x, 0), (center_x, h), (255, 255, 255), 2)

        # Error in pixels
        error = cx - center_x

        if abs(error) <= _params["deadZonePixels"]:
            error = 0

        if isDriving:
            steering = error * _params["steeringPerPixel"]
            steering = max(STEERING_MIN, min(STEERING_MAX, steering))

            # Slow down while turning harder
            turn_scale = max(0.1, 1 - abs(steering) / 100.0)
            throttle = _params["maxThrottle"] * turn_scale

            conn.drive(float(steering), float(throttle))

        cv2.putText(display, f"cx={cx}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(display, f"error={error:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    else:
        # No object found: creep slowly left to search
        if isDriving:
            conn.drive(-10, 10)

        cv2.putText(display, "NO TARGET", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    return display


def on_session_start(data: dict) -> None:
    """Called once when a driving session begins."""
    print(f"[session] Started — car: {data.get('carID')}")
    conn.notice(ub_utils.SEVERITY_INFO, f"Session Started - Car: {data.get('carID')}")
    conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] Session Start Data: {data}")

    # ── YOUR CODE HERE ──────────────────────────────────────────────────── #
    global cam

    port = ub_utils.findOpenPort(8000, options=range(8000, 8011))

    device = data.get("mjpegURL")
    if isinstance(device, str) and device.isdigit():
        device = int(device)

    car_id = data.get("carID")
    cam[car_id] = ub_camera.CameraUSB(device=device)

    if data.get("cameraIntrinsics"):
        for res, params in data["cameraIntrinsics"].items():
            cam[car_id].setIntrinsics(res, **params)

    cam[car_id].frameProcessor = my_pipeline
    cam[car_id].start(startStream=True, port=port)

    conn.set_camera_url(cam[car_id].streamURL)
    conn.notice(ub_utils.SEVERITY_INFO,
                f"Your camera stream is available at {cam[car_id].streamURL}")


def on_session_end(data: dict) -> None:
    """Called when the session ends for any reason.

    data["reason"] is one of: "timeout", "user_exit", "admin_boot",
    "car_disconnect".

    To re-queue automatically after each session, call conn.join() here.
    """
    print(f"[session] Ended — reason: {data.get('reason')}")
    conn.notice(ub_utils.SEVERITY_INFO, f"Session Ended — reason: {data.get('reason')}")
    conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] Session End Data: {data}")

    # ── YOUR CODE HERE ──────────────────────────────────────────────────── #
    global cam

    car_id = data.get("carID")
    if car_id in cam:
        try:
            cam[car_id].stop()
        except Exception as e:
            conn.notice(ub_utils.SEVERITY_WARNING, f"Camera stop warning: {e}")
        finally:
            del cam[car_id]

    time.sleep(1)
    conn.drive(0, 0)

    # Uncomment to re-queue automatically after each session:
    # conn.join()


def on_telemetry(data: dict) -> None:
    """Called at ~10 Hz with the latest car data during a session.

    Call conn.drive(steering, throttle) here to move the car.
    Not called in dev mode (no car connected).

    data keys:
        carID, timestamp,
        steering (current, degrees),
        throttle (current, percent),
        compass  (heading in degrees, or None if unavailable)
    """
    # ── YOUR CODE HERE ──────────────────────────────────────────────────── #
    # Driving is already handled in my_pipeline(), so this callback is
    # mostly for safety and debugging.
    car_id = data.get("carID")
    steering = data.get("steering")
    throttle = data.get("throttle")
    compass = data.get("compass")

    conn.notice(
        ub_utils.SEVERITY_DEBUG,
        f"[telemetry] car={car_id}, steering={steering}, throttle={throttle}, compass={compass}"
    )

    if not isDriving:
        conn.drive(0, 0)


def on_system_status(data: dict) -> None:
    """Called ~1 Hz with queue and car availability info.

    Useful for monitoring your position before a session starts.

    data keys: cars, globalQueuePosition, yourStatus, yourCarID
    """
    # ── YOUR CODE HERE (optional) ────────────────────────────────────────── #
    your_status = data.get("yourStatus")
    queue_pos = data.get("globalQueuePosition")
    your_car = data.get("yourCarID")

    msg = f"Status: {your_status}"
    if queue_pos is not None:
        msg += f" | Queue position: {queue_pos}"
    if your_car is not None:
        msg += f" | Car: {your_car}"

    conn.notice(ub_utils.SEVERITY_INFO, msg)


def on_params(data: dict) -> None:
    """Called when the browser sends updated algorithm parameters.

    The browser Algo Params panel lets you tune these live without restarting
    controller.py.  Values arrive pre-converted to cv2 ranges (see _params
    above).
    """
    global _params
    _params = data

    conn.notice(ub_utils.SEVERITY_DEBUG, f"[DEBUG] params updated: {data}")


def on_estop(is_driving: bool) -> None:
    """Called when the browser E-Stop button is toggled.

    is_driving=False  — E-Stop activated; racerlib has already issued drive(0,0).
    is_driving=True   — driving re-enabled.
    """
    global isDriving
    isDriving = is_driving
    state = "ENABLED" if is_driving else "STOPPED"
    conn.notice(
        ub_utils.SEVERITY_WARNING if not is_driving else ub_utils.SEVERITY_INFO,
        f"E-Stop: Driving {state}"
    )


def on_confirm_required(data: dict) -> None:
    """Called when you have reached the front of the queue.

    You must confirm within data["timeoutSec"] seconds or you will be moved
    to the back of the queue.

    The default behaviour (auto-confirm) is active when you pass
    on_confirm_required=None to Racer().  Override it here if you need
    manual or conditional confirmation.
    """
    print(f"[queue] Confirm required for {data.get('carName')} — auto-confirming.")
    conn.confirm()


# ══════════════════════════════════════════════════════════════════════════════
#  SETUP — create the connection and start
# ══════════════════════════════════════════════════════════════════════════════

conn = Racer(
    on_session_start=on_session_start,
    on_session_end=on_session_end,
    on_telemetry=on_telemetry,
    on_system_status=on_system_status,
    on_confirm_required=on_confirm_required,
    on_params=on_params,
    on_estop=on_estop,
    dev=args.dev,
    port=args.port,
    server=args.server,
)

if __name__ == "__main__":
    try:
        conn.run()
    finally:
        for c in list(cam.values()):
            try:
                c.stop()
            except Exception:
                pass