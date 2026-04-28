#!/usr/bin/env python3
"""
controller.py — UB Racer student controller template.

This is YOUR file.  Implement your AI/control logic in the sections below.
The racerlib backend handles all communication with server.py and the car.

─── Quick start ──────────────────────────────────────────────────────────────

Normal mode (requires a running host):
    python server.py --host https://HOST_IP:8086
    python controller.py

Dev mode (no host required — use your own camera URL):
    python server.py --dev
    python controller.py --dev

─── How it works ─────────────────────────────────────────────────────────────

1. conn.run() connects to server.py and blocks until stopped.
2. The system calls your callbacks as events arrive:
     on_session_start  → a car has been assigned to you
     on_session_end    → the session is over
     on_telemetry      → fresh car data arrived (~10 Hz); call conn.drive() here
     on_system_status  → queue / availability update (~1 Hz)
     on_confirm_required → you are next; confirm within timeoutSec or lose your spot
3. Call conn.join() when you are ready to enter the queue.
4. Call conn.stop() when you are done.
"""

import argparse
from time import time

import cv2
import ub_camera
import numpy as np

from lib.racerlib import Racer

# ── CLI args ──────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="UB Racer controller")
parser.add_argument("--dev",    action="store_true",
                                help="Dev mode — no host or car required")
parser.add_argument("--server", default=None,
                                help="Override server.py URL (auto-detected if omitted)")
args = parser.parse_args()

# ══════════════════════════════════════════════════════════════════════════════
#  YOUR CODE — implement your control logic below
# ══════════════════════════════════════════════════════════════════════════════


def on_session_start(data: dict) -> None:
    """Called once when a driving session begins.

    Use this to initialise per-session state (reset PID integrals, clear
    buffers, etc.).

    Useful keys in data:
        data["carID"]                         — which car you have
        data["timeLimitSec"]                  — seconds until the session ends
        data["mjpegURL"]                      — camera stream URL
        data["steeringLimits"]["min"|"max"]   — hardware steering limits (degrees)
        data["throttleLimits"]["min"|"max"]   — hardware throttle limits (percent)
    """
    print(f"[session] Started — car: {data.get('carID')}  "
          f"limit: {data.get('timeLimitSec')}s")

    # ── YOUR CODE HERE ──────────────────────────────────────────────────── #
    global cam 
    cam{data['carID']} = ub_camera.CameraUSB(device=data['mjpegURL'], apiPref=None)
    cam{data['carID']}.frameProcessor = my_pipeline
    cam{data['carID']}.start(startStream=True, port=8000)  # FIXME -- need to find an empty spot

    # FIXME -- Where to get actual URL?
    conn.set_camera_url('https://localhost:8000/stream.mjpg')  # Tells the browser where to dislay

# NOTE:  This is a "blocking" procedure.
def my_pipeline(frame):	
	
    h, w, d = frame.shape

	# Convert to HSV representation
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	
	#FIXME -- Not very efficient to update each time
	# Apply a binary "mask" for each cell (is it "yellow"?)
    lower_color = np.array([_params['hueTolerance']['min'],
                            _params['satTolerance']['min'],
                            _params["valTolerance"]['min']])
	upper_color = np.array([_params['hueTolerance']['max'],
                            _params['satTolerance']['max'],
                            _params["valTolerance"]['max']])

	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	
	# Mask out the top `search_top` rows of the image
	mask[0:_params['cropTop'], 0:w] = 0
	
	# FIXME -- This isn't a very efficient ordering.  Let's discuss.
	# Could work on subset of image
	
	# Just for visualizing what's happening:		
	frame = cv2.bitwise_and(frame, frame, mask=mask)		
	frame[h-_params['cropBottom']:h, 0:w] = 100  # (darker gray)
	
	# Find "moments" on the binary mask
	M = cv2.moments(mask)
	if M['m00'] > 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cv2.circle(frame, (cx, cy), 20, (0,0,255), -1)
	
	#FIXME -- Need to calculate error and issue a drive command
      
	return frame        # return edited frame -> it streams
	# return None       # return None -> frame is dropped (not streamed)
      
    
    # Convert to HSV representation

def on_session_end(data: dict) -> None:
    """Called when the session ends for any reason.

    data["reason"] is one of: "timeout", "user_exit", "admin_boot",
    "car_disconnect".

    To re-queue automatically after each session, call conn.join() here.
    """
    print(f"[session] Ended — reason: {data.get('reason')}")

    # ── YOUR CODE HERE ──────────────────────────────────────────────────── #
    global cam 
    cam[data['carID']].stop() 

    time.sleep(1)

    conn.drive(0,0)

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
    #
    # Example — drive straight at 20 % throttle:
    #   conn.drive(0.0, 20.0)
    #
    # Example — simple compass-based heading hold:
    #   if data.get("compass") is not None:
    #       error    = TARGET_HEADING - data["compass"]
    #       steering = max(-30, min(30, error * 0.5))
    #       conn.drive(steering, 25.0)
    pass


def on_system_status(data: dict) -> None:
    """Called ~1 Hz with queue and car availability info.

    Useful for monitoring your position before a session starts.

    data keys: cars, globalQueuePosition, yourStatus, yourCarID
    """
    # ── YOUR CODE HERE (optional) ────────────────────────────────────────── #
    pass


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
    dev=args.dev,
    server=args.server,
)

if __name__ == "__main__":
    # conn.run() blocks until conn.stop() is called or Ctrl-C.
    #
    # To join the queue:
    #   - Use the browser UI (click "Join Queue"), OR
    #   - Call conn.join() from inside on_session_end to re-queue automatically, OR
    #   - In interactive/Jupyter mode, call conn.start() then conn.join().
    #
    # In dev mode, call conn.start_dev_session() from on_session_start or
    # from a Jupyter cell after conn.start() to simulate a session.
    conn.run()
