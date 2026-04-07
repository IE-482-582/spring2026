# UB Racer Client — Message Reference

This document lists all messages the client layer has access to, split between
**host-facing** messages (Socket.IO, managed by `server.py`) and **internal**
messages (WebSocket at `wss://CLIENT_IP:PORT/ws`, shared by `index.html` and
`controller.py` / `racerlib`).

---

## 1. Host ↔ server.py  (Socket.IO)

These messages are handled entirely by `server.py`.  Students do not send or
receive them directly; they arrive/depart as internal `/ws` messages instead.

### 1a. server.py → host

| Event | Payload | When |
|-------|---------|------|
| `user_register` | `{username: str, ip: str}` → ack `{status: str, message: str}` | Once, on browser login |
| `user_request` | `{action: "join"\|"exit"\|"confirm", carPreference?: carID}` | Queue management |
| `telemetry` | `{carID, timestamp, steering, throttle, compass}` | Each frame from car, forwarded up |

### 1b. host → server.py

| Event | Payload | Notes |
|-------|---------|-------|
| `session_start` | `{carID, carIP, wsPort, mjpegURL, sessionToken, timeLimitSec, steeringLimits: {min,max}, throttleLimits: {min,max}, cameraIntrinsics, startTime, endTime}` | Triggers car WS connection |
| `session_end` | `{carID, reason: "timeout"\|"user_exit"\|"admin_boot"\|"car_disconnect"}` | Triggers car WS teardown |
| `confirm_required` | `{carID, carName, timeoutSec: 60}` | Must confirm within timeout or move to back |
| `system_status` | `{cars: [...], globalQueuePosition, yourStatus, yourCarID}` | ~1 Hz + on any change |
| `host_notice` | `{severity: 0–7, text: str, carID: str\|null}` | Broadcast message from host operator |

---

## 2. Internal Client  (WebSocket `/ws`)

Both `index.html` (browser) and `controller.py` / `racerlib` connect here.
Controllers connect with `?role=controller`; browsers connect without a role.

### 2a. browser / controller → server.py

| `type` | Payload fields | Sender | server.py action |
|--------|---------------|--------|-----------------|
| `login` | `{username: str}` | browser only | Triggers `user_register` to host (once) |
| `drive` | `{steering: float, throttle: float}` | browser or controller | Forwarded to car WebSocket with session token |
| `user_request` | `{action: "join"\|"exit"\|"confirm", carPreference?: carID}` | browser or controller | Forwarded to host via Socket.IO |
| `local_notice` | `{severity: 0–7, text: str}` | controller only | Broadcast to browsers only (not echoed to controllers) |
| `algo_params` | `{cropTop, cropBottom, color: {h,s,v}, hueTolerance: {min,max}, satTolerance: {min,max}, valTolerance: {min,max}, maxThrottle, steeringPerPixel}` | browser only | Forwarded to controllers only |
| `camera_url` | `{url: str}` | controller only | Forwarded to browsers only |
| `dev_session_start` | `{carID, mjpegURL, steeringLimits: {min,max}, throttleLimits: {min,max}, timeLimitSec, cameraIntrinsics}` | browser only | DEV_MODE only — server synthesises and broadcasts `session_start` |
| `dev_session_stop` | `{}` | browser only | DEV_MODE only — server broadcasts `session_end` |

### 2b. server.py → browser / controller

All of these are broadcast to **all** `/ws` clients (browsers + controllers)
unless noted.

| `type` | Payload fields | Source | Notes |
|--------|---------------|--------|-------|
| `session_start` | `{carID, carIP, wsPort, mjpegURL, sessionToken, timeLimitSec, steeringLimits, throttleLimits, cameraIntrinsics, startTime, endTime}` | host (forwarded) | |
| `session_end` | `{carID, reason}` | host (forwarded) | |
| `confirm_required` | `{carID, carName, timeoutSec}` | host (forwarded) | |
| `system_status` | `{cars, globalQueuePosition, yourStatus, yourCarID}` | host (forwarded) | |
| `host_notice` | `{severity, text, carID}` | host (forwarded) | |
| `telemetry` | `{carID, timestamp, steering, throttle, compass}` | car WS (forwarded) | |
| `local_notice` | `{severity, text}` | controller (forwarded) | **Browsers only** — not re-sent to controllers |
| `algo_params` | (pass-through) | browser (forwarded) | **Controllers only** — not re-sent to browsers |
| `camera_url` | `{url: str}` | controller (forwarded) | **Browsers only** — controller calls `conn.set_camera_url(url)` after its camera stream is ready; browser sets `<img src>` |

---

## 3. server.py ↔ Car  (plain WebSocket `ws://CAR_IP:WS_PORT`)

Handled entirely by `server.py`; students never touch this directly.

| Direction | Payload |
|-----------|---------|
| server.py → car | `{sessionToken: str, steering: float, throttle: float}` |
| car → server.py | `{carID, timestamp, steering, throttle, compass}` |

---

## 4. Dev-mode considerations

When running with `dev=True` in `racerlib` (and `--dev` flag to `server.py`),
no host connection is available.  Changes to the message layer:

### 4a. Resolved

- **`telemetry`** — not generated in dev mode; `on_telemetry` is never called.
- **`drive`** — `conn.drive(s, t)` sends a `local_notice` with the new
  **DEVDRIVE** severity (numeric value TBD, outside the current 0–7 range)
  and text `"drive: steering=X throttle=Y"` instead of forwarding to a car.
  Requires `index.html`/`index.js` to handle the new severity (deferred — see §4b).
- **`session_start` (student-triggered)** — student calls
  `conn.start_dev_session(camera_url, steering_limits, throttle_limits,
  time_limit_sec)` from `controller.py`; racerlib synthesises a `session_start`
  payload and calls `on_session_start` locally.

### 4b. Deferred — browser-side dev mode

1. **Browser form → dev session** — ✅ Implemented: `dev_session_start` /
   `dev_session_stop` messages; Dev card form in `index.html`; handlers in
   `server.py`; wiring in `index.js` (Step 6).

2. **Browser notification of student-triggered dev session** — still deferred.
   When the student calls `conn.start_dev_session(...)` from `controller.py`,
   the browser is not notified.  Design TBD.

3. **DEVDRIVE severity rendering** — ✅ Implemented: `.sev-8 { color: #a78bfa; }`
   in `index.html` CSS; `addNotice()` lookup updated in `index.js` (Step 6).
