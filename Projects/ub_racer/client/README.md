# UB Racer — Client

This directory contains everything that runs on the **student's machine**.

```
client/
├── html/                   Browser UI (index.html + index.js)
├── python/
│   ├── server.py           Client web server — serves the UI and talks to the host
│   ├── controller.py       ← YOUR FILE — implement your control logic here
│   ├── lib/
│   │   └── racerlib.py     Backend library — do not edit
│   ├── ssl/                Auto-generated SSL certificates (git-ignored)
│   └── requirements.txt
└── messages.md             Full message-type reference for server.py / racerlib
```

---

## Quick start

### Normal mode (host + car required)

```bash
# Terminal 1 — start the client web server
cd client/python
venv/bin/python server.py --host https://HOST_IP:8086

# Terminal 2 — start your controller
venv/bin/python controller.py
```

Open the URL printed by `server.py` in your browser, then use the UI to join
the queue.

### Dev mode (no host or car required)

Use this to develop and test your control logic offline.

```bash
# Terminal 1
cd client/python
venv/bin/python server.py --dev

# Terminal 2
venv/bin/python controller.py --dev
```

In dev mode `conn.drive()` commands are printed to the browser console instead
of being sent to a car.  Call `conn.start_dev_session(camera_url=...)` to
simulate a session with your own camera stream.

---

## Writing your controller (`controller.py`)

Implement the five callback functions, then let `conn.run()` drive everything.

```python
def on_session_start(data):
    # A car has been assigned.  Initialise per-session state here.
    pass

def on_session_end(data):
    # Session over (data["reason"]: timeout / user_exit / admin_boot / car_disconnect).
    # Call conn.join() here to re-queue automatically.
    pass

def on_telemetry(data):
    # Called ~10 Hz during a session.  Call conn.drive() here.
    # data keys: carID, timestamp, steering, throttle, compass
    conn.drive(0.0, 20.0)   # example: straight at 20 %

def on_system_status(data):
    # Called ~1 Hz.  Useful for monitoring queue position.
    pass

def on_confirm_required(data):
    # You are at the front of the queue — confirm within data["timeoutSec"].
    conn.confirm()
```

---

## `Racer` API (`lib/racerlib.py`)

### Constructor

```python
from lib.racerlib import Racer

conn = Racer(
    on_session_start=on_session_start,   # callable(data) or None
    on_session_end=on_session_end,       # callable(data) or None
    on_telemetry=on_telemetry,           # callable(data) or None
    on_system_status=on_system_status,   # callable(data) or None
    on_confirm_required=on_confirm_required,  # None → auto-confirm
    dev=False,       # True for dev mode
    server=None,     # server.py WebSocket URL; auto-detected if None
)
```

### Methods

| Method | Description |
|--------|-------------|
| `conn.run()` | **Blocking.** Start the background loop; return only on `stop()` or Ctrl-C. Use this at the end of a script. |
| `conn.start()` | **Non-blocking.** Start the background loop and return immediately. Use this in interactive Python or Jupyter. |
| `conn.stop()` | Gracefully leave any queue/session and shut down. |
| `conn.join(car_preference=None)` | Join the car queue. |
| `conn.leave()` | Leave the queue. |
| `conn.confirm()` | Confirm readiness when prompted. |
| `conn.user_request(action, payload=None)` | Generic queue management (`"join"`, `"exit"`, `"confirm"`). |
| `conn.drive(steering, throttle)` | Send a drive command. In dev mode, prints to the browser console. |
| `conn.notice(severity, text)` | Send a message to the browser notices panel (severity 0–7). |
| `conn.start_dev_session(camera_url, ...)` | **Dev mode only.** Synthesise a session with a custom camera URL. |

### `start_dev_session` signature

```python
conn.start_dev_session(
    camera_url,                      # MJPEG stream URL (required)
    steering_limits=(-90, 90),       # (min, max) degrees
    throttle_limits=(-100, 100),     # (min, max) percent
    time_limit_sec=None,             # None = no countdown
)
```

### Interactive / Jupyter usage

```python
conn = Racer(on_telemetry=my_func, dev=True)
conn.start()                # non-blocking
conn.start_dev_session("http://192.168.0.10:8001/video")
# ... run other cells, explore freely ...
conn.stop()
```

---

## `server.py` flags

| Flag | Description |
|------|-------------|
| `--host URL` | Host server URL, e.g. `https://10.0.0.1:8086`. Required unless `--dev`. |
| `--dev` | Dev mode — skips host connection entirely. Required unless `--host`. |
| `--username NAME` | Pre-fill the browser login field. |
| `--port N` | HTTPS port (default: 8443). |
