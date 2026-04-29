# Host Socket.IO Server — Client API Reference

_Describes how to connect to and interact with the Arbotix host Socket.IO server from user (client-side) code. Covers JavaScript (browser) and Python clients._

---

## Table of Contents

1. [Overview](#1-overview)
2. [Connecting](#2-connecting)
3. [Server → Client Events](#3-server--client-events)
   - [sysinfo](#31-sysinfo)
   - [sessionstart](#32-sessionstart)
   - [status](#33-status)
   - [notice](#34-notice)
4. [Client → Server Events](#4-client--server-events)
   - [userreq — join queue](#41-userreq--join-queue)
   - [userreq — exit queue](#42-userreq--exit-queue)
   - [command](#43-command)
   - [relax](#44-relax)
5. [Typical User Flow](#5-typical-user-flow)

---

## 1. Overview

```
[User browser / Python client]  <-- Socket.IO -->  [server_secure.cjs]  <-- Socket.IO -->  [bridge.py / Robot]
```

- The server runs over **HTTPS** with a self-signed certificate. Clients must accept or disable SSL verification.
- The default port is **8085**.
- A user is identified by their **IP address**. Multiple browser tabs from the same IP share one queue position per robot.
- Each robot has a **queue**. Users join the queue and are promoted to active control in order.

---

## 2. Connecting

Connect with query parameters `role=user` and an optional `name` (display name shown to the admin).

**JavaScript (browser)**
```js
const socket = io("https://10.83.11.58:8085", {
  query: { role: "user", name: "Alice" },
});

socket.on("connect",    () => console.log("Connected:", socket.id));
socket.on("disconnect", () => console.log("Disconnected"));
```

**Python**
```python
import socketio

sio = socketio.Client(ssl_verify=False)

@sio.event
def connect():
    print("Connected, sid:", sio.get_sid())

@sio.event
def disconnect():
    print("Disconnected")

sio.connect(
    "https://10.83.11.58:8085?role=user&name=Alice",
    transports=["websocket"]
)
```

> **Note:** `name` is optional. If omitted, the server will identify you by IP address only.

---

## 3. Server → Client Events

### 3.1 `sysinfo`

Sent by the server immediately on connect, and again after **any state change** (another user joins/leaves, a session ends, availability changes, etc.).

Use this event to keep your UI in sync. It is also your confirmation that a `userreq` join or exit was processed — check `yourPosition` to verify the result.

**Payload:** array of robot objects

```json
[
  {
    "id":           1,
    "name":         "Arm 1",
    "type":         "pan/tilt camera",
    "availability": "available",
    "queueLength":  2,
    "yourPosition": 1,
    "estWaitSec":   95
  },
  ...
]
```

| Field | Type | Description |
|---|---|---|
| `id` | integer | Robot ID |
| `name` | string | Human-readable robot name |
| `type` | string | Robot type description |
| `availability` | `"available"` \| `"admin_only"` \| `"off"` | Whether users can join |
| `queueLength` | integer | Total number of users in queue (not counting active user) |
| `yourPosition` | integer | `0` = you are active; `1`–`N` = your position in queue; `-1` = not in queue |
| `estWaitSec` | integer \| null | Estimated wait in seconds; `null` if `yourPosition` is `-1` or `0` |

**JavaScript**
```js
socket.on("sysinfo", (robots) => {
  for (const robot of robots) {
    console.log(`Robot ${robot.id} (${robot.name}): availability=${robot.availability}, your position=${robot.yourPosition}`);
  }
});
```

**Python**
```python
@sio.on("sysinfo")
def on_sysinfo(robots):
    for robot in robots:
        print(f"Robot {robot['id']} ({robot['name']}): "
              f"availability={robot['availability']}, your position={robot['yourPosition']}")
```

---

### 3.2 `sessionstart`

Sent when you reach the front of the queue and become the **active user** for a robot. Contains everything you need to begin a control session.

**Payload:** object

```json
{
  "robotID":   1,
  "cameraURL": "https://10.83.11.58:8001/stream.mjpg",
  "intrinsics": {
    "640x480": { "fx": 844.49, "fy": 848.10, "cx": 299.46, "cy": 236.49, "dist": [...] }
  },
  "specs": {
    "joints": {
      "arm_shoulder_pan_joint":  { "min_angle": 0,  "max_angle": 114, "max_speed": 300 },
      "arm_shoulder_lift_joint": { "min_angle": 45, "max_angle": 160, "max_speed": 400 }
    }
  },
  "joints": {
    "arm_shoulder_pan_joint":  { "angle_deg": 57.0,  "min_angle": 0,  "max_angle": 114, "OK": true, "torque": false },
    "arm_shoulder_lift_joint": { "angle_deg": 125.0, "min_angle": 45, "max_angle": 160, "OK": true, "torque": false }
  },
  "startTime": "2026-02-12T14:00:00.000Z",
  "endTime":   "2026-02-12T14:01:30.000Z"
}
```

| Field | Type | Description |
|---|---|---|
| `robotID` | integer | Which robot this session is for |
| `cameraURL` | string | MJPEG stream URL (see note below) |
| `intrinsics` | object | Camera intrinsic parameters keyed by resolution |
| `specs.joints` | object | Per-joint limits loaded from config |
| `joints` | object | Current joint state at session start |
| `startTime` | ISO timestamp | Session start time |
| `endTime` | ISO timestamp | Scheduled session end time |

> **Camera stream:** Load `cameraURL` in an `<img>` tag or read it frame-by-frame in Python. The server uses a self-signed TLS certificate — your client must accept it. Your session's `endTime` is recalculated from `startTime` if the admin changes the service time. Your client is responsible for computing the countdown from `startTime`/`endTime`.

**JavaScript**
```js
socket.on("sessionstart", (session) => {
  console.log(`Session started for robot ${session.robotID}`);
  console.log(`Ends at: ${session.endTime}`);
  document.getElementById("camera").src = session.cameraURL;

  const msRemaining = new Date(session.endTime) - new Date();
  console.log(`Time remaining: ${Math.round(msRemaining / 1000)}s`);
});
```

**Python**
```python
@sio.on("sessionstart")
def on_sessionstart(session):
    from datetime import datetime, timezone
    print(f"Session started for robot {session['robotID']}")
    end = datetime.fromisoformat(session['endTime'].replace('Z', '+00:00'))
    now = datetime.now(timezone.utc)
    print(f"Time remaining: {(end - now).seconds}s")
    print(f"Camera URL: {session['cameraURL']}")
    print(f"Current joints: {session['joints']}")
```

---

### 3.3 `status`

Sent continuously while you are the **active user**, carrying the robot's live joint states. Also sent to admins.

**Payload:** `[robotID, jointStateDict]`

```json
[
  1,
  {
    "arm_shoulder_pan_joint":  { "angle_deg": 62.3, "min_angle": 0,  "max_angle": 114, "OK": true, "torque": true },
    "arm_shoulder_lift_joint": { "angle_deg": 130.1, "min_angle": 45, "max_angle": 160, "OK": true, "torque": true }
  }
]
```

Per-joint fields:

| Field | Type | Description |
|---|---|---|
| `angle_deg` | float | Current joint angle in degrees |
| `min_angle` | float | Minimum allowed angle (degrees) |
| `max_angle` | float | Maximum allowed angle (degrees) |
| `OK` | boolean | `true` if the servo is responding normally |
| `torque` | boolean | `true` = rigid (torque on); `false` = relaxed |

**JavaScript**
```js
socket.on("status", ([robotID, joints]) => {
  for (const [joint, state] of Object.entries(joints)) {
    console.log(`${joint}: ${state.angle_deg.toFixed(1)}°  OK=${state.OK}`);
  }
});
```

**Python**
```python
@sio.on("status")
def on_status(msg):
    robot_id, joints = msg
    for joint, state in joints.items():
        print(f"{joint}: {state['angle_deg']:.1f}°  OK={state['OK']}")
```

---

### 3.4 `notice`

Sent by the server to notify you of events affecting your session or queue position.

**Payload:** object

```json
{ "type": "warning", "message": "Your session on Arm 1 has ended. You are first in queue.", "robotID": 1 }
```

| Field | Type | Description |
|---|---|---|
| `type` | `"info"` \| `"warning"` \| `"error"` | Severity |
| `message` | string | Human-readable message |
| `robotID` | integer \| null | Associated robot, if applicable |

Common notices sent to users:

| Situation | `type` |
|---|---|
| It is now your turn (session started) | `info` |
| Session auto-extended (queue was empty) | `info` |
| Session ended, returned to front of queue | `warning` |
| Hard-booted: removed from session and queue | `warning` |
| Robot disconnected, queue released | `warning` |
| Command rejected (invalid angle, etc.) | `error` |

**JavaScript**
```js
socket.on("notice", ({ type, message, robotID }) => {
  const prefix = robotID != null ? `[Robot ${robotID}] ` : "";
  console.warn(`[${type.toUpperCase()}] ${prefix}${message}`);
});
```

**Python**
```python
@sio.on("notice")
def on_notice(msg):
    prefix = f"[Robot {msg['robotID']}] " if msg.get('robotID') is not None else ""
    print(f"[{msg['type'].upper()}] {prefix}{msg['message']}")
```

---

## 4. Client → Server Events

### 4.1 `userreq` — Join Queue

Request to join a robot's queue.

**Emit:** `["join", robotID]`

```json
["join", 1]
```

The server validates:
- Robot exists and `availability == "available"`
- You are not already in this robot's queue or active on it
- You have not exceeded the maximum number of simultaneous queues

**Confirmation / rejection:** Listen to `sysinfo`. If the join succeeded, `yourPosition` for that robot will be `≥ 0`. If rejected, a `notice` with `type: "warning"` or `"error"` will arrive explaining why.

**JavaScript**
```js
socket.emit("userreq", ["join", 1]);
```

**Python**
```python
sio.emit("userreq", ["join", 1])
```

---

### 4.2 `userreq` — Exit Queue

Request to leave a robot's queue, or end your active session early.

**Emit:** `["exit", robotID]`

```json
["exit", 1]
```

If you were the active user, your session is ended and the next queued user is promoted. If you were waiting in the queue, you are simply removed.

**Confirmation:** Listen to `sysinfo`. If the exit succeeded, `yourPosition` for that robot will be `-1`.

**JavaScript**
```js
socket.emit("userreq", ["exit", 1]);
```

**Python**
```python
sio.emit("userreq", ["exit", 1])
```

---

### 4.3 `command`

Send a servo command to a robot. You must be the **active user** for that robot (or an admin).

**Emit:** `[robotID, [cmdDict]]`

`cmdDict` maps one or more joint names to target angles in degrees. You may include any subset of the robot's joints in a single command.

**Generic format:**
```json
[robotID, [{ "joint_name": angle_deg, ... }]]
```

**Example — Arm 1 (pan/tilt, 2 joints):**

| Joint | Min (°) | Max (°) |
|---|---|---|
| `arm_shoulder_pan_joint` | 0 | 114 |
| `arm_shoulder_lift_joint` | 45 | 160 |

```json
[1, [{ "arm_shoulder_pan_joint": 90, "arm_shoulder_lift_joint": 100 }]]
```

Move only one joint:
```json
[1, [{ "arm_shoulder_pan_joint": 45 }]]
```

The server validates angles against the joint limits defined in the robot's config. Out-of-range commands are rejected with a `notice` of `type: "error"`.

**JavaScript**
```js
// Move both joints
socket.emit("command", [1, [{ arm_shoulder_pan_joint: 90, arm_shoulder_lift_joint: 100 }]]);

// Move only pan
socket.emit("command", [1, [{ arm_shoulder_pan_joint: 45 }]]);
```

**Python**
```python
# Move both joints
sio.emit("command", [1, [{"arm_shoulder_pan_joint": 90, "arm_shoulder_lift_joint": 100}]])

# Move only pan
sio.emit("command", [1, [{"arm_shoulder_pan_joint": 45}]])
```

---

### 4.4 `relax`

Set a joint's torque state. You must be the **active user** for that robot (or an admin).

**Emit:** `[robotID, [{ "joint_name": relaxed }]]`

- `true` — relax the joint (torque off; joint moves freely)
- `false` — make the joint rigid (torque on; holds position)

**Generic format:**
```json
[robotID, [{ "joint_name": true_or_false }]]
```

**Example — Arm 1:**
```json
[1, [{ "arm_shoulder_pan_joint": true }]]
```

**JavaScript**
```js
socket.emit("relax", [1, [{ arm_shoulder_pan_joint: true  }]]);  // relax pan
socket.emit("relax", [1, [{ arm_shoulder_pan_joint: false }]]);  // make pan rigid
```

**Python**
```python
sio.emit("relax", [1, [{"arm_shoulder_pan_joint": True}]])   # relax pan
sio.emit("relax", [1, [{"arm_shoulder_pan_joint": False}]])  # make pan rigid
```

---

## 5. Typical User Flow

```
connect
  └─> receive sysinfo          (list of robots, their availability and queue lengths)
        │
        ├─ emit userreq ["join", robotID]
        │     └─> receive sysinfo          (yourPosition updated to queue position)
        │           └─> (wait in queue...)
        │
        └─> receive sessionstart           (your turn — you are now active)
              │
              ├─ receive status            (continuous joint state updates)
              │
              ├─ emit command              (move joints)
              ├─ emit relax               (toggle torque)
              │
              ├─ receive notice (warning)  (session ending soon or ended)
              │
              └─ receive sysinfo          (yourPosition returns to -1 when session ends)
```

**Possible session-ending events:**
- **Timeout:** session `endTime` reached and another user is waiting → soft boot (you return to front of queue)
- **Auto-extend:** session `endTime` reached but queue is empty → `endTime` extended, `notice` of type `"info"` sent
- **Admin boot:** admin ends your session early → `notice` of type `"warning"`
- **You exit:** you emit `userreq ["exit", robotID]`
- **Robot disconnects:** `notice` of type `"warning"` sent; queue released

---


