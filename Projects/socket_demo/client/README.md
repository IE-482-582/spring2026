# Client-Side User Guide

This guide covers the two files you will customize to add computer vision features
to the robot control interface: **`html/scripts/cam_features.js`** (browser UI) and
**`scripts/client.py`** (Python processing).

---

## System Architecture

```
[Browser: index.html]
  │  socket_client (HTTPS/Socket.IO)
  ▼
[server_secure.py]   ←── serves index.html, relays cam_control / camStatus / notice
  │  socket_client (HTTPS/Socket.IO)
  ▼
[client.py]          ←── opens cameras, runs CV, drives the robot

[Browser: index.html]
  │  socket_host (HTTPS/Socket.IO)
  ▼
[Host server]        ←── manages robot queue, relays commands
  │
  ▼
[Robot bridge.py]    ←── drives servos
```

There are two socket connections in play:

| Connection | Event direction | What it carries |
|---|---|---|
| `socket_client` (browser ↔ server_secure.py ↔ client.py) | browser → python | `cam_control` — camera feature commands |
| `socket_client` (browser ↔ server_secure.py ↔ client.py) | python → browser | `camStatus` — available camera streams; `notice` — status messages |
| `socket_host`   (browser ↔ host server)                   | browser → host   | `userreq`, `command`, `relax` |
| `socket_host`   (browser ↔ host server)                   | host → browser   | `sysinfo`, `sessionstart`, `status`, `notice` |

---

## Files You Will Edit

| File | What to add |
|---|---|
| `html/scripts/cam_features.js` | UI panel HTML + JavaScript for each new feature |
| `scripts/client.py` | Python handler for each new feature |

Do not edit `html/scripts/client.js`, `html/scripts/cam_stream.js`, or `html/index.html`
unless you have a specific reason to. They handle the robot controls, queue, and
session management.

---

## How a Feature Works (End-to-End)

The ArUco feature is a good example to follow.

### 1. The browser sends a command

When the user clicks **set ArUco** in the browser, `arucoSet()` runs:

```js
// cam_features.js
function arucoSet()  {
    var data = {};
    data.camID     = document.getElementById('arucoCamID').value;
    data.framerate = document.getElementById('arucoFramerate').value;
    data.tagType   = document.getElementById('arucoTagType').value;
    data.action    = document.getElementById('arucoAction').value;
    data.trackID   = document.getElementById('arucoTrackID').value;

    socket_client.emit('cam_control', ['arucoStart', data]);
}
```

The payload is a two-element list: **`[category, data]`** where `category` is a
string like `'arucoStart'` and `data` is a plain object with the form values.

### 2. `client.py` receives and dispatches

`callback_cam_control` receives the payload and dispatches on `category`:

```python
# client.py
def callback_cam_control(self, msg):
    category = msg[0][0]   # e.g. 'arucoStart'
    data     = msg[0][1]   # the data dict from the browser

    if category == 'arucoStart':
        ...
    elif category == 'arucoStop':
        ...
    elif category == 'myFeatureStart':   # <-- you add this
        ...
```

### 3. The post-function receives results

When a CV task produces a result, a *post-function* you provide is called with
an `argsDict`. For example:

```python
def arucoMoveCamera(self, argsDict):
    camID     = argsDict['camID']
    idName    = argsDict['idName']
    idToTrack = argsDict['idToTrack']

    centers = self.camera[camID].aruco[idName].deque[0]['centers']
    # use centers to compute a servo command...
    sio_host.emit('command', [robotID, [cmd]])
```

### 4. Sending feedback to the browser

Call `self.pubNotice(text)` at any point to send a message to the browser's
Notices panel:

```python
self.pubNotice(f'ArUco monitoring started on camID {camID}')
```

---

## Adding a New Feature

### Step 1 — Add the UI panel in `html/scripts/cam_features.js`

Add three things:

**A.** A function that returns the HTML for your config panel:

```js
function myFeatureConfigDiv()  {
    var html = '<h4 class="subtitle">My Feature</h4>' +
               '<table>' +
               '  <tr><td>camID:</td><td><select id="myFeatureCamID"></select></td></tr>' +
               '  <tr><td>Frame Rate:</td><td><input type="number" id="myFeatureFramerate" value=5></td></tr>' +
               '  <tr><td colspan=2><center>' +
               '    <button onClick="myFeatureStart();">Start</button>' +
               '    <button onClick="myFeatureStop();">Stop</button>' +
               '  </center></td></tr>' +
               '</table>';
    return html;
}
```

> **Tip:** Use a unique ID prefix (e.g. `myFeature`) for every HTML element to
> avoid collisions with the existing features.

**B.** Start and stop functions that emit `cam_control`:

```js
function myFeatureStart()  {
    var data = {};
    data.camID     = document.getElementById('myFeatureCamID').value;
    data.framerate = document.getElementById('myFeatureFramerate').value;

    socket_client.emit('cam_control', ['myFeatureStart', data]);
}

function myFeatureStop()  {
    var data = {};
    data.camID = document.getElementById('myFeatureCamID').value;

    socket_client.emit('cam_control', ['myFeatureStop', data]);
}
```

**C.** Register the feature by adding a line to `loadFeatures()` at the bottom of `cam_features.js`:

```js
function loadFeatures()  {
    new CameraConfig(feature='aruco',      ...);
    new CameraConfig(feature='barcode',    ...);
    new CameraConfig(feature='facedetect', ...);
    new CameraConfig(feature='ultra',      ...);
    new CameraConfig(feature='myFeature', btnDiv='camBtnsDiv', btnText='My Feature',
                     cfgDiv='camConfigDiv', divFunc=myFeatureConfigDiv);  // <-- add this
    ...
}
```

The `camID` select list for your feature will be populated automatically with
all connected cameras when they appear.  To enable this, add a line to
`cameraAdd()` in `client.js`:

```js
function cameraAdd(camID, msg)  {
    ...
    addToSelectList('arucoCamID',      camID);
    addToSelectList('barcodeCamID',    camID);
    addToSelectList('facedetectCamID', camID);
    addToSelectList('ultraCamID',      camID);
    addToSelectList('myFeatureCamID',  camID);   // <-- add this
}
```

---

### Step 2 — Handle the command in `client.py`

Add two `elif` branches inside `callback_cam_control`:

```python
elif category == 'myFeatureStart':
    camID      = data['camID']
    fps_target = data['framerate']

    self.camera[camID].addMyFeature(   # replace with the actual ub_camera method
        fps_target   = fps_target,
        postFunction = self.myFeaturePostFunction,
        postFunctionArgs = {'camID': camID}
    )
    self.pubNotice(f'My feature started on {camID}')

elif category == 'myFeatureStop':
    camID = data['camID']
    self.camera[camID].myFeature['default'].stop()
    self.pubNotice(f'My feature stopped on {camID}')
```

Add the post-function method to the `Main` class:

```python
def myFeaturePostFunction(self, argsDict):
    camID  = argsDict['camID']
    idName = argsDict['idName']   # usually 'default'

    # Access the latest result from the deque:
    result = self.camera[camID].myFeature[idName].deque[0]

    # Do something with result...
    self.pubNotice(f'My feature result: {result}')

    # Optionally send a robot command:
    # cmd = {'arm_shoulder_pan_joint': new_angle}
    # sio_host.emit('command', [robotID, [cmd]])
```

---

## Key References

### Camera data access

After a CV task runs, results are stored in a deque (most recent first):

```python
self.camera[camID].aruco[idName].deque[0]        # latest ArUco result
self.camera[camID].barcode['default'].deque[0]   # latest barcode result
self.camera[camID].facedetect['default'].deque[0]
self.camera[camID].ultralytics[idName].deque[0]
```

### Current joint state

Updated every time the robot sends a `status` event:

```python
self.joint[robotID]['arm_shoulder_pan_joint']['angle_deg']
self.joint[robotID]['arm_shoulder_pan_joint']['min_angle']
self.joint[robotID]['arm_shoulder_pan_joint']['max_angle']
```

Available joints: `arm_shoulder_pan_joint`, `arm_shoulder_lift_joint`,
`arm_elbow_flex_joint`, `arm_wrist_flex_joint`, `gripper_joint`.

### Sending robot commands from Python

```python
cmd = {
    'arm_shoulder_pan_joint':  90.0,
    'arm_shoulder_lift_joint': 120.0,
}
sio_host.emit('command', [robotID, [cmd]])
```

You can include any subset of joints in a single command.

### Sending notices to the browser

```python
self.pubNotice('Something happened')   # appears in the browser Notices panel
```

### `cam_control` event format

```
Browser emits:   socket_client.emit('cam_control', [category, data])
Python receives: msg[0][0]  →  category  (string)
                 msg[0][1]  →  data      (dict)
```

---

## Running the Client

You will need two (2) terminal windows:

### Terminal 1 - Start the Server
**Make sure to activate your Python virtual environment first.**

```bash
cd client/html
python server_secure.py --public
```


### Terminal 2 - Run Python
**Make sure to activate your Python virtual environment first.**

```bash
cd client/scripts
python client.py --host-ip HOST_IP --host-port HOST_PORT --client-ip CLIENT_IP --client-port CLIENT_PORT
```

Then open a browser to `https://localhost:8080` (or the LAN IP shown at startup)
and enter the host server address in the login form.

> **Note:** `server_secure.py` must already be running before you open the browser.
