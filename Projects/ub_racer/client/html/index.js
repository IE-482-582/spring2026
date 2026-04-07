/**
 * index.js — UB Racer client UI
 *
 * Connects to server.py via WebSocket at wss://THIS_HOST/ws.
 * Message protocol (JSON, "type" field):
 *
 *   Inbound (from server.py):
 *     system_status    — queue/car state, ~1 Hz
 *     session_start    — session details + car params
 *     session_end      — {carID, reason}
 *     confirm_required — {carID, carName, timeoutSec}
 *     host_notice      — {severity, text, carID}
 *     local_notice     — {severity, text}   (from controller.py)
 *     telemetry        — {carID, steering, throttle, compass, timestamp}
 *
 *   Outbound (to server.py):
 *     login            — {username}
 *     user_request     — {action: "join"|"exit"|"confirm", carPreference?}
 *     drive            — {steering, throttle}
 *     algo_params      — {cropTop, cropBottom, color, hueTolerance, satTolerance,
 *                          valTolerance, maxThrottle, steeringPerPixel}
 *     dev_session_start — {carID, mjpegURL, reverseAllowed,
 *                          timeLimitSec, cameraIntrinsics}
 *     dev_session_stop  — {}
 */

"use strict";

// ── State ────────────────────────────────────────────────────────────────────

let ws            = null;
let sessionActive = false;
let eStopActive   = true;   // true = driving disabled; false = driving enabled
let sessionData   = null;   // last session_start payload
let username      = "";
let lastStatus    = null;   // last system_status message
let devMode       = false;

// ── Countdown timers ─────────────────────────────────────────────────────────

let sessionCountdownTimer = null;
let confirmCountdownTimer = null;

// ── iro.js color picker ──────────────────────────────────────────────────────

let _pickerUpdating = false;   // guard against sync feedback loops

// ── Element shortcuts ────────────────────────────────────────────────────────

const $ = id => document.getElementById(id);

// ── localStorage persistence ──────────────────────────────────────────────────

const _LS = "ub_racer_";
function _lsSave(id)    { localStorage.setItem(_LS + id, $(id).value); }
function _lsRestore(id) { const v = localStorage.getItem(_LS + id); if (v !== null) $(id).value = v; }

// All fields whose values are persisted across page refreshes.
const _PERSIST_FIELDS = [
  // Dev session form
  "dev-car-id", "dev-camera-url",
  "dev-time-limit",
  "dev-intr-res", "dev-intr-fx", "dev-intr-fy", "dev-intr-cx", "dev-intr-cy", "dev-intr-dist",
  // Algo params
  "ap-crop-top", "ap-crop-bottom",
  "ap-hue", "ap-hue-min", "ap-hue-max",
  "ap-sat", "ap-sat-min", "ap-sat-max",
  "ap-val", "ap-val-min", "ap-val-max",
  "ap-max-throttle", "ap-steer-per-px", "ap-deadzone-px",
];

// ── Login view ────────────────────────────────────────────────────────────────

async function initLoginView() {
  // Pre-fill from server config if --username was supplied at CLI; read devMode
  try {
    const resp = await fetch("/config");
    if (resp.ok) {
      const cfg = await resp.json();
      if (cfg.username) $("username-input").value = cfg.username;
      devMode = !!cfg.devMode;
    }
  } catch (_) { /* server may not be ready yet; ignore */ }
}

$("login-btn").addEventListener("click", doLogin);
$("username-input").addEventListener("keydown", e => { if (e.key === "Enter") doLogin(); });

function doLogin() {
  const uname = $("username-input").value.trim();
  if (!uname) {
    $("login-error").textContent = "Please enter a username.";
    return;
  }
  $("login-error").textContent = "";
  username = uname;
  $("username-display").textContent = username;
  $("login-view").style.display     = "none";
  $("main-view").style.display      = "block";
  setupDevMode();
  initAlgoParams();
  connectWS();
}

// ── Dev mode setup ────────────────────────────────────────────────────────────

function setupDevMode() {
  if (!devMode) {
    $("dev-card").style.display = "none";
    return;
  }
  // Status bar: show badge, hide host-dot + status-text
  $("dev-badge").style.display   = "inline-block";
  $("host-dot").style.display    = "none";
  $("status-text").style.display = "none";
  // Disable queue controls (elements stay visible)
  $("car-pref").disabled  = true;
  $("join-btn").disabled  = true;
  $("leave-btn").disabled = true;
}

// ── WebSocket connection ──────────────────────────────────────────────────────

function connectWS() {
  const url = `wss://${location.host}/ws`;
  ws = new WebSocket(url);

  ws.onopen = () => {
    if (!devMode) setHostDot(true);
    send({ type: "login", username });
    showNotice(6, "Connected to server.");
  };

  ws.onmessage = evt => {
    try {
      dispatch(JSON.parse(evt.data));
    } catch (e) {
      console.error("Bad message:", e);
    }
  };

  ws.onclose = () => {
    if (!devMode) {
      setHostDot(false);
      $("status-text").textContent = "Disconnected. Reconnecting…";
    }
    setTimeout(connectWS, 3000);
  };

  ws.onerror = () => { /* onclose fires next */ };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
  }
}

// ── Message dispatch ──────────────────────────────────────────────────────────

function dispatch(msg) {
  switch (msg.type) {
    case "system_status":    onSystemStatus(msg);    break;
    case "session_start":    onSessionStart(msg);    break;
    case "session_end":      onSessionEnd(msg);      break;
    case "confirm_required": onConfirmRequired(msg); break;
    case "host_notice":      showNotice(msg.severity, msg.text, "host-notices-log");  break;
    case "local_notice":
      if (devMode && msg.severity === 8) onDevDrive(msg.text);
      else showNotice(msg.severity, msg.text, "local-notices-log");
      break;
    case "telemetry":        onTelemetry(msg);       break;
    case "camera_url":       onCameraUrl(msg);       break;
    default:
      console.log("Unknown message type:", msg.type);
  }
}

// ── System status ─────────────────────────────────────────────────────────────

function onSystemStatus(msg) {
  lastStatus = msg;
  setHostDot(true);
  updateStatusBar(msg);
  updateCarsTable(msg.cars);
  updateCarPreferenceSelect(msg.cars);
  updateQueueButtons(msg);
}

function setHostDot(connected) {
  if (devMode) return;
  $("host-dot").classList.toggle("connected", connected);
}

function updateStatusBar(msg) {
  const s = msg.yourStatus;
  let text = "Idle";
  if (s === "queued") {
    text = `In queue — position ${msg.globalQueuePosition}`;
  } else if (s === "confirm_pending") {
    text = "Waiting for your confirmation…";
  } else if (s === "driving") {
    text = `Driving ${msg.yourCarID}`;
  }
  $("status-text").textContent = text;
}

function updateCarsTable(cars) {
  const tbody = $("cars-tbody");
  tbody.innerHTML = "";
  for (const car of cars) {
    const tr   = document.createElement("tr");
    const wait = car.estWaitSec != null ? `~${Math.round(car.estWaitSec)}s` : "—";
    const pos  = car.yourPosition === 0 ? "Driving"
               : car.yourPosition > 0   ? String(car.yourPosition)
               : "—";
    tr.innerHTML = `
      <td>${escHtml(car.name)}</td>
      <td class="avail-${car.availability}">${car.availability}</td>
      <td class="status-${car.status}">${car.status}</td>
      <td>${car.queueLength}</td>
      <td>${wait}</td>
      <td class="${car.yourPosition >= 0 ? "my-pos" : ""}">${pos}</td>
    `;
    tbody.appendChild(tr);
  }
}

let _carsKnown = [];
function updateCarPreferenceSelect(cars) {
  const sel = $("car-pref");
  const ids = cars.map(c => c.carID).join(",");
  if (ids === _carsKnown) return;
  _carsKnown = ids;
  const prev = sel.value;
  sel.innerHTML = `<option value="">Any car</option>`;
  for (const car of cars) {
    const opt = document.createElement("option");
    opt.value       = car.carID;
    opt.textContent = car.name;
    sel.appendChild(opt);
  }
  sel.value = prev; // restore selection if still valid
}

function updateQueueButtons(msg) {
  if (devMode) return;   // queue buttons stay disabled in dev mode
  const inQueue = msg.yourStatus !== "idle";
  $("join-btn").disabled  = inQueue;
  $("leave-btn").disabled = !inQueue;
}

// ── Session ───────────────────────────────────────────────────────────────────

function onSessionStart(msg) {
  sessionActive = true;
  sessionData   = msg;
  setEStop(true);   // E-Stop active by default — student must click Enable to drive

  $("session-panel").style.display  = "block";
  $("session-car-name").textContent = `Session — ${msg.carID}`;

  $("session-meta").textContent = msg.reverseAllowed ? "Reverse: allowed" : "Reverse: not allowed";

  startSessionCountdown(msg.endTime);

  const timeStr = msg.timeLimitSec != null ? `${msg.timeLimitSec}s` : "∞";
  showNotice(6, `Session started on ${msg.carID} (${timeStr})`);

  // Dev card: lock form, flip buttons
  setDevFormDisabled(true);
  $("dev-start-btn").disabled = true;
  $("dev-stop-btn").disabled  = false;

  // Push current algo params to controller.py immediately so _params is in sync
  // with the browser without requiring a manual "Apply" click.
  sendAlgoParams();
}

function onSessionEnd(msg) {
  sessionActive = false;
  sessionData   = null;

  stopSessionCountdown();

  $("session-panel").style.display    = "none";
  $("camera-feed").src               = "";
  $("camera-feed").style.display     = "none";
  $("dev-drive-display").textContent  = "";
  $("dev-drive-display").style.display = "none";
  eStopActive = true;
  $("estop-btn").textContent = "▶ Enable";
  $("estop-btn").className   = "estop-stopped";
  $("countdown").textContent       = "--:--";
  $("countdown").classList.remove("urgent");

  const reasons = {
    timeout:        "Session timed out",
    user_exit:      "You ended the session",
    admin_boot:     "Admin ended the session",
    car_disconnect: "Car disconnected",
  };
  showNotice(5, reasons[msg.reason] ?? `Session ended (${msg.reason})`);

  // Dev card: unlock form, flip buttons
  setDevFormDisabled(false);
  $("dev-start-btn").disabled = false;
  $("dev-stop-btn").disabled  = true;
}

function onCameraUrl(msg) {
  const cam     = $("camera-feed");
  cam.src           = msg.url;
  cam.style.display = "block";
}

function setDevFormDisabled(disabled) {
  for (const id of ["dev-car-id", "dev-camera-url", "dev-reverse-allowed", "dev-time-limit",
                     "dev-intr-res", "dev-intr-fx", "dev-intr-fy",
                     "dev-intr-cx", "dev-intr-cy", "dev-intr-dist"]) {
    $(id).disabled = disabled;
  }
}

function toDateMs(val) {
  // Normalise endTime: host sends ISO strings; dev synthesis sends Unix seconds (float)
  if (val == null) return null;
  if (typeof val === "number") return val < 1e12 ? val * 1000 : val;
  return new Date(val).getTime();
}

function startSessionCountdown(endTime) {
  stopSessionCountdown();
  const endMs = toDateMs(endTime);
  const el    = $("countdown");
  if (!endMs) {
    el.textContent = "∞";
    return;
  }
  sessionCountdownTimer = setInterval(() => {
    const secLeft = Math.max(0, (endMs - Date.now()) / 1000);
    const m = Math.floor(secLeft / 60);
    const s = Math.floor(secLeft % 60);
    el.textContent = `${m}:${String(s).padStart(2, "0")}`;
    el.classList.toggle("urgent", secLeft <= 15);
    if (secLeft <= 0) stopSessionCountdown();
  }, 500);
}

function stopSessionCountdown() {
  if (sessionCountdownTimer) {
    clearInterval(sessionCountdownTimer);
    sessionCountdownTimer = null;
  }
}

// ── Confirm dialog ────────────────────────────────────────────────────────────

function onConfirmRequired(msg) {
  $("confirm-car-name").textContent  = `Car: ${escHtml(msg.carName)}`;
  $("confirm-overlay").style.display = "flex";

  let remaining = msg.timeoutSec ?? 60;
  $("confirm-timer").textContent = remaining;

  stopConfirmCountdown();
  confirmCountdownTimer = setInterval(() => {
    remaining--;
    $("confirm-timer").textContent = remaining;
    if (remaining <= 0) {
      stopConfirmCountdown();
      $("confirm-overlay").style.display = "none";
    }
  }, 1000);
}

function stopConfirmCountdown() {
  if (confirmCountdownTimer) {
    clearInterval(confirmCountdownTimer);
    confirmCountdownTimer = null;
  }
}

$("confirm-btn").addEventListener("click", () => {
  stopConfirmCountdown();
  $("confirm-overlay").style.display = "none";
  send({ type: "user_request", action: "confirm" });
});

// ── E-Stop ────────────────────────────────────────────────────────────────────

function setEStop(active) {
  eStopActive = active;
  const btn = $("estop-btn");
  if (active) {
    btn.textContent = "▶ Enable";
    btn.className   = "estop-stopped";
  } else {
    btn.textContent = "⏹ E-STOP";
    btn.className   = "estop-driving";
  }
  send({ type: "e_stop", isDriving: !active });
}

$("estop-btn").addEventListener("click", () => setEStop(!eStopActive));

// ── Telemetry display ─────────────────────────────────────────────────────────

function onTelemetry(msg) {
  $("tel-steering").textContent = msg.steering != null ? msg.steering.toFixed(1) : "—";
  $("tel-throttle").textContent = msg.throttle != null ? msg.throttle.toFixed(1) : "—";
  $("tel-compass").textContent  = msg.compass  != null ? msg.compass.toFixed(1)  : "—";
}

function onDevDrive(text) {
  const el = $("dev-drive-display");
  el.textContent   = text;
  el.style.display = "block";
}

// ── Queue controls ────────────────────────────────────────────────────────────

$("join-btn").addEventListener("click", () => {
  const pref = $("car-pref").value || null;
  send({ type: "user_request", action: "join", carPreference: pref });
});

$("leave-btn").addEventListener("click", () => {
  send({ type: "user_request", action: "exit" });
});

// ── Dev card ──────────────────────────────────────────────────────────────────

// ── Algo Params info modal ────────────────────────────────────────────────────

$("ap-info-btn").addEventListener("click", () => {
  $("ap-info-modal").style.display = "flex";
});
$("ap-info-close").addEventListener("click", () => {
  $("ap-info-modal").style.display = "none";
});
$("ap-info-modal").addEventListener("click", e => {
  if (e.target === $("ap-info-modal")) $("ap-info-modal").style.display = "none";
});

// ── Dev card ──────────────────────────────────────────────────────────────────

$("dev-start-btn").addEventListener("click", () => {
  const cameraURL = $("dev-camera-url").value.trim();
  if (!cameraURL) {
    showNotice(3, "Camera URL is required.", "local-notices-log");
    return;
  }

  // Build cameraIntrinsics from individual fields — null if any required field is empty.
  let intrinsics = null;
  const intrRes = $("dev-intr-res").value.trim();
  const intrFx  = $("dev-intr-fx").value.trim();
  const intrFy  = $("dev-intr-fy").value.trim();
  const intrCx  = $("dev-intr-cx").value.trim();
  const intrCy  = $("dev-intr-cy").value.trim();
  if (intrRes && intrFx && intrFy && intrCx && intrCy) {
    const distRaw = $("dev-intr-dist").value.trim();
    const dist    = distRaw
      ? distRaw.split(",").map(s => parseFloat(s.trim())).filter(n => !isNaN(n))
      : [];
    intrinsics = { [intrRes]: { fx: Number(intrFx), fy: Number(intrFy),
                                cx: Number(intrCx), cy: Number(intrCy), dist } };
  }

  const timeLimitRaw = $("dev-time-limit").value.trim();
  send({
    type:             "dev_session_start",
    carID:            $("dev-car-id").value.trim() || "dev",
    mjpegURL:         cameraURL,
    reverseAllowed:   $("dev-reverse-allowed").checked,
    timeLimitSec:     timeLimitRaw ? Number(timeLimitRaw) : null,
    cameraIntrinsics: intrinsics,
  });
});

$("dev-stop-btn").addEventListener("click", () => {
  send({ type: "dev_session_stop" });
});

// ── Algo Params ───────────────────────────────────────────────────────────────

function sendAlgoParams() {
  // Convert UI ranges to cv2 ranges and send to controller.py.
  // H: [0,360] → [0,179]  (÷2)    S,V: [0,100] → [0,255]  (×2.55)
  const cv2H = v => Math.round(v / 2);
  const cv2S = v => Math.round(v * 2.55);
  const cv2V = v => Math.round(v * 2.55);
  send({
    type:       "algo_params",
    cropTop:    Number($("ap-crop-top").value),
    cropBottom: Number($("ap-crop-bottom").value),
    color: {
      h: cv2H(Number($("ap-hue").value)),
      s: cv2S(Number($("ap-sat").value)),
      v: cv2V(Number($("ap-val").value)),
    },
    hueTolerance: {
      min: cv2H(Number($("ap-hue-min").value)),
      max: cv2H(Number($("ap-hue-max").value)),
    },
    satTolerance: {
      min: cv2S(Number($("ap-sat-min").value)),
      max: cv2S(Number($("ap-sat-max").value)),
    },
    valTolerance: {
      min: cv2V(Number($("ap-val-min").value)),
      max: cv2V(Number($("ap-val-max").value)),
    },
    maxThrottle:      Number($("ap-max-throttle").value),
    steeringPerPixel: Number($("ap-steer-per-px").value),
    deadZonePixels:   Number($("ap-deadzone-px").value),
  });
}

function initAlgoParams() {
  // Two separate iro.js instances so wheel and sliders can be laid out independently
  const wheelPicker = new iro.ColorPicker("#ap-color-picker-wheel", {
    width: 120,
    color: { h: 50, s: 100, v: 100 },
    layout: [{ component: iro.ui.Wheel }],
  });
  const sliderPicker = new iro.ColorPicker("#ap-color-picker-sliders", {
    width: 130,
    color: { h: 50, s: 100, v: 100 },
    layout: [
      { component: iro.ui.Slider, options: { sliderType: "hue" } },
      { component: iro.ui.Slider, options: { sliderType: "saturation" } },
      { component: iro.ui.Slider, options: { sliderType: "value" } },
    ],
  });

  // Shared updater: push h/s/v into both pickers and all number inputs
  function applyHsv(h, s, v, skipWheel, skipSliders) {
    _pickerUpdating = true;
    if (!skipWheel)   wheelPicker.color.set({ h, s, v });
    if (!skipSliders) sliderPicker.color.set({ h, s, v });
    $("ap-hue").value = Math.round(h);
    $("ap-sat").value = Math.round(s);
    $("ap-val").value = Math.round(v);
    _pickerUpdating = false;
    validateChannel("ap-hue", "ap-hue-min", "ap-hue-max");
    validateChannel("ap-sat", "ap-sat-min", "ap-sat-max");
    validateChannel("ap-val", "ap-val-min", "ap-val-max");
  }

  wheelPicker.on("color:change", c => {
    if (_pickerUpdating) return;
    applyHsv(c.hsv.h, c.hsv.s, c.hsv.v, true, false);
  });
  sliderPicker.on("color:change", c => {
    if (_pickerUpdating) return;
    applyHsv(c.hsv.h, c.hsv.s, c.hsv.v, false, true);
  });

  // Number inputs → both pickers
  function syncPickerFromInputs() {
    if (_pickerUpdating) return;
    applyHsv(Number($("ap-hue").value), Number($("ap-sat").value), Number($("ap-val").value), false, false);
  }
  ["ap-hue", "ap-sat", "ap-val"].forEach(id => $(id).addEventListener("input", syncPickerFromInputs));

  // EyeDropper — pick any pixel on screen (camera feed, etc.)
  if ("EyeDropper" in window) {
    const dropper = new EyeDropper();
    $("ap-eyedropper-btn").addEventListener("click", async () => {
      try {
        const { sRGBHex } = await dropper.open();
        const r = parseInt(sRGBHex.slice(1, 3), 16);
        const g = parseInt(sRGBHex.slice(3, 5), 16);
        const b = parseInt(sRGBHex.slice(5, 7), 16);
        const { h, s, v } = rgbToHsv(r, g, b);
        applyHsv(h, s, v, false, false);
      } catch (_) { /* user cancelled */ }
    });
  } else {
    $("ap-eyedropper-btn").disabled = true;
    $("ap-eyedropper-btn").title    = "EyeDropper not supported in this browser";
  }

  // Range validation: highlight min/max red if target value falls outside [min, max]
  function validateChannel(valId, minId, maxId) {
    const v   = Number($(valId).value);
    const min = Number($(minId).value);
    const max = Number($(maxId).value);
    $(minId).classList.toggle("ap-range-error", v < min);
    $(maxId).classList.toggle("ap-range-error", v > max);
  }
  function wireValidation(valId, minId, maxId) {
    const check = () => validateChannel(valId, minId, maxId);
    $(valId).addEventListener("input", check);
    $(minId).addEventListener("input", check);
    $(maxId).addEventListener("input", check);
  }
  wireValidation("ap-hue", "ap-hue-min", "ap-hue-max");
  wireValidation("ap-sat", "ap-sat-min", "ap-sat-max");
  wireValidation("ap-val", "ap-val-min", "ap-val-max");

  // Sync color pickers to current input values (may have been restored from localStorage)
  applyHsv(Number($("ap-hue").value), Number($("ap-sat").value), Number($("ap-val").value), false, false);

  // Apply button → convert UI ranges to cv2 ranges, send algo_params
  // H: [0,360] → [0,179]  (÷2)    S,V: [0,100] → [0,255]  (×2.55)
  $("ap-apply-btn").addEventListener("click", sendAlgoParams);
}

// ── Notices ───────────────────────────────────────────────────────────────────

function showNotice(severity, text, logId = "host-notices-log") {
  const log  = $(logId);
  const div  = document.createElement("div");
  const time = new Date().toLocaleTimeString();
  div.className   = `notice sev-${severity}`;
  div.textContent = `[${time}] ${text}`;
  log.prepend(div);
  // Keep at most 100 entries
  while (log.children.length > 100) log.removeChild(log.lastChild);
}

// ── Helpers ───────────────────────────────────────────────────────────────────

function rgbToHsv(r, g, b) {
  r /= 255; g /= 255; b /= 255;
  const max = Math.max(r, g, b), min = Math.min(r, g, b), d = max - min;
  let h;
  const s = max === 0 ? 0 : d / max;
  const v = max;
  if (d === 0) {
    h = 0;
  } else {
    switch (max) {
      case r: h = ((g - b) / d + (g < b ? 6 : 0)) / 6; break;
      case g: h = ((b - r) / d + 2) / 6;                break;
      case b: h = ((r - g) / d + 4) / 6;                break;
    }
  }
  return { h: Math.round(h * 360), s: Math.round(s * 100), v: Math.round(v * 100) };
}

function escHtml(str) {
  return String(str)
    .replace(/&/g, "&amp;")
    .replace(/</g, "&lt;")
    .replace(/>/g, "&gt;")
    .replace(/"/g, "&quot;");
}

// ── Boot ──────────────────────────────────────────────────────────────────────

_PERSIST_FIELDS.forEach(_lsRestore);
_PERSIST_FIELDS.forEach(id => $(id).addEventListener("input", () => _lsSave(id)));

initLoginView();
