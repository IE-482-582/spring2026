#!/usr/bin/env python3
"""
server.py — UB Racer client web server.

Responsibilities:
  - Generates a self-signed SSL cert for this machine's LAN IP (Python, cross-platform).
  - Serves index.html / index.js over HTTPS on port 8443.
  - Exposes a /ws WebSocket endpoint used by index.html and controller.py.
  - Connects to the host via python-socketio.
  - Brokers drive commands and telemetry between /ws clients and the car WebSocket.

Usage:
    python server.py --host https://HOST_IP:8086
    python server.py --host https://HOST_IP:8086 --username alice
    python server.py --host https://HOST_IP:8086 --username alice --port 8443
"""

import asyncio
import argparse
import ipaddress
import json
import socket
from datetime import datetime, timedelta, timezone
from pathlib import Path

import socketio
import uvicorn
import websockets as ws_lib
from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.x509.oid import NameOID
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse

# ── CLI args ──────────────────────────────────────────────────────────────────

parser = argparse.ArgumentParser(description="UB Racer client server")
parser.add_argument("--host",     default=None, help="Host URL, e.g. https://10.0.0.1:8086")
parser.add_argument("--dev",      action="store_true",
                                  help="Dev mode — run without a host connection")
parser.add_argument("--username", default=None, help="Your username (or enter it in the browser login)")
parser.add_argument("--port",     type=int,
                    default=8443, help="HTTPS port (default: 8443)")
args = parser.parse_args()

if not args.host and not args.dev:
    parser.error("provide --host <URL> to connect to a host, or --dev to run without one.")

HOST_URL         = args.host
DEV_MODE         = args.dev
CLI_USERNAME     = args.username   # may be None; browser login sets it if omitted
CLIENT_PORT_PREF = args.port

# ── Paths ─────────────────────────────────────────────────────────────────────

BASE_DIR = Path(__file__).parent
HTML_DIR = BASE_DIR.parent / "html"
SSL_DIR  = BASE_DIR / "ssl"

# ── Utilities ─────────────────────────────────────────────────────────────────

def get_lan_ip() -> str:
    """Detect this machine's LAN IP without sending any network traffic.

    Uses a UDP socket: connect() on SOCK_DGRAM never sends a packet — it just
    lets the OS pick the right routing interface for the given destination.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]


def _port_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("", port))
            return True
        except OSError:
            return False


def find_open_port(preferred: int, options=None) -> int:
    """Return preferred if free, else the first free port in options."""
    if _port_free(preferred):
        return preferred
    for p in (options or range(preferred, preferred + 10)):
        if _port_free(p):
            return p
    raise RuntimeError(f"No open port found starting at {preferred}")

# ── SSL cert generation ───────────────────────────────────────────────────────

def _cert_covers_ip(cert_path: Path, ip_str: str) -> bool:
    """Return True if the cert's SubjectAltName includes ip_str."""
    try:
        cert   = x509.load_pem_x509_certificate(cert_path.read_bytes())
        san    = cert.extensions.get_extension_for_class(x509.SubjectAlternativeName)
        target = ipaddress.ip_address(ip_str)
        return target in san.value.get_values_for_type(x509.IPAddress)
    except Exception:
        return False


def ensure_cert(ssl_dir: Path, ip_str: str) -> None:
    """Generate a self-signed cert for ip_str if missing or the IP has changed."""
    key_path  = ssl_dir / "ca.key"
    cert_path = ssl_dir / "ca.crt"
    ssl_dir.mkdir(parents=True, exist_ok=True)

    if key_path.exists() and cert_path.exists() and _cert_covers_ip(cert_path, ip_str):
        return  # still valid for current IP

    print(f"[ssl] Generating self-signed cert for {ip_str} ...")
    key     = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, ip_str)])
    now     = datetime.now(timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + timedelta(days=825))
        .add_extension(
            x509.SubjectAlternativeName([x509.IPAddress(ipaddress.ip_address(ip_str))]),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )
    key_path.write_bytes(
        key.private_bytes(
            serialization.Encoding.PEM,
            serialization.PrivateFormat.TraditionalOpenSSL,
            serialization.NoEncryption(),
        )
    )
    cert_path.write_bytes(cert.public_bytes(serialization.Encoding.PEM))
    print(f"[ssl] Cert written to {ssl_dir}/")

# ── Shared state ──────────────────────────────────────────────────────────────

LAN_IP      = ""
CLIENT_PORT = CLIENT_PORT_PREF
USERNAME    = CLI_USERNAME    # set by CLI arg or browser login
_registered = False           # True once user_register has been sent to host


class _CarState:
    ws        = None   # active websockets.WebSocketClientProtocol | None
    token     = None   # current session token (str | None)
    recv_task = None   # asyncio.Task | None

car = _CarState()

_dev_car_id      = "dev"   # last carID seen in a dev_session_start
_current_session    = None  # cached session_start payload; replayed to reconnecting browsers
_current_camera_url = None  # cached camera_url payload;  replayed to reconnecting browsers

# ── WebSocket connection manager (/ws endpoint) ───────────────────────────────

class WSManager:
    """Tracks /ws clients and routes messages to the right subset."""

    def __init__(self):
        self._browsers:    set[WebSocket] = set()
        self._controllers: set[WebSocket] = set()

    async def connect(self, ws: WebSocket, role: str) -> None:
        await ws.accept()
        if role == "controller":
            self._controllers.add(ws)
        else:
            self._browsers.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self._browsers.discard(ws)
        self._controllers.discard(ws)

    async def _send(self, ws: WebSocket, raw: str) -> bool:
        try:
            await ws.send_text(raw)
            return True
        except Exception:
            return False

    async def broadcast_all(self, msg: dict) -> None:
        """Send to all /ws clients (browsers + controllers)."""
        raw  = json.dumps(msg)
        dead = [ws for ws in list(self._browsers | self._controllers)
                if not await self._send(ws, raw)]
        for ws in dead:
            self.disconnect(ws)

    async def broadcast_browsers(self, msg: dict) -> None:
        """Send only to browser clients."""
        raw  = json.dumps(msg)
        dead = [ws for ws in list(self._browsers)
                if not await self._send(ws, raw)]
        for ws in dead:
            self.disconnect(ws)

    async def broadcast_controllers(self, msg: dict) -> None:
        """Send only to controller clients."""
        raw  = json.dumps(msg)
        dead = [ws for ws in list(self._controllers)
                if not await self._send(ws, raw)]
        for ws in dead:
            self.disconnect(ws)


ws_manager = WSManager()

# ── FastAPI app ───────────────────────────────────────────────────────────────

app = FastAPI()


@app.get("/")
async def serve_index():
    return FileResponse(HTML_DIR / "index.html")


@app.get("/index.js")
async def serve_js():
    return FileResponse(HTML_DIR / "index.js")


@app.get("/scripts/{filename}")
async def serve_script(filename: str):
    return FileResponse(HTML_DIR / "scripts" / filename)


@app.get("/config")
async def serve_config():
    """Return server config so index.html can pre-fill the username."""
    return JSONResponse({"username": USERNAME, "lanIP": LAN_IP, "port": CLIENT_PORT, "devMode": DEV_MODE})


@app.websocket("/ws")
async def ws_endpoint(websocket: WebSocket, role: str = "browser"):
    """
    Local WebSocket endpoint.  Clients connect here with:
      - index.html:      wss://CLIENT_IP:PORT/ws
      - controller.py:   wss://CLIENT_IP:PORT/ws?role=controller

    Inbound message types from clients:
      login             — {username}  — browser login; triggers user_register if not yet done
      user_request      — {action, carPreference?}  — forwarded to host
      drive             — {steering, throttle}       — forwarded to car WS
      local_notice      — {severity, text}           — forwarded to browser clients only
      algo_params       — forwarded to controller clients only
      dev_session_start — (DEV_MODE only) synthesise + broadcast session_start
      dev_session_stop  — (DEV_MODE only) broadcast session_end
    """
    global USERNAME, _registered, _current_session, _current_camera_url

    await ws_manager.connect(websocket, role)
    try:
        while True:
            raw = await websocket.receive_text()
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue

            t = msg.get("type")

            if t == "login":
                new_username = (msg.get("username") or "").strip()
                if new_username and not _registered:
                    USERNAME = new_username
                    if not DEV_MODE:
                        asyncio.create_task(_register_with_host())
                if role != "controller" and _current_session is not None:
                    await websocket.send_text(json.dumps(_current_session))
                    if _current_camera_url is not None:
                        await websocket.send_text(json.dumps(_current_camera_url))

            elif t == "drive":
                if car.ws is not None and car.token is not None:
                    payload = json.dumps({
                        "sessionToken": car.token,
                        "steering":     float(msg.get("steering", 0.0)),
                        "throttle":     float(msg.get("throttle", 0.0)),
                    })
                    try:
                        await car.ws.send(payload)
                    except Exception as e:
                        print(f"[car_ws] send error: {e}")

            elif t == "user_request":
                if _registered:
                    try:
                        await sio.emit("user_request", {
                            "action":        msg.get("action"),
                            "carPreference": msg.get("carPreference"),
                        })
                    except Exception as e:
                        print(f"[sio] user_request error: {e}")

            elif t == "local_notice":
                # controller.py → server.py → browser clients only
                await ws_manager.broadcast_browsers({
                    "type":     "local_notice",
                    "severity": msg.get("severity", 6),
                    "text":     msg.get("text", ""),
                })

            elif t == "algo_params":
                await ws_manager.broadcast_controllers(msg)

            elif t == "e_stop":
                await ws_manager.broadcast_controllers(msg)

            elif t == "camera_url":
                _current_camera_url = msg
                await ws_manager.broadcast_browsers(msg)

            elif t == "dev_session_start":
                if DEV_MODE:
                    global _dev_car_id
                    _dev_car_id  = msg.get("carID", "dev")
                    now          = datetime.now(timezone.utc).timestamp()
                    time_limit   = msg.get("timeLimitSec")
                    session_msg  = {
                        "type":             "session_start",
                        "carID":            _dev_car_id,
                        "carIP":            "dev",
                        "wsPort":           0,
                        "mjpegURL":         msg.get("mjpegURL"),
                        "sessionToken":     "dev-token",
                        "reverseAllowed":   msg.get("reverseAllowed", False),
                        "timeLimitSec":     time_limit,
                        "cameraIntrinsics": msg.get("cameraIntrinsics"),
                        "startTime":        now,
                        "endTime":          now + time_limit if time_limit else None,
                    }
                    _current_session    = session_msg
                    _current_camera_url = None
                    await ws_manager.broadcast_all(session_msg)

            elif t == "dev_session_stop":
                if DEV_MODE:
                    _current_session    = None
                    _current_camera_url = None
                    await ws_manager.broadcast_all({
                        "type":   "session_end",
                        "carID":  _dev_car_id,
                        "reason": "user_exit",
                    })

    except WebSocketDisconnect:
        pass
    finally:
        ws_manager.disconnect(websocket)

# ── Car WebSocket management ──────────────────────────────────────────────────

async def _start_car_session(data: dict) -> None:
    uri   = f"ws://{data['carIP']}:{data['wsPort']}"
    token = data["sessionToken"]
    car.token = token
    try:
        car.ws = await ws_lib.connect(uri, additional_headers={"Authorization": token})
        car.recv_task = asyncio.create_task(_car_recv_loop())
        print(f"[car_ws] Connected to {uri}")
    except Exception as e:
        print(f"[car_ws] Failed to connect to {uri}: {e}")
        car.ws    = None
        car.token = None


async def _end_car_session() -> None:
    if car.recv_task:
        car.recv_task.cancel()
        car.recv_task = None
    if car.ws:
        try:
            await car.ws.close()
        except Exception:
            pass
        car.ws = None
    car.token = None
    print("[car_ws] Session ended, car connection closed")


async def _car_recv_loop() -> None:
    """Receive telemetry from the car and fan it out to /ws clients and host."""
    try:
        async for raw in car.ws:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                continue
            await ws_manager.broadcast_all({"type": "telemetry", **msg})
            try:
                await sio.emit("telemetry", msg)
            except Exception:
                pass
    except Exception as e:
        print(f"[car_ws] receive loop ended: {e}")

# ── Socket.IO client (host connection) ────────────────────────────────────────

sio = socketio.AsyncClient(ssl_verify=False, logger=False, engineio_logger=False)


async def _register_with_host() -> None:
    global _registered
    if _registered or not USERNAME:
        return
    try:
        resp = await sio.call(
            "user_register",
            {"username": USERNAME, "ip": LAN_IP},
            timeout=10,
        )
        status = resp.get("status") if isinstance(resp, dict) else "?"
        print(f"[sio] user_register → {status} (username={USERNAME})")
        _registered = True
    except Exception as e:
        print(f"[sio] user_register failed: {e}")


@sio.event
async def connect():
    print("[sio] Connected to host")
    if USERNAME:
        await _register_with_host()


@sio.event
async def disconnect():
    global _registered
    _registered = False
    print("[sio] Disconnected from host — retrying ...")


@sio.on("session_start")
async def on_session_start(data):
    global _current_session
    _current_session = {"type": "session_start", **data}
    await _start_car_session(data)
    await ws_manager.broadcast_all(_current_session)


@sio.on("session_end")
async def on_session_end(data):
    global _current_session, _current_camera_url
    _current_session    = None
    _current_camera_url = None
    await _end_car_session()
    await ws_manager.broadcast_all({"type": "session_end", **data})


@sio.on("confirm_required")
async def on_confirm_required(data):
    await ws_manager.broadcast_all({"type": "confirm_required", **data})


@sio.on("system_status")
async def on_system_status(data):
    await ws_manager.broadcast_all({"type": "system_status", **data})


@sio.on("host_notice")
async def on_host_notice(data):
    await ws_manager.broadcast_all({"type": "host_notice", **data})

# ── Socket.IO reconnect loop ──────────────────────────────────────────────────

async def _sio_connect_loop() -> None:
    while True:
        try:
            print(f"[sio] Connecting to {HOST_URL} ...")
            await sio.connect(HOST_URL, transports=["websocket"])
            await sio.wait()
        except Exception as e:
            print(f"[sio] Error: {e}. Retrying in 5s ...")
        if sio.connected:
            try:
                await sio.disconnect()
            except Exception:
                pass
        await asyncio.sleep(5)

# ── Uvicorn wrapper (disables signal handler to play nice with asyncio.gather) ─

class _UvicornServer(uvicorn.Server):
    def install_signal_handlers(self) -> None:
        pass  # let asyncio.run() handle SIGINT/SIGTERM

# ── Main ──────────────────────────────────────────────────────────────────────

async def main() -> None:
    global LAN_IP, CLIENT_PORT

    LAN_IP      = get_lan_ip()
    CLIENT_PORT = find_open_port(CLIENT_PORT_PREF, range(CLIENT_PORT_PREF, CLIENT_PORT_PREF + 10))

    ensure_cert(SSL_DIR, LAN_IP)

    config    = uvicorn.Config(
        app,
        host="0.0.0.0",
        port=CLIENT_PORT,
        ssl_keyfile=str(SSL_DIR / "ca.key"),
        ssl_certfile=str(SSL_DIR / "ca.crt"),
        log_level="warning",
    )
    uv_server = _UvicornServer(config)

    print(f"[server] Open in browser: https://{LAN_IP}:{CLIENT_PORT}")
    if USERNAME:
        print(f"[server] Username: {USERNAME}")
    else:
        print(f"[server] No username supplied — enter it in the browser login.")

    if DEV_MODE:
        print("[server] Dev mode — no host connection.")
        print(f"[server] Run controller with port {CLIENT_PORT}:  python controller.py --dev --port {CLIENT_PORT}")
        await uv_server.serve()
    else:
        print(f"[server] Connecting to host: {HOST_URL}")
        print(f"[server] Run controller with port {CLIENT_PORT}:  python controller.py --port {CLIENT_PORT}")
        await asyncio.gather(
            uv_server.serve(),
            _sio_connect_loop(),
        )


if __name__ == "__main__":
    import sys
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[server] Shutting down.")
        sys.exit(0)
