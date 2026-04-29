#!/usr/bin/env python3
"""
HTTPS + Socket.IO server (Python replacement for server_secure.cjs).

Dependencies:
    pip install flask python-socketio gevent gevent-websocket cryptography

NOTE: The /proxy/* endpoint from the original Node.js server has been removed.
If proxy support is needed in the future, implement a Flask /proxy/<path:url>
route using the `requests` library to forward the request and return the response.
"""

import os
import socket
import datetime
import ipaddress
import argparse
import mimetypes

from cryptography import x509
from cryptography.x509.oid import NameOID
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa

from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler
import socketio
from flask import Flask, send_from_directory, make_response

import ub_utils

# ── Custom MIME types (mirrors the original server_secure.cjs configuration) ──
_CUSTOM_MIME_TYPES = {
    '.czml':    'application/json',
    '.geojson': 'application/json',
    '.topojson': 'application/json',
    '.wasm':    'application/wasm',
    '.ktx2':    'image/ktx2',
    '.gltf':    'model/gltf+json',
    '.bgltf':   'model/gltf-binary',
    '.glb':     'model/gltf-binary',
    '.b3dm':    'application/octet-stream',
    '.pnts':    'application/octet-stream',
    '.i3dm':    'application/octet-stream',
    '.cmpt':    'application/octet-stream',
    '.geom':    'application/octet-stream',
    '.vctr':    'application/octet-stream',
    '.glsl':    'text/plain',
}
for _ext, _mime in _CUSTOM_MIME_TYPES.items():
    mimetypes.add_type(_mime, _ext)

# Extensions for which we check the first 3 bytes for a gzip magic header
GZIP_MAGIC   = b'\x1f\x8b\x08'
TILESET_EXTS = {'.b3dm', '.pnts', '.i3dm', '.cmpt', '.glb', '.geom', '.vctr'}

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SSL_CERT   = os.path.join(SCRIPT_DIR, 'ssl', 'ca.crt')
SSL_KEY    = os.path.join(SCRIPT_DIR, 'ssl', 'ca.key')
_SSL_IP_STAMP = os.path.join(SCRIPT_DIR, 'ssl', 'ca.ip')


def _lan_ip():
    """Return the machine's primary outbound LAN IPv4, or '127.0.0.1' on failure."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return '127.0.0.1'


def _ensure_cert():
    """Generate (or regenerate) the SSL cert when the machine's IP has changed."""
    ip = _lan_ip()

    # Read the IP the cert was last built for, if any.
    try:
        stamped_ip = open(_SSL_IP_STAMP).read().strip()
    except FileNotFoundError:
        stamped_ip = None

    if os.path.isfile(SSL_CERT) and os.path.isfile(SSL_KEY) and stamped_ip == ip:
        return ip  # cert is current — nothing to do

    print(f'Generating SSL cert for IP {ip} ...')
    os.makedirs(os.path.dirname(SSL_CERT), exist_ok=True)

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)

    name = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME,             'US'),
        x509.NameAttribute(NameOID.STATE_OR_PROVINCE_NAME,   'New York'),
        x509.NameAttribute(NameOID.LOCALITY_NAME,            'Buffalo'),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME,        'University at Buffalo'),
        x509.NameAttribute(NameOID.COMMON_NAME,              'localhost'),
    ])
    now = datetime.datetime.now(datetime.timezone.utc)
    cert = (
        x509.CertificateBuilder()
        .subject_name(name)
        .issuer_name(name)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(now)
        .not_valid_after(now + datetime.timedelta(days=365))
        .add_extension(
            x509.SubjectAlternativeName([
                x509.DNSName('localhost'),
                x509.IPAddress(ipaddress.IPv4Address('127.0.0.1')),
                x509.IPAddress(ipaddress.IPv4Address(ip)),
            ]),
            critical=False,
        )
        .sign(key, hashes.SHA256())
    )

    with open(SSL_KEY, 'wb') as f:
        f.write(key.private_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PrivateFormat.TraditionalOpenSSL,
            encryption_algorithm=serialization.NoEncryption(),
        ))
    with open(SSL_CERT, 'wb') as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))

    with open(_SSL_IP_STAMP, 'w') as f:
        f.write(ip)
    print(f'SSL cert generated (SANs: localhost, 127.0.0.1, {ip})')
    return ip


# ── Socket.IO server ──────────────────────────────────────────────────────────
sio = socketio.Server(cors_allowed_origins='*', async_mode='gevent')


@sio.event
def connect(sid, environ):
    print(f'socket connection established: {sid}')


@sio.event
def disconnect(sid):
    print(f'socket disconnected: {sid}')


@sio.on('*')
def catch_all(event, sid, data):
    """Re-broadcast every event to all connected clients (dumb relay)."""
    print(f'event={event}  data={data}')
    sio.emit(event, data)


# ── Flask app (static file server) ───────────────────────────────────────────
flask_app = Flask(__name__, static_folder=SCRIPT_DIR, static_url_path='')


@flask_app.after_request
def add_cors_headers(response):
    response.headers['Access-Control-Allow-Origin'] = '*'
    response.headers['Access-Control-Allow-Headers'] = (
        'Origin, X-Requested-With, Content-Type, Accept'
    )
    return response


@flask_app.route('/', defaults={'path': 'index.html'})
@flask_app.route('/<path:path>')
def serve_static(path):
    full_path = os.path.join(SCRIPT_DIR, path)
    if not os.path.isfile(full_path):
        return make_response('Not found', 404)

    response = make_response(send_from_directory(SCRIPT_DIR, path))

    # Mirror the original gzip-detection for known 3D tileset formats
    _, ext = os.path.splitext(path)
    if ext.lower() in TILESET_EXTS:
        try:
            with open(full_path, 'rb') as f:
                if f.read(3) == GZIP_MAGIC:
                    response.headers['Content-Encoding'] = 'gzip'
        except OSError:
            pass

    return response


wsgi_app = socketio.WSGIApp(sio, flask_app)


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == '__main__':
	ap = argparse.ArgumentParser(description='HTTPS + Socket.IO server.')
	ap.add_argument('--port',   type=int,          default=8080,
					help='Port to listen on (default: 8080).')
	ap.add_argument('--public', action='store_true',
					help='Listen on all interfaces (default: localhost only).')
	args = ap.parse_args()

	lan_ip = _ensure_cert()
	host   = '0.0.0.0' if args.public else 'localhost'
	scope  = 'publicly' if args.public else 'locally'

	port = ub_utils.findOpenPort(args.port, options=range(8080,8090))
	print(f'Dev server running {scope}.  Connect to https://localhost:{port}/')
	if args.public:
		print(f'Also reachable at https://{lan_ip}:{port}/')

	server = pywsgi.WSGIServer(
		(host, port), wsgi_app,
		handler_class=WebSocketHandler,
		keyfile=SSL_KEY,
		certfile=SSL_CERT,
	)
	server.serve_forever()
