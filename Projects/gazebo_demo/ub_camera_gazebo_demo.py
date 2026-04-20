#!/usr/bin/env python3

'''
Typical usage:

python3 Projects/gazebo_demo/ub_camera_gazebo_demo.py

A few useful variants:

python3 Projects/gazebo_demo/ub_camera_gazebo_demo.py --no-stream
python3 Projects/gazebo_demo/ub_camera_gazebo_demo.py --port 8001 --protocol websocket
python3 Projects/gazebo_demo/ub_camera_gazebo_demo.py --zoom 2.0
python3 Projects/gazebo_demo/ub_camera_gazebo_demo.py --transport-module gz.transport13 --msgs-module gz.msgs10
'''


import argparse
import os
import sys
import time


def _add_local_ub_code_to_path():
	"""Allow this demo to run directly against a local ub_code checkout."""
	home = os.path.expanduser('~')
	ub_code_root = os.path.join(home, 'Projects', 'ub_code')
	if ub_code_root not in sys.path:
		sys.path.insert(0, ub_code_root)


_add_local_ub_code_to_path()

import ub_camera


# DEFAULT_TOPIC = '/world/default/model/pantilt/link/tilt_link/sensor/camera/image'
DEFAULT_TOPIC = '/world/default/model/car1/link/camera_link/sensor/front_camera/image'

def parse_args():
	parser = argparse.ArgumentParser(
		description='Demo ub_camera.CameraGazebo against a Gazebo Transport camera topic.'
	)
	parser.add_argument('--topic', default=DEFAULT_TOPIC, help='Gazebo image topic to subscribe to')
	parser.add_argument('--port', type=int, default=8000, help='ub_camera stream port')
	parser.add_argument('--protocol', default='mjpeg', choices=('mjpeg', 'websocket', 'webrtc'),
						help='ub_camera streaming protocol')
	parser.add_argument('--no-stream', action='store_true',
						help='Subscribe to Gazebo but do not start the ub_camera stream server')
	parser.add_argument('--rows', type=int, default=480, help='Nominal image height for ub_camera metadata')
	parser.add_argument('--cols', type=int, default=640, help='Nominal image width for ub_camera metadata')
	parser.add_argument('--fps', type=int, default=30, help='Nominal FPS for ub_camera metadata')
	parser.add_argument('--zoom', type=float, default=1.0, help='Optional digital zoom level')
	parser.add_argument('--transport-module', default=None,
						help='Optional explicit Gazebo transport module, e.g. gz.transport13')
	parser.add_argument('--msgs-module', default=None,
						help='Optional explicit Gazebo msgs module, e.g. gz.msgs10')
	return parser.parse_args()


def main():
	args = parse_args()

	camera = ub_camera.CameraGazebo(
		topic=args.topic,
		paramDict={
			'res_rows': args.rows,
			'res_cols': args.cols,
			'fps_target': args.fps,
			'outputPort': args.port,
		},
		transport_module=args.transport_module,
		msgs_module=args.msgs_module,
	)

	if args.zoom > 1.0:
		camera.changeZoom(args.zoom)

	camera.start(
		startStream=not args.no_stream,
		port=args.port,
		protocol=args.protocol,
	)

	if not args.no_stream:
		print(f'Stream URL: {camera.streamURL}')
	print(f'Subscribed to: {args.topic}')
	print('Press Ctrl+C to stop.')

	try:
		while True:
			time.sleep(1.0)
	except KeyboardInterrupt:
		pass
	finally:
		camera.shutdown()


if __name__ == '__main__':
	main()
