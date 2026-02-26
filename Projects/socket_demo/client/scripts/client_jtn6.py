#!/usr/bin/env python3

import argparse
import signal
import time

LOCAL_SSL_PATH  = None
FACE_MODEL_PATH = None

import ub_camera, ub_utils   # Visit https://github.com/optimatorlab/ub_code 
import cv2
import numpy as np


current, latest, is_up_to_date = ub_camera.checkVersion(verbose=False)
if not is_up_to_date:
    print(f"Please update ub_camera from {current} to {latest}")

STATUS_RATE = 1/5  # [Hz] — main loop rate

# ── Default configuration ────────────────────────────────────────────────────
# Edit these to avoid typing them on the command line every time.
# Any value can still be overridden at runtime with the corresponding flag.

HOST_IP     = None        # required — no universal default; set this or pass --host-ip
HOST_PORT   = None        # required — no universal default; set this or pass --host-port
CLIENT_IP   = 'localhost'
CLIENT_PORT = 8080
CAM_DEVICE  = None        # None = no local camera; or 0, '/dev/video0', 'https://...'
CAM_PORT    = 8000

# ─────────────────────────────────────────────────────────────────────────────

# ----------------------
import socketio
    
# We're going to allow for connections to multiple servers
sio_host   = socketio.Client(ssl_verify=False)  # , logger=True, engineio_logger=True)
sio_client = socketio.Client(ssl_verify=False)  # , logger=True, engineio_logger=True)

@sio_host.event
def connect():
    print('host socket connection established')

@sio_host.event
def disconnect():
	print('socket disconnected from host server')

@sio_client.event
def connect():
    print('client socket connection established')

@sio_client.event
def disconnect():
	print('socket disconnected from client server')
    
# ----------------------


class GracefulShutdown:
	# A class-based solution to exit gracefully on sigint/sigterm:
	# https://stackoverflow.com/questions/18499497/how-to-process-sigterm-signal-gracefully	
	def __init__(self):
		self.is_shutdown = False
		self.shutdownFunc = self._shutdownFunc

		signal.signal(signal.SIGINT,  self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)
	
	def _shutdownFunc(self):
		self.is_shutdown = True
		
	def exit_gracefully(self, signum, frame):
		self.is_shutdown = True
		self.shutdownFunc()
		
	def on_shutdown(self, func):
		self.shutdownFunc = func
			
	'''
	# In Main:
	monitor = GracefulShutdown()
	
	# Set shutdown function
	monitor.on_shutdown(self.shutdown)
	
	while not monitor.is_shutdown
	'''



class Main:
	def __init__(self, args):
		# Store CLI configuration
		self.host_ip     = args.host_ip
		self.host_port   = args.host_port
		self.client_ip   = args.client_ip
		self.client_port = args.client_port
		self.cam_device  = args.cam_device
		self.cam_port    = args.cam_port

		# Shutdown handler
		monitor = GracefulShutdown()
		monitor.on_shutdown(self.shutdown)
						
		# Subscribe to HOST socket topics:
		@sio_host.on('notice')
		def on_message(data):
			self.callback_notice(data)

		@sio_host.on('sysinfo')
		def on_message(data):
			self.callback_sysinfo(data)

		@sio_host.on('sessionstart')
		def on_message(data):
			self.callback_sessionstart(data)

		@sio_host.on('status')
		def on_message(data):
			self.callback_status(data)
			
		@sio_client.on('cam_control')
		def on_message(data):
			self.callback_cam_control(data)
			                			
		@sio_client.on('userreq')
		def on_message(data):
			# Relay join/exit requests from the browser to the host server.
			sio_host.emit('userreq', data)

		# Connect to the socket servers
		try:
			sio_host.connect(f'https://{self.host_ip}:{self.host_port}?role=user', transports=['websocket'])
			print('my host sid is', sio_host.get_sid())
		except Exception as e:
			print(f'Could not connect to host socket: {e}')

		try:
			sio_client.connect(f'https://{self.client_ip}:{self.client_port}', transports=['websocket'])
			print('my client sid is', sio_client.get_sid())
		except Exception as e:
			print(f'Could not connect to client socket: {e}')
    
    
		self.robot_id = None
		self.joint    = {}  # self.joint[robotID][jointName]['angle_deg']
		
		self.camera = {}
		
		if self.cam_device is not None:			
			self.startCamera(camID='local_cam', 
							 outputPort=self.cam_port,
							 apiPref=None, 
							 device=self.cam_device,
							 sslPath=LOCAL_SSL_PATH, 
							 intrinsics=None)
		
		
		sleepTime = 1/STATUS_RATE   # [seconds]

		while not monitor.is_shutdown:
			try:
				'''
				...
				Here's where you'd put some code to run forever inside a loop
				...
				self.doSomething()
				...
				'''
				self.pubCamStatus()
			except Exception as e:
				self.pubNotice(f'Error in while loop: {e}')
			
			# Keep the loop looping at a reasonable pace	
			time.sleep(sleepTime)
			
		# When infinite loop is done, call the `shutdown` function	
		self.shutdown()
		

	def callback_cam_control(self, msg):
		'''
		msg is a list like ['arucoStart', {'camID': 'robot_1', 'framerate': '5', 'tagType': 'DICT_APRILTAG_36h11', 'action': 'track', 'trackID': '0'}]
		
		data.camID = document.getElementById('arucoCamID').value;
		data.framerate = document.getElementById('arucoFramerate').value;
		data.tagType = document.getElementById('arucoTagType').value;
		data.action = document.getElementById('arucoAction').value;
		data.trackID = document.getElementById('arucoTrackID').value;
		'''
		
		try:
			category = msg[0]
			data = msg[1]
			
			print(f'FIXME data: {data=}')
			
			if (category == "arucoStart"):	
				camID = data['camID']
				idName = data['tagType']
				fps_target = data['framerate']
				
						
				if (data['action'] == 'track'):	
					postFunction = self.arucoMoveCamera
					ids_of_interest = [int(data['trackID'])]
					idToTrack = data['trackID']
					postFunctionArgs={'camID': camID, 'idName': idName, 'idToTrack': idToTrack}
				elif (data['action'] == 'id'):
					# Just show the IDs on the video feed
					postFunction = self.arucoShowIDs
					ids_of_interest = None  # None really means all
					postFunctionArgs={'camID': camID, 'idName': idName}
					
					print(f'FIXME: {camID=}, {idName=}')
				else:
					self.pubNotice(f"Unknown aruco action: {data['action']}")
					return
						
				self.camera[camID].addAruco(idName=idName, 
								fps_target=fps_target, 
								postFunction=postFunction, 
								postFunctionArgs=postFunctionArgs, 
								configOverrides={}, 
								ids_of_interest=ids_of_interest)  # default is None, or provide a list of IDs to track
								
				self.pubNotice(f'ArUco monitoring started on camID {camID} for {idName}')
								
			elif (category == "arucoStop"):	
				camID = data['camID']
				idName = data['tagType']

				# Stop running the aruco tracking
				self.camera[camID].aruco[idName].stop()

				self.pubNotice(f'ArUco monitoring stopped on camID {camID} for {idName}')
				
			elif (category == "barcodeStart"):
				camID = data['camID']
				fps_target = data['framerate']
				action = data['action']

				if (action == 'read'):
					postFunction = self.barcodePostFunction
					postFunctionArgs={'camID': camID}
				else:
					self.pubNotice(f'Unknown barcode action, "{action}".')
					return
					
				self.camera[camID].addBarcode(fps_target=fps_target,
											  postFunction=postFunction, 
											  postFunctionArgs=postFunctionArgs)

				self.pubNotice(f'Barcode reading started on camID {camID}')
				
			elif (category == "barcodeStop"):
				# Stop running the barcode reader.
				# 'default' is the only type of barcode option.
				camID = data['camID']
				self.camera[camID].barcode['default'].stop()
				self.pubNotice(f'Barcode reading stopped on camID {camID}')
					
			elif (category == "facedetectStart"):
				camID = data['camID']
				fps_target = data['framerate']
				action = data['action']
				conf_threshold = float(data['conf_threshold'])
				dnn = data['dnn']
				device = data['device']

				if (action == 'read'):
					postFunction = self.facedetectPostFunction
					postFunctionArgs={'camID': camID}
				else:
					self.pubNotice(f'Unknown facedetect action, "{action}".')
					return
					
				self.camera[camID].addFaceDetect(fps_target=fps_target,
												 postFunction=postFunction, 
												 postFunctionArgs=postFunctionArgs,
												 conf_threshold=conf_threshold, 
												 dnn=dnn,         # 'caffe' (fp16) or 'pb' (8bit)
												 device=device,   # 'gpu' can be configured
												 modelPath=FACE_MODEL_PATH)

				self.pubNotice(f'Face detection started on camID {camID}')
				
			elif (category == "facedetectStop"):
				# Stop running the face detection.
				# 'default' is the only type of facedetect option.
				camID = data['camID']
				self.camera[camID].facedetect['default'].stop()
				self.pubNotice(f'Face detection stopped on camID {camID}')

			elif (category == "ultraStart"):
				camID = data['camID']
				model_name = data['modelName']
				fps_target = data['framerate']
				action = data['action']
				conf_threshold = float(data['conf_threshold'])
				track = bool(data['track'])
				drawBox = bool(data['drawBox'])
				drawLabel = bool(data['drawLabel'])
				maskOutline = bool(data['maskOutline'])
								
				if (model_name == 'yolo11n.pt'):
					idName = 'detect'
					postFunction = self.ultraPostDetect
				elif (model_name == 'yolo11n-pose.pt'):
					idName = 'pose'
					postFunction = self.ultraPostPose
				elif (model_name == 'yolo11n-seg.pt'):
					idName = 'segment'
					postFunction = self.ultraPostSegment
				elif (model_name == 'yolo11n-obb.pt'):
					idName = 'obb'
					postFunction = self.ultraPostObb
					if (track):
						self.pubNotice(f'Sorry, cannot use tracking on OBB model.')
						return
				else:
					self.pubNotice(f'Unknown ultralytics model, `{model_name}`.')
					return
						
				if (track):
					# Tracking can be done with detect, pose, or segment models.
					idName = 'track'
					postFunction = self.ultraPostTrack
				
				'''		
				if (action == 'read'):
					postFunction = self.ultraPostFunction
					postFunctionArgs={'camID': camID}
				else:
					self.pubNotice(f'Unknown ultralytics action, "{action}".')
					return
				'''	
					
				self.camera[camID].addUltralytics(idName=idName,   # "detect", "pose", "obb", "segment", or "track" 
												  model_name=model_name, 
												  conf_threshold=conf_threshold,  
												  postFunction=postFunction, 
												  drawBox = drawBox, drawLabel=drawLabel, 
												  maskOutline = maskOutline)	

				self.pubNotice(f'Ultralytics {idName} started on camID {camID}')
				
			elif (category == "ultraStop"):
				# Stop running the Ultralytics model.
				camID  = data['camID']
				idName = data['idName']

				if ((camID in self.camera) and (idName in self.camera[camID].ultralytics)):
					self.camera[camID].ultralytics[idName].stop()
					self.pubNotice(f'Ultralytics {idName} stopped on camID {camID}')
				else:
					self.pubNotice(f'Missing ultraStop keys: {camID} and/or {idName}')

			elif (category == "localCameraStart"):
				camID = data['camID']
				outputPort = int(data['outputPort'])
				device = data['device']
				resolution = data['resolution']  # e.g., "640x480"

				# Parse resolution
				try:
					res_parts = resolution.split('x')
					res_cols = int(res_parts[0])
					res_rows = int(res_parts[1])
				except:
					self.pubNotice(f'Invalid resolution format: {resolution}. Using default 640x480.')
					res_cols = 640
					res_rows = 480

				# Convert device to int if it's a numeric string
				try:
					device = int(device)
				except ValueError:
					pass  # Keep as string (e.g., '/dev/video0' or URL)

				# Stop existing camera with same ID if it exists
				if camID in self.camera:
					self.pubNotice(f'Stopping existing camera {camID} before restarting')
					self.stopCamera(camID)

				# Start the camera with custom resolution
				self.startCamera(camID=camID,
								 outputPort=outputPort,
								 apiPref=None,
								 device=device,
								 sslPath=LOCAL_SSL_PATH,
								 intrinsics=None,
								 res_cols=res_cols,
								 res_rows=res_rows)

				self.pubNotice(f'Local camera {camID} started on port {outputPort}')

			elif (category == "localCameraStop"):
				camID = data['camID']

				if camID in self.camera:
					self.stopCamera(camID)
					self.pubNotice(f'Local camera {camID} stopped')
				else:
					self.pubNotice(f'Camera {camID} not found')

			elif (category == "localCameraRestart"):
				camID = data['camID']

				# Verify camera exists before attempting restart
				if camID not in self.camera:
					self.pubNotice(f'Cannot restart camera {camID} - camera does not exist. Please start it first.')
				else:
					# Camera exists - stop it and restart with new parameters
					outputPort = int(data['outputPort'])
					device = data['device']
					resolution = data['resolution']  # e.g., "640x480"

					# Parse resolution
					try:
						res_parts = resolution.split('x')
						res_cols = int(res_parts[0])
						res_rows = int(res_parts[1])
					except:
						self.pubNotice(f'Invalid resolution format: {resolution}. Using default 640x480.')
						res_cols = 640
						res_rows = 480

					# Convert device to int if it's a numeric string
					try:
						device = int(device)
					except ValueError:
						pass  # Keep as string (e.g., '/dev/video0' or URL)

					# Stop the existing camera
					self.camera[camID].stop()

					# Restart with new parameters
					self.startCamera(camID=camID,
									 outputPort=outputPort,
									 apiPref=None,
									 device=device,
									 sslPath=LOCAL_SSL_PATH,
									 intrinsics=None,
									 res_cols=res_cols,
									 res_rows=res_rows)

					self.pubNotice(f'Local camera {camID} restarted on port {outputPort}')

			else:
				self.pubNotice(f'Unknown category: {category}')
				
					
		except Exception as e:
			self.pubNotice(f'Error in callback_cam_control: {e}')
				

	def arucoShowIDs(self, argsDict):
		# This function gets called each time an aruco detection is run
		camID  = argsDict['camID']
		idName = argsDict['idName']

		# There's really nothing to do here.
		# print(camID, idName)
		print(self.camera[camID].aruco[idName].deque[0]['corners'])
		
	def arucoMoveCamera(self, argsDict):
		# This function gets called each time an aruco detection is run
		camID     = argsDict['camID']
		idName    = argsDict['idName']
		idToTrack = argsDict['idToTrack']

		robotID = self.robot_id
		if robotID is None:
			return
					
		centers = self.camera[camID].aruco[idName].deque[0]['centers']
		for i in range(len(centers)):
			# print(self.camera[camID].aruco[idName].deque[0]['ids'][i], centers[i])

			# Only track the ID specified by the user
			if (self.camera[camID].aruco[idName].deque[0]['ids'][i] == idToTrack):
				# data['joints'] = {'arm_shoulder_pan_joint': {'id': 1, 'neutral': 0, 'max_angle': 114, 'min_angle': 0, 'max_speed': 300, 'angle_deg': 88.18359375, 'OK': True, 'torque': False, 'angle_rad': 1.5390940571785934}, 'arm_shoulder_lift_joint': {'id': 2, 'neutral': 0, 'max_angle': 160, 'min_angle': 45, 'max_speed': 400, 'angle_deg': 146.48437500000003, 'OK': True, 'torque': False, 'angle_rad': 2.556634646476069}}
				cmd = {}

				error_x = self.camera[camID].res_cols/2 - centers[i][0]  # > 0 --> we want to rotate left 
				if (abs(error_x) > 5):
					# Servo moves LEFT as angle INCREASES
					cmd['arm_shoulder_pan_joint'] = self.joint[robotID]['arm_shoulder_pan_joint']['angle_deg'] + error_x/25
					cmd['arm_shoulder_pan_joint'] = min(max(cmd['arm_shoulder_pan_joint'], self.joint[robotID]['arm_shoulder_pan_joint']['min_angle']), self.joint[robotID]['arm_shoulder_pan_joint']['max_angle'])
				
				error_y = centers[i][1] - self.camera[camID].res_rows/2  # > 0 --> we want to move down
				if (abs(error_y) > 5):
					# Servo moves DOWN as angle INCREASES
					cmd['arm_shoulder_lift_joint'] = self.joint[robotID]['arm_shoulder_lift_joint']['angle_deg'] + error_y/30
				
				if (len(cmd) > 0):
					# print(cmd)
					sio_host.emit('command', [robotID, [cmd]])


	def barcodePostFunction(self, argsDict):
		# This function gets called each time a barcode detection is run
		camID  = argsDict['camID']
		idName = argsDict['idName']   # Will always be 'default'

		# print(self.camera[camID].barcode['default'].deque[0])
		bc = self.camera[camID].barcode['default'].deque[0]   # alias the long name
		for i in range(len(bc['data'])): 
			myString = f"data: {bc['data'][i]}, codeType: {bc['codeTypes'][i]}, quality: {bc['qualities'][i]}, corners: {bc['corners'][i]}"
			self.pubNotice(myString)
			'''
			Will look like
			data: 0044000051693, codeType: EAN13, quality: 83, corners: [(268, 81), (492, 213)]
			'''
			
	def facedetectPostFunction(self, argsDict):
		# This function gets called each time a face detection is run
		camID  = argsDict['camID']
		idName = argsDict['idName']   # Will always be 'default'

		# print(self.camera[camID].facedetect['default'].deque[0])
		fd = self.camera[camID].facedetect['default'].deque[0]  # alias the long name
		for i in range(len(fd['confidence'])): 
			myString = f"{i} - confidence: {fd['confidence'][i]}, corners: {fd['corners'][i]}"
			self.pubNotice(myString)
			'''
			Will look like
			'''


	def callback_notice(self, data):
		print('You received a "notice" message:')
		print(data)
		
	def callback_sysinfo(self, data):
		# [{'id': '1', 'queueLength': 0, 'yourPosition': 0}]
		'''
		print('You received a "sysinfo" message:')
		print(data)
		'''
		pass
		
	def callback_sessionstart(self, data):
		print('You received a "sessionstart" message:')
		print(data)

		try:
			robotID = data['robotID']
			self.robot_id = robotID

			# Save robot limits
			# data['joints'] = {'arm_shoulder_pan_joint': {'id': 1, 'neutral': 0, 'max_angle': 114, 'min_angle': 0, 'max_speed': 300, 'angle_deg': 88.18359375, 'OK': True, 'torque': False, 'angle_rad': 1.5390940571785934}, 'arm_shoulder_lift_joint': {'id': 2, 'neutral': 0, 'max_angle': 160, 'min_angle': 45, 'max_speed': 400, 'angle_deg': 146.48437500000003, 'OK': True, 'torque': False, 'angle_rad': 2.556634646476069}}
			self.joint[robotID] = data['joints']

			# Use 'client_robot_N' to distinguish from host's 'robot_N' camera
			camID = f'client_robot_{robotID}'
			if camID not in self.camera:
				self.startCamera(camID=camID,
								 outputPort=self.cam_port,
								 apiPref=None,
								 device=data['cameraURL'],
								 sslPath=LOCAL_SSL_PATH,
								 intrinsics=data['intrinsics'])
				print(f'FIXME! cam_port: {self.cam_port}')

			for camID in self.camera:
				print(f'{camID=}')

		except Exception as e:
			self.pubNotice(f'Error in callback_sessionstart: {e}')
			
	def callback_status(self, data):
		# {'arm_shoulder_pan_joint': {'id': 1, 'neutral': 0, 'max_angle': 114, 'min_angle': 0, 'max_speed': 300, 'angle_deg': 88.18359375, 'OK': True, 'torque': False, 'angle_rad': 1.5390940571785934}, 'arm_shoulder_lift_joint': {'id': 2, 'neutral': 0, 'max_angle': 160, 'min_angle': 45, 'max_speed': 400, 'angle_deg': 146.48437500000003, 'OK': True, 'torque': False, 'angle_rad': 2.556634646476069}}
		'''
		FIXME 1 -- I'm receiving this even when not using robot
		FIXME 2 -- Need to do something with this data
		print(data)
		'''
		robotID = data[0]
		if robotID not in self.joint:
			return
		self.joint[robotID] = data[1]
		
	def doSomething(self):
		''' 
		A dummy function that actually does nothing
		'''
		pass
		
	def pubNotice(self, txt):
		'''
		Publishes a socket message to `notice` topic, with payload:
		{'data': <string>}
		'''
		print(f'Notice: {txt}')
		# Emit notices message:
		sio_client.emit('notice', txt)

	
	def pubCamStatus(self):
		data = []
		for camID in self.camera:
			data.append({'camID': camID, 
						 'clientIP': self.client_ip, 
						 'outputPort': self.camera[camID].outputPort, 
						 'url': f'https://{self.client_ip}:{self.camera[camID].outputPort}/stream.mjpg', 
						 'streaming': self.camera[camID].keepStreaming})
		print(data)
		sio_client.emit('camStatus', data)
		
	def startCamera(self, camID, outputPort, apiPref, device, sslPath=None, intrinsics=None, res_cols=640, res_rows=480):
		# Initialize `CameraUSB` Class
		try:
			port = ub_utils.findOpenPort(outputPort, options=range(8000,8011))

			paramDict = {'res_rows':res_rows, 'res_cols':res_cols, 'fps_target':30, 'outputPort': port}

			self.camera[camID] = ub_camera.CameraUSB(paramDict = paramDict,
													 device = device,
													 apiPref = apiPref,
													 sslPath = sslPath)
			self.camera[camID].start(startStream=True, port=port)

			if (intrinsics is not None):
				self.camera[camID].intrinsics = intrinsics
				self.camera[camID].intrinsics = self.camera[camID]._getIntrinsics()

			# FIXME -- Need to let webpage know of this camera stream

		except Exception as e:
			self.pubNotice(f'Error in startCamera for camID {camID}: {e}')

	def stopCamera(self, camID):
		# Stop and remove a specific camera
		try:
			if camID in self.camera:
				self.camera[camID].stop()
				del self.camera[camID]
				self.pubNotice(f'Camera {camID} stopped and removed')
			else:
				self.pubNotice(f'Camera {camID} not found in active cameras')
		except Exception as e:
			self.pubNotice(f'Error stopping camera {camID}: {e}')
			

	def ultraPostDetect(self, argsDict):
		idName = argsDict['idName']
		results = argsDict['results']

		try:
			for result in results:
				'''
				xywh = result.boxes.xywh  # center-x, center-y, width, height
				xywhn = result.boxes.xywhn  # normalized
				xyxy = result.boxes.xyxy  # top-left-x, top-left-y, bottom-right-x, bottom-right-y
				xyxyn = result.boxes.xyxyn  # normalized
				names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # class name of each box
				confs = result.boxes.conf  # confidence score of each box    
				'''

				for i in range(0, len(result.boxes.cls)):
					# print(int(result.boxes.cls[i].item())
					# print(camera.ultralytics[idName].model.names[int(result.boxes.cls[i].item())])
					# print(result.boxes.conf[i].item(), result.boxes.xyxy[i].tolist())
					print(f'{result.names[int(result.boxes.cls[i].item())]} ({result.boxes.conf[i].item()}), {result.boxes.xyxy[i].tolist()}')
				
		except Exception as e:
			self.pubNotice(f'Error in ultraPostDetect: {e}')

	def ultraPostPose(self, argsDict):
		idName = argsDict['idName']
		results = argsDict['results']

		'''
		`keypoints` should have 17 elements:
		0: Nose, 1: Left Eye, 2: Right Eye, 3: Left Ear, 4: Right Ear,
		5: Left Shoulder, 6: Right Shoulder, 7: Left Elbow, 8: Right Elbow, 9: Left Wrist, 10: Right Wrist,
		11: Left Hip, 12: Right Hip, 13: Left Knee, 14: Right Knee, 15: Left Ankle, 16: Right Ankle
		'''

		try:
			for result in results:
				if (result.keypoints.has_visible):
					self.pubNotice(f'conf: {result.keypoints.conf.tolist()}, keypoints: {result.keypoints.xy.tolist()} \n')
		except Exception as e:
			self.pubNotice(f'Error in ultraPostPose: {e}')
		
	
	def ultraPostSegment(self, argsDict):
		idName = argsDict['idName']
		results = argsDict['results']
		
		for result in results:
			for i in range(0, len(result.boxes.cls)):
				try:
					self.pubNotice(f'{result.names[int(result.boxes.cls[i].item())]} ({result.boxes.conf[i].item()}), {result.boxes.xyxy[i].tolist()}')   
				except Exception as e:
					self.pubNotice(f'Error in ultraPostSegment: {e}')
					
	def ultraPostObb(self, argsDict):
		idName = argsDict['idName']
		results = argsDict['results']
		
		try:
			for result in results:
				if (result.obb):
					for i in range(0, len(result.obb.cls)):
						self.pubNotice(f'{result.names[int(result.obb.cls[i].item())]} ({result.obb.conf[i].item()}), Center: {result.obb.xywhr[i][0:2].tolist()}')        
		except Exception as e:
			self.pubNotice(f'Error in ultraPostObb: {e}')
			
	def ultraPostTrack(self, argsDict):
		idName = argsDict['idName']
		results = argsDict['results']
		
		# print(idName)   # "track"
		for result in results:
			'''
			xywh = result.boxes.xywh  # center-x, center-y, width, height
			xywhn = result.boxes.xywhn  # normalized
			xyxy = result.boxes.xyxy  # top-left-x, top-left-y, bottom-right-x, bottom-right-y
			xyxyn = result.boxes.xyxyn  # normalized
			names = [result.names[cls.item()] for cls in result.boxes.cls.int()]  # class name of each box
			confs = result.boxes.conf  # confidence score of each box    
			'''
			for i in range(0, len(result.boxes.cls)):
				try:
					self.pubNotice(f'ID: {result.boxes.id[i].item()} - {result.names[int(result.boxes.cls[i].item())]} ({result.boxes.conf[i].item()}), {result.boxes.xyxy[i].tolist()}')                
				except Exception as e:
					self.pubNotice(f'Error in ultraPostTrack: {e}')


	def shutdown(self):
		# Gracefully shut things down.
		# Close files/processes, end loops, etc.
		print('shutting down')

		time.sleep(1)
			
		for camID in self.camera:
			try:
				self.camera[camID].stop()
			except Exception as e:
				self.pubNotice(f'Error stopping camera {camID}: {e}')
					
		try:
			sio_host.disconnect()
		except Exception as e:
			self.pubNotice(f'Host Socket Disconnect Error: {e}')

		try:
			sio_client.disconnect()
		except Exception as e:
			self.pubNotice(f'Client Socket Disconnect Error: {e}')



if __name__ == "__main__":
	ap = argparse.ArgumentParser(
		description='Client-side Python agent for the Arbotix remote control system.'
	)
	ap.add_argument('--host-ip',     default=HOST_IP,
					help='IP address of the host socket server.')
	ap.add_argument('--host-port',   type=int, default=HOST_PORT,
					help='Port of the host socket server.')
	ap.add_argument('--client-ip',   default=CLIENT_IP,
					help=f'IP of the client socket server (default: {CLIENT_IP}).')
	ap.add_argument('--client-port', type=int, default=CLIENT_PORT,
					help=f'Port of the client socket server (default: {CLIENT_PORT}).')
	ap.add_argument('--cam-device',  default=CAM_DEVICE,
					help='Local camera: integer index (0), path (/dev/video0), '
						 'or stream URL. Omit to start without a local camera.')
	ap.add_argument('--cam-port',    type=int, default=CAM_PORT,
					help=f'Port for the local camera MJPEG stream (default: {CAM_PORT}).')
	args = ap.parse_args()

	'''
	if args.host_ip is None:
		ap.error('--host-ip is required (or set HOST_IP at the top of client.py)')
	if args.host_port is None:
		ap.error('--host-port is required (or set HOST_PORT at the top of client.py)')
	'''
	
	# Convert a numeric device string to int (e.g. "0" → 0 for cv2.VideoCapture)
	if args.cam_device is not None:
		try:
			args.cam_device = int(args.cam_device)
		except ValueError:
			pass

	Main(args)
