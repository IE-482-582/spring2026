"""
ub_camera - Unified Camera Interface Module

This module provides a comprehensive camera interface for USB cameras, Raspberry Pi cameras,
and ROS camera topics with extensive computer vision capabilities.

Main Features:
    - Multiple camera backends (USB, Raspberry Pi Camera Module, ROS topics)
    - HTTP/HTTPS video streaming (MJPEG)
    - ROS topic publishing (compressed and raw images)
    - ArUco marker detection and tracking
    - Barcode/QR code detection
    - Face detection
    - Camera calibration tools
    - Timelapse capture
    - Region of Interest (ROI) tracking
    - YOLO object detection (via Ultralytics)
    - Configurable frame rates and resolutions

Classes:
    Camera: Base camera class with common functionality
    CameraPi: Raspberry Pi camera implementation
    CameraROS: ROS camera topic subscriber/publisher
    CameraUSB: USB camera and RTSP stream implementation

Dependencies:
    - numpy
    - opencv-contrib-python (for ArUco support)
    - rospy, cv_bridge, sensor_msgs (optional, for ROS support)
    - ub_utils (custom utility module)

Basic Usage:
    # USB Camera
    camera = CameraUSB(paramDict={'res_rows': 480, 'res_cols': 640, 'fps_target': 30})
    camera.start()

    # Start HTTP streaming
    camera.startStream(port=8000)
    # Visit https://localhost:8000/stream.mjpg

    # Add ArUco marker detection
    camera.addAruco('DICT_APRILTAG_36h11', fps_target=20)

    # Start ROS publishing
    camera.startROStopic()
    # Publishes to /camera/image/compressed and /camera/image/raw

    # Timelapse capture
    camera.addTimelapse(outputDir="/path/to/output", secBetwPhotos=2)

    # Cleanup
    camera.shutdown()

For more examples and detailed usage, see the module-level comments below.

Author: Optimator Lab
"""

import numpy as np
import cv2   # Try `pip install opencv-contrib-python`
import datetime, time
import threading
import os, platform, sys
import math
from collections import deque
from pathlib import Path

import ub_utils				# A bunch of (somewhat) helpful functions and variables

# This stuff is for streaming only:
# ------------------------------------------------
import socketserver
from functools import partial
from threading import Condition
from http import server
import ssl

STREAM_MAX_WAIT_TIME_SEC = 2  # max time (in seconds) we wait for condition
# ------------------------------------------------

# This stuff is for ROS only:
# ------------------------------------------------
try:
	import rospy     
	from cv_bridge import CvBridge  # NOTE:  Does not support CompressedImage in Python
	from sensor_msgs.msg import Image, CompressedImage
except Exception as e:
	print(f'INFO: rospy is not installed and was not imported.  You may ignore this message.  Unless you are using ROS you do not need rospy.')
	# print(f'NOTE: Could not import rospy:  {e}')

ROSPUB_MAX_WAIT_TIME_SEC = 2  # max time (in seconds) we wait for condition
# ------------------------------------------------

'''
try:
	system = platform.system()
	if (system == 'Linux'):
		# For Linux/Mac
		HOME_DIRECTORY = os.environ['HOME']
	else:
		# For Windows
		HOME_DIRECTORY = os.environ['USERPROFILE']
except Exception as e:
	print(f'Error - Could not set HOME_DIRECTORY: {e}')
'''	
		

'''
import ub_camera
camera = ub_camera.CameraUSB(paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort':8000})
camera.startStream(port=8000)
# Visit https://localhost:8000/stream.mjpg


camera = ub_camera.CameraPi(paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort':8000}, initROSnode=False)
camera = ub_camera.CameraUSB(paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort':8000}, initROSnode=False)
camera = ub_camera.CameraUSB(paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort':8000}, device='/dev/video2', fourcc='MJPG', initROSnode=False)

camera = ub_camera.CameraUSB(paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort':8000}, device='rtsp://192.168.0.114:8900/live', fourcc='MJPG', initROSnode=False)


camera.start()

camera.startStream(port=8000)
# Visit https://localhost:8000/stream.mjpg

camera.addAruco('DICT_APRILTAG_36h11', fps_target=20)

camera.startROStopic()
# Starts publishing `/camera/image/compressed` and `/camera/image/raw`

# camera.addBarcode()
# camera.addCalibrate()

outputDir = f"{os.environ['HOME']}/Downloads/Timelapse/test2"
camera.addTimelapse(outputDir=outputDir, secBetwPhotos=2, timeLimitSec=None, delayStartSec=0, res_rows=None, res_cols=None, postPostFunction=None)
# ... wait some time ...
camera.timelapse['default'].stop()
ub_utils.pics2video(sourcePath=outputDir, filename="myVideo.mp4", fps=2)

camera.shutdown()

exit()

'''










class _Aruco():
	"""
	Internal class for ArUco marker detection and tracking.

	This class runs in a separate thread to detect ArUco/AprilTag markers in camera frames.
	Detected markers are stored in a deque and can be drawn on the video stream.

	Attributes:
		camObject: Parent Camera object
		idName (str): ArUco dictionary name (e.g., 'DICT_APRILTAG_36h11')
		res_rows (int): Height for marker detection
		res_cols (int): Width for marker detection
		fps_target (float): Target detection framerate in Hz
		calcRotations (bool): Whether to calculate marker rotations
		postFunction (callable): Callback function after each detection
		postFunctionArgs (dict): Arguments for postFunction
		config (dict): Drawing configuration (colors, line thickness, etc.)
		ids_of_interest (list): List of specific marker IDs to track (None for all)
		deque (deque): Most recent detection results {'ids', 'corners', 'centers', 'rotations'}

	Methods:
		start(): Start the detection thread
		stop(): Stop the detection thread and cleanup
	"""
	def __init__(self, camObject, idName, res_rows, res_cols, fps_target, calcRotations, postFunction, postFunctionArgs, configDict, ids_of_interest):
		try:
			self.camObject = camObject  # This is the parent!
								
			self.idName   = idName
			self.decorationID = None
			
			self.res_rows = res_rows
			self.res_cols = res_cols		
			self.resolution = f'{res_cols}x{res_rows}'

			self.fps_target  = fps_target		# Hz
			self.threadSleep = 1/fps_target		# seconds
			
			self.calcRotations = calcRotations
				
			self.postFunctionArgs = postFunctionArgs
			self.postFunctionArgs['idName'] = idName
			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction

			self.config = configDict
			# self.color = color
			
			self.ids_of_interest = ids_of_interest
			
			self.fps = _make_fps_dict(recheckInterval=5)

			self.deque = deque(maxlen=1)
			self.deque.append({'ids': None, 'corners': [], 'centers': [], 'rotations': []})

			(major, minor, sub) = cv2.__version__.split(".")[:3]
			if ((int(major) >= 4) and (int(minor) >= 7)):
				self.cv2dict   = cv2.aruco.getPredefinedDictionary(ub_utils.ARUCO_DICT[idName]['dict'])
				self.cv2params = cv2.aruco.DetectorParameters()
			else:
				# This is old:
				self.cv2dict   = cv2.aruco.Dictionary_get(ub_utils.ARUCO_DICT[idName]['dict'])
				self.cv2params = cv2.aruco.DetectorParameters_create()
					
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in aruco init: {e}.', severity=ub_utils.SEVERITY_ERROR)
		

	def _decorate(self, img, **kwargs):
		ub_utils.arucoDrawDetections(img, self.deque[0]['corners'],
									        self.deque[0]['ids'], 
									        self.deque[0]['centers'], 
									        self.deque[0]['rotations'], self.config)		
		
	def _thread_Aruco(self):
		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.aruco[idName] 
		'''
		self.isThreadActive = True

		while self.camObject.camOn:
			try:	
				timeNow = time.time()
							
				# FIXME -- It would be nice to cut out the `if` statements...
				
				# Throttle things if we're going faster than capture speed
				if (self.fps.actual >= self.camObject.fps['capture'].actual):
					with self.camObject.condition:
						self.camObject.condition.wait(1)   # added a timeout, just to keep from getting permanently stuck here
				
				# FIXME -- Why are we calculating this each time (in loop)?
				# We should only set resOption when properties change.
				img_x_y  = (self.res_cols, self.res_rows)
				orig_x_y = (self.camObject.res_cols, self.camObject.res_rows)
				if (img_x_y == orig_x_y):
					resOption = None
				else:
					resOption = img_x_y
				
				img = self.camObject.getFrameCopy(colorOption='gray', resOption=resOption)
								
				# `corners` will be of same scale as original (captured) image
				(corners, ids, rejected, centers, rotations) = ub_utils.arucoDetectMarkers(img, 
																		 self.cv2dict, 
																		 self.cv2params,
																		 img_x_y  = img_x_y,
																		 orig_x_y = orig_x_y)
	
				'''
				centers = []
				rotations = []
				for i in range(0, len(corners)):
					# Find midpoint, using corner points 1 (NE) and 3 (SW)
					# NOTE:  These are not int coordinates.
					mp = ((corners[i][0][3][0] + corners[i][0][1][0])/2, 
						  (corners[i][0][3][1] + corners[i][0][1][1])/2) 
					centers.append(mp)
					if (self.calcRotations):					
						# point 0 is top left, 3 is bottom left.  x increases to right, y increases down
						x = corners[i][0][0][0] - corners[i][0][3][0]
						y = corners[i][0][0][1] - corners[i][0][3][1]
						theta = math.atan2(x, -y)  # NOTE:  This is in [radians]
					
						rotations.append(theta) 
						print(np.rad2deg(theta))
				'''		
				'''
				if (len(corners) > 0):
					print(corners, centers, rotations)
				'''
									
				# Add detection info to deque:
				# print(len(self.deque))
				if (self.ids_of_interest is None):
					self.deque.append({'ids': ids, 'corners': corners, 'centers': centers, 'rotations': rotations})
				else:
					indices = ub_utils.arucoFindTagIndicesList(ids, self.ids_of_interest)
					if (len(indices)):
						self.deque.append({'ids': ids[indices], 'corners': corners[indices], 'centers': centers[indices], 'rotations': rotations[indices]})
					else:
						self.deque.append({'ids': [], 'corners': [], 'centers': [], 'rotations': []})
						
					
					'''
					# self.camObject.logger.log(f'{ids=}, {corners=}, {type(ids)}, {type(corners)}', severity=ub_utils.SEVERITY_DEBUG)	
					if (ids is None):
						self.deque.append({'ids': [], 'corners': [], 'centers': [], 'rotations': []})
					else:
						indices = [i for i in range(len(ids)) if ids[i] in self.ids_of_interest]
						if (len(indices) > 0):
							self.deque.append({'ids': [ids[i] for i in indices], 'corners': [corners[i] for i in indices], 'centers': [centers[i] for i in indices], 'rotations': [rotations[i] for i in indices]})

						else:
							self.deque.append({'ids': [], 'corners': [], 'centers': [], 'rotations': []})
					'''
					
				# Do some post-processing:
				self.postFunction(self.postFunctionArgs)
								
				self.camObject.calcFramerate(self.fps, 'aruco')
				
				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in Aruco {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping ArUco {self.idName} thread - no longer active.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
		
		# If while loop stops, shut down aruco:
		self.stop()	


	def start(self):
		try:			
			self.camObject.logger.log(f'Starting ArUco {self.idName} thread at {self.fps_target} fps', severity=ub_utils.SEVERITY_INFO)
			
			arucoThread = threading.Thread(target=self._thread_Aruco, args=())
			arucoThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			arucoThread.start()

			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

		except Exception as e:
			self.camObject.logger.log(f'Error in aruco start: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def stop(self):
		try:
			if (self.idName in self.camObject.aruco):
				'''
				# Remove idName from self.camObject.decorations['aruco']
				if (self.idName in self.camObject.decorations['aruco']):
					self.camObject.decorations['aruco'].remove(self.idName)
				'''	
				self.camObject.dec['dequeRemove'].append(self.decorationID)	

				self.camObject.logger.log(f'Stopping ArUco {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				
				self.isThreadActive = False
				self.deque.clear()					
			else:
				self.camObject.logger.log(f'In stop, aruco {self.idName} dictionary is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in aruco stop: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def edit(self, res_rows=None, res_cols=None, fps_target=5, postFunction=None, color=None):
		# Note:  `color=None` now implies "do not change color".
		try:
			# change fps_target, resolution, (function?)
			if ((res_rows is not None) and (res_cols is not None)):
				if ((res_cols, res_rows) != (self.res_cols, self.res_rows)):
					(self.res_cols, self.res_rows) = (int(res_cols), int(res_rows))
					self.resolution = f'{res_cols}x{res_rows}'
			
			if (fps_target != self.fps_target):
				self.fps_target = int(fps_target)
				self.threadSleep = 1/self.fps_target
				
			if (postFunction is not None):
				self.postFunction = postFunction
				
			if (color is not None):
				self.color = color
		except Exception as e:
			self.camObject.logger.log(f'Error in aruco edit: {e}.', severity=ub_utils.SEVERITY_ERROR)


class _Calibrate():
	"""Internal camera calibration feature class using checkerboard pattern detection.

	This class performs camera calibration by detecting checkerboard patterns in captured
	frames and computing the camera's intrinsic matrix and distortion coefficients.
	Calibration runs in a separate thread and collects images at specified intervals
	until the required number of valid detections is obtained or a timeout occurs.

	Attributes:
		camObject: Parent Camera instance managing this calibration feature.
		idName (str): Unique identifier for this calibration instance.
		res_rows (int): Target vertical resolution in pixels.
		res_cols (int): Target horizontal resolution in pixels.
		numImages (int): Number of checkerboard detections required for calibration.
		timeoutSec (float): Maximum time in seconds to wait for calibration completion.
		pattern_size (tuple): Checkerboard dimensions as (cols, rows) of internal corners.
		square_size (float): Physical size of checkerboard squares in world units.
		postFunction (callable): Callback function invoked after calibration completes.
		deque (collections.deque): Thread-safe storage for latest detection results.
		isThreadActive (bool): Flag indicating if calibration thread is running.

	Key Methods:
		start(): Initiates calibration thread and begins collecting checkerboard images.
		stop(): Terminates calibration thread and cleans up resources.
	"""
	def __init__(self, camObject, idName, res_rows, res_cols, secBetweenImages, numImages, timeoutSec, pattern_size, square_size, postFunction):
		"""Initialize camera calibration feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Unique identifier for this calibration.
			res_rows (int): Target vertical resolution in pixels.
			res_cols (int): Target horizontal resolution in pixels.
			secBetweenImages (float): Time in seconds between image captures.
			numImages (int): Number of valid checkerboard detections needed.
			timeoutSec (float): Maximum calibration duration in seconds.
			pattern_size (tuple): Checkerboard dimensions (cols, rows) of internal corners.
			square_size (float): Physical size of squares in world units.
			postFunction (callable): Callback executed after calibration with results.
		"""
		try:
			self.camObject = camObject  # This is the parent!
								
			self.idName   = idName
			self.decorationID = None
			
			self.res_rows = res_rows
			self.res_cols = res_cols		
			self.resolution = f'{res_cols}x{res_rows}'

			# self.fps_target  = fps_target		# Hz
			# self.threadSleep = 1/fps_target		# seconds
			# self.fps = _make_fps_dict(recheckInterval=5)
			self.threadSleep = secBetweenImages		# seconds

				
			self.numImages    = numImages
			self.timeoutSec   = timeoutSec
			self.pattern_size = pattern_size
			self.square_size  = square_size

			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction
			
			self.deque = deque(maxlen=1)
			self.deque.append({'checkerboard': None, 'corners': None, 'count': 0, 'img_x_y': (), 'orig_x_y': ()})
								
			self.isThreadActive = False			
		except Exception as e:
			self.camObject.logger.log(f'Error in barcode init: {e}.', severity=ub_utils.SEVERITY_ERROR)
			
	def _decorate(self, img, **kwargs):
		ub_utils.decorateCalibrate(img, 
									 self.deque[0]['checkerboard'], 
									 self.deque[0]['corners'], 
									 self.deque[0]['count'], 
									 self.deque[0]['img_x_y'], 
									 self.deque[0]['orig_x_y'], addText=True)

	def _thread_Calibrate(self):
		# See https://github.com/opencv/opencv/blob/master/samples/python/calibrate.py
		# See https://learnopencv.com/camera-calibration-using-opencv/
		
		# Defining the dimensions of checkerboard
		CHECKERBOARD = self.pattern_size   # e.g., (6,9)
		
		# FIXME -- What is this???
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		
		# Create vector to store vectors of 3D points for each checkerboard image
		objpoints = []
		# Create vector to store vectors of 2D points for each checkerboard image
		imgpoints = [] 
		
		# Define the world coordinates for 3D points
		objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
		objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
		objp *= self.square_size
		
		# Initialize return values
		success    = False
		mtx        = []
		dist       = []
		mean_error = -1
			
		img_x_y  = (self.res_cols, self.res_rows)
		orig_x_y = (self.camObject.res_cols, self.camObject.res_rows)
		if (img_x_y == orig_x_y):
			resOption = None
		else:
			resOption = img_x_y

		timeStart = time.time()
			
		self.isThreadActive = True

		try:
			while self.isThreadActive:
				# We should be going very slowly...no need to wait for next frame.
				timeNow = time.time()
								
				gray = self.camObject.getFrameCopy(colorOption='gray', resOption=resOption)

				# Find the chess board corners
				# If desired number of corners are found in the image then ret = true
				ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
				 
				"""
				If desired number of corners are detected,
				refine the pixel coordinates and display 
				them on the images of checker board
				"""
				if ret == True:
					objpoints.append(objp)
					# refining pixel coordinates for given 2d points.
					corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
					 
					imgpoints.append(corners2)
					
					# Draw and display the corners
					# FIXME -- Need to decorate
					# img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
					# 

					# Add detection info to deque:
					self.deque.append({'checkerboard': CHECKERBOARD, 'corners': corners2, 'count': len(imgpoints), 'img_x_y': img_x_y, 'orig_x_y': orig_x_y})

					# Reset timer
					timeStart = time.time()
				else:
					self.deque.append({'checkerboard': None, 'corners': None, 'count': len(imgpoints), 'img_x_y': img_x_y, 'orig_x_y': orig_x_y})
										
								
				# Do some post-processing:
				# self.postFunction()
				
				# Simplified version of rospy.sleep
				delta = max(0, timeNow + self.threadSleep - time.time())
				if (delta > 0):
					time.sleep(delta)

				self.isThreadActive = self.camObject.camOn 
				if ((time.time() - timeStart >= self.timeoutSec) or (len(imgpoints) >= self.numImages)):
					self.isThreadActive = False
					
					
			if (len(imgpoints) >= self.numImages):
				"""
				Perform camera calibration by 
				passing the value of known 3D points (objpoints)
				and corresponding pixel coordinates of the 
				detected corners (imgpoints)
				"""
				ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
				
				# self.camObject.logger.log(f'Error in barcode init: {e}.', severity=ub_utils.SEVERITY_ERROR)
				print("Camera matrix : \n")
				print(mtx)
				print("dist : \n")
				print(dist)
				print("rvecs : \n")
				print(rvecs)
				print("tvecs : \n")
				print(tvecs)
				print(f"Resolution: {self.resolution}") 

				# Check Reprojection Error.
				# See bottom of https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
				total_error = 0
				for i in range(len(objpoints)):
					imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
					error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
					total_error += error
				print( "\ntotal error: {}".format(total_error) )
				mean_error = total_error/len(objpoints)
				print( "mean error: {}".format(mean_error) )

				success = True

			self.stop()
			
		except Exception as e:
			self.stop()
			self.camObject.logger.log(f'Error in calibration {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				

		finally:
			self.postFunction(success=success, res=f'{self.res_cols}x{self.res_rows}', mtx=mtx, dist=dist, total_error=total_error, mean_error=mean_error)

			
	def start(self):
		"""Start calibration thread to collect checkerboard images.

		Launches a daemon thread that captures images at regular intervals, detects
		checkerboard patterns, and computes calibration parameters. The thread
		terminates when sufficient valid detections are collected or timeout occurs.
		Automatically registers a decoration function to visualize detected corners.
		"""
		try:
			'''
			# Add idName to self.decorations['calibrate']
			if (self.idName not in self.camObject.decorations['calibrate']):
				self.camObject.decorations['calibrate'].append(self.idName)
			'''
			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

			self.camObject.logger.log(f'Starting calibration {self.idName} thread at {self.threadSleep} sec betw images', severity=ub_utils.SEVERITY_INFO)

			calThread = threading.Thread(target=self._thread_Calibrate, args=())
			calThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			calThread.start()
		except Exception as e:
			self.camObject.logger.log(f'Error in calibrate start: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
	def stop(self):
		"""Stop calibration thread and clean up resources.

		Signals the calibration thread to terminate, removes associated decorations,
		and clears the detection deque. Safe to call even if calibration has already
		completed or was never started.
		"""
		try:
			if (self.idName in self.camObject.calibrate):
				'''
				# Remove idName from self.camObject.decorations['calibrate']
				if (self.idName in self.camObject.decorations['calibrate']):
					self.camObject.decorations['calibrate'].remove(self.idName)
				'''
				self.camObject.dec['dequeRemove'].append(self.decorationID)

				self.camObject.logger.log(f'Stopping calibrate {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				self.deque.clear()
			else:
				self.camObject.logger.log(f'In stop, calibrate {self.idName} name is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in calibrate stop: {e}.', severity=ub_utils.SEVERITY_ERROR)	
		
		
class _Barcode():
	"""Internal barcode detection feature class using pyzbar library.

	This class detects and decodes 1D and 2D barcodes (including QR codes) in camera
	frames using the pyzbar library. Detection runs continuously in a separate thread
	at a specified frame rate, with results stored in a thread-safe deque.

	Attributes:
		camObject: Parent Camera instance managing this barcode detector.
		idName (str): Unique identifier for this barcode detection instance.
		res_rows (int): Target vertical resolution in pixels.
		res_cols (int): Target horizontal resolution in pixels.
		fps_target (float): Target detection rate in frames per second.
		postFunction (callable): Callback function invoked after each detection cycle.
		postFunctionArgs (dict): Arguments passed to the post-processing callback.
		color (tuple): RGB color for visualization of detected barcodes.
		deque (collections.deque): Thread-safe storage for latest detection results.
		fps (dict): Frame rate tracking metrics for this detector.
		isThreadActive (bool): Flag indicating if detection thread is running.

	Key Methods:
		start(): Launches barcode detection thread.
		stop(): Terminates detection thread and cleans up resources.
	"""
	def __init__(self, camObject, idName, res_rows, res_cols, fps_target, postFunction, postFunctionArgs, color):
		"""Initialize barcode detection feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Unique identifier for this barcode detector.
			res_rows (int): Target vertical resolution in pixels.
			res_cols (int): Target horizontal resolution in pixels.
			fps_target (float): Target detection rate in Hz.
			postFunction (callable): Callback executed after each detection cycle.
			postFunctionArgs (dict): Arguments for post-processing callback.
			color (tuple): RGB color tuple for barcode visualization.
		"""
		try:
			# https://pypi.org/project/pyzbar/
			from pyzbar import pyzbar
			self.pyzbar = pyzbar
			
			self.camObject = camObject  # This is the parent!
								
			self.idName   = idName
			self.decorationID = None
			
			self.res_rows = res_rows
			self.res_cols = res_cols		
			self.resolution = f'{res_cols}x{res_rows}'

			self.fps_target  = fps_target		# Hz
			self.threadSleep = 1/fps_target		# seconds
				
			self.postFunctionArgs = postFunctionArgs
			self.postFunctionArgs['idName'] = idName	
			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction

			self.color = color
			
			self.fps = _make_fps_dict(recheckInterval=5)

			self.deque = deque(maxlen=1)
			self.deque.append({'data': [], 'codeTypes': [], 'qualities': [], 'corners': [], 'color': self.color})
								
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in barcode init: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def _decorate(self, img, **kwargs):
		# print('idName:', idName, 'barcode[idName]:', self.barcode[idName].deque[0])
		# print(self.barcode[idName].deque[0])
		ub_utils.decorateBarcode(img, 
								   self.deque[0]['corners'], 
								   self.deque[0]['data'], 
								   self.deque[0]['color'], addText=True)


	def _thread_Barcode(self):

		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.barcode['default] 
		'''
		self.isThreadActive = True

		while self.camObject.camOn:
			try:
				timeNow = time.time()
							
				# FIXME -- It would be nice to cut out the `if` statements...
				
				# Throttle things if we're going faster than capture speed
				if (self.fps.actual >= self.camObject.fps['capture'].actual):
					with self.camObject.condition:
						self.camObject.condition.wait(1)   # added a timeout, just to keep from getting permanently stuck here

				'''
				# FIXME -- This was copied from ROI.  Is barcode as brittle?
				# This won't work if cam resolution has changed.
				if ((self.res_cols, self.res_rows) != (self.camObject.res_cols, self.camObject.res_rows)):
					raise Exception('Resolution changed. Stopping Barcode thread')
					# self.stop()
					# break
				'''

				data      = []
				codeTypes = []
				qualities = []
				corners   = []	

				codeList = self.pyzbar.decode(self.camObject.getFrameCopy())	   # Don't need a copy?
				for detections in codeList:
					data.append(str(detections.data, 'utf-8'))
					codeTypes.append(detections.type)
					qualities.append(detections.quality)
					'''
					This was giving really inconsistent results.
					We'll just use the rectangle instead.
					poly = []
					for vertex in detections.polygon:
						poly.append([vertex.x, vertex.y])
					corners.append(np.array(poly, np.int32).reshape((-1, 1, 2)))
					'''
					rect = [(int(detections.rect.left), int(detections.rect.top)), 
							(int(detections.rect.left+detections.rect.width), int(detections.rect.top+detections.rect.height))]
					corners.append(rect)
											
				# Add detection info to deque:
				self.deque.append({'data': data, 'codeTypes': codeTypes, 'qualities': qualities, 'corners': corners, 'color': self.color})
								
				# Do some post-processing:
				self.postFunction(self.postFunctionArgs)
				
				self.camObject.calcFramerate(self.fps, 'barcode')

				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in barcode {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping barcode {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
				
		# If while loop stops, shut down barcode:
		self.stop()


	def start(self):
		"""Start barcode detection thread at the configured frame rate.

		Launches a daemon thread that continuously captures frames, decodes barcodes,
		and stores detection results. The thread automatically throttles itself to
		match the camera's capture rate. Registers a decoration function to visualize
		detected barcodes on the video stream.
		"""
		try:
			'''
			# Add idName to self.decorations['barcode']
			if (self.idName not in self.camObject.decorations['barcode']):
				self.camObject.decorations['barcode'].append(self.idName)
			'''
			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

			self.camObject.logger.log(f'Starting barcode {self.idName} thread at {self.fps_target} fps', severity=ub_utils.SEVERITY_INFO)

			barThread = threading.Thread(target=self._thread_Barcode, args=())
			barThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			barThread.start()

		except Exception as e:
			self.camObject.logger.log(f'Error in barcode start: {e}.', severity=ub_utils.SEVERITY_ERROR)

		
	def stop(self):
		"""Stop barcode detection thread and clean up resources.

		Signals the detection thread to terminate, removes associated decorations,
		and clears the detection deque. Safe to call multiple times.
		"""
		try:
			if (self.idName in self.camObject.barcode):
				'''
				# Remove idName from self.camObject.decorations['barcode']
				if (self.idName in self.camObject.decorations['barcode']):
					self.camObject.decorations['barcode'].remove(self.idName)
				'''
				self.camObject.dec['dequeRemove'].append(self.decorationID)

				self.camObject.logger.log(f'Stopping barcode {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				self.deque.clear()

			else:
				self.camObject.logger.log(f'In stop, barcode {self.idName} name is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in barcode stop: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
		
	def edit(self, fps_target=None, res_rows=None, res_cols=None):
		self.camObject.logger.log('Sorry, barcode editing is not supported.', severity=ub_utils.SEVERITY_WARNING)
		

class _FaceDetect():
	"""Internal face detection feature class using OpenCV DNN models.

	This class detects human faces in camera frames using pre-trained deep neural
	network models (Caffe or TensorFlow). Detection runs continuously in a separate
	thread at a specified frame rate, supporting both CPU and GPU inference.

	Attributes:
		camObject: Parent Camera instance managing this face detector.
		idName (str): Unique identifier for this face detection instance.
		res_rows (int): Target vertical resolution in pixels.
		res_cols (int): Target horizontal resolution in pixels.
		fps_target (float): Target detection rate in frames per second.
		postFunction (callable): Callback function invoked after each detection cycle.
		postFunctionArgs (dict): Arguments passed to the post-processing callback.
		color (tuple): RGB color for visualization of detected faces.
		conf_threshold (float): Minimum confidence threshold for face detections.
		dnn (str): DNN backend type ("caffe" or "tensorflow").
		device (str): Computation device ("cpu" or "gpu").
		modelPath (str): Directory path containing DNN model files.
		deque (collections.deque): Thread-safe storage for latest detection results.
		fps (dict): Frame rate tracking metrics for this detector.
		isThreadActive (bool): Flag indicating if detection thread is running.

	Key Methods:
		start(): Launches face detection thread.
		stop(): Terminates detection thread and cleans up resources.
	"""
	def __init__(self, camObject, idName, res_rows, res_cols, fps_target, postFunction, postFunctionArgs, color, conf_threshold, dnn, device, modelPath):
		"""Initialize face detection feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Unique identifier for this face detector.
			res_rows (int): Target vertical resolution in pixels.
			res_cols (int): Target horizontal resolution in pixels.
			fps_target (float): Target detection rate in Hz.
			postFunction (callable): Callback executed after each detection cycle.
			postFunctionArgs (dict): Arguments for post-processing callback.
			color (tuple): RGB color tuple for face bounding box visualization.
			conf_threshold (float): Minimum confidence (0.0-1.0) for detections.
			dnn (str): Neural network backend ("caffe" or "tensorflow").
			device (str): Computation device ("cpu" or "gpu").
			modelPath (str): Path to directory containing model files, or None for default.
		"""
		try:
			# https://learnopencv.com/face-detection-opencv-dlib-and-deep-learning-c-python/
			# https://pyimagesearch.com/2018/02/26/face-detection-with-opencv-and-deep-learning/
			# https://pyimagesearch.com/2018/09/24/opencv-face-recognition/
			# https://github.com/spmallick/learnopencv/tree/master/FaceDetectionComparison
				
			self.camObject = camObject  # This is the parent!
								
			self.idName   = idName
			self.decorationID = None
			
			self.res_rows = res_rows
			self.res_cols = res_cols		
			self.resolution = f'{res_cols}x{res_rows}'

			self.fps_target  = fps_target		# Hz
			self.threadSleep = 1/fps_target		# seconds
				
			self.postFunctionArgs = postFunctionArgs
			self.postFunctionArgs['idName'] = idName
			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction

			if (modelPath):
				self.modelPath = modelPath
			else:
				self.modelPath = f'{os.getcwd()}/cv2_dnn_models'  # Place where deploy.prototxt, res10_300x300_....caffemodel are saved


			self.color = color
			
			self.conf_threshold = conf_threshold
			self.dnn            = dnn
			self.device         = device
			
			self.fps = _make_fps_dict(recheckInterval=5)

			self.deque = deque(maxlen=1)
			self.deque.append({'confidence': [], 'corners': [], 'color': self.color})
														
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in facedetect init: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def _decorate(self, img, **kwargs):
		# print('idName:', idName, 'facedetect[idName]:', self.facedetect[idName].deque[0])
		# print(self.facedetect[idName].deque[0])
		ub_utils.decorateFaceDetect(img, 
								   self.deque[0]['confidence'], 
								   self.deque[0]['corners'], 
								   self.deque[0]['color'], addText=True)

			
	def _blobCaffe(self, frameCopy):
		return cv2.dnn.blobFromImage(frameCopy, 1.0, (300, 300), [104, 117, 123], False, False,)
		
	def _blobTF(self, frameCopy):
		return cv2.dnn.blobFromImage(frameCopy, 1.0, (300, 300), [104, 117, 123], True, False,)
		
	def _detectFaceOpenCVDnn(self, frameCopy, blobFunction, net):
		frameHeight = frameCopy.shape[0]
		frameWidth = frameCopy.shape[1]
		blob = blobFunction(frameCopy)   # self._blobCaffe or self._blobTF

		net.setInput(blob)
		detections = net.forward()
		confidence = []
		bboxes     = []
		for i in range(detections.shape[2]):
			conf = detections[0, 0, i, 2]
			if conf > self.conf_threshold:
				x1 = int(detections[0, 0, i, 3] * frameWidth)
				y1 = int(detections[0, 0, i, 4] * frameHeight)
				x2 = int(detections[0, 0, i, 5] * frameWidth)
				y2 = int(detections[0, 0, i, 6] * frameHeight)
				bboxes.append([(x1, y1), (x2, y2)])
				confidence.append(conf)

		return confidence, bboxes
    		

	def _thread_FaceDetect(self):

		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.facedetect['default] 
		'''
		self.isThreadActive = True

		# OpenCV DNN supports 2 networks.
		# 1. FP16 version of the original Caffe implementation ( 5.4 MB )
		# 2. 8 bit Quantized version using TensorFlow ( 2.7 MB )

		if (self.dnn == "caffe"):
			modelFile  = f"{self.modelPath}/res10_300x300_ssd_iter_140000_fp16.caffemodel"
			configFile = f"{self.modelPath}/deploy.prototxt"
			net = cv2.dnn.readNetFromCaffe(configFile, modelFile)
			blobFunction = self._blobCaffe
		else:
			modelFile  = f"{self.modelPath}/opencv_face_detector_uint8.pb"
			configFile = f"{self.modelPath}/opencv_face_detector.pbtxt"
			net = cv2.dnn.readNetFromTensorflow(modelFile, configFile)
			blobFunction = self._blobTF

		if (self.device == "cpu"):
			net.setPreferableBackend(cv2.dnn.DNN_TARGET_CPU)
		else:
			net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
			net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)


		while self.camObject.camOn:
			try:
				timeNow = time.time()
							
				# FIXME -- It would be nice to cut out the `if` statements...
				
				# Throttle things if we're going faster than capture speed
				if (self.fps.actual >= self.camObject.fps['capture'].actual):
					with self.camObject.condition:
						self.camObject.condition.wait(1)   # added a timeout, just to keep from getting permanently stuck here

				'''
				# FIXME -- This was copied from ROI and barcode.  Is facedetect as brittle?
				# This won't work if cam resolution has changed.
				if ((self.res_cols, self.res_rows) != (self.camObject.res_cols, self.camObject.res_rows)):
					raise Exception('Resolution changed. Stopping FaceDetect thread')
					# self.stop()
					# break
				'''

				confidence, corners = self._detectFaceOpenCVDnn(self.camObject.getFrameCopy(), blobFunction, net)
				
				'''
				for detections in codeList:
					data.append(str(detections.data, 'utf-8'))
					codeTypes.append(detections.type)
					qualities.append(detections.quality)
					rect = [(int(detections.rect.left), int(detections.rect.top)), 
							(int(detections.rect.left+detections.rect.width), int(detections.rect.top+detections.rect.height))]
					corners.append(rect)
				'''
				
				# Add detection info to deque:
				self.deque.append({'confidence': confidence, 'corners': corners, 'color': self.color})
								
				# Do some post-processing:
				self.postFunction(self.postFunctionArgs)
				
				self.camObject.calcFramerate(self.fps, 'facedetect')

				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in facedetect {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping facedetect {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
				
		# If while loop stops, shut down facedetect:
		self.stop()


	def start(self):
		"""Start face detection thread at the configured frame rate.

		Launches a daemon thread that loads the DNN model, continuously processes
		frames for face detection, and stores results. The thread automatically
		throttles itself to match the camera's capture rate. Registers a decoration
		function to visualize detected faces with bounding boxes.
		"""
		try:
			'''
			# Add idName to self.decorations['facedetect']
			if (self.idName not in self.camObject.decorations['facedetect']):
				self.camObject.decorations['facedetect'].append(self.idName)
			'''
			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

			self.camObject.logger.log(f'Starting facedetect {self.idName} thread at {self.fps_target} fps', severity=ub_utils.SEVERITY_INFO)

			faceThread = threading.Thread(target=self._thread_FaceDetect, args=())
			faceThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			faceThread.start()

		except Exception as e:
			self.camObject.logger.log(f'Error in facedetect start: {e}.', severity=ub_utils.SEVERITY_ERROR)

		
	def stop(self):
		"""Stop face detection thread and clean up resources.

		Signals the detection thread to terminate, removes associated decorations,
		and clears the detection deque. Safe to call multiple times.
		"""
		try:
			if (self.idName in self.camObject.facedetect):
				'''
				# Remove idName from self.camObject.decorations['facedetect']
				if (self.idName in self.camObject.decorations['facedetect']):
					self.camObject.decorations['facedetect'].remove(self.idName)
				'''
				self.camObject.dec['dequeRemove'].append(self.decorationID)

				self.camObject.logger.log(f'Stopping FaceDetect {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				self.deque.clear()

			else:
				self.camObject.logger.log(f'In stop, FaceDetect {self.idName} name is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in FaceDetect stop: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
		
	def edit(self, fps_target=None, res_rows=None, res_cols=None):
		self.camObject.logger.log('Sorry, FaceDetect editing is not supported.', severity=ub_utils.SEVERITY_WARNING)
		


class _Timelapse():
	"""Internal timelapse photography feature class for automated image capture.

	This class captures camera frames at regular intervals and saves them to disk,
	enabling creation of timelapse videos. Capture runs in a separate thread with
	configurable timing, resolution, and duration limits.

	Attributes:
		camObject: Parent Camera instance managing this timelapse feature.
		idName (str): Unique identifier for this timelapse instance.
		outputDir (str): Directory path where captured images will be saved.
		timeLimitSec (float): Maximum capture duration in seconds, or None for unlimited.
		delayStartSec (float): Initial delay before starting capture.
		resOption (tuple): Target resolution as (width, height) in pixels.
		threadSleep (float): Time interval in seconds between photo captures.
		postPostFunction (callable): Callback invoked after timelapse completes.
		isThreadActive (bool): Flag indicating if timelapse thread is running.

	Key Methods:
		start(): Launches timelapse capture thread.
		stop(): Terminates timelapse thread and cleans up resources.
	"""
	def __init__(self, camObject, idName, outputDir, secBetwPhotos, timeLimitSec, delayStartSec, res_rows, res_cols, postPostFunction):
		"""Initialize timelapse capture feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Unique identifier for this timelapse.
			outputDir (str): Directory path for saving captured images.
			secBetwPhotos (float): Time interval in seconds between captures.
			timeLimitSec (float): Maximum duration in seconds, or None for unlimited.
			delayStartSec (float): Initial delay in seconds before first capture.
			res_rows (int): Target vertical resolution in pixels.
			res_cols (int): Target horizontal resolution in pixels.
			postPostFunction (callable): Callback executed after timelapse ends.
		"""
		try:
			self.camObject = camObject  # This is the parent!
						
			self.idName     = idName
			self.decorationID = None   # We're not going to use this.
			
			self.outputDir = outputDir
			# self.secBetwPhotos = secBetwPhotos
			self.timeLimitSec  = timeLimitSec
			self.delayStartSec = delayStartSec
			# self.res_rows = res_rows
			# self.res_cols = res_cols		
			self.resOption = (res_cols, res_rows)   # (width x, height y)

			self.threadSleep = secBetwPhotos   # seconds
		
			# In other threads, this is where we do post-processing (per capture).
			# For timelapse, we have postPostProcessing (after thread ends)
			if (postPostFunction is None):
				self.postPostFunction = ub_utils._passFunction
			else:
				self.postPostFunction = postPostFunction
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in Timelapse init: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def _thread_Timelapse(self):
		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.timelapse['default] 
		'''

		# Add a delayed start
		time.sleep(self.delayStartSec)

		# Create directory (if it does not already exist)
		if (not os.path.exists(self.outputDir)):
			print('Directory {} does not exist.  Making it now.'.format(self.outputDir))            
			os.makedirs(self.outputDir, exist_ok=True)
		
		startTime = time.time()
		
		self.isThreadActive = True

		while self.camObject.camOn:
			try:
				timeNow = time.time()
							
				# Save Photo self.camObject.getFrameCopy( change res )
				self.camObject.takePhotoLocal(path=self.outputDir, filename=None, resOption=self.resOption)
				# (roiSuccess, roiBox) = ub_utils.roiTrack(self.roiTracker, self.camObject.getFrameCopy())
				
				# Add detection info to deque:
				# self.deque.append({'success': roiSuccess, 'box': roiBox, 'color': self.color})
	
				# In other threads, this is where we do post-processing (per capture).
				# For timelapse, we have postPostProcessing (after thread ends)
				# Do some post-processing:
				# self.postFunction()
				
				# self.camObject.calcFramerate(self.fps, 'roi')

				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in Timelapse {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping Timelapse {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
								
			# Check for hitting time limit	
			if (self.timeLimitSec is not None):
				if ((time.time() - startTime) >= self.timeLimitSec):
					self.stop()
					self.camObject.logger.log(f'Stopping Timelapse {self.idName} thread; time limit reached', severity=ub_utils.SEVERITY_INFO)
					break
			
		# If while loop stops, shut down timelapse:
		self.stop()
		
	def start(self):
		"""Start timelapse capture thread with configured intervals.

		Launches a daemon thread that waits for the initial delay, creates the output
		directory if needed, and begins capturing photos at regular intervals. The
		thread automatically stops when the time limit is reached (if specified) or
		when explicitly stopped.
		"""
		try:
			# Not using decorations deque
			'''
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})
			'''

			self.camObject.logger.log(f'Starting Timelapse thread {self.idName} at {self.threadSleep} sec between photos', severity=ub_utils.SEVERITY_INFO)

			tlThread = threading.Thread(target=self._thread_Timelapse, args=())
			tlThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			tlThread.start()
		except Exception as e:
			self.camObject.logger.log(f'Error in Timelapse start: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
	def stop(self):
		"""Stop timelapse capture thread.

		Signals the capture thread to terminate after completing the current photo.
		The post-processing callback is invoked after the thread stops. Safe to call
		multiple times.
		"""
		try:
			if (self.idName in self.camObject.timelapse):

				self.camObject.logger.log(f'Stopping timelapse {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				# self.deque.clear()
			else:
				self.camObject.logger.log(f'In stop, timelapse {self.idName} name is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in timelapse stop: {e}.', severity=ub_utils.SEVERITY_ERROR)	

		
			

class _ROI():
	"""Internal region-of-interest tracking feature class using OpenCV trackers.

	This class tracks a specified rectangular region in camera frames using OpenCV's
	object tracking algorithms. The tracker continuously updates the ROI position as
	objects move, running in a separate thread at a specified frame rate.

	Attributes:
		camObject: Parent Camera instance managing this ROI tracker.
		idName (str): Unique identifier for this ROI tracker instance.
		roiBB (tuple): Initial bounding box as (x, y, width, height).
		roiTracker: OpenCV tracker object for ROI tracking.
		res_rows (int): Vertical resolution in pixels (must match camera).
		res_cols (int): Horizontal resolution in pixels (must match camera).
		fps_target (float): Target tracking rate in frames per second.
		postFunction (callable): Callback function invoked after each tracking update.
		color (tuple): RGB color for visualization of tracked ROI.
		deque (collections.deque): Thread-safe storage for latest tracking results.
		fps (dict): Frame rate tracking metrics for this tracker.
		isThreadActive (bool): Flag indicating if tracking thread is running.

	Key Methods:
		start(): Launches ROI tracking thread.
		stop(): Terminates tracking thread and cleans up resources.

	Note:
		ROI tracking requires the camera resolution to remain constant. The thread
		will terminate if resolution changes are detected.
	"""
	def __init__(self, camObject, idName, roiTrackerName, roiBB, fps_target, postFunction, color):
		"""Initialize region-of-interest tracking feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Unique identifier for this ROI tracker.
			roiTrackerName (str): Name of OpenCV tracker algorithm to use.
			roiBB (tuple): Initial bounding box as (x, y, width, height).
			fps_target (float): Target tracking rate in Hz.
			postFunction (callable): Callback executed after each tracking update.
			color (tuple): RGB color tuple for ROI box visualization.
		"""
		try:
			self.camObject = camObject  # This is the parent!
						
			self.idName     = idName
			self.decorationID = None
							
			self.roiBB      = roiBB  #  (x, y, w, h)
			self.roiTracker = ub_utils.OPENCV_OBJECT_TRACKERS[roiTrackerName]()
			self.roiTracker.init(self.camObject.getFrameCopy(), self.roiBB)

			# We must maintain same resolution as the camera feed.
			self.res_rows = self.camObject.res_rows
			self.res_cols = self.camObject.res_cols		
			self.resolution = f'{self.res_cols}x{self.res_rows}'

			self.fps_target  = fps_target		# Hz
			self.threadSleep = 1/fps_target		# seconds
				
			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction

			self.color = color
			
			self.fps = _make_fps_dict(recheckInterval=5)

			self.deque = deque(maxlen=1)
			self.deque.append({'success': False, 'box': [], 'color': self.color})
								
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in ROI init: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def _decorate(self, img, **kwargs):
		if (self.deque[0]['success']):
			ub_utils.roiDrawBox(img, self.deque[0]['box'], self.deque[0]['color'])
		
	def _thread_ROI(self):
		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.roi['default] 
		'''
		self.isThreadActive = True

		while self.camObject.camOn:
			try:
				timeNow = time.time()
							
				# FIXME -- It would be nice to cut out the `if` statements...
				
				# Throttle things if we're going faster than capture speed
				if (self.fps.actual >= self.camObject.fps['capture'].actual):
					with self.camObject.condition:
						self.camObject.condition.wait(1)   # added a timeout, just to keep from getting permanently stuck here

				# This won't work if cam resolution has changed.
				if ((self.res_cols, self.res_rows) != (self.camObject.res_cols, self.camObject.res_rows)):
					raise Exception('Resolution changed. Stopping ROI thread')
					# self.stop()
					# break
				
				(roiSuccess, roiBox) = ub_utils.roiTrack(self.roiTracker, self.camObject.getFrameCopy())
				
				# Add detection info to deque:
				self.deque.append({'success': roiSuccess, 'box': roiBox, 'color': self.color})
	
				# Do some post-processing:
				self.postFunction()
				
				self.camObject.calcFramerate(self.fps, 'roi')

				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in ROI {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping ROI {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
				
		# If while loop stops, shut down roi:
		self.stop()
	
	
	def start(self):
		"""Start ROI tracking thread at the configured frame rate.

		Launches a daemon thread that continuously tracks the region of interest
		across frames, updating the bounding box position. The thread automatically
		throttles itself to match the camera's capture rate. Registers a decoration
		function to visualize the tracked ROI on the video stream.
		"""
		try:
			'''
			# Add 'default' to self.decorations['roi']
			if (self.idName not in self.camObject.decorations['roi']):
				self.camObject.decorations['roi'].append(self.idName)
			'''
			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

			self.camObject.logger.log(f'Starting ROI thread {self.idName} at {self.fps_target} fps', severity=ub_utils.SEVERITY_INFO)

			roiThread = threading.Thread(target=self._thread_ROI, args=())
			roiThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			roiThread.start()
		except Exception as e:
			self.camObject.logger.log(f'Error in ROI start: {e}.', severity=ub_utils.SEVERITY_ERROR)
				
		
	def stop(self):
		"""Stop ROI tracking thread and clean up resources.

		Signals the tracking thread to terminate, removes associated decorations,
		and clears the tracking deque. Safe to call multiple times.
		"""
		try:
			if (self.idName in self.camObject.roi):
				'''
				# Remove idName from self.camObject.decorations['roi']
				if (self.idName in self.camObject.decorations['roi']):
					self.camObject.decorations['roi'].remove(self.idName)
				'''
				self.camObject.dec['dequeRemove'].append(self.decorationID)

				self.camObject.logger.log(f'Stopping ROI {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				self.deque.clear()
			else:
				self.camObject.logger.log(f'In stop, ROI {self.idName} name is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in ROI stop: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
	
	def edit(self):
		self.camObject.logger.log('Sorry, ROI editing is not supported.', severity=ub_utils.SEVERITY_WARNING)


class _Ultralytics():
	"""Internal Ultralytics YOLO feature class for object detection and tracking.

	This class performs real-time object detection, classification, segmentation, pose
	estimation, or tracking using Ultralytics YOLO models. Processing runs continuously
	in a separate thread at a specified frame rate, supporting various YOLO tasks.

	Attributes:
		camObject: Parent Camera instance managing this YOLO feature.
		idName (str): Task identifier ("detect", "classify", "pose", "obb", "track", "segment").
		model_name (str): YOLO model filename (e.g., "yolo11n.pt", "yolo11n-seg.pt").
		model: Loaded Ultralytics YOLO model instance.
		res_rows (int): Target vertical resolution in pixels.
		res_cols (int): Target horizontal resolution in pixels.
		fps_target (float): Target inference rate in frames per second.
		postFunction (callable): Callback function invoked after each inference cycle.
		postFunctionArgs (dict): Arguments passed to the post-processing callback.
		color (tuple): RGB color for visualization of detections.
		conf_threshold (float): Minimum confidence threshold for detections.
		verbose (bool): Whether to print detailed inference information.
		drawBox (bool): Whether to draw bounding boxes on detections.
		drawLabel (bool): Whether to draw class labels on detections.
		maskOutline (bool): Whether to draw mask outlines for segmentation.
		deque (collections.deque): Thread-safe storage for latest inference results.
		fps (dict): Frame rate tracking metrics for this feature.
		isThreadActive (bool): Flag indicating if inference thread is running.

	Key Methods:
		start(): Launches YOLO inference thread.
		stop(): Terminates inference thread and cleans up resources.
	"""
	def __init__(self, camObject, idName, res_rows, res_cols, fps_target, postFunction, postFunctionArgs, color, conf_threshold, model_name, verbose, drawBox, drawLabel, maskOutline):
		"""Initialize Ultralytics YOLO feature.

		Args:
			camObject: Parent Camera instance.
			idName (str): Task type ("detect", "classify", "pose", "obb", "track", "segment").
			res_rows (int): Target vertical resolution in pixels.
			res_cols (int): Target horizontal resolution in pixels.
			fps_target (float): Target inference rate in Hz.
			postFunction (callable): Callback executed after each inference cycle.
			postFunctionArgs (dict): Arguments for post-processing callback.
			color (tuple): RGB color tuple for visualization.
			conf_threshold (float): Minimum confidence (0.0-1.0) for detections.
			model_name (str): YOLO model filename to load.
			verbose (bool): Enable detailed inference logging.
			drawBox (bool): Draw bounding boxes on detections, or None for auto.
			drawLabel (bool): Draw class labels on detections, or None for auto.
			maskOutline (bool): Draw mask outlines for segmentation tasks.
		"""
		self.camObject = camObject  # This is the parent!

		try:
			from ultralytics import YOLO
		except Exception as e:
			self.camObject.logger.log(f'Error in ultralytics import: {e}.', severity=ub_utils.SEVERITY_ERROR)
			return
			
		try:												
			self.idName   = idName          # "detect", "classify", "pose", "obb", "track", or "segment"
			self.model_name = model_name    # "yolo11n.pt", "yolo11n-cls.pt", etc
			self.model = YOLO(model_name)
			self.verbose = verbose
			if (drawBox is None): 
				if (idName == 'pose'):
					self.drawBox = False
				else:
					self.drawBox = True
			else:
				self.drawBox = drawBox	
			if (drawLabel is None):
				self.drawLabel = self.drawBox
			else:
				self.drawLabel = drawLabel	
			self.maskOutline = maskOutline
						
			self.decorationID = None   # FIXU -- What will this be?
			
			self.res_rows = res_rows
			self.res_cols = res_cols		
			self.resolution = f'{res_cols}x{res_rows}'
			
			self.fps_target  = fps_target		# Hz
			self.threadSleep = 1/fps_target		# seconds
				
			self.postFunctionArgs = postFunctionArgs
			self.postFunctionArgs['idName'] = idName
			if (postFunction is None):
				self.postFunction = ub_utils._passFunction
			else:
				self.postFunction = postFunction

			self.color = color
			
			self.conf_threshold = conf_threshold
			
			self.fps = _make_fps_dict(recheckInterval=5)

			self.deque = deque(maxlen=1)
			self.deque.append(self._initDeque()) 
			
			self.isThreadActive = False

		except Exception as e:
			self.camObject.logger.log(f'Error in ultralytics init: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def _decorate(self, img, **kwargs):
		# print('idName:', idName, 'ultralytics[idName]:', self.ultralytics[idName].deque[0])
		# print(self.ultralytics[idName].deque[0])
		ub_utils.decorateUltralytics(img, self.res_cols, self.res_rows, self.idName, self.deque[0], self.drawBox, self.drawLabel, self.maskOutline)
		# FIXU -- Needs to match deque as defined in __init__

	def _initDeque(self):
		return {'class': [], 'class_conf': [], 'is_track': False, 'id': [], 
				'xywh': [], 'xyxy': [],
				'xywhr': [], 'xyxyxyxy': [],  
				'keypoints': [], 'keypoints_conf': [],  
				'masks_data': [], 'masks_xy': []}

	def _to_np(self, x):
		'''
		Converts Cuda tensor to numpy array
		Tensor -> NumPy on CPU; passthrough for NumPy arrays.
		'''			
		if isinstance(x, np.ndarray):
			return x
		else:
			return x.detach().cpu().numpy()
		
		# I'm trying to avoid importing torch	
		# if isinstance(x, torch.Tensor):
		#	return x.detach().cpu().numpy()

		raise TypeError(f"Unsupported type: {type(x)}")
		
		
	def _processResults(self, results):
		dequeInfo = self._initDeque()

		np_res  = np.array([self.res_cols, self.res_rows])
		np_res2 = np.array([self.res_cols, self.res_rows, self.res_cols, self.res_rows])
		 
		if results[0].boxes is not None:		
			bx = results[0].boxes 
			dequeInfo['xywh'] = (self._to_np(bx.xywhn)*(np_res2)).astype(int).tolist()
			# dequeInfo['xywhn'] = bx.xywhn.tolist()
			# dequeInfo['xywhr'] = []
			dequeInfo['xyxy'] = (self._to_np(bx.xyxyn)*(np_res2)).astype(int).tolist()
			# dequeInfo['xyxyn'] = bx.xyxyn.tolist()
			# dequeInfo['xyxyxyxy'] = []
		elif results[0].obb is not None:
			bx = results[0].obb
			# dequeInfo['xywh'] = []
			dequeInfo['xywhr'] = self._to_np(bx.xywhr).tolist()    # This is the center point of obb, in original resolution
			# dequeInfo['xywhrn'] = bx.xywhrn.tolist()  # There's no such thing as `xywhrn` 
			# dequeInfo['xyxy'] = []
			dequeInfo['xyxyxyxy'] = (self._to_np(bx.xyxyxyxyn)*(np_res)).astype(int).tolist()
			# dequeInfo['xyxyxyxyn'] = bx.xyxyxyxyn.tolist()

		else:
			bx = None
			'''
			dequeInfo['class'] = []
			dequeInfo['class_conf'] = []
			dequeInfo['is_track'] = False 
			dequeInfo['id'] = [] 
			dequeInfo['xywh'] = []
			dequeInfo['xywhr'] = []
			dequeInfo['xyxy'] = []
			dequeInfo['xyxyxyxy'] = []
			'''
						
		if bx is not None:
			dequeInfo['class'] = [results[0].names.get(key) for key in bx.cls.tolist()]
			dequeInfo['class_conf'] = bx.conf.tolist()
			dequeInfo['is_track'] = bx.is_track 
			dequeInfo['id'] = bx.id.tolist() if bx.id is not None else []
		
		if (results[0].keypoints is not None):
			# dequeInfo['keypoints'] = results[0].keypoints.xyn.tolist() if results[0].keypoints.xyn is not None else []
			# dequeInfo['keypoints'] = (results[0].keypoints.xyn*np_res).int().tolist() if results[0].keypoints.xyn is not None else []
			dequeInfo['keypoints'] = np.array(results[0].keypoints.xyn*np_res).astype(int) if results[0].keypoints.has_visible else []
			dequeInfo['keypoints_conf'] = results[0].keypoints.conf.tolist() if results[0].keypoints.conf is not None else []
		# else:
		# 	dequeInfo['keypoints'] = [] 
		#	dequeInfo['keypoints_conf'] = []

		if (results[0].masks is not None):
			for i in range(0, len(results[0].masks.data)):
				dequeInfo['masks_data'].append(
					cv2.resize(np.array(results[0].masks.data[i]), np_res, interpolation=cv2.INTER_LINEAR).round())  
				dequeInfo['masks_xy'].append((results[0].masks.xyn[i]*np_res).astype(int)) 
		# else:
		#	dequeInfo['masks_data'] = [] 
		#	dequeInfo['masks_xy'] = []
		
		return(dequeInfo)
		
	def _thread_Ultralytics(self):

		'''
		THIS IS A THREAD
		rate is in [Hz] (frames/second)
		self.camObject is the parent (from Camera).
		We are in self.camObject.ultralytics[idName] 
		'''
		self.isThreadActive = True

		while self.camObject.camOn:
			try:
				timeNow = time.time()
							
				# FIXME -- It would be nice to cut out the `if` statements...
				
				# Throttle things if we're going faster than capture speed
				if (self.fps.actual >= self.camObject.fps['capture'].actual):
					with self.camObject.condition:
						self.camObject.condition.wait(1)   # added a timeout, just to keep from getting permanently stuck here


				# Predict or Track?
				if (self.idName == 'track'):
					results = self.model.track(self.camObject.getFrameCopy(), stream=False, persist=True, conf=self.conf_threshold, verbose=self.verbose) 
				else:
					results = self.model.predict(self.camObject.getFrameCopy(), stream=False, conf=self.conf_threshold, verbose=self.verbose) 
					# FIXME -- Can also specify a subset of classes/objects to detect.
					# See https://docs.ultralytics.com/modes/predict/#inference-arguments
								
				# Process the results
				dequeInfo = self._processResults(results)
				
				# Add detection info to deque:
				self.deque.append(dequeInfo)
								
				# Do some post-processing:
				self.postFunctionArgs['results'] = results
				self.postFunction(self.postFunctionArgs)
				
				self.camObject.calcFramerate(self.fps, 'ultralytics') 

				self.camObject.reachback_pubCamStatus()
			except Exception as e:
				self.stop()
				self.camObject.logger.log(f'Error in ultralytics {self.idName} thread: {e}', severity=ub_utils.SEVERITY_ERROR)				
				break
	
			if (not self.isThreadActive):
				self.stop()
				self.camObject.logger.log(f'Stopping ultralytics {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)
				break
	
			# Simplified version of rospy.sleep
			delta = max(0, timeNow + self.threadSleep - time.time())
			if (delta > 0):
				time.sleep(delta)
				
		# If while loop stops, shut down ultralytics:
		self.stop()

	def edit(self, *args, **kwargs):
		self.camObject.logger.log('Sorry, ultralytics editing is not yet supported.', severity=ub_utils.SEVERITY_WARNING)

	def start(self):
		"""Start YOLO inference thread at the configured frame rate.

		Launches a daemon thread that continuously runs YOLO inference on camera
		frames, processes results, and stores detections. The thread automatically
		throttles itself to match the camera's capture rate. Registers a decoration
		function to visualize detections with boxes, labels, masks, or keypoints
		depending on the task type.
		"""
		try:
			self.camObject.logger.log(f'Starting Ultralytics {self.idName} thread at {self.fps_target} fps', severity=ub_utils.SEVERITY_INFO)

			ultraThread = threading.Thread(target=self._thread_Ultralytics, args=())
			ultraThread.daemon = True    # Allows your main script to exit, shutting down this thread, too.
			ultraThread.start()

			# Add to decorations deque
			# FIXME -- Maybe we don't necessarily want to decorate?
			self.decorationID = int(time.time()*1000)
			self.camObject.dec['dequeAdd'].append({'function': self._decorate, 'idName': self.idName, 'decorationID': self.decorationID})

		except Exception as e:
			self.camObject.logger.log(f'Error in ultralytics start: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def stop(self):
		"""Stop YOLO inference thread and clean up resources.

		Signals the inference thread to terminate, removes associated decorations,
		and clears the detection deque. Safe to call multiple times.
		"""
		try:
			if (self.idName in self.camObject.ultralytics):
				'''
				# Remove idName from self.camObject.decorations['ultralytics']
				if (self.idName in self.camObject.decorations['ultralytics']):
					self.camObject.decorations['ultralytics'].remove(self.idName)
				'''
				self.camObject.dec['dequeRemove'].append(self.decorationID)

				self.camObject.logger.log(f'Stopping Ultralytics {self.idName} thread.', severity=ub_utils.SEVERITY_INFO)

				self.isThreadActive = False
				self.deque.clear()
			else:
				self.camObject.logger.log(f'In stop, ultralytics {self.idName} dictionary is not defined', severity=ub_utils.SEVERITY_ERROR)
		except Exception as e:
			self.camObject.logger.log(f'Error in ultralytics stop: {e}.', severity=ub_utils.SEVERITY_ERROR)

				
class _make_fps_dict():
	"""Internal frame rate tracking utility class.

	This simple data structure tracks frame rate metrics for camera capture and
	feature threads. It maintains a frame counter, start time, and computed actual
	frame rate, with periodic recalculation based on the recheck interval.

	Attributes:
		numFrames (int): Cumulative count of processed frames.
		startTime (datetime): Timestamp when frame counting began.
		actual (float): Current computed frame rate in frames per second.
		recheckInterval (float): Time interval in seconds between FPS recalculations.
	"""
	def __init__(self, startTime=datetime.datetime.now(), recheckInterval=5):
		"""Initialize frame rate tracking dictionary.

		Args:
			startTime (datetime): Initial timestamp for FPS calculation.
			recheckInterval (float): Seconds between FPS recalculations. Defaults to 5.
		"""
		self.numFrames       = 0
		self.startTime       = startTime
		self.actual          = 0
		self.recheckInterval = recheckInterval  # [seconds]

		
		
class StreamingHandler(server.BaseHTTPRequestHandler):
	"""HTTP request handler for MJPEG video streaming.

	Handles HTTP GET requests for the /stream.mjpg endpoint, providing real-time
	MJPEG (Motion JPEG) video streaming from the camera. Supports IP allowlisting
	and blocklisting for access control.

	The handler waits for new frames from the camera using threading conditions,
	applies decorations (ArUco markers, bounding boxes, etc.), and streams frames
	as a multipart HTTP response.

	Attributes:
		camObject: Parent Camera instance providing frames and configuration.

	Endpoints:
		/stream.mjpg: MJPEG video stream endpoint
	"""
	# See https://stackoverflow.com/questions/21631799/how-can-i-pass-parameters-to-a-requesthandler
	def __init__(self, camObject, *args, **kwargs):
		"""Initialize the streaming handler with a camera object.

		Args:
			camObject: Camera instance to stream from.
			*args: Positional arguments passed to BaseHTTPRequestHandler.
			**kwargs: Keyword arguments passed to BaseHTTPRequestHandler.

		Note:
			BaseHTTPRequestHandler calls do_GET inside __init__, so the camObject
			must be set before calling super().__init__().
		"""
		self.camObject = camObject   # This is an instance of one of our camera classes (like CamUSB)
		# BaseHTTPRequestHandler calls do_GET **inside** __init__ !!!
		# So we have to call super().__init__ after setting attributes.
		super().__init__(*args, **kwargs)
			
	def _error(self):
		self.send_error(404)
		self.end_headers()
					 
	def do_GET(self):
		# print(f'DEBUG: path? {self.path}')
		print(f'DEBUG: clientIP: {self.client_address}')
		if (len(self.camObject.ipAllowlist) > 0):
			if (self.client_address[0] not in self.camObject.ipAllowlist):
				self._error()
		elif (len(self.camObject.ipBlocklist) > 0):
			if (self.client_address[0] in self.camObject.ipBlocklist):
				self._error()
		elif (self.path == '/stream.mjpg'):
			self.send_response(200)
			self.send_header('Age', 0)
			self.send_header('Cache-Control', 'no-cache, private')
			self.send_header('Pragma', 'no-cache')
			self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
			self.end_headers()
			try:
				self.camObject.streamIncr(+1)
				# print(f'DEBUG: keepStreaming? {self.camObject.keepStreaming}')
				while self.camObject.keepStreaming:
					with self.camObject.condition:
						success = self.camObject.condition.wait(STREAM_MAX_WAIT_TIME_SEC)
					
					# We don't get here until the wait condition has finished 
					if (success):
						# Must use a copy if we decorate the frame.
						# Otherwise, our vision processing functions get messed up.
						# myNumpyArray = np.frombuffer(self.camObject.frame, dtype=np.uint8).reshape(self.camObject.res_rows, self.camObject.res_cols, 3)
						myNumpyArray = np.frombuffer(self.camObject.getFrameCopy(), dtype=np.uint8).reshape(self.camObject.res_rows, self.camObject.res_cols, 3)
							
						# Add annotions/decorations
						# updates myNumpyArray in-place
						self.camObject.decorateFrame(myNumpyArray)
															
						frame = cv2.imencode('.jpg',myNumpyArray)[1]
							
						self.wfile.write(b'--FRAME\r\n')
						self.send_header('Content-Type', 'image/jpeg')
						self.send_header('Content-Length', len(frame))
						self.end_headers()
						self.wfile.write(frame)
						self.wfile.write(b'\r\n')
						
						self.camObject.calcFramerate(self.camObject.fps['stream'], 'stream')
			except Exception as e:
				print("ERROR in do_GET: {}".format(e))
				self.camObject.streamIncr(-1)
				# logging.warning('Removed streaming client %s: %s',self.client_address, str(e))
		else:
			self._error()

class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
	"""Threaded HTTP server for MJPEG video streaming.

	Multi-threaded HTTP server that handles multiple concurrent streaming clients.
	Each client connection is handled in a separate daemon thread.

	Attributes:
		allow_reuse_address (bool): Allow immediate reuse of socket address.
		daemon_threads (bool): All client threads are daemon threads.
	"""
	allow_reuse_address = True
	daemon_threads = True

		
class Camera():
	"""Base class for all camera implementations in the UB camera framework.

	This is an abstract base class that provides common functionality for camera operations
	including frame capture, video streaming, ROS integration, and computer vision features.
	Subclasses (CameraPi, CameraROS, CameraUSB, CameraVoxl) implement hardware-specific
	capture mechanisms while inheriting shared streaming, processing, and publishing capabilities.

	Key Features:
		- HTTPS video streaming with SSL/TLS support
		- ROS topic publishing (raw and compressed image formats)
		- Computer vision modules: ArUco marker detection, barcode/QR code scanning,
		  face detection, ROI tracking, camera calibration, Ultralytics YOLO models
		- Timelapse photography
		- Digital zoom functionality
		- Frame decoration/annotation system
		- Multi-threaded architecture for concurrent operations
		- IP allowlisting/blocklisting for stream access control

	Attributes:
		camOn (bool): Whether the camera is currently active and capturing frames.
		fps (dict): Framerate tracking for 'capture', 'stream', and 'publish' threads.
		fps_target (int): Target framerate for camera capture (from paramDict).
		res_rows (int): Camera resolution height in pixels (from paramDict).
		res_cols (int): Camera resolution width in pixels (from paramDict).
		intrinsics (dict): Camera calibration intrinsics (matrix and distortion coefficients)
			organized by resolution (e.g., '640x480': {'matrix': ndarray, 'dist': ndarray}).
		aruco (dict): Active ArUco marker detection instances keyed by idName.
		roi (dict): Active region-of-interest tracking instances.
		barcode (dict): Active barcode/QR code detection instances.
		calibrate (dict): Active camera calibration instances.
		timelapse (dict): Active timelapse photography instances.
		facedetect (dict): Active face detection instances.
		ultralytics (dict): Active Ultralytics YOLO model instances.
		zoomLevel (float): Current digital zoom level (1.0 = no zoom).
		keepStreaming (bool): Flag to control HTTPS streaming thread.
		keepPublishing (bool): Flag to control ROS publishing thread.
		numStreams (int): Count of active HTTPS stream connections.
		frameDeque (deque): Thread-safe deque holding the most recent captured frame.
		condition (Condition): Threading condition variable for frame synchronization.
		logger (Logger): Logging instance for recording events and errors.
		showFPS (bool): Whether to overlay FPS information on streamed frames.
		ipAllowlist (list): List of IP addresses allowed to access streams (empty = all allowed).
		ipBlocklist (list): List of IP addresses blocked from accessing streams.

	Notes:
		- The base Camera class does not implement frame capture. Subclasses must implement
		  their own capture mechanism and populate frameDeque with numpy arrays.
		- All paramDict keys are automatically converted to class attributes.
		- Computer vision features run in separate threads and can operate concurrently.
		- Streaming uses threading.Condition for efficient frame synchronization.
	"""

	# was `cam_capture_initialize`
	def __init__(self, paramDict, logger=None, sslPath=None, pubCamStatusFunction=None,
				 initROSnode=False, showFPS=True, ipAllowlist=[], ipBlocklist=[]):
		"""Initialize the Camera base class with configuration and optional components.

		Args:
			paramDict (dict): Configuration dictionary containing camera parameters.
				Expected keys include:
				- res_rows (int): Image height in pixels.
				- res_cols (int): Image width in pixels.
				- fps_target (int): Target framerate.
				- intrinsics (dict, optional): Camera calibration data by resolution.
				All keys in paramDict are converted to instance attributes.
			logger (Logger, optional): Logger instance for event recording. If None, creates
				a new ub_utils.Logger instance.
			sslPath (str, optional): Path to SSL certificate directory containing ca.key and
				ca.crt files. Defaults to '{cwd}/ssl'.
			pubCamStatusFunction (callable, optional): Callback function to publish camera
				status updates. If None, uses a no-op function.
			initROSnode (bool): Whether to initialize a ROS node on construction. Default False.
			showFPS (bool): Whether to display FPS overlay on streamed frames. Default True.
			ipAllowlist (list): IP addresses allowed to access streams. Empty list allows all.
			ipBlocklist (list): IP addresses blocked from accessing streams.

		Notes:
			- Camera intrinsics are processed and converted to numpy arrays with 'matrix'
			  and 'dist' (distortion) keys.
			- The frameDeque is initialized as a deque with maxlen=1 to hold only the most
			  recent frame.
			- Computer vision feature dictionaries (aruco, roi, barcode, etc.) are initialized
			  as empty and populated when add*() methods are called.
		"""
		# Here's where we put the stuff that was in __init__ from each specific camera class...

		if (logger):
			self.logger = logger
		else:
			self.logger = ub_utils.Logger()
		# Practice:
		self.logger.log(f'{paramDict}')

		if (sslPath):
			self.sslPath = sslPath
		else:
			self.sslPath = f'{os.getcwd()}/ssl'  # Place where ca.key and ca.crt are saved
			
		# If provided, the pubCamStatus function would be in the "main" script.
		# Otherwise, we'll just call `pass`
		if (pubCamStatusFunction):
			self.reachback_pubCamStatus = pubCamStatusFunction
		else:
			self.reachback_pubCamStatus = ub_utils._passFunction

		
		# Turn keys in a dictionary into class attributes
		# https://stackoverflow.com/questions/1639174/creating-class-instance-properties-from-a-dictionary
		for k, v in paramDict.items():
			setattr(self, k, v)

		# Create dictionaries of camera intrinsics, if info was in paramDict.
		# self.intrinsics['640x480']['matrix'] and self.intrinsics['640x480']['dist']
		# Or, self.intrinsics = {}
		self.intrinsics = self._getIntrinsics()

		# FIXME -- Do some validation on inputs (paramDict keys/values)
		# `res_rows` and `res_cols` must be int values
		# `fps_target` must be positive numeric (realistically, within some limits)
				
		# Info for calculating framerates.
		# NOTE: aruco and roi (and barcode, etc) will be defined separately.
		self.fps = {'capture': _make_fps_dict(recheckInterval=3), 
					'stream':  _make_fps_dict(recheckInterval=3),
					'publish': _make_fps_dict(recheckInterval=5)}
		self.showFPS = showFPS
		
		self.ipAllowlist = ipAllowlist
		self.ipBlocklist = ipBlocklist
				
		self.condition = Condition()		# FIXME -- Can we call this self.frameReadyCondition?  NOTE:  This is referenced by camAutoTakePic...If you change names check there, too.
	
		self.frameDeque = deque(maxlen=1) 

		self.camOn = False		# FIXME -- Group the flags together
		
		self.numStreams	    = 0
		self.keepStreaming  = False
		
		self.keepPublishing = False   # _thread_ros
		self.hasROSnode = False	
			
		self.keepCalibrating = False  # _thread_calibrate
			
		self.zoomLevel    = 1.0
		self.zoomFunction = self._zoomFunction_pass
				
		self.camTopicSubscriber = None    # Used by CameraROS (compressed image callback)

		self.aruco       = {}
		self.roi         = {}
		self.barcode     = {}
		self.calibrate   = {}
		self.timelapse   = {}
		self.facedetect  = {}
		self.ultralytics = {}
		# self.decorations = {'aruco': [], 'roi': [], 'barcode': [], 'calibrate': []}
		self.dec = {'active': [], 'dequeAdd': deque(), 'dequeRemove': deque(), 'dequeEdit': deque()}
 
		if (initROSnode):
			self._init_ros_node()
			
	def _getIntrinsics(self):
		'''
		Clean up self.intrinsics, which is populated from the input parameters dictionary.
		We might have something that looks like:
			self.intrinsics = {'640x480': {'cx': 323.09833463, 'cy': 235.34434675, 'fx': 664.11131483, 'fy': 666.96448353, 
										   'dist': [0.0541, -1.545, 0.003, -0.002, 5.536]}}		
		We'll clean this up (remove cx, cy, fx, fy) and add the camera matrix.
		FIXME -- Should we delete cx, cy, fx, and fy?
		If there are no intrinsics, we'll return an empty dictionary
		'''
		if (hasattr(self, 'intrinsics')):
			intr = {}
			for res in self.intrinsics:
				tmp = {}
				if ('dist' in self.intrinsics[res]):
					tmp['dist'] = np.array(self.intrinsics[res]['dist'])
				if (all(k in self.intrinsics[res] for k in ('fx', 'fy', 'cx', 'cy'))):
					tmp['matrix'] = np.array( [[ self.intrinsics[res]['fx'], 0.0,  self.intrinsics[res]['cx']], 
											   [0.0,  self.intrinsics[res]['fy'],  self.intrinsics[res]['cy']], [0.0, 0.0, 1.0]] )
				if (all(k in tmp for k in ('dist', 'matrix'))):
					intr[res] = tmp
			return intr
		else:
			return {}

	def _init_ros_node(self):
		try:
			rospy.init_node('ub_camera', anonymous=True)
		except Exception as e:
			self.logger.log(f'Error in _init_ros_node: {e}.', severity=ub_utils.SEVERITY_ERROR)			
		else:
			self.hasROSnode = True
			
	def defaultFromNone(self, val, default, test=None):
		"""Return a default value if val is None, optionally applying type conversion.

		Utility method for handling optional parameters with defaults and type coercion.

		Args:
			val: Input value to check. If None, default is returned.
			default: Default value to use when val is None.
			test (type, optional): Type conversion function to apply (e.g., int, float, str).
				If specified, the returned value is cast to this type.

		Returns:
			The value (or default) optionally converted to the specified type.

		Notes:
			- Used internally by add*() methods to handle optional resolution and framerate parameters.
		"""

		try:
			if (val is None):
				val = default
				
			if test in (int, float, str):
				return test(val)
			else: 
				return val				
		except Exception as e:
			# raise Exception(f'Error in defaultFromNone: {e}')
			self.logger.log(f'Error in defaultFromNone: {e}.', severity=ub_utils.SEVERITY_ERROR)

		
	def announceCondition(self):
		# Let our web server (and ros video publisher, and camAuto) know we have a new frame:
		with self.condition:
			self.condition.notify_all()

	def addAruco(self, idName=None, res_rows=None, res_cols=None, fps_target=5, calcRotations=True, postFunction=None, postFunctionArgs={}, configOverrides={}, ids_of_interest=None):
		"""Start ArUco marker detection in a separate thread.

		Creates and starts an _Aruco instance that continuously detects ArUco markers in
		camera frames. Results are stored in self.aruco[idName] and can be accessed by
		other threads. Detected markers can be optionally decorated on streamed frames.

		Args:
			idName (str): Unique identifier for this ArUco detection instance. Must match
				a dictionary in ub_utils.ARUCO_DICT.
			res_rows (int, optional): Processing resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Processing resolution width. Defaults to camera's res_cols.
			fps_target (int): Target detection framerate. Default 5.
			calcRotations (bool): Whether to calculate marker rotation vectors. Default True.
			postFunction (callable, optional): Callback function executed after each detection.
				Receives detection results as arguments.
			postFunctionArgs (dict): Additional keyword arguments passed to postFunction.
			configOverrides (dict): Override default ArUco drawing configuration from
				ub_utils.ARUCO_DRAWING_DEFAULTS.
			ids_of_interest (list, optional): List of specific marker IDs to detect. If None,
				detects all markers.

		Notes:
			- Prevents starting multiple instances with the same idName.
			- Detection results include marker corners, IDs, centers, and optionally rotations.
			- Uses camera intrinsics for undistortion if available.
		"""
		# ids_of_interest None --> we don't have any specific IDs we're looking for.
		# Otherwise, this should be a list of integer IDs we're looking for.

		# Set colors to `None` to use the default colors from ub_utils.ARUCO_DICT
		configDefaults = ub_utils.ARUCO_DRAWING_DEFAULTS
								  
		try:
			if (idName is None):
				self.logger.log('Error in addAruco: idName is None', severity=ub_utils.SEVERITY_ERROR)
				return
			
			if (idName in self.aruco):
				if (self.aruco[idName].isThreadActive):
					self.logger.log(f'An aruco thread for {idName} is already running.', severity=ub_utils.SEVERITY_ERROR)
					return

			configDict = configDefaults
			for k,v in configOverrides:
				configDict[k] = v
				if ('Color' in k):
					configDict[k] = self.defaultFromNone(v, ub_utils.ARUCO_DICT[idName]['color'], None)
					
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
						
			self.aruco[idName] = _Aruco(self, idName, res_rows, res_cols, int(fps_target), calcRotations, postFunction, postFunctionArgs, configDict, ids_of_interest)
			
			self.aruco[idName].start()
				
		except Exception as e:
			self.logger.log(f'Error in addAruco: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def addBarcode(self, res_rows=None, res_cols=None, fps_target=5, postFunction=None, postFunctionArgs={}, color=(0,0,255)):
		"""Start barcode and QR code detection using pyzbar in a separate thread.

		Creates and starts a _Barcode instance that continuously scans for 1D/2D barcodes
		and QR codes in camera frames. Supports multiple barcode formats.

		Args:
			res_rows (int, optional): Processing resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Processing resolution width. Defaults to camera's res_cols.
			fps_target (int): Target detection framerate. Default 5.
			postFunction (callable, optional): Callback function executed after each detection.
			postFunctionArgs (dict): Additional keyword arguments passed to postFunction.
			color (tuple): BGR color for drawing barcode bounding boxes. Default (0,0,255) red.

		Notes:
			- Only one barcode detection instance ('default') is allowed at a time.
			- Detection results include barcode data, type, and corner coordinates.
		"""
		# Start pyzbar to track barcodes/QRcodes
		try:
			# self.barcode is a dictionary.  We'll limit ourselves to just 1 barcode thread. though.
			idName = 'default'
			
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			
			self.barcode[idName] = _Barcode(self, idName, res_rows, res_cols, int(fps_target), postFunction, postFunctionArgs, color)
			self.barcode[idName].start() 

		except Exception as e:
			self.logger.log(f'Error in addBarcode: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def addCalibrate(self, res_rows=None, res_cols=None, secBetweenImages=3, numImages=25, timeoutSec=20, pattern_size=(6,8), square_size=0.0254, postFunction=None):
		"""Start camera calibration process using a checkerboard pattern.

		Creates and starts a _Calibrate instance that captures multiple images of a
		checkerboard pattern to compute camera intrinsics (matrix and distortion coefficients).

		Args:
			res_rows (int, optional): Calibration resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Calibration resolution width. Defaults to camera's res_cols.
			secBetweenImages (int): Seconds to wait between capturing calibration images. Default 3.
			numImages (int): Number of checkerboard images to capture for calibration. Default 25.
			timeoutSec (int): Maximum seconds to wait for calibration completion. Default 20.
			pattern_size (tuple): Checkerboard interior corners (columns, rows). Default (6,8).
			square_size (float): Physical size of checkerboard squares in meters. Default 0.0254
				(1 inch).
			postFunction (callable, optional): Callback function executed after calibration
				completes with results.

		Notes:
			- Only one calibration instance ('default') can run at a time.
			- Checkerboard must be held steady and fully visible in each captured frame.
			- Results include camera matrix, distortion coefficients, and reprojection error.
		"""
		# Start an openCV camera calibration thread
		try:
			# self.calibrate is a dictionary.  We'll limit ourselves to just 1 calibration thread, though.
			idName = 'default'
			
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			
			self.calibrate[idName] = _Calibrate(self, idName, res_rows, res_cols, secBetweenImages, numImages, timeoutSec, pattern_size, square_size, postFunction)
			self.calibrate[idName].start() 

		except Exception as e:
			self.logger.log(f'Error in addCalibrate: {e}.', severity=ub_utils.SEVERITY_ERROR)


	def addFaceDetect(self, res_rows=None, res_cols=None, fps_target=5, postFunction=None, postFunctionArgs={}, color=(0,255,255), conf_threshold=0.7, dnn='caffe', device='cpu', modelPath=None):
		"""Start face detection using OpenCV DNN-based models.

		Creates and starts a _FaceDetect instance that detects faces in camera frames using
		deep neural network models (Caffe or TensorFlow).

		Args:
			res_rows (int, optional): Processing resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Processing resolution width. Defaults to camera's res_cols.
			fps_target (int): Target detection framerate. Default 5.
			postFunction (callable, optional): Callback function executed after each detection.
			postFunctionArgs (dict): Additional keyword arguments passed to postFunction.
			color (tuple): BGR color for drawing face bounding boxes. Default (0,255,255) yellow.
			conf_threshold (float): Minimum confidence threshold for detections. Default 0.7.
			dnn (str): DNN framework to use ('caffe' or 'tensorflow'). Default 'caffe'.
			device (str): Compute device ('cpu' or 'gpu'). Default 'cpu'.
			modelPath (str, optional): Custom path to DNN model files. If None, uses default
				models from ub_utils.

		Notes:
			- Only one face detection instance ('default') can run at a time.
			- Detection results include bounding boxes and confidence scores.
			- Caffe models typically offer better performance on CPU.
		"""
		# Start an openCV DNN-based face detector
		try:
			# self.facedetect is a dictionary.  We'll limit ourselves to just 1 face detection thread. though.
			idName = 'default'
			
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			
			self.facedetect[idName] = _FaceDetect(self, idName, res_rows, res_cols, int(fps_target), postFunction, postFunctionArgs, color, conf_threshold, dnn, device, modelPath)
			self.facedetect[idName].start() 
			
		except Exception as e:
			self.logger.log(f'Error in addFaceDetect: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
	
	def addROI(self, roiTrackerName=None, roiBB=None, fps_target=5, postFunction=None, color=(255,255,255)):
		"""Start region-of-interest (ROI) tracking using OpenCV object trackers.

		Creates and starts an _ROI instance that tracks a specified region across frames
		using OpenCV's tracking algorithms (e.g., KCF, CSRT, MedianFlow).

		Args:
			roiTrackerName (str): OpenCV tracker algorithm name. Examples: 'KCF', 'CSRT',
				'MedianFlow', 'MOSSE'.
			roiBB (tuple): Initial bounding box as (x, y, width, height) in pixels.
			fps_target (int): Target tracking framerate. Default 5.
			postFunction (callable, optional): Callback function executed after each tracking
				update with current bounding box.
			color (tuple): BGR color for drawing tracking box. Default (255,255,255) white.

		Notes:
			- Only one ROI tracking instance ('default') can run at a time.
			- Tracker must be initialized with a valid bounding box.
			- Tracking may fail if object moves out of frame or appearance changes drastically.
		"""
		# Start OpenCV object tracker using the supplied bounding box coordinates
		try:
			if (roiTrackerName is None):
				self.logger.log('Error in addROI: tracker is None', severity=ub_utils.SEVERITY_ERROR)
				return

			if (roiBB is None):
				# This should be an integer 4-tuple, of the form `(x, y, w, h)`
				self.logger.log('Error in addROI: bb is None', severity=ub_utils.SEVERITY_ERROR)
				return
				
			# self.roi is a dictionary.  We'll limit ourselves to just 1 ROI thread. though.
			idName = 'default'
			self.roi[idName] = _ROI(self, idName, roiTrackerName, roiBB, int(fps_target), postFunction, color)
			self.roi[idName].start() 

		except Exception as e:
			self.logger.log(f'Error in addROI: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def addTimelapse(self, outputDir=None, secBetwPhotos=30, timeLimitSec=None, delayStartSec=0, res_rows=None, res_cols=None, postPostFunction=None):
		"""Start automatic timelapse photography to capture images at regular intervals.

		Creates and starts a _Timelapse instance that periodically saves camera frames
		to disk for creating timelapse videos.

		Args:
			outputDir (str): Directory path where timelapse images will be saved. Required.
			secBetwPhotos (int): Seconds between capturing consecutive photos. Default 30.
			timeLimitSec (int, optional): Maximum duration in seconds for timelapse capture.
				If None, runs indefinitely until stopped.
			delayStartSec (int): Seconds to delay before starting timelapse. Default 0.
			res_rows (int, optional): Image resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Image resolution width. Defaults to camera's res_cols.
			postPostFunction (callable, optional): Callback function executed after each
				photo is saved.

		Notes:
			- Only one timelapse instance ('default') can run at a time.
			- Output directory is created if it doesn't exist.
			- Images are saved with timestamp filenames.
		"""
		# Start taking pictures periodically
		try:
			if (outputDir is None):
				self.logger.log('Error in addTimelapse: outputDir is None', severity=ub_utils.SEVERITY_ERROR)
				return
			
			if (timeLimitSec is not None):
				if (timeLimitSec <= 0):
					self.logger.log('Error in addTimelapse: timeLimitSec must be None or a positive number.', severity=ub_utils.SEVERITY_ERROR)
					return

			# self.timelapse is a dictionary.  We'll limit ourselves to just 1 timelapse thread, though.
			idName = 'default'
			
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			
			self.timelapse[idName] = _Timelapse(self, idName, outputDir, secBetwPhotos, timeLimitSec, delayStartSec, res_rows, res_cols, postPostFunction)
			self.timelapse[idName].start() 

		except Exception as e:
			self.logger.log(f'Error in addTimelapse: {e}.', severity=ub_utils.SEVERITY_ERROR)
		

	def addUltralytics(self, idName=None, res_rows=None, res_cols=None, fps_target=None, postFunction=None, postFunctionArgs={}, color=(0,255,255), conf_threshold=0.25, model_name=None, verbose=False, drawBox=None, drawLabel=None, maskOutline=False):
		"""Start Ultralytics YOLO model inference for object detection, segmentation, or pose estimation.

		Creates and starts an _Ultralytics instance that runs YOLO models on camera frames
		for various computer vision tasks.

		Args:
			idName (str): Task type - must be one of: 'detect', 'segment', 'classify',
				'pose', 'obb', or 'track'.
			res_rows (int, optional): Processing resolution height. Defaults to camera's res_rows.
			res_cols (int, optional): Processing resolution width. Defaults to camera's res_cols.
			fps_target (int, optional): Target inference framerate. Defaults to camera's fps_target.
			postFunction (callable, optional): Callback function executed after each inference.
			postFunctionArgs (dict): Additional keyword arguments passed to postFunction.
			color (tuple): BGR color for drawing bounding boxes. Default (0,255,255) yellow.
			conf_threshold (float): Minimum confidence threshold for detections. Default 0.25.
			model_name (str): Ultralytics model filename (e.g., 'YOLO11n.pt', 'YOLO11n-seg.pt').
				Required.
			verbose (bool): Whether to print verbose model output. Default False.
			drawBox (bool, optional): Whether to draw bounding boxes on detections.
			drawLabel (bool, optional): Whether to draw class labels on detections.
			maskOutline (bool): For segmentation, draw mask outlines instead of filled masks.
				Default False.

		Notes:
			- Requires Ultralytics library installation.
			- Model is automatically downloaded if not found locally.
			- Different task types require corresponding model suffixes (e.g., -seg for segmentation).
		"""
		# Start an Ultralytics task ("detect", "segment", "classify", "pose", "obb", "track")
		try:
			if (idName not in ["detect", "segment", "classify", "pose", "obb", "track"]):
				# idName in this context is the same as Ultralytics' "task" description
				self.logger.log('Error in addUltralytics: idName not in ["detect", "segment", "classify", "pose", "obb", "track"]', severity=ub_utils.SEVERITY_ERROR)
				return

			if (model_name is None):
				# model_name should be something like "YOLO11n.pt" or "YOLO11n-cls.pt"
				self.logger.log('Error in addUltralytics: model_name must be specified', severity=ub_utils.SEVERITY_ERROR)
				return
				
			res_rows   = self.defaultFromNone(res_rows,   self.res_rows,   int)
			res_cols   = self.defaultFromNone(res_cols,   self.res_cols,   int)
			fps_target = self.defaultFromNone(fps_target, self.fps_target, int)
			
			self.ultralytics[idName] = _Ultralytics(self, idName, res_rows, res_cols, int(fps_target), postFunction, postFunctionArgs, color, conf_threshold, model_name, verbose, drawBox, drawLabel, maskOutline)
			self.ultralytics[idName].start() 
			
		except Exception as e:
			self.logger.log(f'Error in addUltralytics: {e}.', severity=ub_utils.SEVERITY_ERROR)
				

	# FIXME -- Remove this function
	def setCamFunction(self, functionType, framerate):
		# FIXME -- Allow multiple simultaneous cam modes
		#          Each runs in its own thread			
		if (functionType == 'PRECISION_LAND_ARUCO'):
			'''
			self.camMode     = 'P-LAND'
			'''
			# self.arucoDict and self.arucoParams are set in ????() function?
				

						
	def startStream(self, port):
		"""Start HTTPS video streaming server on the specified port.

		Launches a threaded HTTPS server that streams camera frames as MJPEG to connected
		clients. Uses SSL/TLS encryption with certificates from sslPath.

		Args:
			port (int): TCP port number for the streaming server.

		Notes:
			- Server runs in a daemon thread and will stop when the main program exits.
			- Multiple clients can connect simultaneously (tracked via numStreams).
			- Frames are decorated with overlays (FPS, ArUco markers, etc.) before streaming.
			- IP filtering is applied based on ipAllowlist and ipBlocklist.
		"""
		try:
			self.keepStreaming = True

			strThread = threading.Thread(target=self._thread_stream, args=(port,))
			strThread.daemon = True
			strThread.start()
		except Exception as e:
			# raise Exception(f'Error in startStream: {e}')
			self.keepStreaming = False
			self.logger.log(f'Error in startStream: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def stopStream(self):
		"""Stop the HTTPS video streaming server.

		Sets the keepStreaming flag to False, causing the streaming thread to terminate.
		"""
		try:
			self.keepStreaming = False
		except Exception as e:
			self.logger.log(f'Error in stopStream: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def streamIncr(self, incr):
		try:
			self.numStreams += incr
			self.numStreams = max(0, self.numStreams)

			self.reachback_pubCamStatus() 	
		except Exception as e:
			self.logger.log(f'Error in streamIncr: {e}.', severity=ub_utils.SEVERITY_ERROR)
			
	def startROStopic(self, imgTopic='/camera/image/raw', compImgTopic='/camera/image/compressed'):
		"""Start publishing camera frames to ROS image topics.

		Launches a threaded ROS publisher that converts camera frames to ROS Image and
		CompressedImage messages and publishes them to specified topics.

		Args:
			imgTopic (str, optional): ROS topic for raw Image messages. Default
				'/camera/image/raw'. Set to None to disable raw image publishing.
			compImgTopic (str, optional): ROS topic for CompressedImage messages (JPEG format).
				Default '/camera/image/compressed'. Set to None to disable compressed publishing.

		Notes:
			- Requires ROS node to be initialized (initROSnode=True in constructor).
			- At least one topic (imgTopic or compImgTopic) must be specified.
			- Publisher runs in a daemon thread using cv_bridge for message conversion.
			- Compressed images use JPEG encoding for reduced bandwidth.
		"""
		try:
			if (not self.hasROSnode):
				self.logger.log('No ROS node found.  Initialize camera with initROSnode=True.', severity=ub_utils.SEVERITY_WARNING)
				return

			if (imgTopic == compImgTopic == None):
				self.logger.log('No ROS image topic provided.', severity=ub_utils.SEVERITY_WARNING)
				return

			self.keepPublishing = True
			rosThread = threading.Thread(target=self._thread_ros, args=(imgTopic, compImgTopic,))
			rosThread.daemon = True
			rosThread.start()
		except Exception as e:
			# raise Exception(f'Error in startROStopic: {e}')
			self.logger.log(f'Error in startROStopic: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def stopROStopic(self):
		"""Stop publishing camera frames to ROS topics.

		Sets the keepPublishing flag to False, causing the ROS publishing thread to terminate.
		"""
		self.keepPublishing = False
									

	def getFrame(self):
		"""Return the most recent camera frame without copying.

		Returns:
			numpy.ndarray: The current frame from frameDeque (reference, not a copy).

		Warning:
			Returns a reference to the frame, not a copy. Use getFrameCopy() if you need
			to modify the frame or ensure it won't change.
		"""
		# FIXME Need to do some error checking (can't copy `None`)
		# Maybe wait for condition if frame is currently None?
		return self.frameDeque[0]

	def getFrameNext(self, timeout=1):
		"""Wait for and return the next camera frame.

		Blocks until a new frame is captured or timeout expires.

		Args:
			timeout (int): Maximum seconds to wait for next frame. Default 1.

		Returns:
			numpy.ndarray: The next captured frame (reference, not a copy).

		Notes:
			- Uses threading.Condition to efficiently wait for frame updates.
			- Returns current frame if timeout expires before new frame arrives.
		"""

		with self.condition:
			self.condition.wait(timeout)

		return self.frameDeque[0]


	def _frameCopy(self, frame):
		return frame.copy()
		

	def _frameCopyGray(self, frame):
		# FIXME -- cv2.COLOR_BGR2GRAY?  Do we have RGB or BGR?
		return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY);
	

	def getFrameCopyNext(self, colorOption=None, resOption=None, timeout=1):
		"""Wait for the next camera frame and return a copy with optional transformations.

		Combines the waiting behavior of getFrameNext() with the transformation and copy
		functionality of getFrameCopy().

		Args:
			colorOption (str, optional): Color transformation (see getFrameCopy).
			resOption (tuple, optional): Target resolution (see getFrameCopy).
			timeout (int): Maximum seconds to wait for next frame. Default 1.

		Returns:
			numpy.ndarray: Copy of the next captured frame with transformations applied.
		"""

		with self.condition:
			self.condition.wait(timeout)

		return self.getFrameCopy(colorOption=colorOption, resOption=resOption)	
		 
				
	def getFrameCopy(self, colorOption=None, resOption=None):
		"""Return a copy of the most recent camera frame with optional transformations.

		Args:
			colorOption (str, optional): Color space transformation to apply.
				- None: Return frame in original color space (typically BGR).
				- 'gray': Convert to grayscale.
			resOption (tuple, optional): Target resolution as (width, height) in pixels.
				If specified, frame is resized to this resolution. If None, returns original size.

		Returns:
			numpy.ndarray: Copy of the camera frame with requested transformations applied.

		Notes:
			- Always returns a copy, never the original frame from frameDeque.
			- Color conversion happens before resizing if both options are specified.
			- Resizing uses cv2.resize with default interpolation.
		"""
		# FIXME Need to do some error checking (can't copy `None`) and apply options.
		
		if colorOption == resOption == None:
			# Just return a copy of the current frame
			return self._frameCopy(self.frameDeque[0])

		img = None
		if (colorOption == 'gray'):
			img = self._frameCopyGray(self.frameDeque[0])
				
		if (resOption is not None):
			# resize creates a copy
			if (img is None):
				img = cv2.resize(self.frameDeque[0], resOption)
			else:
				img = cv2.resize(img, resOption)
				
		return img	       	
	
	
	def calcFramerate(self, fpsDict, threadType=None):
		'''
		Find the effective framerate for 'capture', 'stream', or 'publish'.
		Also, works for aruco dictionaries, roi, etc, as long as  
		fpsDict is defined by the _make_fps_dict class.
		Ex:  fpsDict = self.fps['capture']
		threadType is a string:  'capture', 'stream', 'aruco', 'roi', 'barcode', 'publish'
		'''
		try:
			if (fpsDict.numFrames == 0):
				fpsDict.startTime = datetime.datetime.now()
				
			fpsDict.numFrames += 1
			t_elapsed = (datetime.datetime.now() - fpsDict.startTime).total_seconds()
			if (t_elapsed >= fpsDict.recheckInterval):
				if (threadType == 'stream'):
					# Streams are inflating our FPS counts.  Divide by number of streams.
					numStreams = max(self.numStreams, 1)
					fpsDict.actual = int((fpsDict.numFrames / numStreams) / t_elapsed)				
				else:	
					fpsDict.actual = int(fpsDict.numFrames / t_elapsed)
				fpsDict.numFrames = 0
				
				self.reachback_pubCamStatus()
		except Exception as e:
			self.logger.log(f'Error in {threadType} calcFramerate: {e}.', severity=ub_utils.SEVERITY_ERROR)
		
	
	
	def manageDecorationsDeque(self):			
		# Add from decorations request add deque
		while self.dec['dequeAdd']:
			self.dec['active'].append(self.dec['dequeAdd'].popleft())
			
		# Remove from decorations request remove deque
		while self.dec['dequeRemove']:
			decorationID = self.dec['dequeRemove'][0]
			
			for q in self.dec['active']:
				if q['decorationID'] == decorationID:
					self.dec['active'].remove(q)
					break
			
			self.dec['dequeRemove'].popleft()
								
		# Remove from decorations request edit deque
		# This should involve a delete and an add.
		while self.dec['dequeEdit']:
			# First remove, then add.
			idRemove = self.dec['dequeEdit'][0]['decorationID']
			
			for q in self.dec['active']:
				if q['decorationID'] == idRemove:
					self.dec['active'].remove(q)
					break

			self.dec['active'].append(self.dec['dequeEdit'].popleft())

		
	def decorateFrame(self, img):
		'''
		FIXME
		Need a list of *active* decoration types.  e.g., ['aruco', 'calibrate'].
		Then, in this function, we'll simply loop over the names in the list.
		Each name should have a function.
		self._decorateProtoFunc = {'aruco': self._decorateAruco, 'roi': self._decorateROI, ...}
		for name in self.activeDecorators:
			self._decorateProtoFunc[name](img)
		'''
			
		try:
			'''
			if (len(self.decorations['aruco']) > 0):
				for idName in self.decorations['aruco']:
					ub_utils.arucoDrawDetections(img, self.aruco[idName].deque[0]['corners'],
												   self.aruco[idName].deque[0]['ids'], 
												   self.aruco[idName].deque[0]['centers'], 
												   self.aruco[idName].deque[0]['rotations'], self.aruco[idName].config)
			if (len(self.decorations['roi']) > 0):
				for idName in self.decorations['roi']:
					if (self.roi[idName].deque[0]['success']):
						ub_utils.roiDrawBox(img, self.roi[idName].deque[0]['box'], self.roi[idName].deque[0]['color'])

			if (len(self.decorations['barcode']) > 0):
				# print(self.decorations['barcode'])
				for idName in self.decorations['barcode']:
					# print('idName:', idName, 'barcode[idName]:', self.barcode[idName].deque[0])
					# print(self.barcode[idName].deque[0])
					ub_utils.decorateBarcode(img, 
											   self.barcode[idName].deque[0]['corners'], 
											   self.barcode[idName].deque[0]['data'], 
											   self.barcode[idName].deque[0]['color'], addText=True)

			if (len(self.decorations['calibrate']) > 0):
				for idName in self.decorations['calibrate']:
					ub_utils.decorateCalibrate(img, 
												 self.calibrate[idName].deque[0]['checkerboard'], 
												 self.calibrate[idName].deque[0]['corners'], 
												 self.calibrate[idName].deque[0]['count'], 
												 self.calibrate[idName].deque[0]['img_x_y'], 
												 self.calibrate[idName].deque[0]['orig_x_y'], addText=True)
			'''

			'''
			self.dec helps us manage decorations
			self.dec['dequeAdd'] - A deque of decorations to be added.
				This will be a list of dictionaries.  
				Each dictionary should have a the following keys:
				- `decorationID`, whose value should be unique across the deque.
				- `decorationFunction` - A convenience function that will later call the appropriate decorator
					self.aruco[idName]._decorate(img, options)
					ub_utils.decorateText(img, options)
				- `idName`
				
			Allow decorating with text, shapes, etc.	
			'''

			# Add to self.dec['active'] from self.dec['dequeAdd'], 
			# Remove from self.dec['active'] from self.dec['dequeRemove']
			# Edit self.dec['active'] from self.dec['dequeEdit']
			self.manageDecorationsDeque()
			
			for d in self.dec['active']:
				d['function'](img = img, function = d['idName'])

		except Exception as e:
			self.logger.log(f'Error in decorateFrame: {e}.', severity=ub_utils.SEVERITY_ERROR)


		if (self.showFPS):
			cv2.putText(img, f"{str(self.fps['stream'].actual)}/{str(self.fps['capture'].actual)} fps",
						(int(20), int(20)),                             # left, down
						cv2.FONT_HERSHEY_SIMPLEX,
						0.5, (255, 255, 255), 1, cv2.LINE_AA)

		
		# FIXME -- Add some other text:
		# stream/capture fps    ArUco     ROI	
						
	def _thread_ros(self, imgTopic, compImgTopic):
		''' 
		See
		* https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
		* https://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber		
		'''
		
		try:
			if (imgTopic):
				# /camera/image/raw
				bridge = CvBridge()
				image_pub = rospy.Publisher(imgTopic, Image, queue_size=2)
			if (compImgTopic):
				# /camera/image/compressed
				comp_image_pub = rospy.Publisher(compImgTopic, CompressedImage, queue_size=2)
				
			while self.keepPublishing:
				with self.condition:
					success = self.condition.wait(ROSPUB_MAX_WAIT_TIME_SEC)

				# We don't get here until the wait condition has finished 
				if (success):
					'''
					FIXME -- Do we want to allow option to stream decorated frames?
					# Must use a copy if we decorate the frame.
					# Otherwise, our vision processing functions get messed up.
					myNumpyArray = np.frombuffer(self.getFrameCopy(), dtype=np.uint8).reshape(self.res_rows, self.res_cols, 3)
					# FIXME -- Do we really need to do all of this conversion?  Isn't getFrameCopy() sufficient?	
						
					# Add annotions/decorations
					# updates myNumpyArray in-place
					self.decorateFrame(myNumpyArray)
					'''
					myNumpyArray = np.frombuffer(self.getFrame(), dtype=np.uint8).reshape(self.res_rows, self.res_cols, 3)
					# FIXME -- Do we really need to do all of this conversion?  Isn't getFrameCopy() sufficient?					
					
					if (imgTopic):
						image_pub.publish(bridge.cv2_to_imgmsg(myNumpyArray, "bgr8"))	
					if (compImgTopic):
						msg = CompressedImage()
						msg.header.stamp = rospy.Time.now()
						msg.format = "jpeg"
						msg.data = np.array(cv2.imencode('.jpg', myNumpyArray)[1]).tostring()
						# Publish new image
						comp_image_pub.publish(msg)	
						
			self.logger.log('_thread_ros stopping', severity=ub_utils.SEVERITY_DEBUG)
					
		except Exception as e:
			# raise Exception(f'_thread_ros error: {e}')
			self.logger.log(f'_thread_ros error: {e}.', severity=ub_utils.SEVERITY_ERROR)
				
	def _thread_stream(self, portNumber):
		'''
		THIS IS A THREAD
		It starts/runs the streaming server
		'''			
		try:
			try:				
				address = ('', portNumber)
				handler = partial(StreamingHandler, self)				# self --> This CamUSB instance
				server = StreamingServer(address, handler)	
				
				# --- make this server secure (ssl/https) ---
				if ((sys.version_info.major == 3) and (sys.version_info.minor <= 7)):
					# ssl.wrap_socket was deprecated in Python 3.7
					# See https://github.com/eventlet/eventlet/issues/795
					server.socket = ssl.wrap_socket(
						server.socket,
						keyfile  = f'{self.sslPath}/ca.key',
						certfile = f'{self.sslPath}/ca.crt',		
						server_side=True)   
				else:
					# This is the newer way:
					ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
					ssl_context.load_cert_chain(
						keyfile  = f'{self.sslPath}/ca.key',
						certfile = f'{self.sslPath}/ca.crt')
					server.socket = ssl_context.wrap_socket(server.socket, server_side = True)
				# -------------------------------------------
				
				server.serve_forever()	
					
			finally:
				self.logger.log('stopping _thread_stream thread', severity=ub_utils.SEVERITY_INFO)
				# self.stop()
					
		except Exception as e:
			# raise Exception(f'_thread_stream error: {e}')
			self.logger.log(f'_thread_stream error: {e}.', severity=ub_utils.SEVERITY_ERROR)	
			
			
	def _zoomFunction_cv2(self, frame):
		''' 
		Apply digital zoom to input frame
		See `cropAndZoom(self, img)` from aaa_camclasses.py
		'''
		# https://stackoverflow.com/questions/50870405/how-can-i-zoom-my-webcam-in-open-cv-python
		try:

			# Crop
			img = frame[ self.zoomCropYmin:self.zoomCropYmax, self.zoomCropXmin:self.zoomCropXmax, :]

			# Resize to original shape
			# This was *close*, but was a couple of pixels off
			# self.frame = cv2.resize( img, (0, 0), fx=self.zoomLevel, fy=self.zoomLevel)
			frame = cv2.resize( img, (self.res_cols, self.res_rows), interpolation = cv2.INTER_LINEAR)
			
			return frame			
		except Exception as e:
			# raise Exception(f'_zoomFunction_cv2 error: {e}')
			self.logger.log(f'_zoomFunction_cv2 error: {e}.', severity=ub_utils.SEVERITY_ERROR)	
			return frame		# Just return the input?

	def _zoomFunction_pass(self, frame):
		return frame
		
			
	def _changeZoom(self, zoomLevel):
		'''
		This is shared between ROS (sim/clover), USB, and Voxl.  Pi has its own zoom.
		
		We need to set the `zoomCrop...` parameters each time the zoom level changes.
		Then, we crop/resize the image before writing/publishing (in the appropriate thread camClass thread).
		'''
		try:
			w = self.res_cols
			h = self.res_rows
			
			cx = w / 2
			cy = h / 2
			
			self.zoomCropXmin = int(round(cx - w/zoomLevel * 0.5))
			self.zoomCropXmax = int(round(cx + w/zoomLevel * 0.5))
			self.zoomCropYmin = int(round(cy - h/zoomLevel * 0.5))
			self.zoomCropYmax = int(round(cy + h/zoomLevel * 0.5))
						
			self.updateZoom(zoomLevel)
			
		except Exception as e:
			# raise Exception(f'Could not _changeZoom to {zoomLevel}x: {e}.')
			self.logger.log(f'Could not _changeZoom to {zoomLevel}x: {e}.', severity=ub_utils.SEVERITY_ERROR)							
			
			
	
	def updateResolution(self, rows, cols):
		"""Update internal resolution attributes after resolution change.

		This method updates the stored resolution values but does not change the actual
		camera resolution. Subclasses should call this after successfully changing hardware
		resolution settings.

		Args:
			rows (int): New image height in pixels.
			cols (int): New image width in pixels.
		"""
		self.res_rows   = int(rows)	# height
		self.res_cols   = int(cols)	# width

	def updateFramerate(self, framerate):
		"""Update internal framerate attribute after framerate change.

		This method updates the stored framerate value but does not change the actual
		camera framerate. Subclasses should call this after successfully changing hardware
		framerate settings.

		Args:
			framerate (int): New target framerate in frames per second.
		"""
		self.fps_target = int(framerate)

	def updateZoom(self, zoomLevel):
		"""Update internal zoom level and configure zoom processing function.

		This method updates the stored zoom level and selects the appropriate zoom
		processing function. For most cameras (USB, ROS, Voxl), this enables digital
		zoom via frame cropping and resizing. For Raspberry Pi cameras, zoom is handled
		in hardware.

		Args:
			zoomLevel (float): New zoom level (1.0 = no zoom, >1.0 = zoomed in).

		Notes:
			- Zoom levels > 1.01 activate digital zoom processing (_zoomFunction_cv2).
			- Zoom levels <= 1.01 use pass-through (_zoomFunction_pass).
			- Digital zoom crops the center region and resizes to original resolution.
		"""
		self.zoomLevel = zoomLevel

		# Set the zoom function to apply to each frame.
		# This is ignored by picam (it has a one-time zoom adjustment)
		if (self.zoomLevel > 1.01):
			self.zoomFunction = self._zoomFunction_cv2
		else:
			self.zoomFunction = self._zoomFunction_pass

	# was `takePhoto()`
	def takePhotoLocal(self, path=None, filename=None, colorOption=None, resOption=None, timeout=-1):
		"""Capture the current camera frame and save it to local disk as a JPEG image.

		Args:
			path (str, optional): Directory path where image will be saved. If None, saves to
				current working directory. Directory is created if it doesn't exist.
			filename (str, optional): Image filename (without path). If None, generates a
				timestamp-based filename in format 'YYYY-MM-DD_HH-MM-SS.jpg'.
			colorOption (str, optional): Color transformation (see getFrameCopy). Default None.
			resOption (tuple, optional): Target resolution as (width, height) (see getFrameCopy).
				Default None.
			timeout (int): If > 0, waits up to timeout seconds for the next frame before
				capturing. If <= 0, captures the current frame immediately. Default -1.

		Returns:
			tuple: (path, filename) if successful, (None, None) if error occurred.

		Notes:
			- Image is saved in JPEG format using cv2.imwrite.
			- If timeout > 0, uses threading.Condition to wait for next frame update.
			- Automatically creates output directory if it doesn't exist.
		"""
		try:
			if (timeout > 0):			
				myNumpyArray = self.getFrameCopyNext(colorOption=colorOption, resOption=resOption, timeout=timeout)
			else:
				myNumpyArray = self.getFrameCopy(colorOption=colorOption, resOption=resOption)
			
			if (filename is None):
				myTimestamp = datetime.datetime.today()
				# myDate = '{}'.format(myTimestamp.strftime('%Y-%m-%d'))
				# myTime = '{}'.format(myTimestamp.strftime('%H:%M:%S'))
					
				filename = "{}.jpg".format(myTimestamp.strftime('%Y-%m-%d_%H-%M-%S'))
			else:
				filename = filename.strip()
								
			if (path is None):
				path = ''
				pathAndFile = f'{filename}'
			else:
				# Make sure path ends in `/`
				path = ub_utils.setEndingChar(path, '/')
				pathAndFile = f'{path}{filename}'
			
			# Create directory (if it does not already exist)
			if (not os.path.exists(path)):
				print(f'Directory {path} does not exist.  Making it now.')            
				os.makedirs(path, exist_ok=True)
										
			cv2.imwrite(f'{pathAndFile}', myNumpyArray)

			# print(myNumpyArray)
			print(f'Saved image: {pathAndFile}')
			
			return (path, filename)
			
		except Exception as e:
			self.logger.log(f'Error taking photo: {e}', severity=ub_utils.SEVERITY_ERROR)							
			return (None, None)
		
			



class CameraPi(Camera):
	"""Raspberry Pi camera implementation using the picamera package.

	This class provides an interface to Raspberry Pi Camera Module hardware (both
	the original camera module and Camera Module v2) using the picamera Python library.
	It supports hardware-accelerated video encoding, dynamic resolution/framerate changes,
	and native hardware zoom capabilities.

	Key Differences from Base Camera:
		- Uses picamera library instead of OpenCV VideoCapture
		- Implements hardware zoom via picamera.zoom property (no frame cropping needed)
		- Supports dynamic resolution/framerate changes via picamera API
		- Camera frames arrive via the write() callback method (picamera stream interface)
		- Requires picamera package (Raspberry Pi only)

	Hardware Support:
		- Raspberry Pi Camera Module v1 (OV5647)
		- Raspberry Pi Camera Module v2 (IMX219)
		- Raspberry Pi Camera Module v3 (IMX708)
		- Raspberry Pi High Quality Camera (IMX477)

	Usage Example:
		>>> # Basic usage with default settings
		>>> cam = CameraPi()
		>>> cam.start(res_rows=720, res_cols=1280, framerate=30, startStream=True, port=8000)
		>>>
		>>> # Capture and save photo
		>>> path, filename = cam.takePhotoLocal(path='/tmp/photos')
		>>>
		>>> # Change zoom level (hardware zoom)
		>>> cam.changeZoom(2.0)  # 2x zoom
		>>>
		>>> # Change resolution and framerate
		>>> cam.changeResolutionFramerate(res_rows=480, res_cols=640, framerate=15)
		>>>
		>>> # Shutdown camera
		>>> cam.shutdown()

	Important Notes:
		- Requires picamera library: `pip install picamera`
		- Only works on Raspberry Pi hardware with camera modules
		- Hardware zoom is more efficient than digital zoom (no quality loss)
		- Resolution changes require stopping and restarting recording
		- Some parameter combinations may not be supported by hardware

	FIXME FIXME FIXME -- Does picamera actually use `device` anywhere???

	Attributes:
		cap (picamera.PiCamera): The picamera instance controlling the hardware.
		picamera (module): Reference to the imported picamera module.
	"""
	def __init__(self, paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort': 8000},
				device='/dev/video0', apiPref=cv2.CAP_V4L2, logger=None, sslPath=None, pubCamStatusFunction=None,
				imgTopic=None, compImgTopic=None, initROSnode=False, showFPS=True, ipAllowlist=[], ipBlocklist=[]):
		"""Initialize Raspberry Pi camera interface.

		Args:
			paramDict (dict, optional): Configuration dictionary. Defaults to 480x640 @ 30fps.
				Supported keys: 'res_rows', 'res_cols', 'fps_target', 'outputPort'.
			device (str, optional): Device path (not used by picamera). Defaults to '/dev/video0'.
			apiPref (int, optional): API preference (not used by picamera). Defaults to cv2.CAP_V4L2.
			logger (Logger, optional): Logger instance. If None, creates default logger.
			sslPath (str, optional): Path to SSL certificates for HTTPS streaming.
			pubCamStatusFunction (callable, optional): Callback function to publish camera status.
			imgTopic (str, optional): ROS image topic name for publishing raw images.
			compImgTopic (str, optional): ROS compressed image topic name.
			initROSnode (bool, optional): Whether to initialize ROS node. Defaults to False.
			showFPS (bool, optional): Whether to display FPS information. Defaults to True.
			ipAllowlist (list, optional): List of allowed IP addresses for streaming.
			ipBlocklist (list, optional): List of blocked IP addresses for streaming.

		Raises:
			Exception: If picamera library cannot be imported (not on Raspberry Pi).

		Notes:
			- The device and apiPref parameters are accepted for API consistency but not used.
			- Picamera library must be installed and available.
			- Camera hardware must be enabled in raspi-config.
		"""
		try:
			import picamera
			self.picamera = picamera	# We have some namespace issues, since importing module inside class.
			# self.logger.log(f'i think picamera has been imported', severity=ub_utils.SEVERITY_DEBUG)
		except Exception as e:
			# raise Exception(f'Failed to init CameraPi: {e}') 
			# self.logger.log(f'Failed to init CameraPi: {e}', severity=ub_utils.SEVERITY_ERROR)
			print(f'Failed to init CameraPi: {e}')
			
		super().__init__(paramDict, logger, sslPath, pubCamStatusFunction, initROSnode, showFPS, ipAllowlist, ipBlocklist)
	
		self.cap = None	
	
	def _changeFramerate(self, req_framerate):
		try:			
			if (req_framerate == self.fps_target):
				# Nothing to change
				return (True, '')

			# FIXME -- Need to show new framerate in Cesium		
			if (self.fpsMin <= req_framerate <= self.fpsMax):
				delta = req_framerate - self.cap.framerate - self.cap.framerate_delta
				self.cap.framerate_delta += delta				
				self.updateFramerate(self.cap.framerate + self.cap.framerate_delta)
				return (True, '')
			else:
				return (False, 'picam framerate is at limit')
					
		except Exception as e:
			return (False, f'Could not change picam framerate: {e}')

		
		
	def _changeResolution(self, req_height, req_width):
		try:
			if (self.cap.resolution != (req_width, req_height)):
				self.cap.stop_recording()
	
				# FIXME -- Do we need to shut off ROI/ArUco threads?
				# self.setCamFunction(None, None)	

				time.sleep(1)
	
				self.cap.resolution = (req_width, req_height)
	
				time.sleep(1)

				self.updateResolution(req_height, req_width) 
	
				self.cap.start_recording(self, format='bgr')
	
				return (True, '')
			else:
				return (False, f'picam resolution is already {req_width}x{req_height}.')

		except Exception as e:
			return (False, f'Could not change picam resolution to {req_width}x{req_height}: {e}.')
				
		
	def changeZoom(self, zoomLevel):
		"""Change camera zoom level using hardware zoom.

		Uses Raspberry Pi's native hardware zoom capability by setting the picamera.zoom
		property. This is more efficient than digital zoom as it crops the sensor region
		before readout, maintaining full resolution without quality loss.

		Args:
			zoomLevel (float): Zoom level where 1.0 = no zoom, 2.0 = 2x zoom, etc.
				Higher values zoom in more (crop more of the sensor).

		Notes:
			- Hardware zoom is implemented by cropping the sensor region.
			- No post-processing or frame manipulation required.
			- Output resolution remains constant regardless of zoom level.
			- Maximum zoom depends on camera hardware and resolution.
			- Zoom is centered on the frame.

		References:
			https://picamera.readthedocs.io/en/release-1.13/api_camera.html#picamera.PiCamera.zoom
		"""
		# This involves a single-line RPi zoom setting
		# No need to manipulate individual frames in numpy
		try:
			# https://picamera.readthedocs.io/en/release-1.13/api_camera.html?highlight=zoom#picamera.PiCamera.zoom
			# https://forums.raspberrypi.com/viewtopic.php?t=254521

			w = h = min(1/zoomLevel, 1)
			x = y = (1 - w)/2
			self.cap.zoom = (x, y, w, h)
			
			self.updateZoom(zoomLevel)			
		except Exception as e:
			# raise Exception(f'Could not change picam zoomLevel to {zoomLevel}x: {e}.')
			self.logger.log(f'Could not change picam zoomLevel to {zoomLevel}x: {e}', severity=ub_utils.SEVERITY_ERROR)
		
	def changeResolutionFramerate(self, res_rows=None, res_cols=None, framerate=None):
		'''
		Change resolution and/or framerate		
		'''
		try:
			# If user didn't provide a parameter, use the default value
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			framerate = self.defaultFromNone(framerate, self.fps_target, int)
			
			(successFr,  msgFr)  = self._changeFramerate(framerate)			
			(successRes, msgRes) = self._changeResolution(res_rows, res_cols)
				
			if ((not successFr) or (not successRes)):
				raise Exception(f'{msgFr} {msgRes}')	
			
		except Exception as e:
			# raise Exception(f'Failed to change to {res_rows} rows, {res_cols} cols, {framerate} framerate: {e}')
			self.logger.log(f'Failed to change to {res_rows} rows, {res_cols} cols, {framerate} framerate: {e}', severity=ub_utils.SEVERITY_ERROR)
					 
	def shutdown(self):
		"""Shutdown camera and release all resources.

		Stops camera recording, closes the picamera instance, and waits for all streaming
		threads to complete. This is the clean way to release the camera hardware.

		Notes:
			- Calls stop() to halt recording and streaming.
			- Closes the picamera.PiCamera instance to release hardware.
			- Waits STREAM_MAX_WAIT_TIME_SEC + 1 second for threads to finish.
			- Should be called before program exit to properly release camera hardware.
		"""
		try:
			if (self.cap):
				self.stop()	
				self.cap.close()
				time.sleep(STREAM_MAX_WAIT_TIME_SEC + 1)

		except Exception as e:
			# raise Exception(f'Error in camera shutdown: {e}')
			self.logger.log(f'Error in camera shutdown: {e}', severity=ub_utils.SEVERITY_ERROR)
					 
	def start(self, assetID=None, res_rows=None, res_cols=None, framerate=None, startStream=False, port=None, imgTopic=None, compImgTopic=None):
		"""Initialize and start Raspberry Pi camera recording.

		This method creates a picamera.PiCamera instance, configures resolution and framerate,
		starts recording in BGR format, and optionally starts HTTP streaming and/or ROS topic
		publishing. The camera begins capturing frames immediately via the write() callback.

		Args:
			assetID (str, optional): Asset identifier (not used by CameraPi).
			res_rows (int, optional): Image height in pixels. If None, uses value from paramDict.
			res_cols (int, optional): Image width in pixels. If None, uses value from paramDict.
			framerate (int, optional): Target framerate in fps. If None, uses value from paramDict.
			startStream (bool, optional): Whether to start HTTP streaming. Defaults to False.
			port (int, optional): Port number for streaming server. Required if startStream=True.
			imgTopic (str, optional): ROS topic name for publishing raw images.
			compImgTopic (str, optional): ROS topic name for publishing compressed images.

		Raises:
			Exception: If camera cannot be initialized or configured.
			Exception: If startStream=True but port=None.

		Notes:
			- Sets self.camOn = True to indicate camera is active.
			- Camera records continuously in BGR format for OpenCV compatibility.
			- Actual resolution/framerate may differ from requested; check self.res_rows,
			  self.res_cols, and self.fps_target after start.
			- Stream uses HTTPS with SSL certificates from self.sslPath.
		"""
		try:
			# If user didn't provide a parameter, use the default value
			res_rows     = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols     = self.defaultFromNone(res_cols,  self.res_cols,   int)
			framerate    = self.defaultFromNone(framerate, self.fps_target, int)
			port         = self.defaultFromNone(port, self.outputPort)
			# compImgTopic =
					
			self.cap = self.picamera.PiCamera(resolution=f'{res_cols}x{res_rows}', framerate=framerate)		

			# FIXME -- Need to verify that the updates actually went thru
			(width, height) = self.cap.resolution
			self.updateResolution(height, width)
			frate = self.cap.framerate
			self.updateFramerate(frate)
			
			# camera.start_recording(output, format='bgr', splitter_port=2, resize=(320,240))		
			self.cap.start_recording(self, format='bgr')
			
			self.camOn = True
			
			# Start streaming?
			if (startStream):
				if (port is None):
					raise Exception('cannot stream when port is None')
				else:	
					self.startStream(port)
			
			# Start publishing to ROS compressed image topic?
			if ((imgTopic is not None) or (compImgTopic is not None)):
				self.startROStopic(imgTopic=imgTopic, compImgTopic=compImgTopic)	
			
			self.reachback_pubCamStatus()				
		except Exception as e:
			# raise Exception(f'Error in camera start: {e}')
			self.logger.log(f'Error in camera start: {e}', severity=ub_utils.SEVERITY_ERROR)
			
	def stop(self):
		'''
		Stop RPi camera from recording
		'''	
		try:
			self.camOn = False		
			self.cap.stop_recording()		
			self.stopStream()			
		except Exception as e:
			raise Exception(f'Error in camera stop: {e}')
			
		
	def write(self, buf):
		"""Callback method for receiving frames from picamera recording stream.

		This method is called automatically by picamera during recording. It receives
		raw BGR frame data, converts it to a numpy array, and adds it to the frame deque.
		It also triggers frame update notifications and framerate calculations.

		Args:
			buf (bytes): Raw BGR frame data from picamera stream.

		Notes:
			- This is a callback method called by picamera, not meant to be called directly.
			- Frame format is BGR (compatible with OpenCV) with shape (res_rows, res_cols, 3).
			- Each frame is appended to self.frameDeque for access by other threads.
			- Triggers threading.Condition notification for threads waiting on new frames.
			- Automatically calculates and updates capture framerate statistics.

		FIXME: Double check this implementation.
		"""
		try:
			# self.myNumpyArray = np.frombuffer(buf, dtype=np.uint8).reshape(self.res_rows, self.res_cols, 3)
			self.frameDeque.append(np.frombuffer(buf, dtype=np.uint8).reshape(self.res_rows, self.res_cols, 3))
			
			'''
			# Only call this if we actually have optical flow capabilities/hardware
			# if (self.vhcl.useOptFlowCam):
			# 	self.vhcl.optFlowPub(self.myNumpyArray, self.vhcl.oflow.camera_matrix, self.vhcl.oflow.dist_coeffs)
			
			# create a copy, as appropriate?
			
			# FIXME -- NEED TO FIX THESE
			#self.vhcl.asset.camAuto['thread_function'][self.vhcl.asset.camAuto['camName']](self.myNumpyArray)
	
			#self.pub()
			'''
			
			self.announceCondition()
			
			self.calcFramerate(self.fps['capture'], 'capture')
			
		except Exception as e:
			self.logger.log(f'Error writing picam frame: {e}', severity=ub_utils.SEVERITY_ERROR)
			

		return		

class CameraROS(Camera):
	"""ROS camera subscriber/publisher implementation for compressed image topics.

	This class provides an interface to cameras that publish to ROS CompressedImage topics,
	including Gazebo simulation cameras and real hardware cameras running ROS (e.g., Clover
	quadcopter). Instead of directly accessing camera hardware, it subscribes to an existing
	ROS topic and makes frames available through the standard Camera interface.

	Key Differences from Base Camera:
		- No direct hardware access - subscribes to ROS CompressedImage topic
		- Cannot change resolution or framerate (determined by publisher)
		- Digital zoom only (crops and resizes frames in software)
		- Requires active ROS master and camera publisher
		- Frame timing depends on topic publication rate

	Supported Sources:
		- Gazebo simulation cameras (e.g., /sim_cam/image_raw/compressed)
		- Clover main camera (e.g., /main_camera/image_raw/compressed)
		- Optical flow debug camera (e.g., /optical_flow/debug/compressed)
		- Any ROS node publishing sensor_msgs/CompressedImage

	Usage Example:
		>>> # Create ROS camera subscriber
		>>> cam = CameraROS(paramDict={'outputPort': 8001})
		>>> cam.topic = "/main_camera/image_raw/compressed"
		>>>
		>>> # Start subscribing and optionally stream via HTTP
		>>> cam.start(startStream=True, port=8001)
		>>>
		>>> # Get current frame
		>>> frame = cam.getFrameCopy()
		>>>
		>>> # Apply digital zoom
		>>> cam.changeZoom(2.0)
		>>>
		>>> # Stop subscribing
		>>> cam.stop()

	Important Notes:
		- Requires ROS environment and active roscore
		- Topic must be publishing sensor_msgs/CompressedImage messages
		- Resolution/framerate cannot be changed (read-only from topic)
		- Frame availability depends on topic publication rate
		- Use {assetID} placeholder in topic string for multi-robot systems
		- Zoom is digital only (crops and resizes frames)

	Attributes:
		topic (str): ROS topic name to subscribe to for compressed images.
		camTopicSubscriber (rospy.Subscriber): ROS subscriber instance.
	"""
	
	def __init__(self, assetID=None, paramDict={}, logger=None, sslPath=None, pubCamStatusFunction=None, showFPS=True,
				 ipAllowlist=[], ipBlocklist=[]):
		"""Initialize ROS camera subscriber interface.

		Args:
			assetID (str, optional): Asset identifier for formatting topic string (replaces {} placeholder).
			paramDict (dict, optional): Configuration dictionary. Defaults to empty dict.
				Supported keys: 'res_rows', 'res_cols', 'fps_target', 'outputPort'.
			logger (Logger, optional): Logger instance. If None, creates default logger.
			sslPath (str, optional): Path to SSL certificates for HTTPS streaming.
			pubCamStatusFunction (callable, optional): Callback function to publish camera status.
			showFPS (bool, optional): Whether to display FPS information. Defaults to True.
			ipAllowlist (list, optional): List of allowed IP addresses for streaming.
			ipBlocklist (list, optional): List of blocked IP addresses for streaming.

		Notes:
			- Does not connect to ROS topic in __init__ (use start() to begin subscription).
			- Topic string should be set on instance before calling start().
			- Resolution and framerate in paramDict are for reference only; actual values
			  come from the ROS topic publisher.
		"""
		super().__init__(paramDict, logger, sslPath, pubCamStatusFunction, showFPS, ipAllowlist, ipBlocklist)

		# See vehicles.json, which includes a topic for Clover and Sim cameras.
		# In make_asset class we replace {} with the assetID (where applicable)
		# self.topic = "/soar_rover/{}/sim_cam/image_raw/compressed"
		# self.topic = "/main_camera/image_raw/compressed"
		# self.topic = "/optical_flow/debug/compressed"			
		
		# from gazebo_msgs.msg import LinkState
		# from gazebo_msgs.srv import SetLinkState	
		from gazebo_msgs.msg import ODEJointProperties
		from gazebo_msgs.srv import SetJointProperties	

		self.camTopicSubscriber = None
		
	def callback_CompressedImage(self, msg):
		"""Callback method for receiving CompressedImage messages from ROS topic.

		This method is called automatically by rospy when a new CompressedImage message
		arrives on the subscribed topic. It decodes the JPEG data, applies zoom if active,
		and adds the frame to the deque.

		Args:
			msg (sensor_msgs.msg.CompressedImage): ROS CompressedImage message containing
				JPEG-encoded frame data.

		Notes:
			- Automatically decodes JPEG data to BGR numpy array using cv2.imdecode.
			- Applies digital zoom (crop and resize) if zoom level > 1.0.
			- Appends frame to self.frameDeque for access by other threads.
			- Triggers threading.Condition notification for threads waiting on new frames.
			- Calculates and updates capture framerate statistics.
			- This is a callback method, not meant to be called directly.
		"""
		try:
			# FIXME -- Do we need to do all of these conversions???
			
			#### direct conversion to CV2 ####
			# np_arr = np.fromstring(msg.data, np.uint8)
			np_arr = np.frombuffer(msg.data, dtype=np.uint8)  # .reshape(self.res_rows, self.res_cols, 3)
			
			# image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
			frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
			# Are we zooming?
			frame = self.zoomFunction(frame)

			self.frameDeque.append(frame) # OpenCV >= 3.0:
					
			# Only call this if we actually have optical flow capabilities/hardware
			# if (self.vhcl.useOptFlowCam):
			#	self.vhcl.optFlowPub(self.myNumpyArray, self.vhcl.oflow.camera_matrix, self.vhcl.oflow.dist_coeffs)

			self.announceCondition()
			
			self.calcFramerate(self.fps['capture'], 'capture')
			
		except Exception as e:
			# raise Exception(f'Error in sim compressed image callback: {e}')
			self.logger.log(f'Error in sim compressed image callback: {e}', severity=ub_utils.SEVERITY_ERROR)

		
	def _changeFramerate(self, req_framerate):
		try:			
			if (req_framerate == self.fps_target):
				# Nothing to change
				return (True, '')

			# FIXME -- I don't think we can actually change ROS framerate	
			if (self.fpsMin <= req_framerate <= self.fpsMax):
				# Do something here if we can?
				return (False, 'cannot change ROS framerate')
			else:
				return (False, 'ROS framerate is at limit')
					
		except Exception as e:
			return (False, f'Could not change ROS framerate: {e}')
		
	def _changeResolution(self, req_height, req_width):
		try:
			# FIXME -- How to get current actual resolution?
			if ((self.res_cols, self.res_rows) != (req_width, req_height)):
				# FIXME -- I don't think we can actually change ROS resolution
				return (False, 'cannot change ROS resolution')
			else:
				return (False, f'ROS resolution is already {req_width}x{req_height}.')
		except Exception as e:
			return (False, f'Could not change ROS resolution to {req_width}x{req_height}: {e}.')
			
	def changeResolutionFramerate(self, res_rows=None, res_cols=None, framerate=None):
		'''
		Change resolution and/or framerate	
		NOTE: I don't think either is possible with ROS compressed image topic	
		'''
		try:
			# If user didn't provide a parameter, use the default value
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			framerate = self.defaultFromNone(framerate, self.fps_target, int)
			
			(successFr,  msgFr)  = self._changeFramerate(framerate)			
			(successRes, msgRes) = self._changeResolution(res_rows, res_cols)
				
			if ((not successFr) or (not successRes)):
				raise Exception(f'{msgFr} {msgRes}')	
			
		except Exception as e:
			self.logger.log(f'Failed to change to {res_rows} rows, {res_cols} cols, {framerate} framerate: {e}', severity=ub_utils.SEVERITY_ERROR)

	def changeZoom(self, zoomLevel):
		"""Change camera zoom level using digital zoom (crop and resize).

		Applies digital zoom by cropping the center region of each frame and resizing
		to the original resolution. This is done in software for each frame via
		the _changeZoom() method shared with CameraUSB and Voxl cameras.

		Args:
			zoomLevel (float): Zoom level where 1.0 = no zoom, 2.0 = 2x zoom, etc.
				Higher values zoom in more (crop more of the frame).

		Notes:
			- Digital zoom reduces effective resolution (not true optical zoom).
			- Zoom is applied to each frame after receiving from ROS topic.
			- Cannot control zoom at the camera source (Gazebo or hardware).
			- Crop is centered on the frame.
			- For Gazebo cameras, consider changing camera properties in simulation instead.
		"""
		# This requires a numpy zoom/crop for each frame?
		# Or, is it possible to change zoom in Gazebo?
		self._changeZoom(zoomLevel)
			

	def shutdown(self):
		'''
		Might be as simple as calling self.stop()
		'''
		self.stop()
		time.sleep(STREAM_MAX_WAIT_TIME_SEC + 1)
			
			
	def start(self, assetID=None, startStream=False, port=None, **kwargs):
		"""Start subscribing to ROS CompressedImage topic.

		Creates a ROS subscriber to the configured topic and begins receiving camera frames.
		Frames arrive via the callback_CompressedImage() callback method. Optionally starts
		HTTP streaming server to re-broadcast frames.

		Args:
			assetID (str, optional): Asset identifier to format into topic string (replaces {}).
			startStream (bool, optional): Whether to start HTTP streaming. Defaults to False.
			port (int, optional): Port number for streaming server. Required if startStream=True.
			**kwargs: Additional keyword arguments (ignored).

		Raises:
			Exception: If startStream=True but port=None.
			Exception: If ROS topic subscription fails.

		Notes:
			- Sets self.camOn = True to indicate active subscription.
			- If self.topic contains {} placeholder, it's replaced with assetID.
			- Frames are received asynchronously via callback.
			- Does not publish to ROS topics (already reading from one).
			- Stream uses HTTPS with SSL certificates from self.sslPath.
		"""
		try:			
			# If user didn't provide a parameter, use the default value
			port         = self.defaultFromNone(port, self.outputPort)
			
			if (hasattr(self, 'topic')):
				# topic = "/soar_rover/{}/sim_cam/image_raw/compressed"
				# topic = "/main_camera/image_raw/compressed"
				# topic = "/optical_flow/debug/compressed"			
				self.topic = self.topic.format(assetID)
					
			print(self.topic)
					
			self.camOn = True

			self.camTopicSubscriber = rospy.Subscriber(self.topic, CompressedImage, self.callback_CompressedImage)

			# Start streaming?
			if (startStream):
				if (port is None):
					raise Exception('cannot stream when port is None')
				else:	
					self.startStream(port)
			# NOTE: No need to publish to compressed image topic (we're already subscribing to it!)

			self.reachback_pubCamStatus()
		except Exception as e:
			# raise Exception(f'Error in camera start: {e}')
			self.logger.log(f'Error in camera start: {e}.', severity=ub_utils.SEVERITY_ERROR)

	def stop(self):
		try:
			self.stopStream()
			if (self.camTopicSubscriber is not None):
				self.camTopicSubscriber.unregister()
		except Exception as e:
			# raise Exception(f'Could not stop cameraROS: {e}')
			self.logger.log(f'Could not stop cameraROS: {e}', severity=ub_utils.SEVERITY_ERROR)
			
						
			
class CameraUSB(Camera):
	"""USB camera and RTSP/HTTP stream implementation using OpenCV VideoCapture.

	This class provides a versatile interface for various video sources using OpenCV's
	VideoCapture API. Despite the name, it supports USB cameras, video device files,
	RTSP streams, HTTP streams, and other sources that OpenCV can read. It uses a
	threaded capture loop to continuously grab frames.

	Key Differences from Base Camera:
		- Uses cv2.VideoCapture for frame acquisition
		- Supports multiple video backends via apiPref parameter (V4L2, FFMPEG, etc.)
		- Threaded capture via _thread_capture() for continuous frame grabbing
		- Digital zoom only (crops and resizes frames in software)
		- Supports dynamic resolution/framerate changes by restarting capture
		- Can connect to RTSP/HTTP streams (not just local devices)

	Supported Sources:
		- USB webcams (e.g., /dev/video0 with V4L2 backend)
		- Raspberry Pi cameras on Ubuntu (e.g., /dev/video0)
		- RTSP streams (e.g., rtsp://192.168.1.100:8554/stream with apiPref=None)
		- HTTP MJPEG streams (e.g., https://localhost:8000/stream.mjpg with apiPref=None)
		- Video files (supported by OpenCV)
		- VOXL camera feeds (RTSP)

	Usage Example:
		>>> # USB webcam on Linux with V4L2
		>>> cam = CameraUSB(device='/dev/video0', apiPref=cv2.CAP_V4L2)
		>>> cam.start(res_rows=720, res_cols=1280, framerate=30, startStream=True, port=8000)
		>>>
		>>> # RTSP stream from VOXL or IP camera
		>>> cam = CameraUSB(device='rtsp://192.168.1.100:8554/stream', apiPref=None)
		>>> cam.start(startStream=True, port=8001)
		>>>
		>>> # Get frame and save photo
		>>> frame = cam.getFrameCopy()
		>>> path, filename = cam.takePhotoLocal(path='/tmp/photos')
		>>>
		>>> # Change resolution and framerate
		>>> cam.changeResolutionFramerate(res_rows=480, res_cols=640, framerate=15)
		>>>
		>>> # Apply digital zoom
		>>> cam.changeZoom(2.0)
		>>>
		>>> # Stop camera
		>>> cam.shutdown()

	Important Notes:
		- For RTSP/HTTP streams, set apiPref=None to let OpenCV choose backend
		- For USB cameras on Linux, use apiPref=cv2.CAP_V4L2 for best performance
		- Resolution/framerate changes restart the capture thread (brief interruption)
		- Not all cameras support all resolutions or framerates
		- FOURCC codec can be specified for compatible cameras (e.g., 'MJPG')
		- Stream sources may ignore resolution/framerate parameters
		- Zoom is digital only (reduces effective resolution)

	Attributes:
		device (str): Video source path or URL (e.g., '/dev/video0' or 'rtsp://...').
		apiPref (int or None): OpenCV VideoCapture API preference (e.g., cv2.CAP_V4L2).
		fourcc (tuple or None): FOURCC codec as 4-char tuple (e.g., ('M','J','P','G')).
		cap (cv2.VideoCapture): OpenCV VideoCapture instance.
	"""
	
	def __init__(self, paramDict={'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort': 8000}, device='/dev/video0',
		apiPref=cv2.CAP_V4L2, fourcc=None, logger=None, sslPath=None, pubCamStatusFunction=None, imgTopic=None, compImgTopic=None,
		initROSnode=False, showFPS=True, ipAllowlist=[], ipBlocklist=[]):
		"""Initialize USB camera or video stream interface.

		Args:
			paramDict (dict, optional): Configuration dictionary. Defaults to 480x640 @ 30fps.
				Supported keys: 'res_rows', 'res_cols', 'fps_target', 'outputPort', 'device', 'fourcc'.
			device (str, optional): Video source path or URL. Examples:
				- '/dev/video0' for USB camera on Linux
				- 'rtsp://192.168.1.100:8554/stream' for RTSP stream
				- 'https://localhost:8000/stream.mjpg' for HTTP stream
				Defaults to '/dev/video0'.
			apiPref (int or None, optional): OpenCV VideoCapture API preference.
				- cv2.CAP_V4L2 for Linux USB cameras (recommended)
				- cv2.CAP_DSHOW for Windows cameras
				- None to let OpenCV auto-detect (for RTSP/HTTP streams)
				Defaults to cv2.CAP_V4L2.
			fourcc (tuple or None, optional): FOURCC codec as 4-character tuple,
				e.g., ('M','J','P','G') for MJPEG. If None, uses camera default.
			logger (Logger, optional): Logger instance. If None, creates default logger.
			sslPath (str, optional): Path to SSL certificates for HTTPS streaming.
			pubCamStatusFunction (callable, optional): Callback function to publish camera status.
			imgTopic (str, optional): ROS image topic name for publishing raw images.
			compImgTopic (str, optional): ROS compressed image topic name.
			initROSnode (bool, optional): Whether to initialize ROS node. Defaults to False.
			showFPS (bool, optional): Whether to display FPS information. Defaults to True.
			ipAllowlist (list, optional): List of allowed IP addresses for streaming.
			ipBlocklist (list, optional): List of blocked IP addresses for streaming.

		Notes:
			- Does not open camera in __init__ (use start() to begin capture).
			- device and fourcc can also be specified in paramDict.
			- For RTSP/HTTP streams, set apiPref=None for better compatibility.
			- FOURCC codec support depends on camera hardware and drivers.
		"""

		super().__init__(paramDict, logger, sslPath, pubCamStatusFunction, initROSnode, showFPS, ipAllowlist, ipBlocklist)
		
		# FIXME -- Do some validation on inputs (in addition to what is in Camera)
		# `device` must be present (but it could be a key in paramDict??)
		# `apiPref` must be present
		# `fourcc` could be a key in paramDict
		
		if (not hasattr(self, 'device')):
			self.device  = device   
		if (not hasattr(self, 'fourcc')):
			self.fourcc  = fourcc   

		self.apiPref = apiPref
		
		self.cap = None
				
				
	def _thread_capture(self, res_rows, res_cols, framerate, fourcc, device, apiPref):
		"""Threaded capture loop for continuously grabbing frames from video source.

		This thread creates a cv2.VideoCapture instance, configures it with the specified
		parameters, and continuously reads frames in a loop. Frames are added to the
		frame deque for access by other parts of the application. The thread runs until
		self.camOn is set to False.

		Args:
			res_rows (int): Requested image height in pixels.
			res_cols (int): Requested image width in pixels.
			framerate (int): Requested framerate in fps.
			fourcc (tuple or None): FOURCC codec as 4-char tuple (e.g., ('M','J','P','G')).
			device (str): Video source path or URL.
			apiPref (int or None): OpenCV VideoCapture API preference.

		Notes:
			- This is a thread method, not meant to be called directly.
			- Started automatically by start() method.
			- For RTSP/HTTP streams (apiPref=None), resolution/framerate may be ignored.
			- For USB cameras (apiPref=cv2.CAP_V4L2), attempts to set resolution/framerate/codec.
			- Actual resolution/framerate stored in self.res_rows, self.res_cols, self.fps_target.
			- Continuously grabs frames via cap.read() until self.camOn becomes False.
			- Each frame has digital zoom applied if zoom level > 1.0.
			- Frames are appended to self.frameDeque with condition notification.
			- Automatically releases VideoCapture when loop exits.
		"""
		try:	
			# See https://www.simonwenkel.com/notes/software_libraries/opencv/opencv-frame-io.html 						
			'''
			self.cap = cv2.VideoCapture(device, apiPref, 
									(cv2.CAP_PROP_FPS,          int(framerate), 
									 cv2.CAP_PROP_FRAME_WIDTH,  int(res_cols),
									 cv2.CAP_PROP_FRAME_HEIGHT, int(res_rows))) 
			'''

			# Update 2024-02-26.  VOXL cameras use rtsp feeds, which prefer different API and don't use v4l2.
			# So, for VOXLs, set `apiPref = None`
			# FIXME -- Might consider using `apiPref = cv2.CAP_FFMPEG`
			if (apiPref is None):
				self.cap = cv2.VideoCapture(device)
			else:
				params = [cv2.CAP_PROP_FRAME_WIDTH,  int(res_cols), 
						  cv2.CAP_PROP_FRAME_HEIGHT, int(res_rows), 
						  cv2.CAP_PROP_FPS,          int(framerate)]
				if (fourcc is not None):
					fourcc = cv2.VideoWriter.fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3])
					params.extend([cv2.CAP_PROP_FOURCC, fourcc])
					
				self.cap = cv2.VideoCapture(device, apiPref, params=params)
				'''
				self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_rows)
				self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  res_cols)
				self.cap.set(cv2.CAP_PROP_FPS, framerate)
				self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
				'''
				
				# cv2.CAP_PROP_ZOOM, 50.0 does not work on Dell laptop camera

				'''
				See https://www.simonwenkel.com/notes/software_libraries/opencv/opencv-frame-io.html
				self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
				self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_rows)
				self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  res_cols)
				self.cap.set(cv2.CAP_PROP_FPS, framerate)
				codec = cv2.VideoWriter.fourcc('M','J', 'P','G')
				self.cap.set(cv2.CAP_PROP_FOURCC, codec)
				'''

			# FIXME -- Need to verify that the updates actually went thru
			self.updateResolution(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) 
			self.updateFramerate(self.cap.get(cv2.CAP_PROP_FPS))
			
			# self.logger.log(f'_thread_capture: {res_rows}, {res_cols}, {framerate}', severity=ub_utils.SEVERITY_DEBUG)

		except Exception as e:
			# raise Exception(f'CameraUSB capture thread Failed: {e}')
			self.logger.log(f'CameraUSB capture thread Failed: {e}', severity=ub_utils.SEVERITY_ERROR)
			
		else:	
			try:
				print(f'Camera Opened? {self.cap.isOpened()}')
				while(self.cap.isOpened()):
					ret, frame = self.cap.read()
					
					if (ret):
						# Are we zooming?
						frame = self.zoomFunction(frame)

						self.frameDeque.append(frame)					

						self.announceCondition()
						
						self.calcFramerate(self.fps['capture'], 'capture')
										
					if (not self.camOn):
						self.cap.release()
						break

				# If we make it here, unset our flag:
				self.camOn = False
			except Exception as e:
				self.logger.log(f'Ugh - Extra exception in _thread_capture: {e}', severity=ub_utils.SEVERITY_ERROR)
				
			
	def start(self, assetID=None, res_rows=None, res_cols=None, framerate=None, device=None, apiPref=None, startStream=False, port=None, imgTopic=None, compImgTopic=None):
		"""Start camera capture thread and optionally start streaming/publishing.

		Creates and starts a daemon thread running _thread_capture() which opens the video
		source and continuously grabs frames. Optionally starts HTTP streaming server and/or
		ROS topic publishing.

		Args:
			assetID (str, optional): Asset identifier (not used by CameraUSB).
			res_rows (int, optional): Image height in pixels. If None, uses value from paramDict.
			res_cols (int, optional): Image width in pixels. If None, uses value from paramDict.
			framerate (int, optional): Target framerate in fps. If None, uses value from paramDict.
			device (str, optional): Video source path/URL. If None, uses value from __init__.
			apiPref (int or None, optional): OpenCV API preference. If None, uses value from __init__.
			startStream (bool, optional): Whether to start HTTP streaming. Defaults to False.
			port (int, optional): Port number for streaming server. Required if startStream=True.
			imgTopic (str, optional): ROS topic name for publishing raw images.
			compImgTopic (str, optional): ROS topic name for publishing compressed images.

		Raises:
			Exception: If startStream=True but port=None.
			Exception: If camera cannot be opened or configured.

		Notes:
			- Sets self.camOn = True to signal capture thread to run.
			- Capture thread is a daemon thread (exits when main program exits).
			- For RTSP/HTTP streams, actual resolution/framerate may differ from requested.
			- Stream uses HTTPS with SSL certificates from self.sslPath.
			- Frames become available in frameDeque shortly after start() returns.
		"""
		try:
			self.camOn = True
			
			# If user didn't provide a parameter, use the default value
			self.res_rows     = self.defaultFromNone(res_rows, self.res_rows, int)
			self.res_cols     = self.defaultFromNone(res_cols, self.res_cols, int)
			self.framerate    = self.defaultFromNone(framerate, self.fps_target, int)
			self.device       = self.defaultFromNone(device, self.device)
			self.apiPref      = self.defaultFromNone(apiPref, self.apiPref)
			self.port         = self.defaultFromNone(port, self.outputPort)
			# compImgTopic =

			# Start capturing
			capThread = threading.Thread(target=self._thread_capture, args=(self.res_rows, self.res_cols, self.framerate, self.fourcc, self.device, self.apiPref,))
			capThread.daemon = True
			capThread.start()

			# Start streaming?
			if (startStream):
				if (self.port is None):
					raise Exception('cannot stream when port is None')
				else:	
					self.startStream(self.port)
								
			# Start publishing to ROS compressed image topic?
			if ((imgTopic is not None) or (compImgTopic is not None)):
				self.startROStopic(imgTopic=imgTopic, compImgTopic=compImgTopic)	


			self.reachback_pubCamStatus()
		except Exception as e:
			self.logger.log(f'Error in camera start: {e}', severity=ub_utils.SEVERITY_ERROR)
	
	def stop(self, stopStream=True):
		'''
		Stop capture thread
		Stop capturing numpy array?
		'''
		self.camOn = False	
		
		# We may choose not to stop the stream if we are changing resolution/framerate.	
		if (stopStream):
			self.stopStream()
		
	def shutdown(self):
		'''
		Might be as simple as calling self.stop()
		'''
		self.stop()
		time.sleep(STREAM_MAX_WAIT_TIME_SEC + 1)
		
			
	def changeResolutionFramerate(self, res_rows=None, res_cols=None, framerate=None):
		"""Change camera resolution and/or framerate.

		Dynamically changes the video source resolution and/or framerate by stopping
		the current capture thread, updating parameters, and restarting capture with
		new settings. The streaming server (if running) continues without interruption.

		Args:
			res_rows (int, optional): New image height in pixels. If None, keeps current value.
			res_cols (int, optional): New image width in pixels. If None, keeps current value.
			framerate (int, optional): New framerate in fps. If None, keeps current value.

		Raises:
			Exception: If framerate is outside configured min/max bounds.
			Exception: If VideoCapture cannot be reconfigured.

		Notes:
			- Briefly interrupts frame capture while restarting (typically ~1 second).
			- Streaming continues during transition (may show same frame briefly).
			- For RTSP/HTTP streams, resolution/framerate changes may be ignored.
			- Actual resolution/framerate are verified after restart.
			- Updates self.res_rows, self.res_cols, and self.fps_target attributes.
			- Not all cameras support all resolutions or framerates.
		"""
		try:
			# If user didn't provide a parameter, use the default value
			res_rows  = self.defaultFromNone(res_rows,  self.res_rows,   int)
			res_cols  = self.defaultFromNone(res_cols,  self.res_cols,   int)
			framerate = self.defaultFromNone(framerate, self.fps_target, int)
			
			if (hasattr(self, 'fpsMin') and hasattr(self, 'fpsMax')):
				if ((framerate < self.fpsMin) or (framerate > self.fpsMax)):
					raise Exception(f'framerate {framerate} ouside of [{self.fpsMin},{self.fpsMin}] bounds.')
				
			if ((framerate != self.cap.get(cv2.CAP_PROP_FPS)) or 
				(res_rows  != self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 
				(res_cols  != self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))):
				
				# Need to stop/release camera to make updates.
				# However, don't stop the stream (if it is running)
				self.stop(stopStream=False)
				time.sleep(1)

				# Now, we'll re-start the thread, re-initializing camera with new params:
				self.start(res_rows=res_rows, res_cols=res_cols, framerate=framerate)
			
			# FIXME -- Need to verify that the updates actually went thru
			self.updateResolution(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT), self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) 
			self.updateFramerate(self.cap.get(cv2.CAP_PROP_FPS))

			fourccText = self.fourcc2text()
			self.logger.log(f'rows: {self.res_rows}, cols: {self.res_cols}, framerate: {framerate}', severity=ub_utils.SEVERITY_DEBUG)
			
		except Exception as e:
			self.logger.log(f'Failed to change to {res_rows} rows, {res_cols} cols, {framerate} framerate: {e}', severity=ub_utils.SEVERITY_ERROR)


	def changeZoom(self, zoomLevel):
		"""Change camera zoom level using digital zoom (crop and resize).

		Applies digital zoom by cropping the center region of each frame and resizing
		to the original resolution. This is done in software for each frame via
		the _changeZoom() method shared with CameraROS and Voxl cameras.

		Args:
			zoomLevel (float): Zoom level where 1.0 = no zoom, 2.0 = 2x zoom, etc.
				Higher values zoom in more (crop more of the frame).

		Notes:
			- Digital zoom reduces effective resolution (not optical zoom).
			- Zoom is applied to each frame in the capture thread.
			- Crop is centered on the frame.
			- Consider using camera's optical zoom if available (hardware-dependent).
			- For USB cameras, cv2.CAP_PROP_ZOOM may work on some hardware (not implemented).
		"""
		# This requires a numpy zoom/crop for each frame?
		self._changeZoom(zoomLevel)
					    

	def fourcc2text(self):
		# Find the 4-letter text description of our FOURCC property
		# See https://stackoverflow.com/questions/61659346/how-to-get-4-character-codec-code-for-videocapture-object-in-opencv
		h = int(self.cap.get(cv2.CAP_PROP_FOURCC))
		return chr(h&0xff) + chr((h>>8)&0xff) + chr((h>>16)&0xff) + chr((h>>24)&0xff) 
		
