'''
A collection of useful classes from the Optimator Lab (OLab)
'''

import numpy as np
import cv2
import os
import math
import socket, errno, time  # For checkPort() function

# https://mavlink.io/en/messages/common.html#MAV_SEVERITY
SEVERITY_EMERGENCY = 0  # System is unusable. This is a "panic" condition.
SEVERITY_ALERT     = 1  # Action should be taken immediately. Indicates error in non-critical systems.
SEVERITY_CRITICAL  = 2  # Action must be taken immediately. Indicates failure in a primary system.
SEVERITY_ERROR     = 3  # Indicates an error in secondary/redundant systems.
SEVERITY_WARNING   = 4  # Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
SEVERITY_NOTICE    = 5  # An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
SEVERITY_INFO      = 6  # Normal operational messages. Useful for logging. No action is required for these messages.
SEVERITY_DEBUG     = 7  # Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
SEVERITY_ALL_CLEAR = 10 # Unique to SOAR.  Indicates that an issue has been resolved (like comms restored).



class Logger():
	'''
	Allows publishing to a console-like topic, or simply printing to screen.
	topicName -- String.  Name of the ROS topic serving as the console.
	topicType -- Object.  An actual topic object.   FIXME -- Document/Explain!
	msgAttr   -- String.  Attribute of the topicType that is associated with the text message
	'''
	def __init__(self, topicName=None, topicType=None, msgAttr=None, queue_size=10):
		try:
			if (topicName and topicType):
				if (hasattr(topicType(), msgAttr)):
					import rospy
					self.consoleTopic = topicType
					self.consolePublisher = rospy.Publisher(topicName, topicType, queue_size=queue_size)
					self.msgAttr = msgAttr
				else:
					self.consolePublisher = None
					print('Logger:  Given msgAttr is not an attrib of given topicType.  Just using print()')
			else:
				self.consolePublisher = None
				print('Logger:  No topic name/type found.  Just using print()')
				
		except Exception as e:
			self.consolePublisher = None
			print(f'Error in Logger init: {e}')


	def write2file(self, msg):
		# FIXME -- Not used.
		try:
			f = open("/home/pi/tmp/debugmain.txt", "a")
			f.write(f"{msg}\n")
			f.close()
		except Exception as e:
			print(f'write2file error: {e}')
			
	def log(self, msgtext, severity=SEVERITY_INFO, **kwargs):
		if (self.consolePublisher):
			# def pubConsole(pub, id, severity, msgtext, userID=0, speakMsg='', tune=px4Tunes.DEFAULT.value):
			try:
				c_msg          = self.consoleTopic()

				c_msg.__setattr__(self.msgAttr, msgtext)
	
				if (hasattr(c_msg, 'severity')):
					c_msg.severity = severity

				for key in c_msg.__slots__:
					if (key in kwargs):
						c_msg.__setattr__(key, kwargs[key])
								
				self.consolePublisher.publish(c_msg)
				
				if (len(kwargs) > 0):
					print(f'DEBUG FROM LOGGER: {msgtext}, {kwargs}')
				else:
					print(f'DEBUG FROM LOGGER: {msgtext}')
			except Exception as e:
				print(f"logger error: {e}.  Could not print {msgtext}")
		else:
			print(f'LOGGER: {msgtext}')	


# https://pyimagesearch.com/2018/12/17/image-stitching-with-opencv-and-python/


# Initialize a dictionary that maps strings to their corresponding
# OpenCV object tracker implementations (depending on cv2 version)
(major, minor, sub) = cv2.__version__.split(".")[:3]
# if ((int(major) >= 4) and (int(minor) >= 5) and (int(sub) >= 1)):
if ((int(major) > 5) or ((int(major) == 4) and (int(minor) >= 5))):
	# xps15 laptop has v4.5.3
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.legacy.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.legacy.TrackerTLD_create,
		"medianflow": cv2.legacy.TrackerMedianFlow_create,
		"mosse": cv2.legacy.TrackerMOSSE_create
	}
else:
	# Our RPis are using v4.4.0
	OPENCV_OBJECT_TRACKERS = {
		"csrt": cv2.TrackerCSRT_create,
		"kcf": cv2.TrackerKCF_create,
		"boosting": cv2.TrackerBoosting_create,
		"mil": cv2.TrackerMIL_create,
		"tld": cv2.TrackerTLD_create,
		"medianflow": cv2.TrackerMedianFlow_create,
		"mosse": cv2.TrackerMOSSE_create
	}
		
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50":         {"dict": cv2.aruco.DICT_4X4_50,         "color": (244, 3, 252)},  # hot pink
	"DICT_4X4_100":        {"dict": cv2.aruco.DICT_4X4_100,        "color": (252, 3, 173)},  # purple
	"DICT_4X4_250":        {"dict": cv2.aruco.DICT_4X4_250,        "color": (252, 3, 98)},   # indigo
	"DICT_4X4_1000":       {"dict": cv2.aruco.DICT_4X4_1000,       "color": (143, 41, 1)},   # navy blue
	"DICT_5X5_50":         {"dict": cv2.aruco.DICT_5X5_50,         "color": (245, 140, 2)},  # bright blue
	"DICT_5X5_100":        {"dict": cv2.aruco.DICT_5X5_100,        "color": (245, 217, 2)},  # light blue
	"DICT_5X5_250":        {"dict": cv2.aruco.DICT_5X5_250,        "color": (196, 245, 2)},  # teal
	"DICT_5X5_1000":       {"dict": cv2.aruco.DICT_5X5_1000,       "color": (1, 138, 24)},   # dark green
	"DICT_6X6_50":         {"dict": cv2.aruco.DICT_6X6_50,         "color": (3, 252, 181)},  # lime green
	"DICT_6X6_100":        {"dict": cv2.aruco.DICT_6X6_100,        "color": (244, 3, 252)},  # hot pink
	"DICT_6X6_250":        {"dict": cv2.aruco.DICT_6X6_250,        "color": (252, 3, 173)},  # purple
	"DICT_6X6_1000":       {"dict": cv2.aruco.DICT_6X6_1000,       "color": (252, 3, 98)},   # indigo
	"DICT_7X7_50":         {"dict": cv2.aruco.DICT_7X7_50,         "color": (143, 41, 1)},   # navy blue
	"DICT_7X7_100":        {"dict": cv2.aruco.DICT_7X7_100,        "color": (245, 140, 2)},  # bright blue
	"DICT_7X7_250":        {"dict": cv2.aruco.DICT_7X7_250,        "color": (245, 217, 2)},  # light blue
	"DICT_7X7_1000":       {"dict": cv2.aruco.DICT_7X7_1000,       "color": (196, 245, 2)},  # teal
	"DICT_ARUCO_ORIGINAL": {"dict": cv2.aruco.DICT_ARUCO_ORIGINAL, "color": (1, 138, 24)},   # dark green
	"DICT_APRILTAG_16h5":  {"dict": cv2.aruco.DICT_APRILTAG_16h5,  "color": (3, 252, 181)},  # lime green
	"DICT_APRILTAG_25h9":  {"dict": cv2.aruco.DICT_APRILTAG_25h9,  "color": (2, 252, 252)},  # yellow
	"DICT_APRILTAG_36h10": {"dict": cv2.aruco.DICT_APRILTAG_36h10, "color": (3, 3, 252)}, 	 # red
	"DICT_APRILTAG_36h11": {"dict": cv2.aruco.DICT_APRILTAG_36h11, "color": (3, 186, 252)}   # orange
}

ARUCO_DRAWING_DEFAULTS = {'borderDraw': True, 'borderColor': (3, 186, 252), 
						  'centerDraw': True, 'centerColor': (3, 186, 252), 'centerRadiusPx':   2, 
						  'arrowDraw':  True, 'arrowColor':  (3, 186, 252), 'arrowThicknessPx': 1, 'arrowLengthPx': 25, 'arrowTipLength': 0.3, 
						  'textDraw':   True, 'textColor':   (3, 186, 252), 'textThicknessPx':  1, 'textScale':     0.5}

# ==================================================================================
# From https://github.com/PyImageSearch/imutils/blob/master/imutils/paths.py

image_types = (".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff")

def list_images(basePath, contains=None):
    # return the set of files that are valid
    return list_files(basePath, validExts=image_types, contains=contains)


def list_files(basePath, validExts=None, contains=None):
    # loop over the directory structure
    for (rootDir, dirNames, filenames) in os.walk(basePath):
        # loop over the filenames in the current directory
        for filename in filenames:
            # if the contains string is not none and the filename does not contain
            # the supplied string, then ignore the file
            if contains is not None and filename.find(contains) == -1:
                continue

            # determine the file extension of the current file
            ext = filename[filename.rfind("."):].lower()

            # check to see if the file is an image and should be processed
            if validExts is None or ext.endswith(validExts):
                # construct the path to the image and yield it
                imagePath = os.path.join(rootDir, filename)
                yield imagePath
# ==================================================================================
                

def cropImageHack(stitched):
	# create a 10 pixel border surrounding the stitched image
	print("Cropping image ...")
	stitched = cv2.copyMakeBorder(stitched, 10, 10, 10, 10,
		cv2.BORDER_CONSTANT, (0, 0, 0))
		
	# convert the stitched image to grayscale and threshold it
	# such that all pixels greater than zero are set to 255
	# (foreground) while all others remain 0 (background)
	gray = cv2.cvtColor(stitched, cv2.COLOR_BGR2GRAY)
	thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY)[1]
	
	# find all external contours in the threshold image then find
	# the *largest* contour which will be the contour/outline of
	# the stitched image
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	# cnts = imutils.grab_contours(cnts)
	cnts = cnts[0] if len(cnts) == 2 else cnts[1]
	c = max(cnts, key=cv2.contourArea)

	# allocate memory for the mask which will contain the
	# rectangular bounding box of the stitched image region
	mask = np.zeros(thresh.shape, dtype="uint8")
	(x, y, w, h) = cv2.boundingRect(c)
	cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)		
	
	# create two copies of the mask: one to serve as our actual
	# minimum rectangular region and another to serve as a counter
	# for how many pixels need to be removed to form the minimum
	# rectangular region
	minRect = mask.copy()
	sub = mask.copy()
	# keep looping until there are no non-zero pixels left in the
	# subtracted image
	while cv2.countNonZero(sub) > 0:
		# erode the minimum rectangular mask and then subtract
		# the thresholded image from the minimum rectangular mask
		# so we can count if there are any non-zero pixels left
		minRect = cv2.erode(minRect, None)
		sub = cv2.subtract(minRect, thresh)		

	# find contours in the minimum rectangular mask and then
	# extract the bounding box (x, y)-coordinates
	cnts = cv2.findContours(minRect.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	# cnts = imutils.grab_contours(cnts)
	cnts = cnts[0] if len(cnts) == 2 else cnts[1]
	c = max(cnts, key=cv2.contourArea)
	(x, y, w, h) = cv2.boundingRect(c)

	# use the bounding box coordinates to extract our final
	# stitched image
	return stitched[y:y + h, x:x + w]
	
	

def stitchImages(imagesDirectory = None, imageFilenamesArray = None, doCrop = False, outputFile = None):
	'''
	Stitches a collection of input images,
	returning a single cv2 image.
	
	Inputs:
	imagesDirectory - A string containing the full path to a directory 
				containing the images you wish to stitch.  
				All image files in this directory will be included.
				Default: None
	imageFilenamsArray - A Python array of strings, where each 
				string is a full-path filename of an image file.
				Default: None
	doCrop - A boolean flag indicating whether the resulting stitched
			 image should be cropped.  See the pyimagesearch link 
			 for details.
			 Default: False
	outputFile - A string containing the full path (and filename) for 
			 the resulting stitched image.
			 Provide this if you want to save the image.
			 Default:  None (no file saved)
			
	You need to provide `imagesDirectory` or `imageFilenamesArray` 
	(or both, if you wish) 
	
	Returns status (0 if no errors) and the stitched image
	'''

	# grab the paths to the input images and initialize our images list
	print("Loading images ...")
	imagePaths = []
	if (imagesDirectory is not None):
		imagePaths = sorted(list(list_images(imagesDirectory)))
	if (imageFilenamesArray is not None):
		imagePaths.extend(imageFilenamesArray)
	if (len(imagePaths) == 0):
		print('Error: No images were found.')
		return (-1, None)

	images = []
	# loop over the image paths, load each one, and add them to our
	# images to stich list
	for imagePath in imagePaths:
		image = cv2.imread(imagePath)
		images.append(image)

	# initialize OpenCV's image sticher object and then perform the image
	# stitching
	print("Stitching images ...")
	stitcher = cv2.Stitcher_create()
	(status, stitched) = stitcher.stitch(images)
	
	# if the status is '0', then OpenCV successfully performed image stitching
	if status == 0:
		# check to see if we supposed to crop out the largest rectangular
		# region from the stitched image
		if (doCrop):
			stiched = cropImageHack(stitched)
		
		if (outputFile is not None):
			# write the output stitched image to disk
			cv2.imwrite(outputFile, stitched)

		# display the output stitched image to our screen
		cv2.imshow("Stitched", stitched)
		cv2.waitKey(0)

	else:
		# otherwise the stitching failed, 
		# likely due to not enough keypoints being detected
		print("Image stitching failed ({})".format(status))		
	
	return(status, stitched)
	

def arucoDrawDetections(img, corners, ids, centers=[], rotations=[], config=ARUCO_DRAWING_DEFAULTS):
	'''
	img is a numpy array of the cv2 image.
		We will update the image itself.
	corners - output from arucoDetectMarkers.  np INT array    FIXME -- Maybe it shouldn't be
	ids - output from arucoDetectMarkers.  np array of size n (number of detected markers)
	
	'''
	
	try:
		for i in range(0, len(corners)):
			# Draw bounding box:
			if (config['borderDraw']):
				cv2.polylines(img, corners[i].astype('int'), True, config['borderColor'], 2, cv2.LINE_AA)
			
			# Add label to bounding box (top right corner)
			if (config['textDraw']):
				cv2.putText(img, str(ids[i]),
					(int(corners[i][0][1][0]), int(corners[i][0][1][1]) - 7),
					cv2.FONT_HERSHEY_SIMPLEX,
					config['textScale'], config['textColor'], config['textThicknessPx'], cv2.LINE_AA)		


		if ((config['centerDraw']) or (config['arrowDraw'])):
			for i in range(0, len(centers)):
				pt1 = (centers[i][0], centers[i][1])
				# Draw a center marker
				if (config['centerDraw']): 
					drawCircle(img, pt1, config['centerRadiusPx'], color=config['centerColor'])
				if (config['arrowDraw']):
					# Draw an arrow marker
					pt2 = ptAndAngleToNewPt(centers[i], rotations[i], config['arrowLengthPx'])
					drawArrow(img, pt1, (int(pt2[0]), int(pt2[1])), config['arrowColor'], config['arrowThicknessPx'], config['arrowTipLength'])
	except Exception as e:
		print(f'ERROR in arucoDrawDetections:  {e}')
		
def arucoDetectMarkers(img, arucoDict, arucoParams, img_x_y=None, orig_x_y=None):
	'''
	Detect ArUco markers in the input frame
	See https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
	
	img is a numpy array of the cv2 image
	arucoDict comes from cv2.aruco.Dictionary_get()
	arucoParams comes from cv2.aruco.DetectorParameters_create()
	img_x_y is a tuple of form (width, height), describing size of img
	orig_x_y is also (width, height), describing size of original image (before scaling)
	
	For example:
		self.arucoDict   = {'RPi':      cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_APRILTAG_16h5']), 
							'HiRes':    cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_APRILTAG_16h5']),
							'Tracking': cv2.aruco.Dictionary_get(ARUCO_DICT['DICT_APRILTAG_16h5'])}
		self.arucoParams = {'RPi':      cv2.aruco.DetectorParameters_create(), 
							'HiRes':    cv2.aruco.DetectorParameters_create(), 
							'Tracking': cv2.aruco.DetectorParameters_create()}
							
	See https://docs.opencv.org/4.x/d2/d1a/classcv_1_1aruco_1_1ArucoDetector.html#a0c1d14251bf1cbb06277f49cfe1c9b61
	NOTE (from above link): The function does not correct lens distortion or takes it into account. It's recommended to undistort input image with corresponding camera model, if camera parameters are known.

	centers -- n x 2 integer array denoting the [x, y] center coords for each of n identified markers.
	rotations -- n x 1 float array denoting each marker's rotation.  0 is up, pi/4 is right.
	corners -- n x 1 x 4 x 2 float array.  
	'''
	try:
		centers   = np.array([], dtype='int')
		rotations = np.array([])
		(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
		if (len(corners) > 0):
			corners = np.array(corners)
			ids = ids.flatten()
			if (img_x_y != orig_x_y):
				# We changed image size.  Change corners to display properly when overlayed on original image
				xscale = orig_x_y[0] / img_x_y[0] 
				yscale = orig_x_y[1] / img_x_y[1]
				
				corners[:,:,:,0] *= xscale
				corners[:,:,:,1] *= yscale
			


			# Find midpoint, using corner points 1 (NE) and 3 (SW)
			'''
			for i in range(0, len(corners)):
				mp = ((corners[i][0][3][0] + corners[i][0][1][0])/2, 
					  (corners[i][0][3][1] + corners[i][0][1][1])/2) 
			'''
			centers = ((corners[:,0,3,:]+corners[:,0,1,:])/2).astype(int)

			'''
			for i in range(0, len(corners)):
				if (self.calcRotations):					
					# point 0 is top left, 3 is bottom left.  x increases to right, y increases down
					x = corners[i][0][0][0] - corners[i][0][3][0]
					y = corners[i][0][0][1] - corners[i][0][3][1]
					theta = math.atan2(x, -y)  # NOTE:  This is in [radians]
			'''
			rotations = np.arctan2( (corners[:,0,3,0]-corners[:,0,0,0]), -(corners[:,0,3,1]-corners[:,0,0,1]) )

			# corners = corners.astype(int)
			
	except Exception as e:
		print('ArUco Tracking failed: {}.'.format(str(e)))
		# (corners, ids, rejected, centers, rotations) = (np.array([], dtype='int'), None, None, np.array([], dtype='int'), np.array([]))
		(corners, ids, rejected, centers, rotations) = (np.array([]), None, None, np.array([], dtype='int'), np.array([]))

		# pubConsole(self.pub_console, self.assetID, MAV_SEVERITY_ERROR, 'ArUco Tracking failed ({}): {}.'.format(camType, str(e)))

	return (corners, ids, rejected, centers, rotations)


def decorateBarcode(img, corners, data, color=(0,0,255), addText=True):
	'''
	img is a numpy array of the cv2 image.
		We will update the image itself.
	corners - output from pyzbar.decode rect.  np INT32 array of arrays.
	data - output from pyzbar.decode data (text of barcode)
	'''
	try:
		for i in range(0, len(corners)):
			# Draw bounding box:
			# print('corners', i, ': ', corners[i])
			# cv2.polylines(img, [corners[i]], True, color, 2, cv2.LINE_AA)
			cv2.rectangle(img, corners[i][0], corners[i][1],
				color, 2, cv2.LINE_AA)

			
			'''
			FIXME -- This is giving an error about index 1..
			# Add label to bounding box (top right corner)
			if (addText):
				cv2.putText(img, str(data[i]),
					(corners[i][0][1][0], corners[i][0][1][1] - 7),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.5, color, 1, cv2.LINE_AA)		
			'''
	except Exception as e:
		print(f'Error in decorateBarcode: {e}')

def decorateCalibrate(img, checkerboard, corners, count, img_x_y, orig_x_y, addText=True):
	try:
		if ((checkerboard is not None) and (corners is not None)):			
			# corners comes from a deque.  Below, we modify the values.  So, let's make a copy.
			cnrs = corners.copy()
			
			if (img_x_y != orig_x_y):
				# We changed image size.  Change corners to display properly when overlayed on original image
				xscale = orig_x_y[0] / img_x_y[0] 
				yscale = orig_x_y[1] / img_x_y[1]

				cnrs[:,:,0] *= xscale
				cnrs[:,:,1] *= yscale

			cv2.drawChessboardCorners(img, checkerboard, cnrs, True)	

		if (addText):
			cv2.putText(img, str(count),
				(15, 65),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (200, 20, 10), 1, cv2.LINE_AA)		

	except Exception as e:
		print(f'Error in decorateCalibrate: {e}')

def decorateFaceDetect(img, confidence, corners, color=(0, 255, 255), addText=True):
	'''
	img is a numpy array of the cv2 image.
		We will update the image itself.
	confidence - array of confidence values
	corners - output from ???.  np INT32 array of arrays.
	'''
	try:
		for i in range(0, len(corners)):
			# Draw bounding box:
			# print('corners', i, ': ', corners[i])
			# cv2.polylines(img, [corners[i]], True, color, 2, cv2.LINE_AA)
			cv2.rectangle(img, corners[i][0], corners[i][1],
				color, 2, cv2.LINE_AA)

			
			'''
			FIXME -- This is giving an error about index 1..
			# Add label to bounding box (top right corner)
			if (addText):
				cv2.putText(img, str(data[i]),
					(corners[i][0][1][0], corners[i][0][1][1] - 7),
					cv2.FONT_HERSHEY_SIMPLEX,
					0.5, color, 1, cv2.LINE_AA)		
			'''
	except Exception as e:
		print(f'Error in decorateFaceDetect: {e}')
	
	
def decorateOptFlow(img, shift):
	'''
	shift[0] x direction
	shift[1] y direction

	'''
	shp = img.shape  # [rows, cols, depth]
	[center_x, center_y] = [int(shp[1]/2), int(shp[0]/2)]
	drawCircle(img, (center_x, center_y), int(5*math.sqrt(shift[0]*shift[0] + shift[1]*shift[1])))
	# drawCircle(img, (center_x, center_y), 20)
	
	drawLine(img, (center_x, center_y), (int(center_x+5*shift[0]), int(center_y+5*shift[1])))
	
def decorateUltralytics(img, w, h, idName, results, drawBox, drawLabel, maskOutline):
	try:
		# FIXME -- assign color based on class
		color=(0, 255, 55)
		
		if (drawBox or drawLabel):
			for i in range(0, len(results['class'])):
				# 'class': [], 'class_conf': [],
				# print(results['xyxy'])
				if (results['xyxy']):
					pt1 = (int(results['xyxy'][i][0]), int(results['xyxy'][i][1]))
					pt2 = (int(results['xyxy'][i][2]), int(results['xyxy'][i][3]))

					if (drawBox):
						cv2.rectangle(img, pt1, pt2, color, 2, cv2.LINE_AA)
					
					if (drawLabel):
						txt = f"{results['class'][i]} {results['class_conf'][i]:.2f}"
						if (results['id']):
							txt = f"ID:{int(results['id'][i])} {txt}" 	
						txtsize = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1)[0]  # scale=0.7, thickness=1
						shp = img.shape  # [rows, cols, depth]
						if (pt1[0] > shp[1]/2):
							# right justify across bounding box
							txt_x = pt2[0] - txtsize[0] - 5 # 5px buffer
							label_pt1 = (min(pt1[0], pt2[0] - txtsize[0] - 5), max(0, pt1[1]-20))
							label_pt2 = (pt2[0], label_pt1[1] + 20)
						else:
							# left justify
							txt_x = pt1[0] + 5
							label_pt1 = (pt1[0], max(0, pt1[1]-20))
							label_pt2 = (max(pt2[0], pt1[0]+txtsize[0]+5), label_pt1[1] + 20)

						cv2.rectangle(img, label_pt1, label_pt2, color, -1)

						cv2.putText(img, txt,
							(txt_x, label_pt2[1]-3),
							cv2.FONT_HERSHEY_SIMPLEX,
							0.7, (0, 0, 0), 1, cv2.LINE_AA)		
				elif (results['xyxyxyxy']):
					# print(np.array(results['xyxyxyxy'][i]).reshape((4,2)))
					cv2.polylines(img, [np.array(results['xyxyxyxy'][i]).reshape((4,2))], isClosed=True, color=(0, 255, 0), thickness=1)
		
		
		# min_conf = 0.75
		radius = 5 # px
		color = (100, 100, 100)
		thickness = 1
		skeleton = {'face':  [5, 3, 1, 0, 2, 4, 6], # [[0, 1], [0, 2], [1, 3], [2, 4], [3, 5], [4, 6]], 
		            'arms':  [9, 7, 5, 6, 8, 10], 
					'leg_left':  [11, 13, 15],
					'leg_right': [12, 14, 16],
					'torso': [5, 11, 12, 6]}
		sk_poly_colors = {'face': (50, 168, 82), 
					 'arms': (0, 154, 196), 
					 'leg_left': (250, 187, 0), 
					 'leg_right': (250, 187, 0), 
					 'torso': (240, 129, 231)}
		sk_points = {'face': [0, 1, 2, 3, 4], 
					 'arms': [5, 6, 7, 8, 9, 10],
					 'legs': [11, 12, 13, 14, 15, 16]}
		sk_points_colors = {'face': sk_poly_colors['face'], 
							'arms': sk_poly_colors['arms'], 
							'legs': sk_poly_colors['leg_left']}
					 
		
		
		# FIXME -- Need to scale the keypoints to image size?  Done?
		for body in range(0, len(results['keypoints'])):
			#print('sk')
			for part in skeleton:
				keep = []
				tmp = []
				for index in skeleton[part]:
					#print(f'{body=}, {index=}')
					#print(f"{results['keypoints'][body]}")
					if ((results['keypoints'][body][index] > [0, 0]).all()):
						tmp.append(results['keypoints'][body][index])
					else:
						if (len(tmp) > 1):
							keep.extend(tmp)
						tmp = []
						
				if (len(tmp) > 1):
					keep.extend(tmp)
				if (len(keep) > 1):
					cv2.polylines(img, [np.int32(keep)], isClosed=False, color=sk_poly_colors[part], thickness=2)
												
			#print('kp')
			for part in sk_points:
				for i in sk_points[part]:
					# print(results['keypoints'][body][i], results['keypoints'][body][i] > [0, 0])
					if ((results['keypoints'][body][i] > [0, 0]).all()):
						cv2.circle(img, results['keypoints'][body][i], radius, sk_points_colors[part], -1)
		
		# Segmentation
		# mask outline
		if (maskOutline):
			for i in range(0, len(results['masks_xy'])):
				cv2.polylines(img, [np.int32(results['masks_xy'][i])], isClosed=True, color=(100, 100, 100), thickness=2)
		
		# channel = 1
		# value = 40
		clr = np.array([100, 100, 100])

		# t = results[0].orig_img
		# t[:,:,channel] = t[:,:,channel]  + mask_stretch*value # mask_stretch*value # t[:,:,channel]#  + mask_stretch*value
		# t[:,:,:] = t[:,:,:]  + np.expand_dims(mask_stretch, axis=-1)*clr # mask_stretch*value # t[:,:,channel]#  + mask_stretch*value
		# success = cv2.imwrite('test00a.jpg', t)	
		try:	
			for i in range(0, len(results['masks_data'])):
				# print(results['masks_data'][i].shape)
				# print(type(results['masks_data'][i]))
				# print(results['masks_data'][i].dtype)
				# print(img.shape)
				# print(max(results['masks_data'][i]))  failsThe truth value of an array with more than one element is ambiguous. Use a.any() or a.all()
				img[:,:,:] = img[:,:,:] + np.expand_dims(results['masks_data'][i], axis=-1)*clr
		except Exception as e:
			print(f'ERRROR: {e}')
			
			
		'''
		if (addText):
			cv2.putText(img, str(idName),
				(15, 65),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (200, 20, 10), 1, cv2.LINE_AA)		
		'''
	except Exception as e:
		print(f'Error in decorateUltralytics: {e}')
		
def degCtoF(degC):
	# Convert degrees Celsius to Fahrenheit
	return (degC*9/5) + 32

def degFtoC(degF):
	# Convert degrees Fahrenheit to Celsius
	return (degF - 32) * 5/9
	
def drawArrow(img, pt1, pt2, color=(255,0,0), thickness=1, tipLength=0.1):
	try:
		cv2.arrowedLine(img, pt1, pt2, color, thickness, cv2.LINE_AA, 0, tipLength) 
	except Exception as e:
		print(f'ERROR in drawArrow: {e}')
		
def drawCircle(img, center, radius, thickness=3, color=(150, 25, 25)):
	'''
	cv2.circle(img, center, radius, color, thickness=1, lineType=8, shift=0)

	img (CvArr) – Image where the circle is drawn
	center (CvPoint) – Center of the circle
	radius (int) – Radius of the circle
	color (CvScalar) – Circle color
	thickness (int) – Thickness of the circle outline if positive, 
		otherwise this indicates that a filled circle is to be drawn
	lineType (int) – Type of the circle boundary, see Line description
		8 (or omitted) - 8-connected line.
		4 - 4-connected line.
		CV_AA - antialiased line.
	shift (int) – Number of fractional bits in the center coordinates and radius value	
	'''
	cv2.circle(img, center, radius, color, thickness, cv2.LINE_AA, 0)
	
def drawLine(img, p1, p2, thickness=3, color=(255,0,0)):
	cv2.line(img, p1, p2, color, thickness, cv2.LINE_AA)


def res2rowscols(res):
	'''
	Split a screen resolution of form `widthxheight` into a list of 
	2 integers: [rows, cols]
	'''
	[cols, rows] = res.split('x')
	return [int(rows.strip()), int(cols.strip())]
	
	
def roiDrawBox(img, box, color=(255,255,255)):
	# check to see if the tracking was a success
	(x, y, w, h) = [int(v) for v in box]
	cv2.rectangle(img, (x, y), (x + w, y + h),
		color, 2, cv2.LINE_AA)

	# initialize the set of information we'll be displaying on
	# the frame
	# (H, W) = output['RPi'].myNumpyArray.shape[:2]
	'''
	cv2.putText(output['RPi'].myNumpyArray, 'tracking', (10, 25),
		cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
	'''		

def roiTrack(roiTracker, img):
	'''
	Identify ROI
	
	This should only be called if ROI is actually active
	(i.e., if the bounding box has been defined, self.roiBB is not None)
	'''
	try:
		# grab the new bounding box coordinates of the object
		# (success, box) = self.roiTracker[camType].update(output[camType].myNumpyArray)
		(success, box) = roiTracker.update(img)			

	except Exception as e:
		print('ROI Tracking failed: {}.'.format(str(e)))
		# pubConsole(self.pub_console, self.assetID, MAV_SEVERITY_ERROR, 'ROI Tracking failed on {}: {}.'.format(camType, str(e)))
		# output[camType].setCamFunction(None)    # FIXME -- Need to update status to indicate we're no longer tracking ROI
		success = False
		box = None
		
	return (success, box)


def countArucoInImage(img, arucoDict, arucoParams, drawDetections=False, labelDetections=False):
	'''
	Count how many ArUco markers are in an image, grouped by marker ID.
	Returns a dictionary, where the keys are detected marker IDs, and the values are 
	the number of observations of the key ID.
	'''
	
	(corners, ids, rejected, centers, rotations) = arucoDetectMarkers(img, arucoDict, arucoParams)
	
	print(ids)
	IDcount = {}
	
	for i in range(0, len(corners)):
		markerID = ids[i]
		if (markerID in IDcount):
			IDcount[markerID] += 1
		else:
			IDcount[markerID]  = 1

	if (drawDetections):
		arucoDrawDetections(img, corners, ids, centers=centers, rotations=rotations)
			
	return IDcount
	


def map_range(x, X_min, X_max, Y_min, Y_max):
	''' 
	Linear mapping between two ranges of values 
	'''

	X_range = X_max - X_min
	Y_range = Y_max - Y_min
	XY_ratio = X_range / Y_range

	# y = ((x-X_min) / XY_ratio + Y_min) //1
	y = ((x-X_min) / XY_ratio + Y_min)

	return y
	
def setEndingChar(string, endingChar):
	'''
	Make sure `string` ends in `endingChar`, without allowing duplicates.
	'''
	string = string.rstrip()
	if (string[-1] != endingChar):
		string += endingChar
	return string
	
def _passFunction(*args, **kwargs):
	'''
	a dummy function that does nothing
	'''
	pass

def ptAndAngleToNewPt(pt, angleRad, length):
	'''
	Find the location of a new point that is `length` units from `pt`, in the direction `angleRad`
	pt -- (x, y)
	NOTE:  y increases DOWN
	'''
	return (pt[0] + length*math.sin(angleRad), pt[1] - length*math.cos(angleRad))


		
		
def arucoFindTagIndices(idArray, targetID):
	'''
	Given a 1-D np array of IDs and a specific reference ID, 
	find the indices of the array that match the reference.
	If no matches are found, return an empty tuple.
	'''
	try:
		if (idArray is None):
			return ()
		
		ids, = np.where(idArray.flatten() == targetID)
		return ids  # This will be a tuple
		
	except Exception as e:
		# raise Exception(e)
		return ()

def arucoFindTagIndicesList(idArray, targetIDlist):
	'''
	Given a 1-D np array of IDs and a python list of reference IDs, 
	find the indices of the array that match the reference.
	If no matches are found, return an empty tuple.
	'''
	try:
		if (idArray is None):
			return ()
		
		ids, = np.where(np.isin(idArray.flatten(), targetIDlist))
		return ids  # This will be a tuple
		
	except Exception as e:
		# raise Exception(e)
		return ()
	

"""	
NO!  arucoDetectMarkers already returns `centers`
def arucoFindTagCenterPixels(corners):
	'''
	Given the corners **for a single tag**, 
	return the (x, y) pixel coordinates of the tag's center.
	x is pixels from left of image; y is pixels from top of image.
	
	NOTE:  I'm pretty sure the x,y values are floats
	'''
	
	try:
		# Find midpoint, using corner points 1 (NE) and 3 (SW)
		mp = ((corners[0][3][0] + corners[0][1][0])/2, 
			  (corners[0][3][1] + corners[0][1][1])/2) 		
		return mp	  
	except Exception as e:
		# raise exception(e)?
		return ()
"""	

def arucoFindPose(objPoints, corners, cameraMatrix, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE):
	'''
	markerLength = 0.45  # [meters], but you can choose any unit you wish
	objPoints = np.array([[-markerLength/2,  markerLength/2, 0], 
						  [ markerLength/2,  markerLength/2, 0], 
						  [ markerLength/2, -markerLength/2, 0], 
						  [-markerLength/2, -markerLength/2, 0]])

	corners **for a single tag**
	cameraMatrix is 3x3, containing fx, fy, cx, and cy
	dist is the array of distortion coefficients
	'''
	(ret, rvecs, tvecs) = cv2.solvePnP(objPoints, corners, cameraMatrix, dist, flags)
	
	return (ret, rvecs, tvecs)
	
	
# self.camera.intrinsics['matrix'], self.camera.intrinsics['dist']

def arucoFindPoseGlobal():
	'''
	NOTE: This wouldn't really be an aruco-specific function.
	
	Estimate the location of an object in the Global/World frame
	
	Need to know 
	- location and orientation of camera (in WORLD coords)
	- location of object (x, y, and z distance from camera)
		This would come from arucoFindPoseLocal.tvecs
		
	Returns location of object in WORLD coords	
	'''	
	
	print('FIXME -- Do we already have this function somewhere?')
	

def checkPort(port):
	# Check if a given local port is available (returns True) or in use (returns False)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(('localhost', port))
        s.close()
        return True
    except socket.error as e:
        if e.errno == errno.EADDRINUSE:
            print(f"Port {port} is already in use.")
        else:
            print(e)
        s.close()
        return False
        
def findOpenPort(port, options=range(8000,8011)):
	# port is our preferred port
	# options is a list of acceptable alternative ports
	if (checkPort(port)):
		return port
	else:
		for p in options:
			if (checkPort(p)):
				return p
				
	return None	
		
def pics2video(sourcePath=None, filename=None, fps=30):
	try:
		
		if sourcePath is None:
			print('Error in pic2video - no sourcePath defined')
			return
			
		sourcePath = setEndingChar(sourcePath, "/")
				
				
		if filename is None:
			myTimestamp = datetime.today()
			filename = f"{myTimestamp.strftime('%Y-%m-%d-%H%M%S')}.mp4"


		# Create a list of image files, ordered by timestamp:
		# See https://stackoverflow.com/questions/30121222/convert-all-images-in-directory-to-mp4-using-ffmpeg-and-a-timestamp-order
		# ls /path/to/*.jpg | sort -V | xargs -I {} echo "file '{}'" > list.txt
		os.system(f"ls {sourcePath}*.jpg | sort -V | xargs -I {{}} echo \"file '{{}}'\" > {sourcePath}list.txt")

		# Create video:
		# ffmpeg -r 1/5 -f concat -i list.txt -c:v libx264 -r 25 -pix_fmt yuv420p -t 15 out.mp4
		# ffmpeg -r 1/5 -f concat -safe 0 -i list.txt -c:v libx264 -r 30 -pix_fmt yuv420p -vf scale=540:-2 -t 15 out.mp4 
		os.system(f"ffmpeg -r {fps} -f concat -safe 0 -i {sourcePath}list.txt -c:v libx264 -pix_fmt yuv420p -y {sourcePath}{filename}")

		# Twitter doesn't like the format created above.
		# We'll post-process with ffmpeg:
		# os.system("ffmpeg -i {}/{} -vcodec libx264 -f mp4 {}/{} -y".format(dirpath, tmpFilename, dirpath, filename))
		# os.system("rm {}/{}".format(dirpath, tmpFilename))
	except Exception as e:
		print(f'Error in pics2video: {e}')


def inches2meters(inches):
    return inches * 0.0254

def meters2inches(meters):
    return meters * 39.3701
