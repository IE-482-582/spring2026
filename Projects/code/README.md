# Introduction to the `ub_camera.py` module

This document describes some basic functionality of the `ub_camera` module.  **This is very much a work-in-progress**.  The code below was previously part of a Jupyter notebook, but it was tough to keep everyone's notebooks in sync.

---

### 1.  Import the package:
```
import ub_camera
```

### 2. Initialize your camera
There are 3 types of camera classes:
1. `CameraUSB` - This is for any camera that has a device path (like `/dev/video0`).  Examples include webcams, internal laptop cams, and even Raspberry Pi cameras.
2. `CameraROS` - This is for cameras that subscribe to compressedImage topic, including Gazebo simulations and the Clover drone (real hardware).
3. `CameraPi` - This is exclusive to Raspberry Pi cameras that use the `picamera` package.  This option is deprecated.

If you're unsure, chances are `CameraUSB` is the appropriate class for you.

``` 
# Initialize `CameraUSB` Class
paramDict = {'res_rows':480, 'res_cols':640, 'fps_target':30, 'outputPort': 8000}
apiPref   = None
device    = 0          # '/dev/video0'

camera = ub_camera.CameraUSB(paramDict = paramDict, device = device, apiPref = apiPref)
```
- **FIXME** -- Need to document the arguments in the `CameraUSB` class.

### 3.  Start the camera
```
camera.start()
```
- **Before you exit, make sure you stop your camera.**  See code below.

### 4. Stream the camera feed to be viewed in a browser
```
camera.startStream(port=8000)
```

- Visit https://localhost:8000/stream.mjpg
- NOTE:  You could combine the start and stream options into one command:
    ```
    camera.start(startStream=True, port=8000)
    ```
  
### 5.  When you're done with the camera, stop it:
```
camera.stop()
```

    
---  

## Aruco Tags
**NOTE** You will need to calibrate the camera if you want to be able to determine the distance from a tag.
- See `addCalibrate()` function    
    
### Start ArUco detection (choose the appropriate dictionary for your tags):    
```    
camera.addAruco('DICT_4X4_250', fps_target=20)
# camera.addAruco('DICT_APRILTAG_36h11', fps_target=20)
# camera.addAruco('DICT_APRILTAG_16h5', fps_target=20)
```

### Stop ArUco detection (make sure to use the same dictionary as above):
```
camera.aruco['DICT_4X4_250'].stop()
# camera.aruco['DICT_APRILTAG_36h11'].stop()
# camera.aruco['DICT_APRILTAG_16h5'].stop()
```

--- 

## Detect Barcodes

First, create a function that will be called each time a barcode is detected:
```
def barcodePost():
    try:
        if (len(camera.barcode['default'].deque) > 0):
            if (len(camera.barcode['default'].deque[0]['data']) > 0):
                print(camera.barcode['default'].deque)
    except Exception as e:
        pass
```

Next, start the barcode reader, pointing to the `barcodePost()` function:
```
camera.addBarcode(postFunction=barcodePost)
```

When you're done, stop the barcode reader:
```
camera.barcode['default'].stop()
```

--- 

## Face Detection

Start:
```
camera.addFaceDetect()
```

Stop:
```
camera.facedetect['default'].stop()
```




    
