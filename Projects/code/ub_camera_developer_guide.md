# ub_camera Developer Guide

**Version:** 1.0
**Last Updated:** 2026-01-29

This guide explains how to extend the `ub_camera` module by adding new camera classes or new feature classes. It can be used as reference documentation for human developers or as a prompt for Claude Code.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Adding a New Camera Class](#adding-a-new-camera-class)
3. [Adding a New Feature Class](#adding-a-new-feature-class)
4. [Code Organization Recommendations](#code-organization-recommendations)
5. [Best Practices](#best-practices)
6. [Testing Your Changes](#testing-your-changes)

---

## Architecture Overview

### System Design

The `ub_camera` module uses an **object-oriented architecture** with:

- **Base Camera Class**: `Camera` - Provides common functionality (streaming, ROS, decorations, frame management)
- **Camera Implementations**: `CameraPi`, `CameraROS`, `CameraUSB` - Hardware-specific camera interfaces
- **Feature Classes**: `_Aruco`, `_Barcode`, `_Calibrate`, etc. - Pluggable computer vision features
- **Streaming Infrastructure**: `StreamingHandler`, `StreamingServer` - HTTP/HTTPS MJPEG streaming

### Key Concepts

#### Threading Model
- **Capture Thread**: Each camera implementation runs a capture loop in its own thread
- **Feature Threads**: Each feature (ArUco, barcode, etc.) runs in a separate daemon thread
- **Frame Synchronization**: Uses `threading.Condition` to notify consumers when new frames are available
- **Frame Storage**: `deque(maxlen=1)` stores the most recent frame

#### Decoration System
- Features can register "decoration" functions to draw on frames during streaming
- Decorations are applied in `decorateFrame()` method
- Uses a deque-based system for adding/removing/editing decorations dynamically

#### Parent-Child Relationship
- Feature classes store a reference to their parent camera: `self.camObject`
- Features access camera frames via `self.camObject.getFrameCopy()`
- Features use the parent's logger: `self.camObject.logger.log()`

---

## Adding a New Camera Class

Camera classes inherit from `Camera` and implement hardware-specific frame capture.

### Step-by-Step Guide

#### 1. Choose Your Camera Source
Determine what type of camera/video source you're implementing:
- Physical hardware (like `CameraPi`)
- Network stream (like `CameraUSB` with RTSP)
- Simulation/virtual camera (like `CameraROS`)
- Video file playback
- Other sources

#### 2. Create the Class Structure

```python
class CameraYourName(Camera):
    """Your camera implementation.

    Describe what hardware/sources this camera supports, key features,
    and any special requirements or dependencies.

    Args:
        paramDict (dict): Configuration dictionary with required keys:
            - res_rows (int): Frame height in pixels
            - res_cols (int): Frame width in pixels
            - fps_target (float): Target frame rate in Hz
            - Additional camera-specific parameters
        device: Camera-specific device identifier (path, URL, etc.)
        logger: Optional ub_utils.Logger instance
        sslPath (str): Path to SSL certificates for streaming
        initROSnode (bool): Whether to initialize ROS node
        ipAllowlist (list): IP addresses allowed to stream
        ipBlocklist (list): IP addresses blocked from streaming
        **kwargs: Additional camera-specific parameters

    Example:
        camera = CameraYourName(
            paramDict={'res_rows': 480, 'res_cols': 640, 'fps_target': 30},
            device='/dev/your_device'
        )
        camera.start()
        camera.startStream(port=8000)

    Notes:
        - List any special hardware requirements
        - List any software dependencies (pip packages, system libraries)
        - Note any platform limitations (Linux-only, etc.)
    """

    def __init__(self, paramDict, device=None, logger=None, sslPath=None,
                 initROSnode=False, ipAllowlist=[], ipBlocklist=[], **kwargs):
        """Initialize camera instance.

        Args:
            paramDict: See class docstring
            device: Camera-specific device parameter
            logger: Optional logger instance
            sslPath: SSL certificate path
            initROSnode: Whether to initialize ROS node
            ipAllowlist: IP whitelist for streaming
            ipBlocklist: IP blacklist for streaming
            **kwargs: Additional parameters
        """
        # IMPORTANT: Call parent __init__ first
        super().__init__(paramDict, logger, sslPath, None, initROSnode,
                        True, ipAllowlist, ipBlocklist)

        # Store camera-specific parameters
        self.device = device
        # Add any other camera-specific attributes

        # Initialize hardware connection (if needed)
        # But don't start capturing yet - that happens in start()
```

#### 3. Implement the Capture Thread

This is the core of your camera class - it continuously captures frames.

```python
    def _thread_capture(self):
        """Capture thread that continuously grabs frames.

        This method runs in its own daemon thread and should:
        1. Initialize the camera hardware/connection
        2. Loop while self.camOn is True
        3. Capture frames at the target framerate
        4. Store frames in self.frameDeque
        5. Notify waiting threads via self.announceCondition()
        6. Calculate actual framerate
        7. Handle errors gracefully
        8. Clean up on exit
        """
        try:
            # Initialize your camera/video source here
            # Example:
            # self.capture = YourCameraAPI.open(self.device)
            # self.capture.set_resolution(self.res_cols, self.res_rows)
            # self.capture.set_framerate(self.fps_target)

            self.logger.log(f'Starting capture thread', severity=ub_utils.SEVERITY_INFO)

            # Main capture loop
            while self.camOn:
                timeNow = time.time()

                # Capture a frame from your camera
                # frame should be a numpy array of shape (res_rows, res_cols, 3) in BGR format
                frame = None  # Replace with actual capture code
                # Example:
                # frame = self.capture.read()

                if frame is not None:
                    # Convert to bytes if needed (for consistency with other cameras)
                    self.frameDeque.append(frame.tobytes())

                    # Notify waiting threads (streaming, ROS, etc.)
                    self.announceCondition()

                    # Calculate and update framerate
                    self.calcFramerate(self.fps['capture'], 'capture')
                else:
                    self.logger.log('Failed to capture frame', severity=ub_utils.SEVERITY_WARNING)

                # Throttle to target framerate
                delta = max(0, timeNow + (1.0/self.fps_target) - time.time())
                if delta > 0:
                    time.sleep(delta)

        except Exception as e:
            self.logger.log(f'Error in capture thread: {e}', severity=ub_utils.SEVERITY_ERROR)
        finally:
            # Clean up camera resources
            # Example:
            # if hasattr(self, 'capture') and self.capture:
            #     self.capture.release()
            pass
```

#### 4. Implement the start() Method

```python
    def start(self, doStream=False, port=8000, doROSpub=False):
        """Start the camera capture thread.

        Args:
            doStream (bool): Whether to start HTTPS streaming immediately
            port (int): Port for HTTPS streaming server
            doROSpub (bool): Whether to start ROS publishing immediately

        Example:
            camera.start()  # Just start capture
            camera.start(doStream=True, port=8000)  # Start with streaming
        """
        try:
            # Set the camera active flag
            self.camOn = True

            # Start the capture thread
            captureThread = threading.Thread(target=self._thread_capture, args=())
            captureThread.daemon = True
            captureThread.start()

            self.logger.log('Camera started', severity=ub_utils.SEVERITY_INFO)

            # Optionally start streaming
            if doStream:
                self.startStream(port=port)

            # Optionally start ROS publishing
            if doROSpub:
                self.startROStopic()

        except Exception as e:
            self.logger.log(f'Error starting camera: {e}', severity=ub_utils.SEVERITY_ERROR)
```

#### 5. Implement shutdown() Method (if needed)

```python
    def shutdown(self):
        """Stop camera capture and clean up resources.

        Stops all feature threads, streaming, ROS publishing, and the capture thread.
        Releases hardware resources.
        """
        try:
            self.logger.log('Shutting down camera', severity=ub_utils.SEVERITY_INFO)

            # Stop all features first
            for arucoName in list(self.aruco.keys()):
                self.aruco[arucoName].stop()
            for barcodeName in list(self.barcode.keys()):
                self.barcode[barcodeName].stop()
            for calibrateName in list(self.calibrate.keys()):
                self.calibrate[calibrateName].stop()
            # ... etc for all feature types

            # Stop streaming and ROS
            self.stopStream()
            self.stopROStopic()

            # Signal capture thread to stop
            self.camOn = False

            # Give threads time to clean up
            time.sleep(0.5)

            # Release camera-specific resources
            # Example:
            # if hasattr(self, 'capture') and self.capture:
            #     self.capture.release()

            self.logger.log('Camera shutdown complete', severity=ub_utils.SEVERITY_INFO)

        except Exception as e:
            self.logger.log(f'Error during shutdown: {e}', severity=ub_utils.SEVERITY_ERROR)
```

#### 6. Implement Camera-Specific Features (Optional)

If your camera supports hardware-specific features, add methods for them:

```python
    def changeZoom(self, zoomLevel):
        """Change camera zoom level.

        Args:
            zoomLevel (float): Zoom factor (1.0 = no zoom, 2.0 = 2x zoom)

        Notes:
            Override this if your camera has hardware zoom support.
            Otherwise, use the base class digital zoom.
        """
        # Hardware zoom implementation
        # Or fall back to parent class:
        super().changeZoom(zoomLevel)

    def changeResolutionFramerate(self, res_rows, res_cols, fps_target):
        """Dynamically change resolution and framerate.

        Args:
            res_rows (int): New frame height
            res_cols (int): New frame width
            fps_target (float): New target framerate
        """
        # Implementation depends on your camera hardware
        pass
```

### Complete Camera Class Example

See `CameraUSB` in `ub_camera.py` for a complete, working example that supports:
- USB cameras
- RTSP/HTTP streams
- Video files
- Dynamic resolution/framerate changes
- Multiple backend APIs

---

## Adding a New Feature Class

Feature classes add computer vision capabilities (detection, tracking, etc.) that run in parallel with frame capture.

### Step-by-Step Guide

#### 1. Define Your Feature

Determine what your feature does:
- Object detection/tracking
- Image analysis
- Visual marker detection
- Image enhancement/filtering
- Other computer vision tasks

#### 2. Create the Class Structure

```python
class _YourFeature():
    """Brief description of what this feature does.

    Detailed description of the algorithm, purpose, and use cases.
    Explain what gets stored in the deque and what the postFunction receives.

    Attributes:
        camObject: Parent Camera instance
        idName (str): Unique identifier for this feature instance
        res_rows (int): Processing height in pixels
        res_cols (int): Processing width in pixels
        fps_target (float): Target processing framerate
        postFunction (callable): Callback function after each processing cycle
        postFunctionArgs (dict): Arguments passed to postFunction
        deque (deque): Most recent detection results
        isThreadActive (bool): Whether processing thread is running

    Methods:
        start(): Start the feature processing thread
        stop(): Stop the feature thread and cleanup
    """

    def __init__(self, camObject, idName, res_rows, res_cols, fps_target,
                 postFunction, postFunctionArgs, **kwargs):
        """Initialize the feature instance.

        Args:
            camObject: Parent Camera instance
            idName (str): Unique name for this feature instance
            res_rows (int): Height for image processing (None = use camera resolution)
            res_cols (int): Width for image processing (None = use camera resolution)
            fps_target (float): Target processing rate in Hz
            postFunction (callable): Function called after each processing cycle.
                Signature: postFunction(postFunctionArgs)
            postFunctionArgs (dict): Arguments for postFunction. The feature
                automatically adds 'idName' to this dict.
            **kwargs: Additional feature-specific parameters
        """
        try:
            # Store reference to parent camera
            self.camObject = camObject

            # Store configuration
            self.idName = idName
            self.decorationID = None  # Set when decoration is registered

            self.res_rows = res_rows
            self.res_cols = res_cols
            self.resolution = f'{res_cols}x{res_rows}'

            self.fps_target = fps_target
            self.threadSleep = 1.0 / fps_target

            # Setup post-processing callback
            self.postFunctionArgs = postFunctionArgs
            self.postFunctionArgs['idName'] = idName
            if postFunction is None:
                self.postFunction = ub_utils._passFunction
            else:
                self.postFunction = postFunction

            # Initialize result storage
            # Store whatever data structure makes sense for your feature
            self.deque = deque(maxlen=1)
            self.deque.append({'detected': False, 'results': []})

            # FPS tracking for this feature
            self.fps = _make_fps_dict(recheckInterval=5)

            # Initialize any models, detectors, or resources needed
            # Example:
            # self.detector = YourDetectorClass()

            # Thread state
            self.isThreadActive = False

        except Exception as e:
            self.camObject.logger.log(f'Error in {idName} init: {e}',
                                     severity=ub_utils.SEVERITY_ERROR)
```

#### 3. Implement the Processing Thread

```python
    def _thread_YourFeature(self):
        """Processing thread for this feature.

        Continuously processes camera frames to detect/analyze/track objects.
        Runs while self.camObject.camOn is True and self.isThreadActive is True.
        """
        self.isThreadActive = True

        while self.camObject.camOn:
            try:
                timeNow = time.time()

                # Throttle if processing faster than capture
                if self.fps.actual >= self.camObject.fps['capture'].actual:
                    with self.camObject.condition:
                        self.camObject.condition.wait(1)

                # Determine if we need to resize the frame
                img_x_y = (self.res_cols, self.res_rows)
                orig_x_y = (self.camObject.res_cols, self.camObject.res_rows)
                if img_x_y == orig_x_y:
                    resOption = None
                else:
                    resOption = img_x_y

                # Get a frame copy (handles color conversion and resizing)
                # colorOption: 'color', 'gray', 'bgr', etc.
                img = self.camObject.getFrameCopy(colorOption='color', resOption=resOption)

                # Process the frame - YOUR ALGORITHM GOES HERE
                # Example:
                # results = self.detector.detect(img)
                results = []  # Replace with actual processing

                # Store results in deque
                # Coordinates should be scaled back to original resolution if needed
                self.deque.append({
                    'detected': len(results) > 0,
                    'results': results,
                    # Add any other data your feature produces
                })

                # Call post-processing function
                self.postFunction(self.postFunctionArgs)

                # Update framerate tracking
                self.camObject.calcFramerate(self.fps, self.idName)

                # Optionally notify parent of status changes
                self.camObject.reachback_pubCamStatus()

            except Exception as e:
                self.stop()
                self.camObject.logger.log(f'Error in {self.idName} thread: {e}',
                                         severity=ub_utils.SEVERITY_ERROR)
                break

            # Check if we should stop
            if not self.isThreadActive:
                self.stop()
                self.camObject.logger.log(f'Stopping {self.idName} thread - no longer active.',
                                         severity=ub_utils.SEVERITY_INFO)
                break

            # Sleep to maintain target framerate
            delta = max(0, timeNow + self.threadSleep - time.time())
            if delta > 0:
                time.sleep(delta)

        # Cleanup when loop exits
        self.stop()
```

#### 4. Implement the Decoration Function (Optional)

If your feature should draw on the video stream, implement `_decorate()`:

```python
    def _decorate(self, img, **kwargs):
        """Draw feature results on the frame.

        This function is called by the Camera's decorateFrame() method
        during streaming/ROS publishing to visualize detections.

        Args:
            img (numpy.ndarray): Frame to draw on (modified in-place)
            **kwargs: Additional arguments (currently unused)
        """
        # Get the latest results
        results = self.deque[0]['results']

        # Draw on the frame
        # Example: draw bounding boxes, labels, keypoints, etc.
        for result in results:
            # Drawing code here using cv2.rectangle, cv2.circle, etc.
            pass
```

#### 5. Implement start() Method

```python
    def start(self):
        """Start the feature processing thread.

        Launches a daemon thread that continuously processes camera frames.
        Optionally registers a decoration function to visualize results.
        """
        try:
            self.camObject.logger.log(f'Starting {self.idName} thread at {self.fps_target} fps',
                                     severity=ub_utils.SEVERITY_INFO)

            # Start the processing thread
            featureThread = threading.Thread(target=self._thread_YourFeature, args=())
            featureThread.daemon = True
            featureThread.start()

            # Register decoration function (if applicable)
            self.decorationID = int(time.time() * 1000)
            self.camObject.dec['dequeAdd'].append({
                'function': self._decorate,
                'idName': self.idName,
                'decorationID': self.decorationID
            })

        except Exception as e:
            self.camObject.logger.log(f'Error in {self.idName} start: {e}',
                                     severity=ub_utils.SEVERITY_ERROR)
```

#### 6. Implement stop() Method

```python
    def stop(self):
        """Stop the feature thread and clean up resources.

        Signals the thread to stop, removes decorations, and clears the result deque.
        Safe to call multiple times.
        """
        try:
            if self.idName in self.camObject.yourfeature:  # Replace with actual dict name
                # Remove decoration
                self.camObject.dec['dequeRemove'].append(self.decorationID)

                self.camObject.logger.log(f'Stopping {self.idName} thread.',
                                         severity=ub_utils.SEVERITY_INFO)

                # Signal thread to stop
                self.isThreadActive = False

                # Clear results
                self.deque.clear()

                # Clean up any resources (models, detectors, etc.)
                # Example:
                # if hasattr(self, 'detector'):
                #     self.detector.cleanup()
            else:
                self.camObject.logger.log(f'In stop, {self.idName} dictionary is not defined',
                                         severity=ub_utils.SEVERITY_ERROR)
        except Exception as e:
            self.camObject.logger.log(f'Error in {self.idName} stop: {e}',
                                     severity=ub_utils.SEVERITY_ERROR)
```

#### 7. Implement edit() Method (Optional)

```python
    def edit(self, res_rows=None, res_cols=None, fps_target=None, postFunction=None):
        """Modify feature parameters while running.

        Args:
            res_rows (int): New processing height
            res_cols (int): New processing width
            fps_target (float): New target framerate
            postFunction (callable): New callback function
        """
        try:
            if res_rows is not None and res_cols is not None:
                if (res_cols, res_rows) != (self.res_cols, self.res_rows):
                    self.res_cols = int(res_cols)
                    self.res_rows = int(res_rows)
                    self.resolution = f'{res_cols}x{res_rows}'

            if fps_target is not None and fps_target != self.fps_target:
                self.fps_target = int(fps_target)
                self.threadSleep = 1.0 / self.fps_target

            if postFunction is not None:
                self.postFunction = postFunction

        except Exception as e:
            self.camObject.logger.log(f'Error in {self.idName} edit: {e}',
                                     severity=ub_utils.SEVERITY_ERROR)
```

#### 8. Add Camera Integration Method

Add a method to the `Camera` class to create instances of your feature:

```python
# In the Camera class:

def addYourFeature(self, idName='default', res_rows=None, res_cols=None,
                   fps_target=5, postFunction=None, postFunctionArgs={}, **kwargs):
    """Add your feature to the camera.

    Args:
        idName (str): Unique identifier for this feature instance
        res_rows (int): Processing height (None = use camera resolution)
        res_cols (int): Processing width (None = use camera resolution)
        fps_target (float): Target processing framerate in Hz
        postFunction (callable): Callback after each processing cycle
        postFunctionArgs (dict): Arguments for postFunction
        **kwargs: Additional feature-specific parameters

    Returns:
        _YourFeature: The created feature instance

    Example:
        camera.addYourFeature('my_detector', fps_target=10)
        # Access results:
        results = camera.yourfeature['my_detector'].deque[0]['results']
        # Stop feature:
        camera.yourfeature['my_detector'].stop()
    """
    try:
        # Set defaults
        res_rows = self.defaultFromNone(res_rows, self.res_rows, int)
        res_cols = self.defaultFromNone(res_cols, self.res_cols, int)

        # Create feature instance
        if not hasattr(self, 'yourfeature'):
            self.yourfeature = {}

        self.yourfeature[idName] = _YourFeature(
            self, idName, res_rows, res_cols, fps_target,
            postFunction, postFunctionArgs, **kwargs
        )

        # Start the feature
        self.yourfeature[idName].start()

        return self.yourfeature[idName]

    except Exception as e:
        self.logger.log(f'Error in addYourFeature: {e}', severity=ub_utils.SEVERITY_ERROR)
        return None
```

### Complete Feature Class Example

See `_Aruco` or `_Ultralytics` in `ub_camera.py` for complete, working examples.

---

## Code Organization Recommendations

### Current State
The current `ub_camera.py` is ~3600 lines in a single file. This works but can be difficult to maintain.

### Recommended Split (Moderate Approach)

```
ub_camera/
├── __init__.py                 # Package initialization, exports
├── camera_base.py              # Camera base class
├── camera_pi.py                # CameraPi implementation
├── camera_ros.py               # CameraROS implementation
├── camera_usb.py               # CameraUSB implementation
├── streaming.py                # StreamingHandler, StreamingServer
├── features/
│   ├── __init__.py
│   ├── aruco.py               # _Aruco class
│   ├── barcode.py             # _Barcode class
│   ├── calibrate.py           # _Calibrate class
│   ├── facedetect.py          # _FaceDetect class
│   ├── roi.py                 # _ROI class
│   ├── timelapse.py           # _Timelapse class
│   └── ultralytics.py         # _Ultralytics class
└── utils.py                    # _make_fps_dict, helper functions
```

### Migration Strategy

1. **Create the package structure** above
2. **Move classes** to appropriate files
3. **Update imports** in each file
4. **Export public API** in `__init__.py`:
   ```python
   from .camera_base import Camera
   from .camera_pi import CameraPi
   from .camera_ros import CameraROS
   from .camera_usb import CameraUSB
   from .streaming import StreamingHandler, StreamingServer
   ```
5. **Update client code** to use: `from ub_camera import CameraUSB`
6. **Test thoroughly** to ensure nothing broke

### Fine-Grained Split (Alternative)

For maximum modularity, put each camera and feature in its own file:
```
ub_camera/
├── __init__.py
├── base.py                     # Camera class
├── cameras/
│   ├── pi.py
│   ├── ros.py
│   └── usb.py
├── features/
│   ├── aruco.py
│   ├── barcode.py
│   ├── calibrate.py
│   ├── facedetect.py
│   ├── roi.py
│   ├── timelapse.py
│   └── ultralytics.py
└── streaming/
    ├── handler.py
    └── server.py
```

This maximizes separation but creates more files to manage.

---

## Best Practices

### Threading
- ✅ Always use daemon threads: `thread.daemon = True`
- ✅ Use `threading.Condition` for frame synchronization
- ✅ Check `self.camOn` and `self.isThreadActive` in loops
- ✅ Use `try/except` to handle errors gracefully
- ✅ Call `self.stop()` on errors to clean up
- ❌ Don't use `thread.join()` - can cause deadlocks

### Frame Access
- ✅ Use `getFrameCopy()` for processing - never modify the original
- ✅ Specify `colorOption` explicitly ('color', 'gray', 'bgr')
- ✅ Use `resOption` to resize frames for processing efficiency
- ✅ Scale coordinates back to original resolution if needed
- ❌ Don't access `self.camObject.frame` directly

### Error Handling
- ✅ Wrap all code in try/except blocks
- ✅ Use `self.camObject.logger.log()` for all messages
- ✅ Use appropriate severity levels (INFO, WARNING, ERROR)
- ✅ Clean up resources in `except` and `finally` blocks
- ❌ Don't use `print()` - use the logger

### Configuration
- ✅ Accept None for resolution parameters to use camera defaults
- ✅ Use `defaultFromNone()` to handle None values
- ✅ Validate input parameters in `__init__`
- ✅ Document all parameters with types and defaults
- ❌ Don't assume parameters are valid without checking

### Decorations
- ✅ Draw on the frame in-place - don't return a new image
- ✅ Use OpenCV drawing functions (cv2.rectangle, cv2.circle, etc.)
- ✅ Check if results exist before drawing
- ✅ Use consistent colors and line thickness
- ❌ Don't do heavy computation in `_decorate()` - just draw

### Resource Management
- ✅ Clean up resources in `stop()` method
- ✅ Set flags before starting threads
- ✅ Clear deques on stop
- ✅ Release hardware resources (cameras, models, etc.)
- ❌ Don't leave threads running or resources allocated

### Documentation
- ✅ Add comprehensive docstrings to all classes and methods
- ✅ Use Google-style docstring format
- ✅ Include usage examples in class docstrings
- ✅ Document threading behavior and requirements
- ✅ Explain what goes in the deque and what format
- ❌ Don't assume users will read the code

---

## Testing Your Changes

### Basic Testing Checklist

#### For New Camera Classes:
```python
# 1. Basic initialization
camera = CameraYourName(paramDict={'res_rows': 480, 'res_cols': 640, 'fps_target': 30})

# 2. Start capture
camera.start()
time.sleep(2)

# 3. Get a frame
frame = camera.getFrameCopy()
assert frame is not None
print(f"Frame shape: {frame.shape}")

# 4. Check framerate
print(f"Capture FPS: {camera.fps['capture'].actual}")

# 5. Test streaming
camera.startStream(port=8000)
# Visit https://localhost:8000/stream.mjpg in browser

# 6. Test with a feature
camera.addAruco('DICT_APRILTAG_36h11', fps_target=20)
time.sleep(5)
print(f"Detected markers: {camera.aruco['default'].deque[0]['ids']}")

# 7. Cleanup
camera.shutdown()
```

#### For New Feature Classes:
```python
# 1. Create camera (use any camera type)
camera = CameraUSB(paramDict={'res_rows': 480, 'res_cols': 640, 'fps_target': 30})
camera.start()

# 2. Add your feature
camera.addYourFeature('test', fps_target=10)
time.sleep(5)

# 3. Check results
results = camera.yourfeature['test'].deque[0]
print(f"Results: {results}")

# 4. Check framerate
print(f"Feature FPS: {camera.yourfeature['test'].fps.actual}")

# 5. Test with streaming (check decorations)
camera.startStream(port=8000)
# Visit https://localhost:8000/stream.mjpg - verify decorations appear

# 6. Test stop/restart
camera.yourfeature['test'].stop()
time.sleep(1)
camera.addYourFeature('test2', fps_target=5)

# 7. Cleanup
camera.shutdown()
```

### Edge Cases to Test
- ✅ Start/stop multiple times
- ✅ Add multiple instances of the same feature
- ✅ Change resolution/framerate dynamically
- ✅ Run with no features, one feature, many features
- ✅ Test with streaming on/off
- ✅ Test with ROS publishing on/off
- ✅ Test error conditions (bad parameters, missing hardware)
- ✅ Test resource cleanup (no memory leaks, no hanging threads)

### Performance Testing
```python
# Check CPU usage
import psutil
import time

camera = CameraYourName(paramDict={'res_rows': 480, 'res_cols': 640, 'fps_target': 30})
camera.start()
camera.addYourFeature('test', fps_target=10)

process = psutil.Process()
for i in range(10):
    cpu_percent = process.cpu_percent(interval=1)
    print(f"CPU usage: {cpu_percent}%")
    print(f"Capture FPS: {camera.fps['capture'].actual}")
    print(f"Feature FPS: {camera.yourfeature['test'].fps.actual}")

camera.shutdown()
```

---

## Quick Reference

### Common Code Patterns

#### Get a frame for processing:
```python
img = self.camObject.getFrameCopy(colorOption='gray', resOption=(640, 480))
```

#### Log a message:
```python
self.camObject.logger.log('Message here', severity=ub_utils.SEVERITY_INFO)
```

#### Wait for next frame:
```python
with self.camObject.condition:
    self.camObject.condition.wait(timeout_seconds)
```

#### Calculate framerate:
```python
self.camObject.calcFramerate(self.fps, 'feature_name')
```

#### Store results in deque:
```python
self.deque.append({'key': value, 'key2': value2})
```

#### Draw on frame:
```python
cv2.rectangle(img, (x1, y1), (x2, y2), color=(0, 255, 0), thickness=2)
cv2.putText(img, 'Text', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
```

---

## Additional Resources

- **OpenCV Documentation**: https://docs.opencv.org/
- **Threading Documentation**: https://docs.python.org/3/library/threading.html
- **ROS cv_bridge**: http://wiki.ros.org/cv_bridge
- **Existing Examples**: See `CameraUSB`, `_Aruco`, `_Ultralytics` in `ub_camera.py`

---

## Support

For questions or issues:
1. Check existing camera/feature classes for examples
2. Review this guide
3. Check the module docstrings in `ub_camera.py`
4. Test with minimal examples before adding complexity

---

**End of Developer Guide**
