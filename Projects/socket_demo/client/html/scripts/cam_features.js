
function toggleConfig(feature)  {
	// Make note of this div's visibility:
	var disp = document.getElementById(feature).style.display;

	// Hide all divs
	for (const f of configDivs)  {
		// console.log(f);
		document.getElementById(f).style.display = '';
	}
	
	// If the given div was originally hidden, display it now
	if (disp == '')  {
		document.getElementById(feature).style.display = 'block';
	}
}


function arucoConfigDiv()  {
	var html = 	'<h4 class="subtitle">ArUco Config</h4>' +
				'<table>' +
				'	<!--<tr><td>Camera Angle:</td><td>???</td></tr>-->' +
				'	<!--<tr><td>Resolution:</td><td><select id="arucoResolution"><option value="320x240" selected>320x240 (default)</option><option value="640x480">640x480</option></select></td></tr>-->' +
				'	<tr><td>camID:</td><td><select id="arucoCamID"></select></td></tr>' +
				'	<tr><td>Frame Rate:</td><td><input type="number" id="arucoFramerate" value=5></td></tr>' +
				'	<tr><td>Tag Type:</td><td><select id="arucoTagType"><option>DICT_APRILTAG_16h5</option><option selected>DICT_APRILTAG_36h11</option></select></td></td></tr>' +
				'	<tr><td>Tag Size:</td><td><select id="arucoTagSize" disabled><option>unknown</option></select></td></tr>' +
				'	<tr><td>Action:</td><td><select id="arucoAction">' +
				'									<option value="id">ID tags in view</option>' +
				'									<option value="track" selected>Track specific tag ID</option>' + 
				'									<option value="search any" disabled>Search w/in Geofence for any tag</option>' +
				'									<option value="search this" disabled>Search w/in Geofence for specific tag</option>' +
				'									<option value="count" disabled>Count Inventory</option>' +
				'									</select></td></tr>' +
				'	<tr><td>ID to Track:</td><td><input type="number" id="arucoTrackID"></td></tr>' +
				'	<tr><td colspan=2><center>' +
				'		<button id="btnArucoSet" onClick="arucoSet();">set ArUco</button>' +
				'		<button id="btnArucoClear" onClick="arucoRemove();">remove ArUco</button>' +
				'	</center></td></tr>' +
				'</table>';

	return html;
}
	
function arucoRemove()  {
	var data = {};
	data.camID   = document.getElementById('arucoCamID').value;
	data.tagType = document.getElementById('arucoTagType').value;
	
	socket_client.emit('cam_control', ['arucoStop', data]);	
}


function arucoSet()  {
	var data = {};
	data.camID = document.getElementById('arucoCamID').value;
	data.framerate = document.getElementById('arucoFramerate').value;
	data.tagType = document.getElementById('arucoTagType').value;
	data.action = document.getElementById('arucoAction').value;
	data.trackID = document.getElementById('arucoTrackID').value;

	console.log(data);
	
	socket_client.emit('cam_control', ['arucoStart', data]);
}



function barcodeConfigDiv()  {
	var html =  '<h4 class="subtitle">Barcode Config</h4>' +
				'<table>' +
				'	<tr><td>camID:</td><td><select id="barcodeCamID"></select></td></tr>' +
				'	<tr><td>Frame Rate:</td><td><input type="number" id="barcodeFramerate" value=5></td></tr>' +
				'	<tr><td>Action:</td><td><select id="barcodeAction">' +
				'									<option value="read">Read barcodes in view</option>' +
				'									<option value="count" disabled>Count Inventory</option>' +
				'									</select></td></tr>' +
				'	<tr><td colspan=2><center>' +
				'		<button id="btnBarcodeSet" onClick="barcodeStart();">Start Barcode</button>' +
				'		<button id="btnBarcodeClear" onClick="barcodeStop();">Stop Barcode</button>' +
				'	</center></td></tr>' +
				'</table>';
	return html;						
}


function barcodeStart()  {
	var data = {};
	data.camID = document.getElementById('barcodeCamID').value;
	data.framerate = document.getElementById('barcodeFramerate').value;
	data.action    = document.getElementById('barcodeAction').value;
	
	socket_client.emit('cam_control', ['barcodeStart', data]);	
}

function barcodeStop()  {
	var data = {};
	data.camID = document.getElementById('barcodeCamID').value;
	
	socket_client.emit('cam_control', ['barcodeStop', data]);		
}


function facedetectConfigDiv()  {
	var html =  '<h4 class="subtitle">Face Detection Config</h4>' +
				'<table>' +
				'	<tr><td>camID:</td><td><select id="facedetectCamID"></select></td></tr>' +
				'	<tr><td>Frame Rate:</td><td><input type="number" id="facedetectFramerate" value=5></td></tr>' +
				'	<tr><td>Action:</td><td><select id="facedetectAction">' +
				'									<option value="read">Display detected faces in view</option>' +
				'									</select></td></tr>' +
				'	<tr><td>Confidence Threshold:</td><td><input type="number" id="facedetectConfThreshold" value=0.7 min=0.0 max=1.0, step=0.05></td></tr>' +
				'	<tr><td>DNN:</td><td><select id="facedetectDNN">' +
				'									<option value="caffe">Caffe (fp16)</option>' +
				'									<option value="pb">PB (8bit)</option>' +
				'									</select></td></tr>' +
				'	<tr><td>Device:</td><td><select id="facedetectDevice">' +
				'									<option value="cpu" default>CPU</option>' +
				'									<option value="gpu">GPU (experimental)</option>' +
				'									</select></td></tr>' +
				'	<tr><td colspan=2><center>' +
				'		<button id="btnFacedetectSet" onClick="facedetectStart();">Start Face Detection</button>' +
				'		<button id="btnFacedetectClear" onClick="facedetectStop();">Stop Face Detection</button>' +
				'	</center></td></tr>' +
				'</table>';
	return html;						
}

function facedetectStart()  {
	var data = {};
	data.camID          = document.getElementById('facedetectCamID').value;
	data.framerate      = document.getElementById('facedetectFramerate').value;
	data.action         = document.getElementById('facedetectAction').value;
	data.conf_threshold = document.getElementById('facedetectConfThreshold').value;
	data.dnn            = document.getElementById('facedetectDNN').value;
	data.device         = document.getElementById('facedetectDevice').value;
	
	socket_client.emit('cam_control', ['facedetectStart', data]);	
}

function facedetectStop()  {
	var data = {};
	data.camID = document.getElementById('facedetectCamID').value;
	
	socket_client.emit('cam_control', ['facedetectStop', data]);		
}



function ultraConfigDiv()  {
	var html =  '<h4 class="subtitle">Ultralytics Config</h4>' +
				'<table>' +
				'	<tr><td>camID:</td><td><select id="ultraCamID"></select></td></tr>' +
				'	<tr><td>modelName:</td><td><select id="ultraModelName">' +
				'									<option value="yolo11n.pt">yolo11n.pt (detect)</option>' +
				'									<option value="yolo11n-pose.pt">yolo11n-pose.pt (pose)</option>' +
				'									<option value="yolo11n-seg.pt">yolo11n-seg.pt (segment)</option>' +
				'									<option value="yolo11n-obb.pt">yolo11n-obb.pt (oriented bounding boxes)</option>' +	
				'									</select></td></tr>' +
				'	<tr><td>Track?</td><td><input type="checkbox" id="ultraTrack"></td></tr>' +
				'	<tr><td>Frame Rate:</td><td><input type="number" id="ultraFramerate" value=5></td></tr>' +
				'	<tr><td>Action:</td><td><select id="ultraAction">' +
				'									<option value="read">?????</option>' +
				'									</select></td></tr>' +
				'	<tr><td>Confidence Threshold:</td><td><input type="number" id="ultraConfThreshold" value=0.7 min=0.0 max=1.0, step=0.05></td></tr>' +
				'	<tr><td>Draw Box?</td><td><input type="checkbox" id="ultraDrawBox" checked></td></tr>' +
				'	<tr><td>Draw Label?</td><td><input type="checkbox" id="ultraDrawLabel" checked></td></tr>' +
				'	<tr><td>Draw Mask Outline?</td><td><input type="checkbox" id="ultraMaskOutline" unchecked></td></tr>' +				
				'	<tr><td colspan=2><center>' +
				'		<button id="btnUltraSet" onClick="ultraStart();">Start Ultralytics</button>' +
				'		<button id="btnUltraClear" onClick="ultraStop();">Stop Ultralytics</button>' +
				'	</center></td></tr>' +
				'</table>';
	return html;						
}

function ultraStart()  {
	var data = {};
	data.camID          = document.getElementById('ultraCamID').value;
	data.modelName      = document.getElementById('ultraModelName').value;
	data.track          = document.getElementById('ultraTrack').checked;
	data.framerate      = parseInt(document.getElementById('ultraFramerate').value);
	data.action         = document.getElementById('ultraAction').value;
	data.conf_threshold = parseFloat(document.getElementById('ultraConfThreshold').value);
	data.drawBox        = document.getElementById('ultraDrawBox').checked;
	data.drawLabel      = document.getElementById('ultraDrawLabel').checked;
	data.maskOutline    = document.getElementById('ultraMaskOutline').checked;
		
	socket_client.emit('cam_control', ['ultraStart', data]);	
}

function ultraStop()  {
	var data = {};
	data.camID = document.getElementById('ultraCamID').value;

	if (document.getElementById('ultraTrack').checked)  {
		data.idName = "track";
	}  else if (document.getElementById('ultraModelName').value.includes("-seg"))  {
		data.idName = "segment";
	}  else if (document.getElementById('ultraModelName').value.includes("-pose"))  {
		data.idName = "pose";
	}  else if (document.getElementById('ultraModelName').value.includes("-obb"))  {
		data.idName = "obb";
	}  else  {
		data.idName = "detect";		
	}	
	
	socket_client.emit('cam_control', ['ultraStop', data]);		
}


class CameraConfig  {
	// new CameraConfig(feature='barcode', btnDiv='camBtnsDiv', btnText='Barcode', cfgDiv='camConfigDiv', divFunc=barcodeConfigDiv);
	constructor(feature='barcode', btnDiv=null, btnText='Barcode', cfgDiv=null, divFunc=null) {
		if (feature == null)  {  console.error('feature cannot be null');  return;  }
		if (btnDiv == null)   {  console.error('btnDiv cannot be null');   return;  }
		if (btnText == null)  {  console.error('btnText cannot be null');  return;  }
		if (cfgDiv == null)   {  console.error('cfgDiv cannot be null');   return;  }
		if (divFunc == null)  {  console.error('divFunc cannot be null');  return;  }
		
		// Create a pop-up div so we can configure this feature
		this.createConfigDiv(feature, cfgDiv);
		
		// Make note of the config div, so we can track all of these divs
		configDivs.push(feature);

		this.addMainButton(feature, btnDiv, btnText);
	}
		
	addMainButton(feature, parentDiv, btnText)  {
		/*
		Add a button to the main page.
		feature: Text label categorizing this button (e.g., 'aruco', 'roi', 'face', 'barcode')
		parentDiv: Our button will be added to this div
		btnText: Text to appear in the button
		*/

		const myDiv = document.getElementById(parentDiv); 
		const myBtn = document.createElement("button");

		// Set button properties
		myBtn.textContent = btnText; // Set the text displayed on the button
		// FIXME!
		// myBtn.setAttribute("name", "myButton");  // Set the name attribute
		// myBtn.setAttribute("id",   "myButton");  // Set the id attribute
		// myBtn.setAttribute("type", "button"); // Set the type attribute to "button" to prevent form submission if needed
		// set title attribute?
		myBtn.setAttribute("title", btnText);  // Set the name attribute
		myBtn.setAttribute("class", "camConfigBtn");
		
		// Add an event listener for this button:
		myBtn.addEventListener("click", function() {
			toggleConfig(feature); 
		});
		// FIXME -- Also add "touchstart" event listener for mobile?
		// (I think 'click' will also work on mobile.

		// Append the button to the div
		myDiv.appendChild(myBtn);	
	}

	createConfigDiv(feature, parentDiv)  {

		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", feature);      // Give the config div an ID
		
		newDiv.innerHTML = divFunc();
			
		// If we didn't specify a parent div, just add it to the main document body:	
		if (parentDiv == null)  {  
			parentDiv = document.body;  
			newDiv.setAttribute("class", "configFloat");  // Specify the style properties of this div
		}  else  {
			parentDiv = document.getElementById(parentDiv);
			newDiv.setAttribute("class", "configInline");  // Specify the style properties of this div
		}	
		parentDiv.appendChild(newDiv);

	}
}


// ─── Local Camera ─────────────────────────────────────────────────────────────

function localCameraConfigDiv()  {
	var html =  '<h4 class="subtitle">Local Camera Config</h4>' +
				'<table>' +
				'	<tr><td>camID:</td><td><input type="text" id="localCamID" value="local_cam" placeholder="local_cam"></td></tr>' +
				'	<tr><td>Output Port:</td><td><input type="number" id="localCamPort" value="8000" placeholder="8000"></td></tr>' +
				'	<tr><td>Device:</td><td><input type="text" id="localCamDevice" value="0" placeholder="0 or /dev/video0"></td></tr>' +
				'	<tr><td>Resolution:</td><td><select id="localCamResolution">' +
				'									<option value="320x240">320x240</option>' +
				'									<option value="640x480" selected>640x480</option>' +
				'									<option value="800x600">800x600</option>' +
				'									<option value="1280x720">1280x720</option>' +
				'									<option value="1920x1080">1920x1080</option>' +
				'									</select></td></tr>' +
				'	<tr><td colspan=2><center>' +
				'		<button id="btnLocalCameraStart" onClick="localCameraStart();">Start Camera</button>' +
				'		<button id="btnLocalCameraStop" onClick="localCameraStop();">Stop Camera</button>' +
				'		<button id="btnLocalCameraRestart" onClick="localCameraRestart();">Restart Camera</button>' +
				'	</center></td></tr>' +
				'</table>';
	return html;
}


function localCameraStart()  {
	var data = {};
	data.camID = document.getElementById('localCamID').value;
	data.outputPort = parseInt(document.getElementById('localCamPort').value);
	data.device = document.getElementById('localCamDevice').value;
	data.resolution = document.getElementById('localCamResolution').value;

	console.log('Starting local camera:', data);
	socket_client.emit('cam_control', ['localCameraStart', data]);
}

function localCameraStop()  {
	var data = {};
	data.camID = document.getElementById('localCamID').value;

	console.log('Stopping local camera:', data);
	socket_client.emit('cam_control', ['localCameraStop', data]);
}

function localCameraRestart()  {
	var data = {};
	data.camID = document.getElementById('localCamID').value;
	data.outputPort = parseInt(document.getElementById('localCamPort').value);
	data.device = document.getElementById('localCamDevice').value;
	data.resolution = document.getElementById('localCamResolution').value;

	console.log('Restarting local camera:', data);
	socket_client.emit('cam_control', ['localCameraRestart', data]);
}


// ─── Feature registration ─────────────────────────────────────────────────────
// Add a new CameraConfig line here for each feature you create.

function loadFeatures()  {
	new CameraConfig(feature='localcam',   btnDiv='camBtnsDiv', btnText='Local Camera',  cfgDiv='camConfigDiv', divFunc=localCameraConfigDiv);
	new CameraConfig(feature='aruco',      btnDiv='camBtnsDiv', btnText='ArUco',         cfgDiv='camConfigDiv', divFunc=arucoConfigDiv);
	new CameraConfig(feature='barcode',    btnDiv='camBtnsDiv', btnText='Barcode',        cfgDiv='camConfigDiv', divFunc=barcodeConfigDiv);
	new CameraConfig(feature='facedetect', btnDiv='camBtnsDiv', btnText='Face Detect',    cfgDiv='camConfigDiv', divFunc=facedetectConfigDiv);
	new CameraConfig(feature='ultra',      btnDiv='camBtnsDiv', btnText='Ultralytics',    cfgDiv='camConfigDiv', divFunc=ultraConfigDiv);

	new Stream(1, 'camStreamDiv', 'https://localhost:8001/stream.mjpg');
	new Stream(2, 'camStreamDiv', 'https://localhost:8002/stream.mjpg');
	new Stream(3, 'camStreamDiv', 'https://localhost:8003/stream.mjpg');
}
