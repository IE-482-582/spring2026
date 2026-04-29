var stream = [];

function showStream(id, show)  {
	try {
		if (show)  {
			document.getElementById('vidStream_' + id).src = document.getElementById('txt_stream_addr_' + id).value;
		}  else  {
			document.getElementById('vidStream_' + id).src = "images/transparent_pixel.png";			
			return;						
		}			
	}  catch (error) {
		console.error("ERROR: ", error.message);
	}		
}

function toggleStream(id)  {
	try {
		const stream = document.getElementById('vidStream_' + id);
		const btn = document.getElementById('btn_tgl_stream_' + id);
		
		if (stream.src == document.getElementById('txt_stream_addr_' + id).value)  {
			// Now we want to hide
			stream.src = "images/transparent_pixel.png";	
			btn.innerText = 'show';
		}  else  {
			// Now we want to show
			stream.src = document.getElementById('txt_stream_addr_' + id).value;
			btn.innerText = 'hide';
		}		
	}  catch (error) {
		console.error("ERROR: ", error.message);
	}		
}

class Stream {
	constructor(id=null, parentDiv=null, addr=null) {
		if (id == null)   {  console.error('id should be an integer');  return;  }
		if (parentDiv == null)   {  console.error('must specify parentDiv');  return;  }
		
		stream[id] = {addr: addr};
		
		this.addStreamDiv(id, parentDiv, addr);
	}
	
	addStreamDiv(id, parentDiv, addr)  {
		var newDiv = document.createElement("div");
		newDiv.setAttribute("id", "stream_" + id);   // Give the stream div an ID
		newDiv.setAttribute("class", "streamParent");      // Specify the style properties of this div
		
		newDiv.innerHTML =  '<div style="text-align:center;background:#3d3d3d;color:white;font-weight:bold;border-radius:5px;margin-bottom:4px;">' + id + '</div>' +
							'<!--<div>camID: <input type="text" id="txt_stream_camID_' + id + '" style="width:100px;"></div>-->' +
							'<div>URL: <input type="text" id="txt_stream_addr_' + id + '" style="width:250px;">' +
							'   <button onClick="toggleStream(' + id + ');" id="btn_tgl_stream_' + id + '">show</button></div>' + 
							'<div>' +
							'  <img id="vidStream_' + id + '" src="images/transparent_pixel.png" style="width:320px;">' +
							'</div>';
			
		document.getElementById(parentDiv).appendChild(newDiv);	

		if (addr != null)  {
			document.getElementById("txt_stream_addr_" + id).value = addr;
			showStream(id);
		}
		
	}		
}


/*
	function startCam(id, ipAddr, port)  {
		if (port > 0)  {
			document.getElementById('vidStream_' + id).src = "https://" + ipAddr + ":" + port + "/stream.mjpg";
		}  else  {
			document.getElementById('vidStream_' + id).src = "images/transparent_pixel.png";			
			return;						
		}			
	}
	function showVideo()  {
		document.getElementById('vidStream_1').src = "https://localhost:8001/stream.mjpg";		
	}
*/
