// ─── Globals ─────────────────────────────────────────────────────────────────

var configDivs = [];
var robots     = {};
var activeRobot    = null;
var sessionTimer   = null;
var sessionEndTime = null;
var camStreams  = {};

var socket_host;
var socket_client;

// ─── Helpers ──────────────────────────────────────────────────────────────────

function addToSelectList(lst, itmVal, itmTxt=null)  {
	var selList = document.getElementById(lst);
	if (!isValueInOptions(selList, itmVal))  {
		var el = document.createElement("option");
		if (itmTxt == null)  { itmTxt = itmVal; }
		el.textContent = itmTxt;
		el.value = itmVal;
		selList.appendChild(el);
	}
}

function isValueInOptions(sel, val)  {
	for (var i = 0; i < sel.options.length; i++)  {
		if (sel.options[i].value == val)  { return true; }
	}
	return false;
}

function fmtWait(sec)  {
	if (sec == null || sec < 0)  { return '\u2014'; }
	const h = Math.floor(sec / 3600);
	const m = Math.floor((sec % 3600) / 60);
	const s = sec % 60;
	return [h, m, s].map(v => String(v).padStart(2, '0')).join(':');
}

// ─── Camera table ─────────────────────────────────────────────────────────────

function cameraAdd(camID, msg, source = 'client')  {
	const table  = document.getElementById("tbl_cameras");
	const newRow = table.insertRow();

	const cell1 = newRow.insertCell();
	cell1.id = "td_camera_id_" + camID;
	cell1.textContent = camID;

	const cell2 = newRow.insertCell();
	cell2.id = "td_camera_url_" + camID;
	cell2.textContent = msg.url;

	const cell3 = newRow.insertCell();
	cell3.innerHTML = "<select id='sel_camera_" + camID + "'><option>1</option><option>2</option><option>3</option></select>";

	const cell4 = newRow.insertCell();
	cell4.innerHTML = "<button id='btn_camera_watch" + camID + "' onClick='cameraStream(\"" + camID + "\");'>Stream</button>";
	cell4.id = "td_robot_action_" + camID;

	// Only add client cameras to CV feature select lists (host cameras are not accessible to client.py)
	if (source === 'client')  {
		addToSelectList('arucoCamID',      camID);
		addToSelectList('barcodeCamID',    camID);
		addToSelectList('facedetectCamID', camID);
		addToSelectList('ultraCamID',      camID);
	}
}

function cameraStream(camID)  {
	const streamID = document.getElementById('sel_camera_' + camID).value;
	document.getElementById('txt_stream_addr_' + streamID).value = camStreams[camID].url;
	toggleStream(streamID);
}

// ─── Robot table ──────────────────────────────────────────────────────────────

function robotAdd(robotID, msg)  {
	if (document.getElementById('tr_robot_' + robotID))  { return; }

	const table  = document.getElementById("tbl_robots");
	const newRow = table.insertRow();

	const cell1 = newRow.insertCell();
	cell1.id = "td_robot_id_" + robotID;
	cell1.textContent = robotID;

	const cell2 = newRow.insertCell();
	cell2.id = "td_robot_name_" + robotID;

	const cell3 = newRow.insertCell();
	cell3.id = "td_robot_type_" + robotID;

	const cell4 = newRow.insertCell();
	cell4.id = "td_robot_status_" + robotID;

	const cell5 = newRow.insertCell();
	cell5.id = "td_robot_queueLength_" + robotID;

	const cell6 = newRow.insertCell();
	cell6.id = "td_robot_yourPosition_" + robotID;

	const cell7 = newRow.insertCell();
	cell7.id = "td_robot_estWait_" + robotID;

	const cell8 = newRow.insertCell();
	cell8.innerHTML  = "<button id='btn_robot_join" + robotID + "' onClick='joinQueue(" + robotID + ");'>Join</button>";
	cell8.innerHTML += "<button id='btn_robot_exit" + robotID + "' onClick='exitQueue(" + robotID + ");'>Exit</button>";
	cell8.id = "td_robot_action_" + robotID;
}

function robotRemove(robotID)  {
	document.getElementById('btn_robot_join' + robotID).disabled = true;
	document.getElementById('td_robot_status_' + robotID).innerText = 'OFFLINE';
}

function robotUpdate(robotID)  {
	const r = robots[robotID];
	document.getElementById('td_robot_name_'         + robotID).innerText = r.name;
	document.getElementById('td_robot_type_'         + robotID).innerText = r.type;
	document.getElementById('td_robot_status_'       + robotID).innerText = r.availability.toUpperCase();
	document.getElementById('td_robot_queueLength_'  + robotID).innerText = r.queueLength;
	document.getElementById('td_robot_yourPosition_' + robotID).innerText = r.yourPosition;
	document.getElementById('td_robot_estWait_'      + robotID).innerText =
		r.yourPosition === 0  ? 'active' :
		r.yourPosition  <  0  ? '\u2014' :
		fmtWait(r.estWaitSec);

	const joinBtn = document.getElementById('btn_robot_join' + robotID);
	const exitBtn = document.getElementById('btn_robot_exit' + robotID);
	if (r.yourPosition < 0)  {
		// Not yet in queue
		joinBtn.disabled = (r.availability !== 'available');
		exitBtn.disabled = true;
		exitBtn.innerText = 'Exit';
		if (activeRobot == robotID)  {
			activeRobot = null;
			if (sessionTimer)  { clearInterval(sessionTimer); sessionTimer = null; }
			document.getElementById('lbl_active_robot').innerText = '\u00a0';
		}
	}  else if (r.yourPosition > 0)  {
		// In queue
		if (activeRobot == robotID)  { activeRobot = null; }
		joinBtn.disabled = true;
		exitBtn.disabled = false;
		exitBtn.innerText = 'Leave Queue';
	}  else  {
		// In service (yourPosition == 0)
		joinBtn.disabled = true;
		exitBtn.disabled = false;
		exitBtn.innerText = 'End Session';
	}
}

// ─── Queue actions ────────────────────────────────────────────────────────────

function joinQueue(robotID)  {
	socket_host.emit('userreq', ["join", robotID]);
}

function exitQueue(robotID)  {
	socket_host.emit('userreq', ["exit", robotID]);
}

// ─── Robot arm controls ───────────────────────────────────────────────────────

var joints = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'gripper_joint'];

var joint = {};
for (const j of joints)  {
	joint[j] = {torque: undefined, angle_deg: undefined, min_angle: undefined, max_angle: undefined, sliding: false};
}

function slidingOn(j)  {
	joint[j].sliding = true;
	document.getElementById('txt_' + j).value = document.getElementById('rng_' + j).value;
}

function slidingOff(j)  {
	joint[j].sliding = false;
	document.getElementById('txt_' + j).value = document.getElementById('rng_' + j).value;
	var cmd = {};
	cmd[j] = document.getElementById('txt_' + j).value;
	socket_host.emit('command', [activeRobot, [cmd]]);
}

function move()  {
	var cmd = {};
	for (const j of joints)  {
		if (document.getElementById('chk_' + j).checked)  {
			cmd[j] = document.getElementById('txt_' + j).value;
		}
	}
	socket_host.emit('command', [activeRobot, [cmd]]);
}

function increment(j, degrees)  {
	try  {
		const new_angle = joint[j].angle_deg + degrees;
		if (joint[j].min_angle <= new_angle && new_angle <= joint[j].max_angle)  {
			var cmd = {};
			cmd[j] = new_angle;
			socket_host.emit('command', [activeRobot, [cmd]]);
		}
	}  catch (error)  {
		console.error("Error: ", error.message);
	}
}

function relax(j)  {
	var cmd = {};
	cmd[j] = true;
	socket_host.emit('relax', [activeRobot, [cmd]]);
}

function rigid(j)  {
	var cmd = {};
	cmd[j] = false;
	socket_host.emit('relax', [activeRobot, [cmd]]);
}

// ─── Session countdown ────────────────────────────────────────────────────────

function updateCountdown()  {
	const secRemaining = Math.round((new Date(sessionEndTime) - Date.now()) / 1000);
	const lbl = document.getElementById('lbl_active_robot');
	if (secRemaining > 0)  {
		lbl.innerText = 'Robot ' + activeRobot + ' \u2014 ' + secRemaining + 's remaining';
	}  else  {
		lbl.innerText = 'Robot ' + activeRobot + ' \u2014 session ended';
		clearInterval(sessionTimer);
		sessionTimer = null;
	}
}

// ─── Joint display (shared by sessionstart and status) ────────────────────────

function applyJointStatus(joints)  {
	var disableMove = true;
	for (const [key, value] of Object.entries(joints))  {
		if (document.getElementById('chk_' + key))  {
			joint[key].angle_deg = value.angle_deg;
			joint[key].min_angle = value.min_angle;
			joint[key].max_angle = value.max_angle;

			document.getElementById('chk_'        + key).disabled = !value.OK;
			document.getElementById('txt_'        + key).disabled = !value.OK;
			document.getElementById('rng_'        + key).disabled = !value.OK;
			document.getElementById('btn_incr_a_' + key).disabled = !value.OK;
			document.getElementById('btn_incr_b_' + key).disabled = !value.OK;
			document.getElementById('btn_relax_'  + key).disabled = !value.OK;
			document.getElementById('btn_rigid_'  + key).disabled = !value.OK;

			if (!value.OK)  { document.getElementById('chk_' + key).checked = false; }

			document.getElementById('pos_' + key).textContent = value.angle_deg.toFixed(1);
			document.getElementById('min_' + key).textContent = value.min_angle.toFixed(1);
			document.getElementById('max_' + key).textContent = value.max_angle.toFixed(1);

			document.getElementById('rng_' + key).min = value.min_angle;
			document.getElementById('rng_' + key).max = value.max_angle;
			if (!joint[key].sliding)  {
				document.getElementById('rng_' + key).value = value.angle_deg;
			}

			if (document.getElementById('chk_' + key).checked)  { disableMove = false; }
		}  else  {
			console.log('need to add ' + key + ' to table');
		}
	}
	document.getElementById('btn_robot_move').disabled = disableMove;
}

// ─── Connection ───────────────────────────────────────────────────────────────

function connect()  {
	const host_server   = 'https://' + document.getElementById('host_ip').value   + ':' + document.getElementById('host_port').value;
	const client_server = 'https://' + document.getElementById('client_ip').value + ':' + document.getElementById('client_port').value;

	document.getElementById('loginBkgrnd').style.display = 'none';

	loadFeatures();

	// Socket to the host (robot broker) server
	socket_host = io(host_server, {
		ackTimeout: 10000,
		retries: 0,
		query: { role: "user", name: document.getElementById('user_name').value.trim() || undefined },
	});

	// Socket to the local client server
	socket_client = io(client_server, {
		ackTimeout: 10000,
		retries: 0,
	});

	// ── Host events ──

	socket_host.on('notice', (msg) => {
		const newNotice = document.createElement("li");
		const prefix = (msg.robotID != null) ? '[Robot ' + msg.robotID + '] ' : '';
		newNotice.textContent = prefix + msg.message;
		newNotice.className = 'notice-' + msg.type;
		ul_notices.insertBefore(newNotice, ul_notices.firstChild);
	});

	socket_host.on('sessionstart', (msg) => {
		activeRobot    = msg.robotID;
		sessionEndTime = msg.endTime;

		if (sessionTimer)  { clearInterval(sessionTimer); }
		updateCountdown();
		sessionTimer = setInterval(updateCountdown, 1000);

		const urlInput = document.getElementById('txt_stream_addr_1');
		if (urlInput && msg.cameraURL)  {
			urlInput.value = msg.cameraURL;
			showStream(1, true);
			const btn = document.getElementById('btn_tgl_stream_1');
			if (btn)  { btn.innerText = 'hide'; }
		}

		const camID = 'robot_' + msg.robotID;
		if (msg.cameraURL && !camStreams[camID])  {
			camStreams[camID] = { camID, url: msg.cameraURL };
			cameraAdd(camID, { url: msg.cameraURL }, 'host');
		}

		if (msg.joints)  { applyJointStatus(msg.joints); }
	});

	socket_host.on('sysinfo', (msg) => {
		var removedRobotIDs = Object.keys(robots);
		for (var i = 0; i < msg.length; i++)  {
			const robotID = msg[i].id;
			if (robotID in robots)  {
				const index = removedRobotIDs.indexOf(String(robotID));
				if (index > -1)  { removedRobotIDs.splice(index, 1); }
				document.getElementById('td_robot_status_' + robotID).innerText = msg[i].availability.toUpperCase();
			}  else  {
				robotAdd(robotID, msg[i]);
			}
			robots[robotID] = msg[i];
			robotUpdate(robotID);
		}
		for (const robotID of removedRobotIDs)  {
			robotRemove(robotID);
		}
	});

	socket_host.on('status', (msg) => {
		applyJointStatus(msg[1]);
	});

	// ── Client events ──

	socket_client.on('camStatus', (msg) => {
		for (var i = 0; i < msg.length; i++)  {
			if (!camStreams[msg[i].camID])  {
				camStreams[msg[i].camID] = msg[i];
				cameraAdd(msg[i].camID, msg[i], 'client');
			}
		}
	});

	socket_client.on('notice', (msg) => {
		const newNotice = document.createElement("li");
		newNotice.textContent = msg;
		ul_notices.insertBefore(newNotice, ul_notices.firstChild);
	});
}
