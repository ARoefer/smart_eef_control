String.prototype.format = function () {
  var i = 0, args = arguments;
  return this.replace(/{}/g, function () {
    return typeof args[i] != 'undefined' ? args[i++] : '';
  });
};

var lastUpdate = Date.now();

var lin_input    = {x: 0.0, y: 0.0, z: 0.0};
var device_ori   = {x: 0.0, y: 0.0, z: 0.0};
var ori_zero     = {x: 0.0, y: 0.0, z: 0.0};
var linear_slider = null;
var deadzone_slider = null;
var njs_manager = null;
var current_robot = null;

// Robot settings
var controlled_eef = null;
var robot_forward = 0.0;
var calibrated = false;
var controller_yaw = 0.0;

// EEF settings
var coordMode = true;
var controlMode = 'XY';
var deadzone = 0.2
var sensitivity = 1.0;
var controlPosition = true;
var controlRotation = true;

var serverIp = window.location.href.substring(7, window.location.href.lastIndexOf(':'));

$(function() {
  var deadmanWidth = window.innerWidth * 0.6;
  var ja = document.getElementById('joystickArea');
  var cm = document.getElementById('controlMode');
  linear_slider = document.getElementById('linScale');
  deadzone_slider = document.getElementById('deadzoneScale');
  ja.style.width  = window.innerWidth * 0.8 + 'px';
  ja.style.height = ja.offsetWidth + 'px';
  cm.style.width  = window.innerWidth * 0.2 + 'px';
  cm.style.height = ja.offsetWidth + 'px';

  connectToROS();

  njs_manager = nipplejs.create({
    zone: document.getElementById('joystickArea'),
    mode: 'static',
    position: {left: '50%', top: '50%'},
    size: deadmanWidth,
    restOpacity: 255
  });

  njs_manager.on('start', function(event, data) {
    capture_data = true;
    ori_zero.x = device_ori.x;
    ori_zero.y = device_ori.y;
    ori_zero.z = device_ori.z;
    console.log('starting data capture...');
  });

  njs_manager.on('end', function(event, data) {
    capture_data = false;
    lin_input    = {x: 0.0, y: 0.0, z: 0.0};
    ori_zero.x = device_ori.x;
    ori_zero.y = device_ori.y;
    ori_zero.z = device_ori.z;
    sendMotion();
    console.log('...stopping data capture');
  });

  njs_manager.on('move', function(event, data) {
    if (data.force <= 0.1) {
      x = 0.0;
      y = 0.0;
    } else {
      s = (Math.min(1.0, data.force) - deadzone) / (1.0 - deadzone);
      x = Math.cos(data.angle.radian) * s;
      y = Math.sin(data.angle.radian) * s;
    }

    if (controlMode == 'XY') {
      lin_input = {x: y, y: -x, z: 0.0};
    } else if (controlMode == 'XZ') {
      lin_input = {x: x, y: 0.0, z: y};
    } else {
      lin_input = {x: 0.0, y: -x, z: y};
    }
  });

  window.addEventListener('deviceorientation', function(event) {
    //console.log(event)
    device_ori.x =  event.gamma;
    device_ori.y = -event.beta;
    device_ori.z =  event.alpha;
  });

  window.setInterval(function() {
    if (capture_data) {
      sendMotion();
    }
  }, 25);
});

var capture_data = false;

function sendMotion() {
  var final_lin_input = {x: 0.0, y: 0.0, z: 0.0};
  if (controlPosition)
    final_lin_input = {x: lin_input.x, y: lin_input.y, z: lin_input.z};

  var final_ang_input = {x: 0.0, y: 0.0, z: 0.0};
  if (controlRotation) {
    final_ang_input = {x: device_ori.x - ori_zero.x, 
                      y: device_ori.y - ori_zero.y,
                      z: device_ori.z - ori_zero.z};
  }

  var cmd = new ROSLIB.Message({
      linear_input:   final_lin_input,
      angular_input:  final_ang_input,
      controller_yaw: ori_zero.z - robot_forward, 
      linear_scale:   sensitivity,
      controlled_id:  controlled_eef,
      command_type: coordMode
  });

  motionTopic.publish(cmd);
}

// Connecting to ROS
// -----------------
var ros = new ROSLIB.Ros();
// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  $('.statusDisplay').hide();
  document.getElementById('error').style.display = 'block';
  console.log(error);
});
// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  $('.statusDisplay').hide();
  document.getElementById('connected').style.display = 'block';
  showCalibration(true);

  var get_eefs_request = new ROSLIB.ServiceRequest({});
  srv_get_eefs.callService(get_eefs_request, function(res) {
    current_robot  = res.robot;
    var eef_to_select = localStorage.getItem('{}:eef'.format(current_robot));
    var sel_index = 0;

    var list = document.getElementById('eefList');
    if (res.endeffectors.length > 1) {
      $('.eefRelated').show();
      for (x in res.endeffectors) {
        list.add(new Option(res.endeffectors[x], res.endeffectors[x], false, false));
        if (res.endeffectors[x] == eef_to_select)
          sel_index = x;
      }
      list.selectedIndex = sel_index;
    } else {
      $('.eefRelated').hide();
    }
    controlled_eef = res.endeffectors[sel_index];
    loadEEFSettings(current_robot, controlled_eef);
  });
});
ros.on('close', function() {
  console.log('Connection closed.');
  $('.statusDisplay').hide();
  document.getElementById('closed').style.display = 'block';
});
// Create a connection to the rosbridge WebSocket server.

function connectToROS() {
  ros.connect('ws://{}:9097'.format(serverIp));
}

var motionTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/remote_control',
    messageType : 'smart_eef_control/RemoteControl'
});

var srv_get_eefs = new ROSLIB.Service({
  ros: ros,
  name: '/get_endeffectors',
  serviceType: 'smart_eef_control/GetEndeffectors'
});

function selectMode(id, mode) {
  $('.mode-btn').removeClass('btn-selected');
  $('#' + id).addClass('btn-selected');
  controlMode = mode;
  localStorage.setItem('{}:{}:controlMode'.format(current_robot, controlled_eef), controlMode);
}

function selectCoordMode(id, mode) {
  $('.frame-btn').removeClass('btn-selected');
  $('#' + id).addClass('btn-selected');
  coordMode = mode;
  localStorage.setItem('{}:{}:coordMode'.format(current_robot, controlled_eef), coordMode);
  if (coordMode == 'relative' && !calibrated) {
    onMenuBtnClicked(document.getElementById('btnSettings'));
    showCalibration(true);
  }
}

function changeThemeColor(color) {
    var metaThemeColor = document.querySelector("meta[name=theme-color]");
    metaThemeColor.setAttribute("content", color);
}

function eefChanged() {
  var list = document.getElementById('eefList');
  controlled_eef = list.options[list.selectedIndex].value
  localStorage.setItem('{}:eef'.format(current_robot), controlled_eef);
  loadEEFSettings(current_robot, controlled_eef);
}

function sensitivityChanged() {
  sensitivity = linear_slider.value / linear_slider.max;
  localStorage.setItem('{}:{}:sensitivity'.format(current_robot, controlled_eef), sensitivity);
}

function deadzoneChanged() {
  deadzone = deadzone_slider.value / deadzone_slider.max;
  localStorage.setItem('deadzone', deadzone);
}

function boolFromLS(key, defVal) {
  var strVal = localStorage.getItem(key);
  if (strVal)
    return strVal == 'true';
  else
    return defVal;
}

function loadEEFSettings(robot, eef) {
  deadzone    = parseFloat(localStorage.getItem('deadzone')) || deadzone;
  coordMode   = localStorage.getItem('{}:{}:coordMode'.format(robot, eef)) || coordMode;
  controlMode = localStorage.getItem('{}:{}:controlMode'.format(robot, eef)) || controlMode;
  sensitivity = parseFloat(localStorage.getItem('{}:{}:sensitivity'.format(robot, eef))) || sensitivity;
  controlPosition = boolFromLS('{}:{}:controlPosition'.format(robot, eef), controlPosition);
  controlRotation = boolFromLS('{}:{}:controlRotation'.format(robot, eef), controlRotation);

  $('#ckPos').prop('checked', controlPosition);
  $('#ckRot').prop('checked', controlRotation);

  selectMode('btn' + controlMode, controlMode);
  selectCoordMode('btn' + coordMode, coordMode);

  linear_slider.value = sensitivity * linear_slider.max;
  deadzone_slider.value = deadzone * deadzone_slider.max;
}

function onMenuBtnClicked(x) {
  var menu = document.getElementById('settingsMenu');
  if (menu.style.height != '100%') {
    x.classList.add("change");
    menu.style.height = '100%';
  } else {
    x.classList.remove("change");
    menu.style.height = '0%';
  }
}

function setControlPosition(state) {
  controlPosition = state;
  localStorage.setItem('{}:{}:controlPosition'.format(current_robot, controlled_eef), controlPosition);
}

function setControlRotation(state) {
  controlRotation = state;
  localStorage.setItem('{}:{}:controlRotation'.format(current_robot, controlled_eef), controlRotation);
}

function showCalibration(show) {
  if (calibrated) {
    $('#btnCaliCancel').show();
  } else {
    $('#btnCaliCancel').hide();
  }

  $('#calibrationRow').toggle(!show);
  $('#calibrationAnimation').toggle(show);
}

function calibrate() {
  robot_forward = device_ori.z;
  calibrated = true;
}