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
var globalFrame = true;
var controlled_eef = null;

var njs_manager = null;
var controlMode = 'XY';

$(function() {
  var deadmanWidth = window.innerWidth * 0.6;
  var ja = document.getElementById('joystickArea');
  var cm = document.getElementById('controlMode');
  linear_slider = document.getElementById('linScale');
  ja.style.width  = window.innerWidth * 0.8 + 'px';
  ja.style.height = ja.offsetWidth + 'px';
  cm.style.width  = window.innerWidth * 0.2 + 'px';
  cm.style.height = ja.offsetWidth + 'px';

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
      s = (Math.min(1.0, data.force) - 0.1) / 0.9;
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
  var cmd = new ROSLIB.Message({
      linear_input:  {x: lin_input.x, y: lin_input.y, z: lin_input.z},
      angular_input: {x: device_ori.x - ori_zero.x, 
                      y: device_ori.y - ori_zero.y,
                      z: device_ori.z - ori_zero.z},
      linear_scale: linear_slider.value / linear_slider.max,
      controlled_id: controlled_eef,
      global_command: globalFrame
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

  var get_eefs_request = new ROSLIB.ServiceRequest({});
  srv_get_eefs.callService(get_eefs_request, function(res) {
    var list = document.getElementById('eefList');
    if (res.endeffectors.length > 1) {
      list.style.display = 'block';
      for (x in res.endeffectors) {
        list.add(new Option(res.endeffectors[x], res.endeffectors[x], false, false));
      }
      list.selectedIndex = 0;
    }
    controlled_eef = res.endeffectors[0];
  });
});
ros.on('close', function() {
  console.log('Connection closed.');
  $('.statusDisplay').hide();
  document.getElementById('closed').style.display = 'block';
});
// Create a connection to the rosbridge WebSocket server.

var serverIp = window.location.href.substring(7, window.location.href.lastIndexOf(':'));


ros.connect('ws://{}:9097'.format(serverIp));

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
}

function selectNavMode(id, mode) {
  $('.frame-btn').removeClass('btn-selected');
  $('#' + id).addClass('btn-selected');
  globalFrame = mode == 'global';
}

function changeThemeColor(color) {
    var metaThemeColor = document.querySelector("meta[name=theme-color]");
    metaThemeColor.setAttribute("content", color);
}

function eefChanged() {
  var list = document.getElementById('eefList');
  controlled_eef = list.options[list.selectedIndex].value
}