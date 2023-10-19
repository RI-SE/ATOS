//const common = require('./public/javascripts/commonfuns');
// Find a way to import common here to remove the need for sendCommand.

// Websocket connection
var ws;

// Helper functions
function getOBCState(intState){
    switch(intState){
        case 0:
            return "UNDEFINED";
        case 1:
            return "IDLE";
        case 2:
            return "INITIALIZED";
        case 3:
            return "CONNECTED";
        case 4:
            return "ARMED";
        case 5:
            return "DISARMING";
        case 6:
            return "RUNNING";
        case 7:
            return "REMOTECTRL";
        case 8:
            return "ERROR";
        case 9:
            return "ABORTING";
        default:
            return "UNDEFINED";
    }
}

function sendCommand(command, ws){
    var clientCommand = new Object();
    clientCommand.msg_type = command;
    ws.send(JSON.stringify(clientCommand));
}

// Functions relating to visual stuff, move into own .js file. 
function setOBCStatusText(new_text){
    const obcStateText = document.getElementById('OBCState');
    obcStateText.innerHTML = new_text;
}

function updateATOSStatusIcon(status, new_text) {
    const statusIcon = document.querySelector('.status-icon');
    const icon = statusIcon.querySelector('.status-icon__icon');
    const text = statusIcon.querySelector('.status-icon__text');
    text.textContent = new_text;
    switch (status) {
      case 'pending':
        icon.style.backgroundColor = 'gray';
        text.style.color = 'gray';
        break;
      case 'active':
        icon.style.backgroundColor = 'green';
        text.style.color = 'green';
        break;
      case 'inactive':
        icon.style.backgroundColor = 'red';
        text.style.color = 'red';
        break;
      default:
        break;
    }
  }

// HTML-Button callbacks
function sendInit(){
    sendCommand("send_init", ws);
}
function sendConnect(){
    sendCommand("send_connect", ws);
}
function sendDisconnect(){
    sendCommand("send_disconnect", ws);
}
function sendArm(){
    sendCommand("send_arm", ws);
}
function sendDisarm(){
    sendCommand("send_disarm", ws);
}
function sendStart(){
    sendCommand("send_start", ws);
}
function sendAbort(){
    sendCommand("send_abort", ws);
}
function sendAllClear(){
    sendCommand("send_all_clear", ws);
}
function sendResetTestObjects(){
    sendCommand("send_reset_test_objects", ws);
}
function sendReloadObjectSettings(){
    sendCommand("send_reload_object_settings", ws);
}

// Websocket callbacks and reconnect functionality
var wsConnect = function(){

    // Depending on the connection type, open different websockets.
    const currentUrl = new URL(window.location.href);
    if (currentUrl.protocol == "https:"){
        ws = new WebSocket('wss://' + currentUrl.hostname + ':8082');
    }
    else{
        ws = new WebSocket('ws://' + currentUrl.hostname + ':8081');
    }
    ws.addEventListener('open', function (event) {
        console.log("Connection opened");
        updateATOSStatusIcon("active", "Connected to ATOS");
        sendCommand("get_obc_state", ws); // Get state of OBC on connection immediately
        intervalGetOBCState = setInterval(function() { // also start a timer to get the state periodically
            sendCommand("get_obc_state", ws);
          }, 500);

    });
    ws.addEventListener('close', function (event) {
        console.log("Connection lost!");
        clearInterval(intervalGetOBCState); // Stop timer
        setOBCStatusText("State: UNDEFINED");
        updateATOSStatusIcon("inactive", "Connection to ATOS lost, retrying...");
        setTimeout(wsConnect, 1000); // Try to reconnect websocket
    });
    ws.addEventListener('message', function (event) {
        var serverResponse = JSON.parse(event.data);
        switch (serverResponse.msg_type) {
            case "get_obc_state_response":
                setOBCStatusText("State: " + getOBCState(parseInt(serverResponse.state)));
                break;
            // Add more callbacks here, e.g. feedback of successfully executed ros2 commands
            default:
                break;
        }
    });
};
wsConnect();

