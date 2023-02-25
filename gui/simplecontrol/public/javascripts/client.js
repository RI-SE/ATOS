//const common = require('./public/javascripts/commonfuns');
// Find a way to import common here to remove the need for sendCommand.

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
            return "RUNNING";
        case 6:
            return "REMOTECTRL";
        case 7:
            return "ERROR";
        case 8:
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

// Create WebSocket connection with the server
const ws = new WebSocket('ws://localhost:8082');

// Periodically execute this callback
var intervalId = setInterval(function() {
    sendCommand("get_obc_state", ws);
  }, 500);

// HTML-Button callbacks
function sendResetTest(){
    sendCommand("send_reset_test", ws);
}
function sendInit(){
    sendCommand("send_init", ws);
}
function sendConnect(){
    sendCommand("send_connect", ws);
}
function sendArm(){
    sendCommand("send_arm", ws);
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

ws.addEventListener('open', function (event) {
    console.log("Connection opened");
    document.getElementById("isConnected").innerText = "Connected to ATOS";
    sendCommand("get_obc_state", ws); // Get state of OBC on connection
});

ws.addEventListener('close', function (event) {
    console.log("Connection closed");
    clearInterval(intervalId);
    document.getElementById("isConnected").innerText = "Connection to ATOS lost";
});

// Websocket callbacks from server
ws.addEventListener('message', function (event) {
    var serverResponse = JSON.parse(event.data);
    switch (serverResponse.msg_type) {
        case "get_obc_state_response":
            document.getElementById("OBCState").innerText = "OBC State: " + getOBCState(parseInt(serverResponse.state));
            break;
        // Add more callbacks here, e.g. feedback of successfully executed ros2 commands
        default:
            break;
    }
});
