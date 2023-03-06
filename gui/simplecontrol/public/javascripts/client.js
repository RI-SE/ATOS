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

function setTextAndColor(id, color, text) {
    document.getElementById(id).innerText = text;
    document.getElementById(id).style.color = color;
}

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

// Websocket callbacks and reconnect functionality
var wsConnect = function(){
    ws = new WebSocket('wss://' + window.location.hostname + ':8082');
    ws.addEventListener('open', function (event) {
        console.log("Connection opened");
        setTextAndColor("isConnected", "green", "Connected to ATOS");
        sendCommand("get_obc_state", ws); // Get state of OBC on connection immediately
        intervalGetOBCState = setInterval(function() { // also start a timer to get the state periodically
            sendCommand("get_obc_state", ws);
          }, 500);

    });
    ws.addEventListener('close', function (event) {
        console.log("Connection lost!");
        clearInterval(intervalGetOBCState); // Stop timer
        setTextAndColor("OBCState", "black", "OBC State: UNDEFINED")
        setTextAndColor("isConnected", "red", "Connection to ATOS lost, retrying...");
        setTimeout(wsConnect, 1000); // Try to reconnect websocket
    });
    ws.addEventListener('message', function (event) {
        var serverResponse = JSON.parse(event.data);
        switch (serverResponse.msg_type) {
            case "get_obc_state_response":
                setTextAndColor("OBCState", "black", "OBC State: " + getOBCState(parseInt(serverResponse.state)));
                break;
            // Add more callbacks here, e.g. feedback of successfully executed ros2 commands
            default:
                break;
        }
    });
};
wsConnect();

