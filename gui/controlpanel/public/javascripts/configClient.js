const { set } = require("../../server");

// Websocket connection
var ws;

function sendParam(param, value, ws){
    var clientParam = new Object();
    clientParam.msg_type = param;
    clientParam.value = value;

    ws.send(JSON.stringify(clientParam));
}

// HTML-Button callbacks
function sendScenarioParam(value){
    sendParam("send_scenario_param", value, ws);
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
    });

    ws.addEventListener('close', function (event) {
        console.log("Connection lost!");
    });

    ws.addEventListener('message', function (event) {
        var serverResponse = JSON.parse(event.data);
        switch (serverResponse.msg_type) {
            case "send_scenario_param_response":
                setScenarioParamStatusText(serverResponse.success);
                break;
            // Add more callbacks here, e.g. feedback of successfully executed ros2 commands
            default:
                break;
        }
    });
};
wsConnect();

