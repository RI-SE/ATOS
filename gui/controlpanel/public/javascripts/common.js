module.exports = {
    wsSend: function(command, ws, data={}) {
        var wsResponse = new Object();
        wsResponse.msg_type = command + "_response";
        wsResponse = Object.assign(wsResponse, data);
        ws.send(JSON.stringify(wsResponse));
    }
}