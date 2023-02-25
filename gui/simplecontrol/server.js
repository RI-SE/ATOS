var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
const WebSocket = require('ws'); // Websocket lib
const rclnodejs = require('rclnodejs'); // ROS2 lib
const common = require('./public/javascripts/common'); // Server/client common functions

/* Start of boilerplate */
var indexRouter = require('./routes/index');
var usersRouter = require('./routes/users');

var app = express();

// signal handler for graceful shutdown
process.on('SIGINT', () => process.exit(1));

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'jade');

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', indexRouter);
app.use('/users', usersRouter);

// catch 404 and forward to error handler
app.use(function(req, res, next) {
  next(createError(404));
});

// error handler
app.use(function(err, req, res, next) {
  // set locals, only providing error in development
  res.locals.message = err.message;
  res.locals.error = req.app.get('env') === 'development' ? err : {};

  // render the error page
  res.status(err.status || 500);
  res.render('error');
});
app.get('/public/javascripts/client.js', function(req, res) {
  res.sendFile(path.join(__dirname + '/client.js'));
});

module.exports = app;

/* End of boilerplate */


// simple_control logic:

// Initialize ros2
rclnodejs.init().then(() => {
  // Create a node
  const node = new rclnodejs.Node('simple_control','atos');

  // Publishers
  const resetTestPub = node.createPublisher('std_msgs/msg/Empty', '/atos/disconnect');
  const initPub = node.createPublisher('std_msgs/msg/Empty', '/atos/init');
  const connectPub = node.createPublisher('std_msgs/msg/Empty', '/atos/connect');
  const armPub = node.createPublisher('std_msgs/msg/Empty', '/atos/arm');
  const startPub = node.createPublisher('std_msgs/msg/Empty', '/atos/start');
  const abortPub = node.createPublisher('std_msgs/msg/Empty', '/atos/abort');
  const allClearPub = node.createPublisher('std_msgs/msg/Empty', '/atos/all_clear');
  var commandToPublisher = {
    "send_reset_test": resetTestPub,
    "send_init": initPub,
    "send_connect": connectPub,
    "send_arm": armPub,
    "send_start": startPub,
    "send_abort": abortPub,
    "send_all_clear": allClearPub
  };

  // Service Clients
  const obcStateClient = node.createClient('atos_interfaces/srv/GetObjectControlState', '/atos/get_object_control_state');
  var commandToSrvClient = {
    "get_obc_state": obcStateClient
  };

  // Start the ros2 event loop
  node.spin();

  // Helper functions
  function respondToClient(command, data ,ws) {
    var wsResponse = new Object();
    wsResponse.msg_type = command + "_response";
    wsResponse = Object.assign(wsResponse, data);
    ws.send(JSON.stringify(wsResponse));
  }

  function requestService(serviceClient, command, ws){
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(1000)) {
      console.log('Service not available after waiting');
      common.wsSend(command, ws, {"success" : "false"})
      //respondToClient(command, {"success" : "false"}, ws)

    }
    else{
      console.log('Service available, sending request...');
      obcStateClient.sendRequest({}, (response) => {
        console.log(`Result: ${typeof response}`, response);
        // Let websocket client know that the service call is done
        common.wsSend(command, ws, response)
        //respondToClient(command, response, ws)
      });
    }
  }

  function executeCommand(command, ws){
    if (command in commandToPublisher){ // If the command should be published
      console.log("Sending command: " + command);
      commandToPublisher[command].publish({}); // Publish an empty message
      common.wsSend(command, ws, {"success" : "true"})
      //respondToClient(command, {"success" : "true"}, ws)
    }
    else if (command in commandToSrvClient){ // if the command should be called as a service
      requestService(commandToSrvClient[command], command, ws);
    }
    else { // Undefined
      console.log("Undefined command: " + command);
    }
  }

  // Start a websocket server
  const wss = new WebSocket.Server({ port: 8082 });

  // Wire up some logic for the connection event (when a client connects) 
  wss.on('connection', function connection(ws) {

    // Wire up logic for the message event (when a client sends something)
    ws.on('message', function incoming(message) {
      console.log('received: %s', message);
      var clientCommand = JSON.parse(message).msg_type;
      executeCommand(clientCommand,ws);
    });

  });
});



