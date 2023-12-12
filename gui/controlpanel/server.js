var createError = require('http-errors');
var express = require('express');
var path = require('path');
var cookieParser = require('cookie-parser');
var logger = require('morgan');
var favicon = require('serve-favicon');
const fs = require('fs');
const https = require('https');
const os = require('os');
const WebSocket = require('ws'); // Websocket lib
const rclnodejs = require('rclnodejs'); // ROS2 lib
const common = require('./public/javascripts/common'); // Server/client common functions

/* Start of boilerplate */
var indexRouter = require('./routes/index');

var app = express();

// signal handler for graceful shutdown
process.on('SIGINT', () => process.exit(1));

// view engine setup
app.set('views', path.join(__dirname, 'views'));

// Display favicon
app.use(favicon(path.join(__dirname,'public','images','favicon.ico')));

app.set('view engine', 'pug');

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));

app.use('/', indexRouter);

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
  const ParameterType = rclnodejs.ParameterType;
  const Parameter = rclnodejs.Parameter;

  // Create nodes
  const control_node = new rclnodejs.Node('simple_control','atos');
  const config_node = new rclnodejs.Node('config_panel','atos');

  // Publishers
  const initPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/init');
  const connectPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/connect');
  const disconnectPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/disconnect');
  const armPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/arm');
  const disarmPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/disarm');
  const startPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/start');
  const abortPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/abort');
  const allClearPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/all_clear');
  const resetTestObjectsPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/reset_test_objects');
  const reloadObjectSettingsPub = control_node.createPublisher('std_msgs/msg/Empty', '/atos/reload_object_settings');
  var commandToPublisher = {
    "send_init": initPub,
    "send_connect": connectPub,
    "send_disconnect": disconnectPub,
    "send_arm": armPub,
    "send_disarm": disarmPub,
    "send_start": startPub,
    "send_abort": abortPub,
    "send_all_clear": allClearPub,
    "send_reset_test_objects": resetTestObjectsPub,
    "send_reload_object_settings": reloadObjectSettingsPub
  };

  // Control Service Clients
  const obcStateClient = control_node.createClient('atos_interfaces/srv/GetObjectControlState', '/atos/get_object_control_state');
  var commandToSrvClient = {
    "get_obc_state": obcStateClient
  };

  // Config Service Clients
  const scenarioClient = config_node.createClient('rcl_interfaces/srv/SetParametersAtomically', '/atos/esmini_adapter/set_parameters_atomically');
  var commandToConfigClient = {
    "set_scenario_param": scenarioClient
  };

  // Start the ros2 event loop
  control_node.spin();
  config_node.spin();

  // Helper functions
  function requestService(serviceClient, command, ws){
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(1000)) {
      console.log('Service not available after waiting');
      common.wsSend(command, ws, {"success" : "false"})
    }
    else{
      // Following is async
      obcStateClient.sendRequest({}, (response) => {
        // Let websocket client know that the service call is done
        common.wsSend(command, ws, response)
      });
    }
  }

  function executeCommand(command, ws){
    if (command in commandToPublisher){ // If the command should be published
      commandToPublisher[command].publish({}); // Publish an empty message
      common.wsSend(command, ws, {"success" : "true"})
    }
    else if (command in commandToSrvClient){ // if the command should be called as a service
      requestService(commandToSrvClient[command], command, ws);
    }
    else if (command in commandToConfigClient){ // if the command should be called as a service
      var scenarioParam = new Parameter(
        'open_scenario_file',
        ParameterType.PARAMETER_STRING,
        'GaragePlanScenario.xosc'
      );
      const request = {
        parameters: scenarioParam,
      };
      scenarioClient.sendRequest(request, (response) => {
        // Let websocket client know that the service call is done
        common.wsSend(command, ws, response)
      };
    }
    else { // Undefined
      console.log("Undefined command: " + command);
    }
  }

  function wsInit(wss){
    // Wire up some logic for the connection event (when a client connects) 
    wss.on('connection', function connection(ws) {

      // Wire up logic for the message event (when a client sends something)
      ws.on('message', function incoming(message) {
        var clientCommand = JSON.parse(message).msg_type;
        executeCommand(clientCommand,ws);
      });
    });
  };


  // Set up a websocket server
  const ws = new WebSocket.Server({ port: 8081 });


  // Set up TLS, create secure websocket server
  const homeDir = os.homedir();
  
  const httpsServer = https.createServer({
    cert: fs.readFileSync(homeDir + '/.astazero/ATOS/certs/selfsigned.crt', 'utf8'),
    key: fs.readFileSync(homeDir + '/.astazero/ATOS/certs/selfsigned.key', 'utf8'),
  })

  httpsServer.listen(8082);

  // Start the secure websocket server
  const wsSecure = new WebSocket.Server({
      perMessageDeflate: false,
      server: httpsServer,
  })

  wsInit(ws); // start the insecure websocket server
  wsInit(wsSecure); // start the secure websocket server
  
});
