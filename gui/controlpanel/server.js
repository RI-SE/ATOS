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

/* Start of boilerplate */
var indexRouter = require('./routes/index');

var app = express();

// signal handler for graceful shutdown
process.on('SIGINT', () => process.exit(1));

// view engine setup
app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'pug');

// Display favicon
app.use(favicon(path.join(__dirname,'public','images','favicon.ico')));

app.use(logger('dev'));
app.use(express.json());
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());
app.use(express.static(path.join(__dirname, 'public')));
app.use(express.static(path.join(__dirname, 'node_modules')));

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


module.exports = app;

/* End of boilerplate */

// Initialize ros2
rclnodejs.init().then(() => {
  const ControlNode = require('./ros_nodes/control_node');
  control_node = new ControlNode();
  const ConfigNode = require('./ros_nodes/config_node');
  config_node = new ConfigNode();

  function wsInit(wss){
    // Wire up some logic for the connection event (when a client connects) 
    wss.on('connection', function connection(ws) {

      // Wire up logic for the message event (when a client sends something)
      ws.on('message', function incoming(message) {
        var clientMessage = JSON.parse(message).msg_type;
        if (clientMessage.includes("param")) {
          var param_value = JSON.parse(message).value;
          config_node.handleMessage(clientMessage, param_value, ws);
        }
        else {
          control_node.handleMessage(clientMessage,ws);
        }
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
