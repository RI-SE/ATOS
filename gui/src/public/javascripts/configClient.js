// Websocket connection
var ws;

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
          case "set_config_response":
            console.log("set config response: ", serverResponse)
            if (serverResponse.success){
              window.alert('All parameters were successfully set');
            }
            else{
              window.alert('Not all parameters were successfully set');
            }
          case "get_config_response":
            console.log("get config response: ", serverResponse)
            // generateForm(serverResponse.value); // TODO: This is not working yet. Update the form with the current values when all have been fetched.
          // Add more callbacks here, e.g. feedback of successfully executed ros2 commands
          default:
              break;
      }
  });
};
wsConnect();


/**
 * Loads the example identified by the given name
 */
function loadJsonFile(example) {
  var jsonSchema;
  $.ajax({
    url: 'json/' + example + '.json',
    dataType: 'text',
    async: false
  }).done(function (code) {
    console.log("Loading example: " + example);
    jsonSchema = code;
  }).fail(function () {
    $('#form').html('Sorry, I could not retrieve the example!');
  });
  return jsonSchema
};

/**
 * Displays the form entered by the user
 * (this function runs whenever once per second whenever the user
 * changes the contents of the ACE input field)
 */
function generateForm(jsonString) {
  var createdForm = JSON.parse(jsonString)

  // Reset form pane
  $('#form').html('');

  // Render the resulting form, binding to onSubmitValid
  try {
    createdForm.onSubmitValid = function (values) {
      if (console && console.log) {
        console.log('Values extracted from submitted form', values);
      }
    };
    createdForm.onSubmit = function (errors, values) {
      if (errors) {
        console.log('Validation errors', errors);
        return false;
      }
      sendConfig(values, ws)
      return true;
    };
    $('#form').jsonForm(createdForm);
  }
  catch (e) {
    $('#form').html('<pre>Entered content is not yet a valid' +
      ' JSON Form object.\n\nThe JSON Form library returned:\n' +
      e + '</pre>');
    return;
  }
};

function updateForm(jsonString){
  var createdForm = JSON.parse(jsonString);
  console.log("update form: ", createdForm);
  // $('#form').jsonForm(createdForm); // TODO: This is not working yet. Update the form with the current values when all have been fetched.
}

// Wait until ACE is loaded
var itv = window.setInterval(function() {
  if (window.ace) {
    var example = 'atos-param-schema';
    window.clearInterval(itv);
    const jsonSchema = loadJsonFile(example);
    generateForm(jsonSchema);
    getCurrentConfig(jsonSchema, ws); // Get initial config from server to init the form
  }
}, 1000);

function getCurrentConfig(jsonString, ws){
  var jsonObject = JSON.parse(jsonString);
  var nodes = [];
  for (var node in jsonObject.schema.properties){
    nodes.push(node);
  }
  var clientParam = new Object();
  clientParam.msg_type = 'get_config';
  clientParam.value = nodes;

  ws.send(JSON.stringify(clientParam));
}

function sendConfig(value, ws){
    var clientParam = new Object();
    clientParam.msg_type = 'set_config';
    clientParam.value = value;

    ws.send(JSON.stringify(clientParam));
}
