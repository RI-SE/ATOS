
/**
 * Loads the example identified by the given name
 */
function loadJsonFile(example) {
  var jsonPlaceholder;
  $.ajax({
    url: 'json/' + example + '.json',
    dataType: 'text',
    async: false
  }).done(function (code) {
    console.log("Loading example: " + example);
    jsonPlaceholder = code;
  }).fail(function () {
    $('#form').html('Sorry, I could not retrieve the example!');
  });
  return jsonPlaceholder
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
      window.alert('Form submitted. Values object:\n' +
        JSON.stringify(values, null, 2));
    };
    createdForm.onSubmit = function (errors, values) {
      if (errors) {
        console.log('Validation errors', errors);
        return false;
      }
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

// Wait until ACE is loaded
var itv = window.setInterval(function() {
  if (window.ace) {
    var example = 'atos-param-schema';
    window.clearInterval(itv);
    const jsonPlaceholder = loadJsonFile(example);
    generateForm(jsonPlaceholder);
  }
}, 1000);

// Websocket connection
var ws;

// function sendParam(param, value, ws){
//     var clientParam = new Object();
//     clientParam.msg_type = param;
//     clientParam.value = value;

//     ws.send(JSON.stringify(clientParam));
// }

// HTML-Button callbacks
function requestParameters(parameters){
    tmpName = parameters.Name;
    console.log("tmpName: " + tmpName);
    scenarioPath = parameters.ScenarioPath;
    console.log("Sending scenario file: " + scenarioPath);
}

function sendScenarioParam(scenarioPath){
    const elem = document.getElementById('scenario_file_path');
    if (not (scenarioPath.includes(".xosc"))){
        console.log("Scenario file does not end with .xosc!");
        return;
    }
    console.log("Sending scenario file: " + filePath);
    sendParam("send_scenario_param", filePath, ws);
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

