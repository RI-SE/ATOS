const rclnodejs = require('rclnodejs'); // ROS2 lib
// const parameter_lib = require('rclnodejs/lib/parameter.js')
const common = require('../public/javascripts/common'); // Server/client common functions

const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;

class ConfigNode {
  constructor(){
    this._config_node = new rclnodejs.Node('config_panel','atos');
    this._parametersSrvMsg = 'rcl_interfaces/srv/SetParametersAtomically';
    this._journalControlClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/journal_control/set_parameters_atomically');
    this._esminiAdapterClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/esmini_adapter/set_parameters_atomically');
    this._objectControlClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/object_control/set_parameters_atomically');
    this._systemControlClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/system_control/set_parameters_atomically');
    this._osiAdapterClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/osi_adapter/set_parameters_atomically');
    this._mqttBridgeClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/mqtt_bridge/set_parameters_atomically');
    this._trajectoryletStreamerClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/trajectorylet_streamer/set_parameters_atomically');
    this._pointcloudPublisherClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/pointcloud_publisher/set_parameters_atomically');
    this._backToStartClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/back_to_start/set_parameters_atomically');
    this._integrationTestingHandlerClient = this._config_node.createClient(this._parametersSrvMsg, '/atos/integration_testing_handler/set_parameters_atomically');
    this._nodeToClient = {
      "journal_control": this._journalControlClient,
      "esmini_adapter": this._esminiAdapterClient,
      "object_control": this._objectControlClient,
      "system_control": this._systemControlClient,
      "osi_adapter": this._osiAdapterClient,
      "mqtt_bridge": this._mqttBridgeClient,
      "trajectorylet_streamer": this._trajectoryletStreamerClient,
      "pointcloud_publisher": this._pointcloudPublisherClient,
      "back_to_start": this._backToStartClient,
      "integration_testing_handler": this._integrationTestingHandlerClient
    };
    this._dataType_to_parameterType = {
      "boolean": ParameterType.PARAMETER_BOOL,
      "number": ParameterType.PARAMETER_DOUBLE,
      "string": ParameterType.PARAMETER_STRING,
    }
    // Start the ros2 event loop
    this._config_node.spin();
  };

  // Helper functions
  requestParamService(serviceClient, ros_parameters, node, ws){
    const wait_duration_s = 1 // Wait duration for service to appear in seconds.
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(wait_duration_s*1000)) {
      console.log(`${node} set parameters service not available after waiting ${wait_duration_s} second(s)`);
      common.wsSend(node, ws, {"success" : "false"});
      return false;
    }
    else{
      // Following is async
      var request = new Object();
      request.parameters = [];
      for (var param_name in ros_parameters){
        var param_value = ros_parameters[param_name];
        var parameter_type = this._dataType_to_parameterType[typeof(param_value)] // TODO: See if parameterTypeFromValue() can be used somehow. https://github.com/RobotWebTools/rclnodejs/blob/dc7f54bd543d834d6ce36b17238cf257489752d1/lib/parameter.js#L702
        var param = new Parameter(param_name, parameter_type, param_value);
        var param_msg = param.toParameterMessage()
        request.parameters.push(param_msg);
      }
      serviceClient.sendRequest(request, (response) => {
        // Let websocket client know that the service call is done
        return response.result.successful
      });
    };
  };

  handleMessage(config_message, ws){
    var all_successful_params = true;
    for (var node in config_message){
      if (node in this._nodeToClient){
        var ros_parameters = config_message[node].ros__parameters;
        var success = this.requestParamService(this._nodeToClient[node], ros_parameters, node, ws); // TODO: Fix asynch request...
        all_successful_params = success & all_successful_params;
      }
      else { // Undefined
        console.log(`Skipping ${node}. Node is not defined in service client list`);
      };
    }
    common.wsSend("config", ws, {"success": all_successful_params})
  };
};

module.exports = ConfigNode