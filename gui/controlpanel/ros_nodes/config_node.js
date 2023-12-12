const rclnodejs = require('rclnodejs'); // ROS2 lib
const common = require('../public/javascripts/common'); // Server/client common functions

const ParameterType = rclnodejs.ParameterType;
const Parameter = rclnodejs.Parameter;

class ConfigNode {
  constructor(){
    this.config_node = new rclnodejs.Node('config_panel','atos');
    this.scenarioClient = this.config_node.createClient('rcl_interfaces/srv/SetParametersAtomically', '/atos/esmini_adapter/set_parameters_atomically');
    this.commandToParamClient = {
      "set_scenario_param": this.scenarioClient
    };
  };

  init(){
    // Start the ros2 event loop
    this.config_node.spin();
    };

  // Helper functions
  requestParamService(serviceClient, param_name, param_value, command, ws){
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(1000)) {
      console.log('Service not available after waiting');
      common.wsSend(command, ws, {"success" : "false"});
    }
    else{
      // Following is async
      var param = new Parameter(param_name, ParameterType.PARAMETER_STRING, param_value);
      const request = {parameters: param};
      obcStateClient.sendRequest(request, (response) => {
        // Let websocket client know that the service call is done
        common.wsSend(command, ws, response);
      });
    };
  };

  handleMessage(command, ws){
    if (command in commandToParamClient){ // if the command should be called as a service
      this.requestParamService(commandToParamClient[command], command, ws);
    }
    else { // Undefined
      console.log("Undefined command: " + clientCommand);
    };
  };
};

module.exports = ConfigNode