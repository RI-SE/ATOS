const fs = require('fs');
const https = require('https');
const os = require('os');
const WebSocket = require('ws'); // Websocket lib
const rclnodejs = require('rclnodejs'); // ROS2 lib
const common = require('../public/javascripts/common'); // Server/client common functions

class ConfigNode {
  constructor(){
    this.config_node = new rclnodejs.Node('config_panel','atos');
    this.scenarioClient = this.config_node.createClient('rcl_interfaces/srv/SetParametersAtomically', '/atos/esmini_adapter/set_parameters_atomically');
    this.commandToConfigClient = {
      "set_scenario_param": this.scenarioClient
    };
  };

  init(){
    // Start the ros2 event loop
    this.config_node.spin();
    };

  // Helper functions
  requestService(serviceClient, command, ws){
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(1000)) {
      console.log('Service not available after waiting');
      common.wsSend(command, ws, {"success" : "false"});
    }
    else{
      // Following is async
      obcStateClient.sendRequest({}, (response) => {
        // Let websocket client know that the service call is done
        common.wsSend(command, ws, response);
      });
    };
  };

  executeCommand(command, ws){
    if (command in commandToPublisher){ // If the command should be published
      commandToPublisher[command].publish({}); // Publish an empty message
      common.wsSend(command, ws, {"success" : "true"});
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
        });
      }
    else { // Undefined
      console.log("Undefined command: " + command);
    };
  };
};

module.exports = {
    ConfigNode: ConfigNode
}