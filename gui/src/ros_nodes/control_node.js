const rclnodejs = require('rclnodejs'); // ROS2 lib
const common = require('../public/javascripts/common'); // Server/client common functions

class ControlNode {
  constructor(){
    this.control_node = new rclnodejs.Node('control_panel','atos');

    // Publishers
    this.initPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/init');
    this.connectPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/connect');
    this.disconnectPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/disconnect');
    this.armPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/arm');
    this.disarmPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/disarm');
    this.startPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/start');
    this.abortPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/abort');
    this.allClearPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/all_clear');
    this.resetTestObjectsPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/reset_test_objects');
    this.reloadObjectSettingsPub = this.control_node.createPublisher('std_msgs/msg/Empty', '/atos/reload_object_settings');
    this.commandToPublisher = {
        "send_init": this.initPub,
        "send_connect": this.connectPub,
        "send_disconnect": this.disconnectPub,
        "send_arm": this.armPub,
        "send_disarm": this.disarmPub,
        "send_start": this.startPub,
        "send_abort": this.abortPub,
        "send_all_clear": this.allClearPub,
        "send_reset_test_objects": this.resetTestObjectsPub,
        "send_reload_object_settings": this.reloadObjectSettingsPub
      };

    // Service Clients
    this.obcStateClient = this.control_node.createClient('atos_interfaces/srv/GetObjectControlState', '/atos/get_object_control_state');
    this.commandToSrvClient = {
        "get_obc_state": this.obcStateClient
      };
    
    // Start the ros2 event loop
    this.control_node.spin();
  };

  // Helper functions
  requestService(serviceClient, command, ws){
    if ( !serviceClient.isServiceServerAvailable() || !serviceClient.waitForService(1000)) {
      console.log('Service not available after waiting');
      common.wsSend(command, ws, {"success" : "false"});
    }
    else{
      // Following is async
      this.obcStateClient.sendRequest({}, (response) => {
      // Let websocket client know that the service call is done
      common.wsSend(command, ws, response);
      });
    };
  };
  
  handleMessage(command, ws){
    if (command in this.commandToPublisher){ // If the command should be published
      this.commandToPublisher[command].publish({}); // Publish an empty message
      common.wsSend(command, ws, {"success" : "true"});
    }
    else if (command in this.commandToSrvClient){ // if the command should be called as a service
      this.requestService(this.commandToSrvClient[command], command, ws);
    }
    else { // Undefined
      console.log("Undefined command: " + command);
    };
  };
};

module.exports = ControlNode