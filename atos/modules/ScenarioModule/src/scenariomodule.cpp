#include "scenariomodule.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "roschannels/commandchannels.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

ScenarioModule::ScenarioModule()
    : Module(moduleName), initSub(*this, std::bind(&ScenarioModule::onInitMessage, this, _1)) {

  declare_parameter("open_scenario_file", "default_scenario");
  declare_parameter("active_object_ids", std::vector<int>());

  getObjectIdsService = create_service<atos_interfaces::srv::GetObjectIds>(
      ServiceNames::getObjectIds, std::bind(&ScenarioModule::onRequestObjectIDs, this, _1, _2));
}

void ScenarioModule::onInitMessage(const Init::message_type::SharedPtr) {
  // Update the scenario name
  RCLCPP_INFO(this->get_logger(), "Scenario name: %s", get_parameter("open_scenario_file").value_to_string().c_str());
  RCLCPP_INFO(this->get_logger(), "Active object IDs: %s",
              get_parameter("active_object_ids").value_to_string().c_str());
}

void ScenarioModule::onRequestObjectIDs(std::shared_ptr<atos_interfaces::srv::GetObjectIds::Request> _,
                                        const std::shared_ptr<atos_interfaces::srv::GetObjectIds::Response> &res) {
  RCLCPP_DEBUG(get_logger(), "Received object ID information request");
  auto active_object_ids_param = get_parameter("active_object_ids").as_integer_array();

  // Get current param value
  if (active_object_ids_param.empty()) {
    RCLCPP_WARN(get_logger(), "No active object IDs are set!");
    res->success = false;
    res->ids.clear();
  } else {
    for (const auto id : active_object_ids_param) {
      res->ids.push_back(static_cast<uint32_t>(id));
    }
    res->success = true;
  }
}