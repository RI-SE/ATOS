#include "scenariomodule.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "roschannels/commandchannels.hpp"

using namespace ROSChannels;
using namespace std::chrono_literals;
using namespace std::placeholders;

ScenarioModule::ScenarioModule()
    : Module(moduleName),
      initSub(*this, std::bind(&ScenarioModule::onInitMessage, this, _1)) {}

void ScenarioModule::onInitMessage(const Init::message_type::SharedPtr) {
  RCLCPP_DEBUG(this->get_logger(), "Received init message");
}