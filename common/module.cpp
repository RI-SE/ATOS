#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

using namespace std_msgs::msg;
using namespace std_srvs::srv;

bool Module::shouldExit(){
    return this->quit;
}

void Module::onExitMessage(const Empty::SharedPtr){
    this->quit=true;
}

/*!
 * \brief A try/catch wrapper that logs messages 
 * \param tryExecute function to execute
 * \param executeIfFail if executing tryExecute fails, this function is executed
 * \param topic The topic to print.
 * \param logger The logger to use.
 * \return true if the initialization was successful, false otherwise
 */
void Module::tryHandleMessage(
    std::function<void()> tryExecute,
    std::function<void()> executeIfFail,
    const std::string& topic,
    const rclcpp::Logger& logger)
{
	try {
		RCLCPP_DEBUG(logger, "Handling command on %s", topic.c_str());
		tryExecute();
	} catch (std::invalid_argument& e) {
		RCLCPP_ERROR(logger, "Handling command on %s failed - %s", topic.c_str(), e.what());
		executeIfFail();
	}
}

/*!
 * \brief requestDataDictInitialization Sends a request to initialize the data dictionary
 * \param maxRetries Maximum number of retries before returning failure
 * \return true if the initialization was successful, false otherwise
 */
bool Module::requestDataDictInitialization(int maxRetries) {
    SetBool::Response::SharedPtr response;
    using namespace std::chrono_literals;
    auto successful = nShotServiceRequest<SetBool>(maxRetries, 1s, ServiceNames::initDataDict, response);
    if (successful) {
        if (response->success){
            RCLCPP_INFO(get_logger(), "Data dictionary successfully initialized");
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to initialize data dictionary, with message: %s", response->message.c_str());
        }
    }
    else{
        RCLCPP_ERROR(get_logger(), "Failed to initialize data dictionary");
    }
    return response->success && successful;
}
