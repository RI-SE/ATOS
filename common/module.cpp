#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

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
    int retries = 0;
    bool success = false;
    auto client = create_client<std_srvs::srv::SetBool>(ServiceNames::initDataDict);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    do {
        while (client->wait_for_service(std::chrono::seconds(1)) != true) {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for service " + ServiceNames::initDataDict);
            }
            RCLCPP_INFO(get_logger(), "Waiting for service " + ServiceNames::initDataDict + " ...");
        }
        RCLCPP_DEBUG(get_logger(), "Service " + ServiceNames::initDataDict + " found");
        
        auto response = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), response, std::chrono::seconds(1)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            success = response.get()->success;
            if (success) {
                RCLCPP_INFO(get_logger(), "Data dictionary successfully initialized");
                break;
            } else {
                RCLCPP_ERROR(get_logger(), "Data dictionary initialization failed: " + response.get()->message);
            }
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to call service %s", client->get_service_name());
        }
    } while (++retries < maxRetries);
    
    return success;
}