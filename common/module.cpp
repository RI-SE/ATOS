#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

void Module::tryHandleMessage(
    COMMAND commandCode,
    std::function<void()> tryExecute,
    std::function<void()> executeIfFail)
{
	try {
		LogMessage(LOG_LEVEL_DEBUG, "Handling %s command", topicNames.at(commandCode).c_str());
		tryExecute();
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Handling %s command failed - %s", topicNames[commandCode].c_str(),
				   e.what());
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
    auto initClient = create_client<std_srvs::srv::SetBool>(ServiceNames::initDataDict);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();

    do {
        while (initClient->wait_for_service(std::chrono::seconds(1)) != true) {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for service " + ServiceNames::initDataDict);
            }
            RCLCPP_INFO(get_logger(), "Waiting for service " + ServiceNames::initDataDict + " ...");
        }
        RCLCPP_DEBUG(get_logger(), "Service " + ServiceNames::initDataDict + " found");
        
        auto response = initClient->async_send_request(request);
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
            RCLCPP_ERROR(get_logger(), "Failed to call service %s", initClient->get_service_name());
        }
    } while (++retries < maxRetries);
    
    return success;
}