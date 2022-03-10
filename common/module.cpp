#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

void Module::tryHandleMessage(
    COMMAND commandCode,
    std::function<void()> tryExecute,
    std::function<void()> executeIfFail)
{
	try {
		LogMessage(LOG_LEVEL_DEBUG, "Handling %s command", topicNames[commandCode].c_str());
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
    auto initClient = create_client<std_srvs::srv::SetBool>(ServiceNames::initDataDict);
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    auto response = std::shared_future<std::shared_ptr<std_srvs::srv::SetBool::Response>>();
    do {
        while (initClient->wait_for_service(std::chrono::seconds(1)) != true) {
            if (!rclcpp::ok()) {
                throw std::runtime_error("Interrupted while waiting for service " + ServiceNames::initDataDict);
            }
            RCLCPP_INFO(get_logger(), "Waiting for service " + ServiceNames::initDataDict + " ...");
        }
        response = initClient->async_send_request(request);
        if (response.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for data dictionary initialization");
            continue;
        }
        if (!response.get()->success) {
            RCLCPP_ERROR(get_logger(), "Data dictionary initialization failed");
        }
    } while (!response.get()->success && ++retries < maxRetries);
    
    return response.valid() && response.get()->success;
}