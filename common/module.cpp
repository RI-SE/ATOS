#include "module.hpp"
#include <std_srvs/srv/set_bool.hpp>

Module::Module(const std::string name) : 
        Node(name)
        {
    exitSub = create_subscription<Empty>(topicNames[COMM_EXIT], queueSize, bind(&JournalControl::onExitMessage, this, _1));
    this->quit = false;
}

Module::shouldExit(){
    return this->quit;
}

Module::onExitMessage(){
    this->quit=true;
}

/*!
 * \brief A try/catch wrapper that logs messages 
 * \param commandCode The message type to be processed
 * \param tryExecute function to execute
 * \param executeIfFail if executing tryExecute fails, this function is executed
 * \return true if the initialization was successful, false otherwise
 */
void Module::tryHandleMessage(
    COMMAND commandCode,
    std::function<void()> tryExecute,
    std::function<void()> executeIfFail)
{
	try {
		LogMessage(LOG_LEVEL_DEBUG, "Handling %s command", topicNames.at(commandCode).c_str());
		tryExecute();
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Handling %s command failed - %s", topicNames.at(commandCode).c_str(),
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