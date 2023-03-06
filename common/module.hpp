/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#pragma once

#include <functional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "roschannels/commandchannels.hpp"

namespace ServiceNames {
const std::string initDataDict = "init_data_dictionary";
const std::string getObjectIds = "get_object_ids";
const std::string getObjectIp = "get_object_ip";
const std::string getTestOrigin = "get_test_origin";
const std::string getObjectTrajectory = "get_object_trajectory";
const std::string getObjectTriggerStart = "get_object_trigger_start";
const std::string getObjectControlState = "get_object_control_state";
}

// TODO move somewhere else? also make generic to allow more args (variadic template)?
/*!
 * \brief Facilitates one-line intialization
 * of a ros message with one argument.
 */
template <typename Msg_T, typename MsgData_T>
Msg_T msgCtr1(MsgData_T data) {
	Msg_T ret;
	ret.data = data;
	return ret;
}

/*!
 * \brief The Module class
 * This class is the base class for all modules.
 * It is a derived class of ROS2 node.
 * It provides an interface for all modules.
 * \param name name of the module. Is passed to the ROS node.
 */
class Module : public rclcpp::Node {
   public:
	Module(const std::string name) : Node(name), getStatusResponsePub(*this) {};
	Module() = default;
	bool shouldExit();

   protected:
	bool quit=false;

	ROSChannels::GetStatusResponse::Pub getStatusResponsePub;

	virtual void onFailureMessage(const ROSChannels::Failure::message_type::SharedPtr){};
	virtual void onGetStatusResponse(const ROSChannels::GetStatusResponse::message_type::SharedPtr){};
	virtual void onGetStatusMessage(const ROSChannels::GetStatus::message_type::SharedPtr) {
		auto msg = std_msgs::msg::String();
		msg.data = this->get_name();
		getStatusResponsePub.publish(msg);
	};

	virtual void onInitMessage(const ROSChannels::Init::message_type::SharedPtr){};
	virtual void onConnectMessage(const ROSChannels::Connect::message_type::SharedPtr){};
	virtual void onDisconnectMessage(const ROSChannels::Disconnect::message_type::SharedPtr){};
	virtual void onArmMessage(const ROSChannels::Arm::message_type::SharedPtr){};
	virtual void onDisarmMessage(const ROSChannels::Disarm::message_type::SharedPtr){};
	virtual void onObjectsConnectedMessage(const ROSChannels::ObjectsConnected::message_type::SharedPtr){};
	virtual void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr){};
	virtual void onStartMessage(const ROSChannels::Start::message_type::SharedPtr){};
	virtual void onStartObjectMessage(const ROSChannels::StartObject::message_type::SharedPtr){};
	virtual void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr){};
	virtual void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr){};
	virtual void onReplayMessage(const ROSChannels::Replay::message_type::SharedPtr){};
	virtual void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr);


	static void tryHandleMessage(std::function<void()> tryExecute,
								 std::function<void()> executeIfFail,
								 const std::string& topic,
								 const rclcpp::Logger& logger);
								 
	bool requestDataDictInitialization(int maxRetries = 3);
	
	/*! \brief This helper function is performs a service call given a client and yields a response.
	*  \tparam Srv The name of the service to request.
	*  \param timeout The timeout for the service call.
	*  \param client The client to use to request the service.
	*  \param response The response of the service.
	* 	\return The response of the service.
	*/
	template <typename Srv>
	bool callService( const std::chrono::duration< double > &timeout,
						std::shared_ptr<rclcpp::Client<Srv>> &client,
						std::shared_ptr<typename Srv::Response> &response)
{
		auto request = std::make_shared<typename Srv::Request>();
		auto promise = client->async_send_request(request);
		if (rclcpp::spin_until_future_complete(get_node_base_interface(), promise, timeout) ==
			rclcpp::FutureReturnCode::SUCCESS) {
			response = promise.get();
			return true;
		} else {
			RCLCPP_ERROR(get_logger(), "Failed to call service %s", client->get_service_name());
			return false;
		}
	}

	/*! \brief This helper function waits for a service to become available and returns a client.
	*  \tparam Srv The name of the service to request.
	*  \param n The number of times to retry.
	*  \param timeout The timeout to wait for the service.
	*  \param serviceName The name of the service to request.
	* 	\return The response of the service.
	*/
	template <typename Srv>
	typename std::shared_ptr<rclcpp::Client<Srv>> nTimesWaitForService(int n,
							const std::chrono::duration< double > &timeout,
							const std::string &serviceName)
{
		int retries = 0;
		auto client = create_client<Srv>(serviceName);

		do {
			while (client->wait_for_service(timeout) != true) {
				if (!rclcpp::ok()) {
					throw std::runtime_error("Interrupted while waiting for service " + serviceName);
				}
				RCLCPP_INFO(get_logger(), "Waiting for service %s ...", serviceName.c_str());
			}
			RCLCPP_DEBUG(get_logger(), "Service %s found", serviceName.c_str());
			return client;
		} while (++retries < n);
		throw std::runtime_error("Failed to initialize service " + serviceName);
	}


	/*! \brief This helper function is used by rosnodes to request a service.
	*  \tparam Srv The name of the service to request.
	*  \param n Number of times to retry until accepting the service is not available.
	*  \param timeout The timeout to wait for the service.
	*  \param serviceName The name of the service to request.
	*  \param response Response of the service, to be returned.
	* 	\return The response of the service.
	*/
	template <typename Srv>
	bool nShotServiceRequest(int n, 
							const std::chrono::duration< double > &timeout,
							const std::string &serviceName,
							std::shared_ptr<typename Srv::Response> &response) {
		std::shared_ptr<rclcpp::Client<Srv>> client;
		try{
			client = nTimesWaitForService<Srv>(n, timeout, serviceName);
		}
		catch (std::runtime_error &e){
			RCLCPP_ERROR(get_logger(), "Failed to initialize service %s", serviceName.c_str());
			return false;
		}
		return callService<Srv>(timeout, client, response);
	}
};
