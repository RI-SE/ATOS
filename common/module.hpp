#include <functional>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "util.h"
#include "roschannel.hpp"

#include "atos_interfaces/msg/action_configuration.hpp"
#include "atos_interfaces/msg/execute_action.hpp"
#include "atos_interfaces/msg/trigger_event_occurred.hpp"
#include "atos_interfaces/msg/monitor.hpp"
#include "atos_interfaces/msg/manoeuvre_command.hpp"
#include "atos_interfaces/msg/object_enabled.hpp"
#include "atos_interfaces/msg/control_signal_percentage.hpp"
#include "atos_interfaces/msg/object_id_array.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

// TODO move somewhere else
static std::map<COMMAND, std::string> topicNames = {
	{COMM_STRT, "/start"},
	{COMM_ARM, "/arm"},
	{COMM_STOP, "/stop"},
	{COMM_EXIT, "/exit"},
	{COMM_REPLAY, "/replay"},
	{COMM_ABORT, "/abort"},
	{COMM_ABORT_DONE, "/all_clear"},
	{COMM_INIT, "/init"},
	{COMM_CONNECT, "/connect"},
	{COMM_OBC_STATE, "/obc_state"},
	{COMM_DISCONNECT, "/disconnect"},
	{COMM_VIOP, "/viop"},
	{COMM_TRAJ, "/traj"},
	{COMM_TRAJ_TOSUP, "/traj_tosup"},
	{COMM_TRAJ_FROMSUP, "/traj_fromsup"},
	{COMM_ASP, "/asp"},
	{COMM_OSEM, "/osem"},
	{COMM_DATA_DICT, "/data_dict"},
	{COMM_EXAC, "/execute_action"},
	{COMM_TREO, "/event_occurred"},
	{COMM_ACCM, "/action_configuration"},
	{COMM_TRCM, "/trigger_configuration"},
	{COMM_DISARM, "/disarm"},
	{COMM_GETSTATUS, "/get_status"},
	{COMM_GETSTATUS_OK, "/get_status_response"},
	{COMM_BACKTOSTART_CALL, "/back_to_start"},
	{COMM_BACKTOSTART_RESPONSE, "/back_to_start_response"},
	{COMM_REMOTECTRL_ENABLE, "/remote_control_enable"},
	{COMM_REMOTECTRL_DISABLE, "/remote_control_disable"},
	{COMM_REMOTECTRL_MANOEUVRE, "/remote_control_manoeuvre"},
	{COMM_ENABLE_OBJECT, "/enable_object"},
	{COMM_CONTROL_SIGNAL_PERCENTAGE, "/control_signal_perc"},
	{COMM_OBJECTS_CONNECTED, "/objects_connected"},
	{COMM_FAILURE, "/failure"}
};

namespace ServiceNames {
const std::string initDataDict = "init_data_dictionary";
const std::string getObjectIds = "get_object_ids";
const std::string getObjectIp = "get_object_ip";
const std::string getTestOrigin = "get_test_origin";
const std::string getObjectTrajectory = "get_object_trajectory";
const std::string getObjectTriggerStart = "get_object_trigger_start";
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
	virtual void onRemoteControlEnableMessage(const ROSChannels::RemoteControlEnable::message_type::SharedPtr){};
	virtual void onRemoteControlDisableMessage(const ROSChannels::RemoteControlDisable::message_type::SharedPtr){};
	virtual void onRemoteControlManoeuvreMessage(const ROSChannels::RemoteControlManoeuvre::message_type::SharedPtr){};
	virtual void onEnableObjectMessage(const ROSChannels::EnableObject::message_type::SharedPtr){};
	virtual void onObjectsConnectedMessage(const ROSChannels::ObjectsConnected::message_type::SharedPtr){};
	virtual void onDataDictMessage(const ROSChannels::DataDictionary::message_type::SharedPtr){};
	virtual void onOSEMMessage(const ROSChannels::Init::message_type::SharedPtr){}; // TODO remove
	virtual void onASPMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO once channel defined swap type
	virtual void onPathMessage(const ROSChannels::Path::message_type::SharedPtr, uint32_t){};
	virtual void onTrajectoryMessage(const ROSChannels::CartesianTrajectory::message_type::SharedPtr, uint32_t){};
	virtual void onMonitorMessage(const ROSChannels::Monitor::message_type::SharedPtr, uint32_t){};
	virtual void onTrajToSupMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO once channel defined swap type
	virtual void onTrajFromSupMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO once channel defined swap type
	virtual void onAllClearMessage(const ROSChannels::AllClear::message_type::SharedPtr){};
	virtual void onOBCStateMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO remove
	virtual void onVIOPMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO remove or swap type
	virtual void onStartMessage(const ROSChannels::Start::message_type::SharedPtr){};
	virtual void onStopMessage(const ROSChannels::Stop::message_type::SharedPtr){};
	virtual void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) = 0;
	virtual void onACCMMessage(const ROSChannels::ActionConfiguration::message_type::SharedPtr){};
	virtual void onTRCMMessage(const std_msgs::msg::Empty::SharedPtr){}; // TODO
	virtual void onEXACMessage(const ROSChannels::ExecuteAction::message_type::SharedPtr){};
	virtual void onReplayMessage(const ROSChannels::Replay::message_type::SharedPtr){};
	virtual void onBackToStartMessage(const ROSChannels::BackToStart::message_type::SharedPtr){};
	virtual void onBackToStartResponse(const ROSChannels::BackToStartResponse::message_type::SharedPtr){};
	virtual void onDataDictResponse(const std_msgs::msg::Empty::SharedPtr){}; // TODO remove
	virtual void onControlSignalMessage(const ROSChannels::ControlSignal::message_type::SharedPtr){};
	virtual void onTriggerEventMessage(const ROSChannels::TriggerEventOccurred::message_type::SharedPtr){};

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

   private:
	static void printUnhandledMessage(const std::string& topic, const std::string& message) {
		std::cout << "Unhandled message on topic: " << topic << " (" << message << ")" << std::endl;
	}
	static void printUnhandledMessage(const std::string& topic) {
		std::cout << "Unhandled empty message on topic: " << topic << std::endl;
	}
};
