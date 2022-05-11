#include <functional>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "util.h"
#include "roschannel.hpp"

#include "maestro_interfaces/msg/accm.hpp"
#include "maestro_interfaces/msg/exac.hpp"
#include "maestro_interfaces/msg/manoeuvre_command.hpp"
#include "maestro_interfaces/msg/object_enabled.hpp"
#include "maestro_interfaces/msg/monitor.hpp"
#include "maestro_interfaces/msg/control_signal_percentage.hpp"
#include "maestro_interfaces/msg/trigger_event.hpp"
#include "maestro_interfaces/msg/object_id_array.hpp"
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
	virtual void onTrajectoryMessage(const ROSChannels::Trajectory::message_type::SharedPtr, uint32_t){};
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
	virtual void onTriggerEventMessage(const ROSChannels::TriggerEvent::message_type::SharedPtr){};

	virtual void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr);


	static void tryHandleMessage(std::function<void()> tryExecute,
								 std::function<void()> executeIfFail,
								 const std::string& topic,
								 const rclcpp::Logger& logger);
	bool requestDataDictInitialization(int maxRetries = 3);

   private:
	static void printUnhandledMessage(const std::string& topic, const std::string& message) {
		std::cout << "Unhandled message on topic: " << topic << " (" << message << ")" << std::endl;
	}
	static void printUnhandledMessage(const std::string& topic) {
		std::cout << "Unhandled empty message on topic: " << topic << std::endl;
	}
};
