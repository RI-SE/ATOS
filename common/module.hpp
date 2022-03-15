#include <functional>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "util.h"

#include "maestro_interfaces/msg/accm.hpp"
#include "maestro_interfaces/msg/exac.hpp"
#include "maestro_interfaces/msg/manoeuvre_command.hpp"
#include "maestro_interfaces/msg/object_enabled.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

using maestro_interfaces::msg::Accm;
using maestro_interfaces::msg::Exac;
using maestro_interfaces::msg::ManoeuvreCommand;
using maestro_interfaces::msg::ObjectEnabled;
using rclcpp::Node;
using std::placeholders::_1;
using std_msgs::msg::Empty;
using std_msgs::msg::Int8;
using std_msgs::msg::String;
using std_msgs::msg::UInt8;

// TODO: Move somewhere else
namespace TopicNames {
const std::string start = "start";
const std::string arm = "arm";
const std::string stop = "stop";
const std::string exit = "exit";
const std::string replay = "replay";
const std::string abort = "abort";
const std::string abortDone = "all_clear";
const std::string init = "init";
const std::string connect = "connect";
const std::string obc_state = "obc_state";
const std::string disconnect = "disconnect";
const std::string viop = "viop";
const std::string traj = "traj";
const std::string trajToSup = "traj_tosup";
const std::string trajFromSup = "traj_fromsup";
const std::string asp = "asp";
const std::string osem = "osem";
const std::string dataDict = "data_dict";
const std::string executeAction = "execute_action";
const std::string eventOccurred = "event_occurred";
const std::string actionConfiguration = "action_configuration";
const std::string triggerConfiguration = "trigger_configuration";
const std::string disarm = "disarm";
const std::string getStatus = "get_status";
const std::string getStatusResponse = "get_status_response";
const std::string backToStart = "back_to_start";
const std::string backToStartResponse = "back_to_start_response";
const std::string remoteControlEnable = "remote_control_enable";
const std::string remoteControlDisable = "remote_control_disable";
const std::string remoteControlManoeuvre = "remote_control_manoeuvre";
const std::string enableObject = "enable_object";
const std::string objectsConnected = "objects_connected";
const std::string failure = "failure";
}  // namespace TopicNames

namespace ServiceNames {
const std::string initDataDict = "init_data_dictionary";
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
class Module : public Node {
   public:
	Module(const std::string name) : Node(name){};
	Module() = default;

   protected:
	rclcpp::Publisher<String>::SharedPtr getStatusResponsePub;
	rclcpp::Subscription<String>::SharedPtr getStatusResponseSub;

	rclcpp::Publisher<Empty>::SharedPtr getStatusPub;
	rclcpp::Subscription<Empty>::SharedPtr getStatusSub;

	rclcpp::Publisher<UInt8>::SharedPtr failurePub;
	rclcpp::Subscription<UInt8>::SharedPtr failureSub;

	rclcpp::Publisher<Empty>::SharedPtr initPub;
	rclcpp::Subscription<Empty>::SharedPtr initSub;

	rclcpp::Publisher<Empty>::SharedPtr connectPub;
	rclcpp::Subscription<Empty>::SharedPtr connectSub;

	rclcpp::Publisher<Empty>::SharedPtr armPub;
	rclcpp::Subscription<Empty>::SharedPtr armSub;

	rclcpp::Publisher<Empty>::SharedPtr startPub;
	rclcpp::Subscription<Empty>::SharedPtr startSub;

	rclcpp::Publisher<Empty>::SharedPtr stopPub;
	rclcpp::Subscription<Empty>::SharedPtr stopSub;

	rclcpp::Publisher<Empty>::SharedPtr exitPub;
	rclcpp::Subscription<Empty>::SharedPtr exitSub;

	rclcpp::Publisher<Empty>::SharedPtr replayPub;
	rclcpp::Subscription<Empty>::SharedPtr replaySub;

	rclcpp::Publisher<Empty>::SharedPtr abortPub;
	rclcpp::Subscription<Empty>::SharedPtr abortSub;

	rclcpp::Publisher<Empty>::SharedPtr allClearPub;
	rclcpp::Subscription<Empty>::SharedPtr allClearSub;

	rclcpp::Publisher<Empty>::SharedPtr obcStatePub;
	rclcpp::Subscription<Empty>::SharedPtr obcStateSub;

	rclcpp::Publisher<Empty>::SharedPtr disconnectPub;
	rclcpp::Subscription<Empty>::SharedPtr disconnectSub;

	rclcpp::Subscription<Empty>::SharedPtr viopSub;
	rclcpp::Publisher<Empty>::SharedPtr viopPub;

	rclcpp::Subscription<Empty>::SharedPtr trajSub;
	rclcpp::Publisher<Empty>::SharedPtr trajPub;

	rclcpp::Subscription<Empty>::SharedPtr trajToSupSub;
	rclcpp::Publisher<Empty>::SharedPtr trajToSupPub;

	rclcpp::Subscription<Empty>::SharedPtr trajFromSupSub;
	rclcpp::Publisher<Empty>::SharedPtr trajFromSupPub;

	rclcpp::Subscription<Empty>::SharedPtr aspSub;
	rclcpp::Publisher<Empty>::SharedPtr aspPub;

	rclcpp::Subscription<Empty>::SharedPtr osemSub;
	rclcpp::Publisher<Empty>::SharedPtr osemPub;

	rclcpp::Subscription<Exac>::SharedPtr exacSub;
	rclcpp::Publisher<Exac>::SharedPtr exacPub;

	rclcpp::Subscription<Accm>::SharedPtr accmSub;
	rclcpp::Publisher<Accm>::SharedPtr accmPub;

	rclcpp::Subscription<Empty>::SharedPtr treoSub;
	rclcpp::Publisher<Empty>::SharedPtr treoPub;

	rclcpp::Subscription<Empty>::SharedPtr trcmSub;
	rclcpp::Publisher<Empty>::SharedPtr trcmPub;

	rclcpp::Subscription<Empty>::SharedPtr disarmSub;
	rclcpp::Publisher<Empty>::SharedPtr disarmPub;

	rclcpp::Subscription<ManoeuvreCommand>::SharedPtr backToStartSub;
	rclcpp::Publisher<ManoeuvreCommand>::SharedPtr backToStartPub;

	rclcpp::Subscription<Empty>::SharedPtr backToStartResponseSub;
	rclcpp::Publisher<Empty>::SharedPtr backToStartResponsePub;

	rclcpp::Subscription<Empty>::SharedPtr remoteControlEnableSub;
	rclcpp::Publisher<Empty>::SharedPtr remoteControlEnablePub;

	rclcpp::Subscription<Empty>::SharedPtr remoteControlDisableSub;
	rclcpp::Publisher<Empty>::SharedPtr remoteControlDisablePub;

	rclcpp::Subscription<Empty>::SharedPtr remoteControlManoeuvreSub;
	rclcpp::Publisher<Empty>::SharedPtr remoteControlManoeuvrePub;

	rclcpp::Subscription<ObjectEnabled>::SharedPtr enableObjectSub;
	rclcpp::Publisher<ObjectEnabled>::SharedPtr enableObjectPub;

	rclcpp::Subscription<Empty>::SharedPtr objectsConnectedSub;
	rclcpp::Publisher<Empty>::SharedPtr objectsConnectedPub;

	rclcpp::Subscription<Empty>::SharedPtr dataDictSub;
	rclcpp::Publisher<Empty>::SharedPtr dataDictPub;

	rclcpp::Subscription<Empty>::SharedPtr dataDictResponseSub;
	rclcpp::Publisher<Empty>::SharedPtr dataDictResponsePub;

	virtual void onFailureMessage(const UInt8::SharedPtr){};
	virtual void onGetStatusResponse(const String::SharedPtr){};
	virtual void onGetStatusMessage(const Empty::SharedPtr) {
		auto msg = String();
		msg.data = this->get_name();
		getStatusResponsePub->publish(msg);
	};
	virtual void onInitMessage(const Empty::SharedPtr){};
	virtual void onConnectMessage(const Empty::SharedPtr){};
	virtual void onDisconnectMessage(const Empty::SharedPtr){};
	virtual void onArmMessage(const Empty::SharedPtr){};
	virtual void onDisarmMessage(const Empty::SharedPtr){};
	virtual void onRemoteControlEnableMessage(const Empty::SharedPtr){};
	virtual void onRemoteControlDisableMessage(const Empty::SharedPtr){};
	virtual void onRemoteControlManoeuvreMessage(const Empty::SharedPtr){};
	virtual void onEnableObjectMessage(const ObjectEnabled::SharedPtr){};
	virtual void onObjectsConnectedMessage(const Empty::SharedPtr){};
	virtual void onDataDictMessage(const Empty::SharedPtr){};
	virtual void onOSEMMessage(const Empty::SharedPtr){};
	virtual void onASPMessage(const Empty::SharedPtr){};
	virtual void onTrajMessage(const Empty::SharedPtr){};
	virtual void onTrajToSupMessage(const Empty::SharedPtr){};
	virtual void onTrajFromSupMessage(const Empty::SharedPtr){};
	virtual void onAllClearMessage(const Empty::SharedPtr){};
	virtual void onOBCStateMessage(const Empty::SharedPtr){};
	virtual void onVIOPMessage(const Empty::SharedPtr){};
	virtual void onStartMessage(const Empty::SharedPtr){};
	virtual void onStopMessage(const Empty::SharedPtr){};
	virtual void onAbortMessage(const Empty::SharedPtr) = 0;
	virtual void onACCMMessage(const Accm::SharedPtr){};
	virtual void onTRCMMessage(const Empty::SharedPtr){};
	virtual void onEXACMessage(const Exac::SharedPtr){};
	virtual void onTREOMessage(const Empty::SharedPtr){};
	virtual void onExitMessage(const Empty::SharedPtr){};
	virtual void onReplayMessage(const Empty::SharedPtr){};
	virtual void onBackToStartMessage(const Empty::SharedPtr){};
	virtual void onBackToStartResponse(const Int8::SharedPtr){};
	virtual void onDataDictResponse(const Empty::SharedPtr){};

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
