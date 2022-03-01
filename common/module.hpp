#include <iostream>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include "util.h"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int8.hpp"
#include "maestro_msgs/msg/exac.hpp"
#include "maestro_msgs/msg/accm.hpp"
#include "maestro_msgs/msg/manoeuvre_command.hpp"
#include "maestro_msgs/msg/object_enabled.hpp"


using std_msgs::msg::Empty;
using std_msgs::msg::String;
using std_msgs::msg::UInt8;
using std_msgs::msg::Int8;
using maestro_msgs::msg::Exac;
using maestro_msgs::msg::Accm;
using maestro_msgs::msg::ManoeuvreCommand;
using maestro_msgs::msg::ObjectEnabled;
using std::placeholders::_1;
using rclcpp::Node;

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
	{COMM_OBJECTS_CONNECTED, "/objects_connected"},
	{COMM_FAILURE, "/failure"}
};

//TODO move somewhere else? also make generic to allow more args (variadic template)?
/*!
 * \brief Facilitates one-line intialization
 * of a ros message with one argument. 
 */
template <typename Msg_T,typename MsgData_T>
Msg_T msgCtr1(MsgData_T data){
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
	Module(const std::string name) : Node(name) {};
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

	virtual void onFailureMessage(const UInt8::SharedPtr) {};
	virtual void onGetStatusResponse(const String::SharedPtr) { };
	virtual void onGetStatusMessage(const Empty::SharedPtr) { 
		auto msg = String();
		msg.data = this->get_name();
		getStatusResponsePub->publish(msg);
	};
	virtual void onInitMessage(const Empty::SharedPtr) {};
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

private:
	static void printUnhandledMessage(const std::string& topic, const std::string& message) {
		std::cout << "Unhandled message on topic: " << topic << " (" << message << ")" << std::endl;
	}
	static void printUnhandledMessage(const std::string& topic) {
		std::cout << "Unhandled empty message on topic: " << topic << std::endl;
	}
};
