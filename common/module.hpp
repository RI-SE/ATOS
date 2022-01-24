#include <iostream>
#include <string>

#include "util.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

using std_msgs::Empty;
using std_msgs::String;

/**
 *  Topic for publishing and subscribing to messages
 *
 *  \tparam Msg Type of the message to be sent / recevied on this topic
 *  \tparam Node ROS node that should subscribe / publish on this topic
 *
 */
template <typename Msg, class Node>
class Topic {
   public:
	/**
	 * Default constructor
	 * \param topicName name of topic
	 * \param queueSize maximum number of messages the topic can buffer before deleting new messages
	 * \param cb callback function
	 * \param n ROS node subscribing/publishing to this topic
	 */
	Topic(const std::string topicName, int queueSize, void (Node::*cb)(typename Msg::ConstPtr), Node* n) {
		this->pub = n->template advertise<Msg>(topicName, queueSize);
		this->sub = n->subscribe(topicName, queueSize, cb, n);
	}
	void publish(Msg message) { pub.publish(message); }

   private:
	ros::Publisher pub;
	ros::Subscriber sub;
};

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

/*!
 * \brief The Module class
 * This class is the base class for all modules.
 * It provides the basic functionality for all modules.
 */
class Module : public ros::NodeHandle {
public:
	Module(const std::string name) : name(name){};
	Module() = default;
	
	Topic<String, Module> getStatusResponseTopic
		= Topic<String, Module>(topicNames[COMM_GETSTATUS_OK], 100, &Module::onGetStatusResponse, this);
	Topic<Empty, Module> failureTopic
		= Topic<Empty, Module>(topicNames[COMM_FAILURE], 100, &Module::onFailureMessage, this);
	Topic<Empty, Module> initTopic
		= Topic<Empty, Module>(topicNames[COMM_INIT], 1, &Module::onInitMessage, this);
	Topic<Empty, Module> connectTopic
		= Topic<Empty, Module>(topicNames[COMM_CONNECT], 1, &Module::onConnectMessage, this);
	Topic<Empty, Module> armTopic
		= Topic<Empty, Module>(topicNames[COMM_ARM], 1, &Module::onArmMessage, this);
	Topic<Empty, Module> startTopic
		= Topic<Empty, Module>(topicNames[COMM_STRT], 1, &Module::onStartMessage, this);
	Topic<Empty, Module> stopTopic
		= Topic<Empty, Module>(topicNames[COMM_STOP], 1, &Module::onStopMessage, this);
	Topic<Empty, Module> exitTopic
		= Topic<Empty, Module>(topicNames[COMM_EXIT], 1, &Module::onExitMessage, this);
	Topic<Empty, Module> replayTopic
		= Topic<Empty, Module>(topicNames[COMM_REPLAY], 1, &Module::onReplayMessage, this);
	Topic<Empty, Module> abortTopic
		= Topic<Empty, Module>(topicNames[COMM_ABORT], 1, &Module::onAbortMessage, this);
	Topic<Empty, Module> allClearTopic
		= Topic<Empty, Module>(topicNames[COMM_ABORT_DONE], 1, &Module::onAllClearMessage, this);
	Topic<Empty, Module> obcStateTopic
		= Topic<Empty, Module>(topicNames[COMM_OBC_STATE], 1, &Module::onOBCStateMessage, this);
	Topic<Empty, Module> disconnectTopic
		= Topic<Empty, Module>(topicNames[COMM_DISCONNECT], 1, &Module::onDisconnectMessage, this);
	Topic<Empty, Module> viopTopic
		= Topic<Empty, Module>(topicNames[COMM_ABORT], 1, &Module::onVIOPMessage, this);
	Topic<Empty, Module> trajTopic
		= Topic<Empty, Module>(topicNames[COMM_TRAJ], 1, &Module::onTrajMessage, this);
	Topic<Empty, Module> trajToSupTopic
		= Topic<Empty, Module>(topicNames[COMM_TRAJ_TOSUP], 1, &Module::onTrajToSupMessage, this);
	Topic<Empty, Module> trajFromSupTopic
		= Topic<Empty, Module>(topicNames[COMM_TRAJ_FROMSUP], 1, &Module::onTrajFromSupMessage, this);
	Topic<Empty, Module> aspTopic
		= Topic<Empty, Module>(topicNames[COMM_ASP], 1, &Module::onASPMessage, this);
	Topic<Empty, Module> osemTopic
		= Topic<Empty, Module>(topicNames[COMM_OSEM], 1, &Module::onOSEMMessage, this);
	Topic<Empty, Module> dataDictTopic
		= Topic<Empty, Module>(topicNames[COMM_DATA_DICT], 1, &Module::onDataDictMessage, this);
	Topic<Empty, Module> exacTopic
		= Topic<Empty, Module>(topicNames[COMM_EXAC], 1, &Module::onEXACMessage, this);
	Topic<Empty, Module> accmTopic
		= Topic<Empty, Module>(topicNames[COMM_ACCM], 1, &Module::onACCMMessage, this);
	Topic<Empty, Module> treoTopic
		= Topic<Empty, Module>(topicNames[COMM_TREO], 1, &Module::onTREOMessage, this);
	Topic<Empty, Module> trcmTopic
		= Topic<Empty, Module>(topicNames[COMM_TRCM], 1, &Module::onTRCMMessage, this);
	Topic<Empty, Module> disarmTopic
		= Topic<Empty, Module>(topicNames[COMM_DISARM], 1, &Module::onDisarmMessage, this);
	Topic<Empty, Module> backToStartTopic
		= Topic<Empty, Module>(topicNames[COMM_BACKTOSTART_CALL], 1, &Module::onBackToStartMessage, this);
	Topic<Empty, Module> backToStartResponseTopic
		= Topic<Empty, Module>(topicNames[COMM_BACKTOSTART_RESPONSE], 1, &Module::onBackToStartResponse, this);
	Topic<Empty, Module> remoteControlEnableTopic
		= Topic<Empty, Module>(topicNames[COMM_REMOTECTRL_ENABLE], 1, &Module::onRemoteControlEnableMessage, this);
	Topic<Empty, Module> remoteControlDisableTopic
		= Topic<Empty, Module>(topicNames[COMM_REMOTECTRL_DISABLE], 1, &Module::onRemoteControlEnableMessage, this);
	Topic<Empty, Module> remoteControlManoeuvreTopic
		= Topic<Empty, Module>(topicNames[COMM_REMOTECTRL_MANOEUVRE], 1, &Module::onRemoteControlManoeuvreMessage, this);
	Topic<Empty, Module> enableObjectTopic
		= Topic<Empty, Module>(topicNames[COMM_ENABLE_OBJECT], 1, &Module::onEnableObjectMessage, this);
	Topic<Empty, Module> objectsConnectedTopic
		= Topic<Empty, Module>(topicNames[COMM_OBJECTS_CONNECTED], 1, &Module::onObjectsConnectedMessage, this);

	std::string getName() const { return name; }

private:
	std::string name;

	static void printUnhandledMessage(const std::string& topic, const std::string& message) {
		std::cout << "Unhandled message on topic: " << topic << " (" << message << ")" << std::endl;
	}
	static void printUnhandledMessage(const std::string& topic) {
		std::cout << "Unhandled message on topic: " << topic << std::endl;
	}

	virtual void onFailureMessage(Empty::ConstPtr) { };
	virtual void onGetStatusResponse(String::ConstPtr) { };
	virtual void onGetStatusMessage(Empty::ConstPtr) { 
		String msg;
		msg.data = name;
		getStatusResponseTopic.publish(msg); };
	virtual void onInitMessage(Empty::ConstPtr){};
	virtual void onConnectMessage(Empty::ConstPtr){};
	virtual void onDisconnectMessage(Empty::ConstPtr){};
	virtual void onArmMessage(Empty::ConstPtr){};
	virtual void onDisarmMessage(Empty::ConstPtr){};
	virtual void onRemoteControlEnableMessage(Empty::ConstPtr){};
	virtual void onRemoteControlDisableMessage(Empty::ConstPtr){};
	virtual void onRemoteControlManoeuvreMessage(Empty::ConstPtr){};
	virtual void onEnableObjectMessage(Empty::ConstPtr){};
	virtual void onObjectsConnectedMessage(Empty::ConstPtr){};
	virtual void onDataDictMessage(Empty::ConstPtr){};
	virtual void onOSEMMessage(Empty::ConstPtr){};
	virtual void onASPMessage(Empty::ConstPtr){};
	virtual void onTrajMessage(Empty::ConstPtr){};
	virtual void onTrajToSupMessage(Empty::ConstPtr){};
	virtual void onTrajFromSupMessage(Empty::ConstPtr){};
	virtual void onAllClearMessage(Empty::ConstPtr){};
	virtual void onOBCStateMessage(Empty::ConstPtr){};
	virtual void onVIOPMessage(Empty::ConstPtr){};
	virtual void onStartMessage(Empty::ConstPtr){};
	virtual void onStopMessage(Empty::ConstPtr){};
	virtual void onAbortMessage(Empty::ConstPtr){};
	virtual void onACCMMessage(Empty::ConstPtr){};
	virtual void onTRCMMessage(Empty::ConstPtr){};
	virtual void onEXACMessage(Empty::ConstPtr){};
	virtual void onTREOMessage(Empty::ConstPtr){};
	virtual void onExitMessage(Empty::ConstPtr){};
	virtual void onReplayMessage(Empty::ConstPtr){};
	virtual void onBackToStartMessage(Empty::ConstPtr){};
	virtual void onBackToStartResponse(Empty::ConstPtr){};
};
