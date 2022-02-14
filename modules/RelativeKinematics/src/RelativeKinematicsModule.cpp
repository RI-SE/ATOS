#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "state.hpp"
#include "logging.h"
#include "util.h"
#include "journal.h"
#include "datadictionary.h"
#include "RelativeKinematicsModule.hpp"

using std_msgs::msg::Empty;
using std_msgs::msg::String;
using std_msgs::msg::UInt8;


void RelativeKinematicsModule::tryHandleMessage(COMMAND commandCode, std::function<void()>& tryExecute,std::function<void()>& executeIfFail){
	try{
		LogMessage(LOG_LEVEL_DEBUG, "Handling %s command", topicNames[commandCode]);
		tryExecute();
	}
	catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Handling %s command failed - %s", topicNames[commandCode], e.what());
		executeIfFail();
	}
}

RelativeKinematicsModule::RelativeKinematicsModule() : Module(RelativeKinematicsModule::module_name){
	int queueSize=0;
	// ** Subscriptions
	this->initSub = this->create_subscription<Empty>(topicNames[COMM_INIT], queueSize, std::bind(&RelativeKinematicsModule::onInitMessage, this, _1));
	this->connectSub = this->create_subscription<Empty>(topicNames[COMM_CONNECT], queueSize, std::bind(&RelativeKinematicsModule::onConnectMessage, this, _1));
	this->armSub = this->create_subscription<Empty>(topicNames[COMM_ARM], queueSize, std::bind(&RelativeKinematicsModule::onArmMessage, this, _1));
	this->startSub = this->create_subscription<Empty>(topicNames[COMM_STRT], queueSize, std::bind(&RelativeKinematicsModule::onStartMessage, this, _1));
	this->disconnectSub = this->create_subscription<Empty>(topicNames[COMM_DISCONNECT], queueSize, std::bind(&RelativeKinematicsModule::onDisconnectMessage, this, _1));
	this->stopSub = this->create_subscription<Empty>(topicNames[COMM_STOP], queueSize, std::bind(&RelativeKinematicsModule::onStopMessage, this, _1));
	this->abortSub = this->create_subscription<Empty>(topicNames[COMM_ABORT], queueSize, std::bind(&RelativeKinematicsModule::onAbortMessage, this, _1));
	this->allClearSub = this->create_subscription<Empty>(topicNames[COMM_ABORT_DONE], queueSize, std::bind(&RelativeKinematicsModule::onAllClearMessage, this, _1));
	this->accmSub = this->create_subscription<Accm>(topicNames[COMM_ACCM], queueSize, std::bind(&RelativeKinematicsModule::onACCMMessage, this, _1));
	this->exacSub = this->create_subscription<Exac>(topicNames[COMM_EXAC], queueSize, std::bind(&RelativeKinematicsModule::onEXACMessage, this, _1));
	this->getStatusSub = this->create_subscription<Empty>(topicNames[COMM_GETSTATUS], queueSize, std::bind(&RelativeKinematicsModule::onGetStatusMessage, this, _1));

	// ** Publishers
	this->failurePub = this->create_publisher<UInt8>(topicNames[COMM_FAILURE],queueSize);
	this->abortPub = this->create_publisher<Empty>(topicNames[COMM_ABORT],queueSize);
	this->getStatusResponsePub = this->create_publisher<String>(topicNames[COMM_GETSTATUS_OK],queueSize);
	
};

void RelativeKinematicsModule::onInitMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_INIT;
	std::function<void()> f_try = [&]() { scenarioHandler.handleInitCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onConnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_CONNECT;
	std::function<void()> f_try = [&]() { scenarioHandler.handleConnectCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onArmMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ARM;
	std::function<void()> f_try = [&]() { scenarioHandler.handleArmCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onStartMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_STRT;
	std::function<void()> f_try = [&]() { scenarioHandler.handleStartCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onDisconnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_DISCONNECT;
	std::function<void()> f_try = [&]() { scenarioHandler.handleDisconnectCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onStopMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_STOP;
	std::function<void()> f_try = [&]() { scenarioHandler.handleStopCommand(); };
	std::function<void()> f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
			abortPub->publish(Empty());
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void RelativeKinematicsModule::onAbortMessage(const Empty::SharedPtr){	
	// Any exceptions here should crash the program
	scenarioHandler.handleAbortCommand();
}

void RelativeKinematicsModule::onAllClearMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ABORT_DONE;
	std::function<void()> f_try = [&]() { scenarioHandler.handleAllClearCommand(); };
	std::function<void()> f_catch = [&]() {	failurePub->publish(msgCtr1<UInt8>(cmd));};
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onACCMMessage(const Accm::SharedPtr accm){
	try {
		if (accm->action_type == ACTION_TEST_SCENARIO_COMMAND) {
			ScenarioHandler::TestScenarioCommandAction cmdAction;
			cmdAction.command = static_cast<ActionTypeParameter_t>(accm->action_type_parameter1);
			cmdAction.actionID = accm->action_id;
			cmdAction.objectID = scenarioHandler.getVehicleIDByIP(accm->ip);
			scenarioHandler.handleActionConfigurationCommand(cmdAction);
		}
	}
	catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Failed action configuration - %s", e.what());
		this->failurePub->publish(msgCtr1<UInt8>(COMM_ACCM));
	}
}

void RelativeKinematicsModule::onEXACMessage(const Exac::SharedPtr exac){	
	try {
		using namespace std::chrono;
		quartermilliseconds qmsow(exac->executiontime_qmsow);
		auto now = to_timeval(system_clock::now().time_since_epoch());
		auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
		scenarioHandler.handleExecuteActionCommand(exac->action_id, startOfWeek+qmsow);
	}
	catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Failed action execution - %s", e.what());
		this->failurePub->publish(msgCtr1<UInt8>(COMM_EXAC));
	}
}
