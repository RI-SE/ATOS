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

RelativeKinematicsModule::RelativeKinematicsModule(LOG_LEVEL logLevel) : Module(RelativeKinematicsModule::module_name){
	this->initialize(logLevel);
	scenarioHandler = std::make_unique<ScenarioHandler>();
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
	this->getStatusResponsePub = this->create_publisher<String>(topicNames[COMM_GETSTATUS_OK],queueSize);	
};

void RelativeKinematicsModule::tryHandleMessage(COMMAND commandCode, std::function<void()> tryExecute, std::function<void()> executeIfFail){
	try{
		LogMessage(LOG_LEVEL_DEBUG, "Handling %s command", topicNames[commandCode].c_str());
		tryExecute();
	}
	catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Handling %s command failed - %s", topicNames[commandCode].c_str(), e.what());
		executeIfFail();
	}
}

int RelativeKinematicsModule::initialize(const LOG_LEVEL logLevel) {
	int retval = 0;

	// Initialize log
	LogInit(module_name.c_str(), logLevel);
	LogMessage(LOG_LEVEL_INFO, "%s task running with PID: %d",module_name.c_str(), getpid());

	// Create test journal
	if (JournalInit(module_name.c_str()) == -1) {
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR, "Unable to create test journal");
	}
	if (DataDictionaryInitStateData() != READ_OK) {
		DataDictionaryFreeStateData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for state data");
	}
	if (DataDictionaryInitObjectData() != READ_OK) {
		DataDictionaryFreeObjectData();
		retval = -1;
		LogMessage(LOG_LEVEL_ERROR,
					"Found no previously initialized shared memory for object data");
	}

	return retval;
}

void RelativeKinematicsModule::onInitMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_INIT;
	auto f_try = [&]() { scenarioHandler->handleInitCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onConnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_CONNECT;
	auto f_try = [&]() { scenarioHandler->handleConnectCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onArmMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ARM;
	auto f_try = [&]() { scenarioHandler->handleArmCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onStartMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_STRT;
	auto f_try = [&]() { scenarioHandler->handleStartCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onDisconnectMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_DISCONNECT;
	auto f_try = [&]() { scenarioHandler->handleDisconnectCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onStopMessage(const Empty::SharedPtr){
	COMMAND cmd = COMM_STOP;
	auto f_try = [&]() { scenarioHandler->handleStopCommand(); };
	auto f_catch = [&]() {
			failurePub->publish(msgCtr1<UInt8>(cmd));
			abortPub->publish(Empty());
	};
	this->tryHandleMessage(cmd,f_try,f_catch);	
}

void RelativeKinematicsModule::onAbortMessage(const Empty::SharedPtr){	
	// Any exceptions here should crash the program
	scenarioHandler->handleAbortCommand();
}

void RelativeKinematicsModule::onAllClearMessage(const Empty::SharedPtr){	
	COMMAND cmd = COMM_ABORT_DONE;
	auto f_try = [&]() { scenarioHandler->handleAllClearCommand(); };
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onACCMMessage(const Accm::SharedPtr accm){
	COMMAND cmd = COMM_ACCM;
	auto f_try = [&]() {
		if (accm->action_type == ACTION_TEST_SCENARIO_COMMAND) {
			ScenarioHandler::TestScenarioCommandAction cmdAction;
			cmdAction.command = static_cast<ActionTypeParameter_t>(accm->action_type_parameter1);
			cmdAction.actionID = accm->action_id;
			cmdAction.objectID = scenarioHandler->getVehicleIDByIP(accm->ip);
			scenarioHandler->handleActionConfigurationCommand(cmdAction);
		}
	};
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}

void RelativeKinematicsModule::onEXACMessage(const Exac::SharedPtr exac){
	COMMAND cmd = COMM_EXAC;
	auto f_try = [&]() {
		using namespace std::chrono;
		quartermilliseconds qmsow(exac->executiontime_qmsow);
		auto now = to_timeval(system_clock::now().time_since_epoch());
		auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
		scenarioHandler->handleExecuteActionCommand(exac->action_id, startOfWeek+qmsow);	
	};
	auto f_catch = [&]() { failurePub->publish(msgCtr1<UInt8>(cmd)); };
	this->tryHandleMessage(cmd,f_try,f_catch);
}
