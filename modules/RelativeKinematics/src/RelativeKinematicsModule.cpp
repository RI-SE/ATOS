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

void RelativeKinematicsModule::onInitMessage(const Empty::SharedPtr){
    try {
        scenarioHandler.handleInitCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Initialization failed - %s", e.what());
		UInt8 msg = std_msgs::msg::UInt8();
		msg.data = COMM_INIT;
        failurePub->publish(msg);
    }
}

void RelativeKinematicsModule::onConnectMessage(Empty::SharedPtr){	
    try {
        scenarioHandler.handleConnectCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Connection failed - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_CONNECT;
        this->failurePub->publish(msg);
    }
}

void RelativeKinematicsModule::onArmMessage(Empty::SharedPtr){	
    try {
        scenarioHandler.handleArmCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Arm failed - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_ARM;
        this->failurePub->publish(msg);
    }
}

void RelativeKinematicsModule::onStartMessage(Empty::SharedPtr){	
    try {
        scenarioHandler.handleStartCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Start failed - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_STRT;
        this->failurePub->publish(msg);
    }
}

void RelativeKinematicsModule::onDisconnectMessage(Empty::SharedPtr){	
    try {
        scenarioHandler.handleDisconnectCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Disconnect failed - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_DISCONNECT;
        this->failurePub->publish(msg);
    }
}

void RelativeKinematicsModule::onStopMessage(Empty::SharedPtr){	
    try {
        scenarioHandler.handleStopCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Stop failed - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_STOP;
        scenarioHandler.handleAbortCommand();
        this->failurePub->publish(msg);
        this->abortPub->publish(Empty());
    }
}

void RelativeKinematicsModule::onGetStatusMessage(Empty::SharedPtr){
	String msg;
	msg.data = MODULE_NAME;
	this->getStatusResponsePub->publish(msg);
}

void RelativeKinematicsModule::onAbortMessage(Empty::SharedPtr){	
	// Any exceptions here should crash the program
    scenarioHandler.handleAbortCommand();
}

void RelativeKinematicsModule::onAllClearMessage(Empty::SharedPtr){	
	try {
		scenarioHandler.handleAllClearCommand();
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Failed clear abort - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_ABORT_DONE;
		this->failurePub->publish(msg);
	}
}

void RelativeKinematicsModule::onACCMMessage(Accm::SharedPtr accm){
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
		auto msg = UInt8();
		msg.data = COMM_ACCM;
        this->failurePub->publish(msg);
	}
}

void RelativeKinematicsModule::onEXACMessage(Exac::SharedPtr exac){	
	try {
		using namespace std::chrono;
		quartermilliseconds qmsow(exac->executiontime_qmsow);
		auto now = to_timeval(system_clock::now().time_since_epoch());
		auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
		scenarioHandler.handleExecuteActionCommand(exac->action_id, startOfWeek+qmsow);
	}
	catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Failed action execution - %s", e.what());
		auto msg = UInt8();
		msg.data = COMM_EXAC;
        this->failurePub->publish(msg);
	}
}
