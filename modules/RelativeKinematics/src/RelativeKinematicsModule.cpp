#include <iostream>
#include <unistd.h>
#include <signal.h>

#include "state.hpp"
#include "logging.h"
#include "util.h"
#include "journal.h"
#include "datadictionary.h"
#include "RelativeKinematicsModule.hpp"

using std_msgs::Empty;
using std_msgs::String;

void RelativeKinematicsModule::onInitMessage(Empty::ConstPtr){
    try {
        scenarioHandler.handleInitCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Initialization failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onConnectMessage(Empty::ConstPtr){	
    try {
        scenarioHandler.handleConnectCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Connection failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onArmMessage(Empty::ConstPtr){	
    try {
        scenarioHandler.handleArmCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Arm failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onStartMessage(Empty::ConstPtr){	
    try {
        scenarioHandler.handleStartCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Start failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onDisconnectMessage(Empty::ConstPtr){	
    try {
        scenarioHandler.handleDisconnectCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Disconnect failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onStopMessage(Empty::ConstPtr){	
    try {
        scenarioHandler.handleStopCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Stop failed - %s", e.what());
        scenarioHandler.handleAbortCommand();
        this->failureTopic.publish(Empty());
        this->abortTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::onAbortMessage(Empty::ConstPtr){	
	// Any exceptions here should crash the program
    scenarioHandler.handleAbortCommand();
}

void RelativeKinematicsModule::onAllClearMessage(Empty::ConstPtr){	
	try {
		scenarioHandler.handleAllClearCommand();
	} catch (std::invalid_argument& e) {
		LogMessage(LOG_LEVEL_ERROR, "Failed clear abort - %s", e.what());
		this->failureTopic.publish(Empty());
	}
}

void RelativeKinematicsModule::onACCMMessage(Empty::ConstPtr){
	/*
			try {
				ACCMData accm;
				UtilPopulateACCMDataStructFromMQ(mqRecvData, sizeof (mqRecvData), &accm);
				if (accm.actionType == ACTION_TEST_SCENARIO_COMMAND) {
					ScenarioHandler::TestScenarioCommandAction cmdAction;
					cmdAction.command = static_cast<ActionTypeParameter_t>(accm.actionTypeParameter1);
					cmdAction.actionID = accm.actionID;
					cmdAction.objectID = scenarioHandler.getVehicleIDByIP(accm.ip);
					scenarioHandler.handleActionConfigurationCommand(cmdAction);
				}
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Failed action configuration - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
			*/
}

void RelativeKinematicsModule::onEXACMessage(Empty::ConstPtr){
	/*
			
			try {
				using namespace std::chrono;
				EXACData exac;
				UtilPopulateEXACDataStructFromMQ(mqRecvData, sizeof (mqRecvData), &exac);
				quartermilliseconds qmsow(exac.executionTime_qmsoW);
				auto now = to_timeval(system_clock::now().time_since_epoch());
				auto startOfWeek = system_clock::time_point(weeks(TimeGetAsGPSweek(&now)));
				scenarioHandler.handleExecuteActionCommand(exac.actionID, startOfWeek+qmsow);
			}
			catch (std::invalid_argument& e) {
				LogMessage(LOG_LEVEL_ERROR, "Failed action execution - %s", e.what());
				iCommSend(COMM_FAILURE, nullptr, 0);
			}
			break;
			*/
}
