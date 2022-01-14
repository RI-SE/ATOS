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
void RelativeKinematicsModule::initCB(const Empty&){
    try {
        scenarioHandler.handleInitCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Initialization failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::connectCB(const Empty&){	
    try {
        scenarioHandler.handleConnectCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Connection failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::armCB(const Empty&){	
    try {
        scenarioHandler.handleArmCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Arm failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}

void RelativeKinematicsModule::startCB(const Empty&){	
    try {
        scenarioHandler.handleStartCommand();
    } catch (std::invalid_argument& e) {
        LogMessage(LOG_LEVEL_ERROR, "Start failed - %s", e.what());
        this->failureTopic.publish(Empty());
    }
}