/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <thread>

#include "scenario.h"
#include "module.hpp"
#include "braketrigger.h"
#include "trigger.h"
#include "datadictionary.h"
#include "scenario.h"
#include "util.h"
#include "journal.hpp"

namespace ATOS {

class ScenarioControl : public Module
{
public:
	explicit ScenarioControl();
	~ScenarioControl();
	int initialize();
private:
	static inline std::string const moduleName = "scenario_control";

	ROSChannels::Init::Sub initSub;
	ROSChannels::Arm::Sub armSub;
	ROSChannels::Start::Sub startSub;
	ROSChannels::Abort::Sub abortSub;
	ROSChannels::Exit::Sub exitSub;
	ROSChannels::ObjectsConnected::Sub objectsConnectedSub;
	ROSChannels::Disconnect::Sub disconnectSub;
	ROSChannels::TriggerEventOccurred::Sub triggerEventSub;
	ROSChannels::GetStatus::Sub getStatusSub;

	ROSChannels::ActionConfiguration::Pub accmPub;
	ROSChannels::TriggerConfiguration::Pub trcmPub;
	ROSChannels::ExecuteAction::Pub exacPub;

	std::unique_ptr<std::thread> manageTriggersThread;

	void sendConfiguration();
	enum state_t {UNINITIALIZED, INITIALIZED, CONNECTED, RUNNING};
	state_t state = UNINITIALIZED;
	const std::map<state_t,std::string> stateToString = {
		{UNINITIALIZED,"Uninitialized"},
		{INITIALIZED,"Initialized"},
		{CONNECTED,"Connected"},
		{RUNNING,"Running"}
	};
	std::unique_ptr<Scenario> scenario;
	static inline const std::string triggerActionFileName = "triggeraction.conf";
	static inline const std::string openDriveFileName = "opendrive.xodr";
	static inline const std::string openScenarioFileName = "openscenario.xosc";
	char triggerActionConfigPath[MAX_FILE_PATH];
	char openDriveConfigPathPath[MAX_FILE_PATH];
	char openScenarioConfigPath[MAX_FILE_PATH];

	//! Scenario is executed with x hz
	static constexpr auto scenarioCheckPeriod = std::chrono::milliseconds(1);
	//! Shmem is read with x hz
	static constexpr auto shmemReadPeriod = std::chrono::milliseconds(10);

	std::chrono::time_point<std::chrono::steady_clock> nextShmemReadTime;

	void onInitMessage(const ROSChannels::Init::message_type::SharedPtr) override;
	void onArmMessage(const ROSChannels::Arm::message_type::SharedPtr) override;
	void onStartMessage(const ROSChannels::Start::message_type::SharedPtr) override;
	void onAbortMessage(const ROSChannels::Abort::message_type::SharedPtr) override;
	void onExitMessage(const ROSChannels::Exit::message_type::SharedPtr) override;
	void onObjectsConnectedMessage(const ROSChannels::ObjectsConnected::message_type::SharedPtr) override;
	void onDisconnectMessage(const ROSChannels::Disconnect::message_type::SharedPtr) override;
	void onTriggerEventMessage(const ROSChannels::TriggerEventOccurred::message_type::SharedPtr) override;

	void manageTriggers();
	int updateTriggers();
	std::chrono::time_point<std::chrono::steady_clock> getNextReadTime(std::chrono::time_point<std::chrono::steady_clock> now);
};

} // namespace ATOS