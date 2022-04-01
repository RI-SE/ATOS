#include <thread>

#include "scenario.h"
#include "module.hpp"
#include "braketrigger.h"
#include "trigger.h"
#include "datadictionary.h"
#include "scenario.h"
#include "util.h"
#include "journal.h"

using namespace std::chrono;

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
	ROSChannels::TriggerEvent::Sub triggerEventSub;
	ROSChannels::GetStatus::Sub getStatusSub;

	std::unique_ptr<std::thread> manageTriggersThread;

	enum state_t {UNINITIALIZED, INITIALIZED, CONNECTED, RUNNING};
	state_t state = UNINITIALIZED;
	const std::map<state_t,std::string> stateToString = {
		{UNINITIALIZED,"Uninitialized"},
		{INITIALIZED,"Initialized"},
		{CONNECTED,"Connected"},
		{RUNNING,"Running"}
	};
	Scenario scenario;
	static inline const std::string triggerActionFileName = "triggeraction.conf";
	char configPath[MAX_FILE_PATH];

	//! Scenario is executed with x hz
	typedef duration<int, std::ratio<1, 1000>> scenarioDuration;
	//! Shmem is read with x hz
	typedef duration<int, std::ratio<1, 100>> shmemReadDuration;

	time_point<steady_clock> nextShmemReadTime;

	void onInitMessage(const Empty::SharedPtr) override;
	void onArmMessage(const Empty::SharedPtr) override;
	void onStartMessage(const Empty::SharedPtr) override;
	void onAbortMessage(const Empty::SharedPtr) override;
	void onExitMessage(const Empty::SharedPtr) override;
	void onObjectsConnectedMessage(const maestro_interfaces::msg::ObjectIdArray::SharedPtr) override;
	void onDisconnectMessage(const Empty::SharedPtr) override;
	void onTriggerEventMessage(const TriggerEvent::SharedPtr) override;

	void manageTriggers();
	int updateTriggers(Scenario& scenario);
	time_point<steady_clock> getNextReadTime(time_point<steady_clock> now);
};
