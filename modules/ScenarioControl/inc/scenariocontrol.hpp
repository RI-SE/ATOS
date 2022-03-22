#include <thread>

#include "scenario.h"
#include "module.hpp"
#include "braketrigger.h"
#include "trigger.h"
#include "datadictionary.h"
#include "maestroTime.h"
#include "scenario.h"
#include "logging.h"
#include "util.h"
#include "journal.h"

class ScenarioControl : public Module
{
public:
	explicit ScenarioControl();
  ~ScenarioControl();
	void initialize();
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

  std::unique_ptr<std::thread> actionExecutorThread;

  enum {UNINITIALIZED, INITIALIZED, CONNECTED, RUNNING} state = UNINITIALIZED;
  Scenario scenario;
	TREOData treo;
	ObjectDataType monr;
	char configPath[MAX_FILE_PATH];
  int SHMEM_READ_RATE_HZ=100;

	struct timeval tvTime;
	struct timeval nextSHMEMreadTime = { 0, 0 };

  void onInitMessage(const Empty::SharedPtr) override;
	void onArmMessage(const Empty::SharedPtr) override;
  void onStartMessage(const Empty::SharedPtr) override;
	void onAbortMessage(const Empty::SharedPtr) override;
	void onExitMessage(const Empty::SharedPtr) override;
  void onObjectsConnectedMessage(const Empty::SharedPtr) override;
  void onDisconnectMessage(const Empty::SharedPtr) override;
  void onTriggerEventMessage(const TriggerEvent::SharedPtr) override;

  void executeScenario();
  int updateTriggers(Scenario& scenario);
  void updateObjectCheckTimer(struct timeval *currentSHMEMReadTime, uint8_t SHMEMReadRate_Hz);
};
