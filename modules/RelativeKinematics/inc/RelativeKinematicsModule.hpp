#include "module.hpp"
#include "scenariohandler.hpp"

class RelativeKinematicsModule : public Module
{
public:
	int initialize(LOG_LEVEL logLevel);
	RelativeKinematicsModule(LOG_LEVEL logLevel);
private:
	static inline std::string const module_name = "RelativeKinematics";
	std::unique_ptr<ScenarioHandler> scenarioHandler;
	void onInitMessage(const Empty::SharedPtr) override;
	void onConnectMessage(const Empty::SharedPtr) override;
	void onArmMessage(const Empty::SharedPtr) override;
	void onStartMessage(const Empty::SharedPtr) override;
	void onDisconnectMessage(const Empty::SharedPtr) override;
	void onStopMessage(const Empty::SharedPtr) override;
	void onAbortMessage(const Empty::SharedPtr) override;
	void onAllClearMessage(const Empty::SharedPtr) override;
	void onACCMMessage(const Accm::SharedPtr) override;
	void onEXACMessage(const Exac::SharedPtr) override;

	void tryHandleMessage(COMMAND commandCode, std::function<void()> tryExecute, std::function<void()> executeIfFail);
};
