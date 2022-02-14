#include "module.hpp"
#include "scenariohandler.hpp"

#define MODULE_NAME "RelativeKinematics"
class RelativeKinematicsModule : public Module
{
private:
	ScenarioHandler scenarioHandler;
	void handleMessage(std::function<void()>& handleCommand, std::function<void()>& catchC);
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

	void tryHandleMessage(COMMAND commandCode, std::function<void()>& tryExecute,std::function<void()>& executeIfFail);

public:
	static constexpr char* module_name = "RelativeKinematics";
	RelativeKinematicsModule();
};
