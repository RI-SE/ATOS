#include "module.hpp"
#include "scenariohandler.hpp"

using std_msgs::msg::Empty;
using std_msgs::msg::String;

class RelativeKinematicsModule : public Module
{
	using Module::Module;
	private:
		ScenarioHandler scenarioHandler;
		virtual void onInitMessage(Empty::ConstSharedPtr) override;
		virtual void onConnectMessage(Empty::ConstSharedPtr) override;
		virtual void onArmMessage(Empty::ConstSharedPtr) override;
        virtual void onStartMessage(Empty::ConstSharedPtr) override;
		virtual void onDisconnectMessage(Empty::ConstSharedPtr) override;
		virtual void onStopMessage(Empty::ConstSharedPtr) override;
		virtual void onAbortMessage(Empty::ConstSharedPtr) override;
		virtual void onAllClearMessage(Empty::ConstSharedPtr) override;
		virtual void onACCMMessage(Empty::ConstSharedPtr) override;
		virtual void onEXACMessage(Empty::ConstSharedPtr) override;
};
