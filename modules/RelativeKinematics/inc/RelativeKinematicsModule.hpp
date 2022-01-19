//base class
#include "module.hpp"
//
#include "scenariohandler.hpp"
// roscpp
#include "ros/ros.h"
// messages
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

using std_msgs::Empty;
using std_msgs::String;
class RelativeKinematicsModule : public Module
{
	using Module::Module;
	private:
		ScenarioHandler scenarioHandler;
		virtual void onInitMessage(Empty::ConstPtr) override;
		virtual void onConnectMessage(Empty::ConstPtr) override;
		virtual void onArmMessage(Empty::ConstPtr) override;
        virtual void onStartMessage(Empty::ConstPtr) override;
		virtual void onDisconnectMessage(Empty::ConstPtr) override;
		virtual void onStopMessage(Empty::ConstPtr) override;
		virtual void onAbortMessage(Empty::ConstPtr) override;
		virtual void onAllClearMessage(Empty::ConstPtr) override;
		virtual void onACCMMessage(Empty::ConstPtr) override;
		virtual void onEXACMessage(Empty::ConstPtr) override;
};
