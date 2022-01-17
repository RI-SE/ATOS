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
		void initCB(const Empty&) override;
		void connectCB(const Empty&) override;
		void armCB(const Empty&) override;
        void startCB(const Empty&) override;

	public:
		RelativeKinematicsModule(const RelativeKinematicsModule& rtm) : Module(rtm.name) { }; // explicit copy ctor
		//Topic<Empty, RelativeKinematicsModule> invTopic = Topic<Empty, RelativeKinematicsModule> ("/inv",1000,&RelativeKinematicsModule::invCB,this);
		//Topic<Empty, RelativeKinematicsModule> startTopic = Topic<Empty, RelativeKinematicsModule>("/start",1000,&RelativeKinematicsModule::startCB,this);
};
