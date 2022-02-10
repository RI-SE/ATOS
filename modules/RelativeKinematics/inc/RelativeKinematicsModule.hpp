#include "module.hpp"
#include "scenariohandler.hpp"

#define MODULE_NAME "RelativeKinematics"

class RelativeKinematicsModule : public Module
{
private:
	ScenarioHandler scenarioHandler;
	virtual void onInitMessage(const Empty::SharedPtr) override;
	virtual void onConnectMessage(Empty::SharedPtr) override;
	virtual void onArmMessage(Empty::SharedPtr) override;
	virtual void onStartMessage(Empty::SharedPtr) override;
	virtual void onDisconnectMessage(Empty::SharedPtr) override;
	virtual void onStopMessage(Empty::SharedPtr) override;
	virtual void onAbortMessage(Empty::SharedPtr) override;
	virtual void onAllClearMessage(Empty::SharedPtr) override;
	virtual void onACCMMessage(Accm::SharedPtr) override;
	virtual void onEXACMessage(Exac::SharedPtr) override;
	virtual void onGetStatusMessage(Empty::SharedPtr) override;

	public: 
	RelativeKinematicsModule(std::string name) : Module(name){
		
		int queueSize=0;
		// ** Subscriptions
		this->initSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onInitMessage, this, _1));
		this->connectSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onConnectMessage, this, _1));
		this->armSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onArmMessage, this, _1));
		this->startSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onStartMessage, this, _1));
		this->disconnectSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onDisconnectMessage, this, _1));
		this->stopSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onStopMessage, this, _1));
		this->abortSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onAbortMessage, this, _1));
		this->allClearSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onAllClearMessage, this, _1));
		this->accmSub = this->create_subscription<Accm>(name, queueSize, std::bind(&RelativeKinematicsModule::onACCMMessage, this, _1));
		this->exacSub = this->create_subscription<Exac>(name, queueSize, std::bind(&RelativeKinematicsModule::onEXACMessage, this, _1));
		this->getStatusSub = this->create_subscription<Empty>(name, queueSize, std::bind(&RelativeKinematicsModule::onGetStatusMessage, this, _1));

		// ** Publishers
		this->failurePub = this->create_publisher<UInt8>(topicNames[COMM_FAILURE],queueSize);
		this->abortPub = this->create_publisher<Empty>(topicNames[COMM_ABORT],queueSize);
		this->getStatusResponsePub = this->create_publisher<String>(topicNames[COMM_GETSTATUS_OK],queueSize);
		
	};
};
