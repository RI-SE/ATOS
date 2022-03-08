#pragma once

#include "objectcontrol.hpp"
#include "util.h"
#include "type.h"
#include <stdexcept>
#include <rclcpp/logging.hpp>

class ObjectControl;

class ObjectControlState {
public:
	//! The below transitions represent user commands and can be expected
	//! any time - thus any inheriting class must handle them all nicely
	virtual void initializeRequest(ObjectControl&) = 0;
	virtual void disconnectRequest(ObjectControl&) = 0;
	virtual void connectRequest(ObjectControl&) = 0;
	virtual void armRequest(ObjectControl&) = 0;
	virtual void disarmRequest(ObjectControl&) = 0;
	virtual void startRequest(ObjectControl&) = 0;
	virtual void stopRequest(ObjectControl&) = 0;
	virtual void abortRequest(ObjectControl&) = 0;
	virtual void allClearRequest(ObjectControl&) = 0;

	//! The below transitions represent spontaneous actions uninitiated by
	//! the user - inheriting classes may throw exceptions if transitions
	//! are deemed unreasonable
	virtual void connectedToObject(ObjectControl&, uint32_t) { throw std::runtime_error("Unexpected connection in state " + type(*this)); }
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) { throw std::runtime_error("Unexpected disconnection in state " + type(*this)); }
	virtual void connectedToLiveObject(ObjectControl&, uint32_t) { throw std::runtime_error("Unexpected connection to live object in state " + type(*this)); }
	virtual void connectedToArmedObject(ObjectControl&, uint32_t) { throw std::runtime_error("Unexpected connection to armed object in state " + type(*this)); }
	virtual void allObjectsDisarmed(ObjectControl&) { throw std::runtime_error("Unexpected all objects disarmed in state " + type(*this)); }
	virtual void allObjectsAbortDisarmed(ObjectControl&) { throw std::runtime_error("Unexpected all objects disarmed in state " + type(*this)); }
	virtual void allObjectsConnected(ObjectControl&) { throw std::runtime_error("Unexpected all objects connected in state " + type(*this)); }
	virtual void testCompleted(ObjectControl&) { throw std::runtime_error("Unexpected test completion in state " + type(*this)); }
	virtual void objectDisarmed(ObjectControl&,uint32_t) { throw std::runtime_error("Unexpected object disarmed in state " + type(*this)); }
	virtual void objectArmed(ObjectControl&,uint32_t) { throw std::runtime_error("Unexpected object armed in state " + type(*this)); }
	virtual void objectAborting(ObjectControl&,uint32_t) { throw std::runtime_error("Unexpected object aborting in state " + type(*this)); }
	virtual void objectAbortDisarmed(ObjectControl&,uint32_t) { throw std::runtime_error("Unexpected object abort disarmed in state " + type(*this)); }
	virtual void postProcessingCompleted(ObjectControl&) { throw std::runtime_error("Unexpected postprocessing completion in state " + type(*this)); }
	virtual void settingModificationRequested(ObjectControl&) { throw std::runtime_error("Unexpected setting modification in state " + type(*this)); }
	virtual void actionExecutionRequested(ObjectControl&) { throw std::runtime_error("Unexpected action execution in state " + type(*this)); }

	//! Enter/exit functionality - defaults to nothing
	virtual void onEnter(ObjectControl&) {}
	virtual void onExit(ObjectControl&) {}

	virtual OBCState_t asNumber() const { return OBC_STATE_UNDEFINED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_ABORT; }

	ObjectControlState(){}
	virtual ~ObjectControlState() {}
protected:
	void setState(ObjectControl& handler, ObjectControlState *st);
	void controlModeInitialization(ObjectControl& handler);
};

namespace AbstractKinematics {

class Idle : public ObjectControlState {
public:
	Idle();
	//! Handle initialization requests
	virtual void initializeRequest(ObjectControl&) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void disconnectRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Allow modifications of the settings to occur
	void settingModificationRequested(ObjectControl&) override {}
	//! All other spontaneous events unexpected
	//

	OBCState_t asNumber() const override { return OBC_STATE_IDLE; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Initialized : public ObjectControlState {
public:
	Initialized();
	//! Handle connect/disconnect requests
	virtual void disconnectRequest(ObjectControl&) override;
	virtual void connectRequest(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Allow modifications of the settings to occur
	void settingModificationRequested(ObjectControl&) override {}
	//! Other spontaneous events unexpected
	//

	OBCState_t asNumber() const override { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Connecting : public ObjectControlState {
public:
	Connecting();
	//! Handle only connect/disconnect and abort requests
	virtual void disconnectRequest(ObjectControl&) override;
	virtual void connectRequest(ObjectControl&) override;
	virtual void abortRequest(ObjectControl&) override;
	virtual void connectedToObject(ObjectControl&, uint32_t) override;
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) override;
	virtual void connectedToLiveObject(ObjectControl&, uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;
	virtual void objectArmed(ObjectControl&,uint32_t) override;
	virtual void connectedToArmedObject(ObjectControl&, uint32_t) override;
	virtual void allObjectsConnected(ObjectControl&) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Other spontaneous events unexpected
	// TODO perhaps settings may be modified here?

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Ready : public ObjectControlState {
public:
	Ready();
	//! Handle arm and disconnect requests
	virtual void armRequest(ObjectControl&) override;
	virtual void disconnectRequest(ObjectControl&) override;
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;
	virtual void objectAbortDisarmed(ObjectControl&,uint32_t) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Allow modifications of the settings to occur
	void settingModificationRequested(ObjectControl&) override;
	//! Other spontaneous events unexpected
	//

	OBCState_t asNumber() const override { return OBC_STATE_CONNECTED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_READY; }
};

class Aborting : public ObjectControlState {
public:
	Aborting();
	//! Only handle all clear signal
	virtual void allClearRequest(ObjectControl&) override;
	virtual void connectedToObject(ObjectControl&, uint32_t) override;
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) override;
	virtual void connectedToLiveObject(ObjectControl&, uint32_t) override;
	virtual void connectedToArmedObject(ObjectControl&, uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;
	virtual void objectAbortDisarmed(ObjectControl&,uint32_t) override;
	virtual void allObjectsAbortDisarmed(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void disconnectRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}

	//! Handle strange object transitions
	void objectDisarmed(ObjectControl&, uint32_t) override;
	void objectArmed(ObjectControl&, uint32_t) override;

	virtual void onExit(ObjectControl&) override;

	//! Other spontaneous events unexpected
	//

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_ABORTING; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_ABORT; }
};

class TestLive : public ObjectControlState {
public:
	TestLive();
	//! Only stop/abort allowed in live state
	virtual void stopRequest(ObjectControl&) override;
	virtual void abortRequest(ObjectControl&) override;
	virtual void testCompleted(ObjectControl&) override;
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) override;
	virtual void objectDisarmed(ObjectControl&,uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void disconnectRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}
	void actionExecutionRequested(ObjectControl&) override {}

	//! Other spontaneous events unexpected
	//

	OBCState_t asNumber() const override { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; }
};

class Disarming : public ObjectControlState {
public:
	Disarming();
	//! Only allow disconnect command
	virtual void disconnectRequest(ObjectControl&) override;
	virtual void connectedToObject(ObjectControl&,uint32_t) override;
	virtual void disconnectedFromObject(ObjectControl&,uint32_t) override;
	virtual void connectedToArmedObject(ObjectControl&,uint32_t) override;
	virtual void connectedToLiveObject(ObjectControl&,uint32_t) override;
	virtual void allObjectsDisarmed(ObjectControl&) override;
	virtual void objectDisarmed(ObjectControl&,uint32_t) override;
	virtual void objectArmed(ObjectControl&,uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Other spontaneous events unexpected
	//

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Armed : public ObjectControlState {
public:
	Armed();
	//! Only allow start/disarm
	virtual void startRequest(ObjectControl&) override;
	virtual void disarmRequest(ObjectControl&) override;
	virtual void disconnectedFromObject(ObjectControl&, uint32_t) override;
	virtual void objectArmed(ObjectControl&,uint32_t) override;
	virtual void objectDisarmed(ObjectControl&,uint32_t) override;
	virtual void objectAborting(ObjectControl&,uint32_t) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void disconnectRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {}
	void allClearRequest(ObjectControl&) override {}

	//! Other spontaneous events unexpected
	//

	OBCState_t asNumber() const override { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Done : public ObjectControlState {
public:
	Done();
	//! Completing postprocessing allows exiting this state
	virtual void postProcessingCompleted(ObjectControl&) override;

	virtual void onEnter(ObjectControl&) override;

	//! Ignore other commands
	void initializeRequest(ObjectControl&) override {}
	void disconnectRequest(ObjectControl&) override {}
	void connectRequest(ObjectControl&) override {}
	void armRequest(ObjectControl&) override {}
	void disarmRequest(ObjectControl&) override {}
	void startRequest(ObjectControl&) override {}
	void stopRequest(ObjectControl&) override {}
	void abortRequest(ObjectControl&) override {} // safe?
	void allClearRequest(ObjectControl&) override {}

	//! Other spontaneous events unexpected
	//

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_TEST_DONE; }
};

}

namespace RelativeKinematics {
using Idle = AbstractKinematics::Idle;

class Initialized : public AbstractKinematics::Initialized {
	void disconnectRequest(ObjectControl&) override;
	void connectRequest(ObjectControl&) override;
};

class Connecting : public AbstractKinematics::Connecting {
	void disconnectRequest(ObjectControl&) override;
	void connectRequest(ObjectControl&) override;
	void abortRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void allObjectsConnected(ObjectControl&) override;
	void objectArmed(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
};

class Ready : public AbstractKinematics::Ready {
	void armRequest(ObjectControl&) override;
	void disconnectRequest(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
	void objectAbortDisarmed(ObjectControl&, uint32_t) override;
	void settingModificationRequested(ObjectControl&) override;
};

class Aborting : public AbstractKinematics::Aborting {
	void allClearRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
	void objectAbortDisarmed(ObjectControl&, uint32_t) override;
	void allObjectsAbortDisarmed(ObjectControl&) override;
};

class TestLive : public AbstractKinematics::TestLive {
	void stopRequest(ObjectControl&) override;
	void abortRequest(ObjectControl&) override;
	void testCompleted(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectDisarmed(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
};

class Disarming : public AbstractKinematics::Disarming {
	void disconnectRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void allObjectsDisarmed(ObjectControl&) override;
	void objectDisarmed(ObjectControl&,uint32_t) override;
	void objectArmed(ObjectControl&,uint32_t) override;
	void objectAborting(ObjectControl&,uint32_t) override;
};

class Armed : public AbstractKinematics::Armed {
	void startRequest(ObjectControl&) override;
	void disarmRequest(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectArmed(ObjectControl&,uint32_t) override;
	void objectDisarmed(ObjectControl&,uint32_t) override;
	void objectAborting(ObjectControl&,uint32_t) override;
};

class Done : public AbstractKinematics::Done {
	void postProcessingCompleted(ObjectControl&) override;
};

}

namespace AbsoluteKinematics {
using Idle = AbstractKinematics::Idle;

class Initialized : public AbstractKinematics::Initialized {
	void disconnectRequest(ObjectControl&) override;
	void connectRequest(ObjectControl&) override;
};

class Connecting : public AbstractKinematics::Connecting {
	void disconnectRequest(ObjectControl&) override;
	void connectRequest(ObjectControl&) override;
	void abortRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void allObjectsConnected(ObjectControl&) override;
	void objectArmed(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
};

class Ready : public AbstractKinematics::Ready {
	void armRequest(ObjectControl&) override;
	void disconnectRequest(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
	void objectAbortDisarmed(ObjectControl&, uint32_t) override;
	void settingModificationRequested(ObjectControl&) override;
};

class Aborting : public AbstractKinematics::Aborting {
	void allClearRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
	void objectAbortDisarmed(ObjectControl&, uint32_t) override;
	void allObjectsAbortDisarmed(ObjectControl&) override;
};

class TestLive : public AbstractKinematics::TestLive {
	void stopRequest(ObjectControl&) override;
	void abortRequest(ObjectControl&) override;
	void testCompleted(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectDisarmed(ObjectControl&, uint32_t) override;
	void objectAborting(ObjectControl&, uint32_t) override;
};


class Disarming : public AbstractKinematics::Disarming {
	void disconnectRequest(ObjectControl&) override;
	void connectedToObject(ObjectControl&, uint32_t) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void connectedToArmedObject(ObjectControl&, uint32_t) override;
	void connectedToLiveObject(ObjectControl&, uint32_t) override;
	void allObjectsDisarmed(ObjectControl&) override;
	void objectDisarmed(ObjectControl&,uint32_t) override;
	void objectArmed(ObjectControl&,uint32_t) override;
	void objectAborting(ObjectControl&,uint32_t) override;
};


class Armed : public AbstractKinematics::Armed {
	void startRequest(ObjectControl&) override;
	void disarmRequest(ObjectControl&) override;
	void disconnectedFromObject(ObjectControl&, uint32_t) override;
	void objectArmed(ObjectControl&,uint32_t) override;
	void objectDisarmed(ObjectControl&,uint32_t) override;
	void objectAborting(ObjectControl&,uint32_t) override;
};

class Done : public AbstractKinematics::Done {
	void postProcessingCompleted(ObjectControl&) override;
};
}
