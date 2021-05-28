#ifndef STATE_H
#define STATE_H
#pragma once

#include "scenariohandler.hpp"
#include "util.h"
#include <stdexcept>

class ScenarioHandler;

class ObjectControlState {
public:
	//! The below transitions represent user commands and can be expected
	//! any time - thus any inheriting class must handle them all nicely
	virtual void initializeRequest(ScenarioHandler&) = 0;
	virtual void disconnectRequest(ScenarioHandler&) = 0;
	virtual void connectRequest(ScenarioHandler&) = 0;
	virtual void armRequest(ScenarioHandler&) = 0;
	virtual void disarmRequest(ScenarioHandler&) = 0;
	virtual void startRequest(ScenarioHandler&) = 0;
	virtual void stopRequest(ScenarioHandler&) = 0;
	virtual void abortRequest(ScenarioHandler&) = 0;
	virtual void allClearRequest(ScenarioHandler&) = 0;

	//! The below transitions represent spontaneous actions uninitiated by
	//! the user - inheriting classes may throw exceptions if transitions
	//! are deemed unreasonable
	virtual void connectedToObject(ScenarioHandler&,uint32_t) = 0;
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t) = 0;
	virtual void connectedToLiveObject(ScenarioHandler&,uint32_t) = 0;
	virtual void connectedToArmedObject(ScenarioHandler&,uint32_t) = 0;
	virtual void allObjectsDisarmed(ScenarioHandler&) = 0;
	virtual void allObjectsConnected(ScenarioHandler&) = 0;
	virtual void testCompleted(ScenarioHandler&) = 0;
	virtual void objectDisarmed(ScenarioHandler&,uint32_t) = 0;
	virtual void objectArmed(ScenarioHandler&,uint32_t) = 0;
	virtual void objectAborting(ScenarioHandler&,uint32_t) = 0;
	virtual void postProcessingCompleted(ScenarioHandler&) = 0;

	//! Enter/exit functionality - defaults to nothing
	virtual void onEnter(ScenarioHandler&) {}
	virtual void onExit(ScenarioHandler&) {}

	virtual OBCState_t asNumber() const { return OBC_STATE_UNDEFINED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_ABORT; }

	ObjectControlState(){}
	virtual ~ObjectControlState() {}
protected:
	void setState(ScenarioHandler& handler, ObjectControlState *st);
};

namespace ObjectControl {

class Idle : public ObjectControlState {
public:
	Idle();
	//! Handle initialization requests
	virtual void initializeRequest(ScenarioHandler&) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void disconnectRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! All spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void objectAborting(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object aborting"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const override { return OBC_STATE_IDLE; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Initialized : public ObjectControlState {
public:
	Initialized();
	//! Handle connect/disconnect requests
	virtual void disconnectRequest(ScenarioHandler&) override;
	virtual void connectRequest(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void objectAborting(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object aborting"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const override { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Connecting : public ObjectControlState {
public:
	Connecting();
	//! Handle only connect/disconnect and abort requests
	virtual void disconnectRequest(ScenarioHandler&) override;
	virtual void connectRequest(ScenarioHandler&) override;
	virtual void abortRequest(ScenarioHandler&) override;
	virtual void connectedToObject(ScenarioHandler&, uint32_t) override;
	virtual void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	virtual void connectedToLiveObject(ScenarioHandler&, uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;
	virtual void objectArmed(ScenarioHandler&,uint32_t) override;
	virtual void connectedToArmedObject(ScenarioHandler&, uint32_t) override;
	virtual void allObjectsConnected(ScenarioHandler&) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }


	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_INIT; }
};

class Ready : public ObjectControlState {
public:
	Ready();
	//! Handle arm and disconnect requests
	virtual void armRequest(ScenarioHandler&) override;
	virtual void disconnectRequest(ScenarioHandler&) override;
	virtual void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const override { return OBC_STATE_CONNECTED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_READY; }
};

class Aborting : public ObjectControlState {
public:
	Aborting();
	//! Only handle all clear signal
	virtual void allClearRequest(ScenarioHandler&) override;
	virtual void connectedToObject(ScenarioHandler&, uint32_t) override;
	virtual void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	virtual void connectedToLiveObject(ScenarioHandler&, uint32_t) override;
	virtual void connectedToArmedObject(ScenarioHandler&, uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void disconnectRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_ERROR; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_ABORT; }
};

class TestLive : public ObjectControlState {
public:
	TestLive();
	//! Only stop/abort allowed in live state
	virtual void stopRequest(ScenarioHandler&) override;
	virtual void abortRequest(ScenarioHandler&) override;
	virtual void testCompleted(ScenarioHandler&) override;
	virtual void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	virtual void objectDisarmed(ScenarioHandler&,uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void disconnectRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const override { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; }
};

class Disarming : public ObjectControlState {
public:
	Disarming();
	//! Only allow disconnect command
	virtual void disconnectRequest(ScenarioHandler&) override;
	virtual void connectedToObject(ScenarioHandler&,uint32_t) override;
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t) override;
	virtual void connectedToArmedObject(ScenarioHandler&,uint32_t) override;
	virtual void connectedToLiveObject(ScenarioHandler&,uint32_t) override;
	virtual void allObjectsDisarmed(ScenarioHandler&) override;
	virtual void objectDisarmed(ScenarioHandler&,uint32_t) override;
	virtual void objectArmed(ScenarioHandler&,uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Armed : public ObjectControlState {
public:
	Armed();
	//! Only allow start/disarm
	virtual void startRequest(ScenarioHandler&) override;
	virtual void disarmRequest(ScenarioHandler&) override;
	virtual void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	virtual void objectArmed(ScenarioHandler&,uint32_t) override;
	virtual void objectDisarmed(ScenarioHandler&,uint32_t) override;
	virtual void objectAborting(ScenarioHandler&,uint32_t) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void disconnectRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {}
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const override { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Done : public ObjectControlState {
public:
	Done();
	//! Completing postprocessing allows exiting this state
	virtual void postProcessingCompleted(ScenarioHandler&) override;

	virtual void onEnter(ScenarioHandler&) override;

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) override {}
	void disconnectRequest(ScenarioHandler&) override {}
	void connectRequest(ScenarioHandler&) override {}
	void armRequest(ScenarioHandler&) override {}
	void disarmRequest(ScenarioHandler&) override {}
	void startRequest(ScenarioHandler&) override {}
	void stopRequest(ScenarioHandler&) override {}
	void abortRequest(ScenarioHandler&) override {} // safe?
	void allClearRequest(ScenarioHandler&) override {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) override { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) override { throw std::runtime_error("Unexpected test completion"); }
	void objectDisarmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object disarmed"); }
	void objectArmed(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object armed"); }
	void objectAborting(ScenarioHandler&,uint32_t) override { throw std::runtime_error("Unexpected object aborting"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const override { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const override { return CONTROL_CENTER_STATUS_TEST_DONE; }
};

}

namespace RelativeKinematics {
class Idle : public ObjectControl::Idle {
	void initializeRequest(ScenarioHandler&) override;
};

class Initialized : public ObjectControl::Initialized {
	void disconnectRequest(ScenarioHandler&) override;
	void connectRequest(ScenarioHandler&) override;
};

class Connecting : public ObjectControl::Connecting {
	void disconnectRequest(ScenarioHandler&) override;
	void connectRequest(ScenarioHandler&) override;
	void abortRequest(ScenarioHandler&) override;
	void connectedToObject(ScenarioHandler&, uint32_t) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override;
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override;
	void allObjectsConnected(ScenarioHandler&) override;
	void objectArmed(ScenarioHandler&, uint32_t) override;
	void objectAborting(ScenarioHandler&, uint32_t) override;
};

class Ready : public ObjectControl::Ready {
	void armRequest(ScenarioHandler&) override;
	void disconnectRequest(ScenarioHandler&) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void objectAborting(ScenarioHandler&, uint32_t) override;
};

class Aborting : public ObjectControl::Aborting {
	void allClearRequest(ScenarioHandler&) override;
	void connectedToObject(ScenarioHandler&, uint32_t) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override;
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override;
	void objectAborting(ScenarioHandler&, uint32_t) override;
};

class TestLive : public ObjectControl::TestLive {
	void stopRequest(ScenarioHandler&) override;
	void abortRequest(ScenarioHandler&) override;
	void testCompleted(ScenarioHandler&) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void objectDisarmed(ScenarioHandler&, uint32_t) override;
	void objectAborting(ScenarioHandler&, uint32_t) override;
};

class Disarming : public ObjectControl::Disarming {
	void disconnectRequest(ScenarioHandler&) override;
	void connectedToObject(ScenarioHandler&, uint32_t) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void connectedToArmedObject(ScenarioHandler&, uint32_t) override;
	void connectedToLiveObject(ScenarioHandler&, uint32_t) override;
	void allObjectsDisarmed(ScenarioHandler&) override;
	void objectDisarmed(ScenarioHandler&,uint32_t) override;
	void objectArmed(ScenarioHandler&,uint32_t) override;
	void objectAborting(ScenarioHandler&,uint32_t) override;
};

class Armed : public ObjectControl::Armed {
	void startRequest(ScenarioHandler&) override;
	void disarmRequest(ScenarioHandler&) override;
	void disconnectedFromObject(ScenarioHandler&, uint32_t) override;
	void objectArmed(ScenarioHandler&,uint32_t) override;
	void objectDisarmed(ScenarioHandler&,uint32_t) override;
	void objectAborting(ScenarioHandler&,uint32_t) override;
};

class Done : public ObjectControl::Done {
	void postProcessingCompleted(ScenarioHandler&) override;
};

}
