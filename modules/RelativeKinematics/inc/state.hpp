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
	virtual void initializeRequest(ScenarioHandler&);

	virtual void onEnter(ScenarioHandler&);

	//! Ignore other commands
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! All spontaneous events unexpected
	void connectedToObject(ScenarioHandler&, uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const { return OBC_STATE_IDLE; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_INIT; }
};

class Initialized : public ObjectControlState {
public:
	Initialized();
	//! Handle connect/disconnect requests
	virtual void disconnectRequest(ScenarioHandler&);
	virtual void connectRequest(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_INIT; }
};

class Connecting : public ObjectControlState {
public:
	Connecting();
	//! Handle only connect/disconnect and abort requests
	virtual void disconnectRequest(ScenarioHandler&);
	virtual void connectRequest(ScenarioHandler&);
	virtual void abortRequest(ScenarioHandler&);
	virtual void connectedToObject(ScenarioHandler&,uint32_t);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);
	virtual void connectedToLiveObject(ScenarioHandler&,uint32_t);
	virtual void connectedToArmedObject(ScenarioHandler&,uint32_t);
	virtual void allObjectsConnected(ScenarioHandler&);

	virtual void onEnter(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }


	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const { return OBC_STATE_INITIALIZED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_INIT; }
};

class Ready : public ObjectControlState {
public:
	Ready();
	//! Handle arm and disconnect requests
	virtual void armRequest(ScenarioHandler&);
	virtual void disconnectRequest(ScenarioHandler&);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const { return OBC_STATE_CONNECTED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_READY; }
};

class Aborting : public ObjectControlState {
public:
	Aborting();
	//! Only handle all clear signal
	virtual void allClearRequest(ScenarioHandler&);
	virtual void connectedToObject(ScenarioHandler&,uint32_t);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);
	virtual void connectedToLiveObject(ScenarioHandler&,uint32_t);
	virtual void connectedToArmedObject(ScenarioHandler&,uint32_t);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const { return OBC_STATE_ERROR; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_ABORT; }
};

class TestLive : public ObjectControlState {
public:
	TestLive();
	//! Only stop/abort allowed in live state
	virtual void stopRequest(ScenarioHandler&);
	virtual void abortRequest(ScenarioHandler&);
	virtual void testCompleted(ScenarioHandler&);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_RUNNING; }
};

class Disarming : public ObjectControlState {
public:
	Disarming();
	//! Only allow disconnect command
	virtual void disconnectRequest(ScenarioHandler&);
	virtual void connectedToObject(ScenarioHandler&,uint32_t);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);
	virtual void connectedToArmedObject(ScenarioHandler&,uint32_t);
	virtual void connectedToLiveObject(ScenarioHandler&,uint32_t);
	virtual void allObjectsDisarmed(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Armed : public ObjectControlState {
public:
	Armed();
	//! Only allow start/disarm
	virtual void startRequest(ScenarioHandler&);
	virtual void disarmRequest(ScenarioHandler&);
	virtual void disconnectedFromObject(ScenarioHandler&,uint32_t);

	virtual void onEnter(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }

	OBCState_t asNumber() const { return OBC_STATE_ARMED; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_RUNNING; } // TODO
};

class Done : public ObjectControlState {
public:
	Done();
	//! Completing postprocessing allows exiting this state
	virtual void postProcessingCompleted(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&,uint32_t) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }

	// TODO integrate this state into the enum variable
	OBCState_t asNumber() const { return OBC_STATE_RUNNING; }
	virtual ControlCenterStatusType asControlCenterStatus() const { return CONTROL_CENTER_STATUS_TEST_DONE; }
};

}

namespace RelativeKinematics {
class Idle : public ObjectControl::Idle {
	void initializeRequest(ScenarioHandler&);
};

class Initialized : public ObjectControl::Initialized {
	void disconnectRequest(ScenarioHandler&);
	void connectRequest(ScenarioHandler&);
};

class Connecting : public ObjectControl::Connecting {
	void disconnectRequest(ScenarioHandler&);
	void connectRequest(ScenarioHandler&);
	void abortRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&,uint32_t);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
	void connectedToLiveObject(ScenarioHandler&,uint32_t);
	void connectedToArmedObject(ScenarioHandler&,uint32_t);
	void allObjectsConnected(ScenarioHandler&);
};

class Ready : public ObjectControl::Ready {
	void armRequest(ScenarioHandler&);
	void disconnectRequest(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
};

class Aborting : public ObjectControl::Aborting {
	void allClearRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&,uint32_t);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
	void connectedToLiveObject(ScenarioHandler&,uint32_t);
	void connectedToArmedObject(ScenarioHandler&,uint32_t);
};

class TestLive : public ObjectControl::TestLive {
	void stopRequest(ScenarioHandler&);
	void abortRequest(ScenarioHandler&);
	void testCompleted(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
};

class Disarming : public ObjectControl::Disarming {
	void disconnectRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&,uint32_t);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
	void connectedToArmedObject(ScenarioHandler&,uint32_t);
	void connectedToLiveObject(ScenarioHandler&,uint32_t);
	void allObjectsDisarmed(ScenarioHandler&);
};

class Armed : public ObjectControl::Armed {
	void startRequest(ScenarioHandler&);
	void disarmRequest(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&,uint32_t);
};

class Done : public ObjectControl::Done {
	void postProcessingCompleted(ScenarioHandler&);
};

}
