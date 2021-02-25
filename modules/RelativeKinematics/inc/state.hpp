#pragma once

#include "objecthandler.hpp"
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
	virtual void connectedToObject(ScenarioHandler&) = 0;
	virtual void disconnectedFromObject(ScenarioHandler&) = 0;
	virtual void connectedToLiveObject(ScenarioHandler&) = 0;
	virtual void connectedToArmedObject(ScenarioHandler&) = 0;
	virtual void allObjectsDisarmed(ScenarioHandler&) = 0;
	virtual void allObjectsConnected(ScenarioHandler&) = 0;
	virtual void testCompleted(ScenarioHandler&) = 0;
	virtual void postProcessingCompleted(ScenarioHandler&) = 0;

	virtual ~ObjectControlState();
protected:

	void setState(ScenarioHandler& handler, ObjectControlState *st);
};

namespace RelativeKinematics {

class Idle : public ObjectControlState {
public:
	Idle();
	//! Handle initialization requests
	void initializeRequest(ScenarioHandler&);

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
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Initialized : public ObjectControlState {
public:
	Initialized();
	//! Handle connect/disconnect requests
	void disconnectRequest(ScenarioHandler&);
	void connectRequest(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Connecting : public ObjectControlState {
public:
	Connecting();
	//! Handle only connect/disconnect requests
	void disconnectRequest(ScenarioHandler&);
	void connectRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);
	void connectedToLiveObject(ScenarioHandler&);
	void connectedToArmedObject(ScenarioHandler&);
	void allObjectsConnected(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Ready : public ObjectControlState {
public:
	Ready();
	//! Handle arm and disconnect requests
	void armRequest(ScenarioHandler&);
	void disconnectRequest(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Aborting : public ObjectControlState {
public:
	Aborting();
	//! Only handle all clear signal
	void allClearRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);
	void connectedToLiveObject(ScenarioHandler&);
	void connectedToArmedObject(ScenarioHandler&);

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
};

class TestLive : public ObjectControlState {
public:
	TestLive();
	//! Only stop/abort allowed in live state
	void stopRequest(ScenarioHandler&);
	void abortRequest(ScenarioHandler&);
	void testCompleted(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void disarmRequest(ScenarioHandler&) {}
	void startRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Disarming : public ObjectControlState {
public:
	Disarming();
	//! Only allow disconnect command
	void disconnectRequest(ScenarioHandler&);
	void connectedToObject(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);
	void connectedToArmedObject(ScenarioHandler&);
	void connectedToLiveObject(ScenarioHandler&);
	void allObjectsDisarmed(ScenarioHandler&);

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
};

class Armed : public ObjectControlState {
public:
	Armed();
	//! Only allow start/disarm
	void startRequest(ScenarioHandler&);
	void disarmRequest(ScenarioHandler&);
	void disconnectedFromObject(ScenarioHandler&);

	//! Ignore other commands
	void initializeRequest(ScenarioHandler&) {}
	void disconnectRequest(ScenarioHandler&) {}
	void connectRequest(ScenarioHandler&) {}
	void armRequest(ScenarioHandler&) {}
	void stopRequest(ScenarioHandler&) {}
	void abortRequest(ScenarioHandler&) {}
	void allClearRequest(ScenarioHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Done : public ObjectControlState {
public:
	Done();
	//! Completing postprocessing allows exiting this state
	void postProcessingCompleted(ScenarioHandler&);

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
	void connectedToObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ScenarioHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ScenarioHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ScenarioHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ScenarioHandler&) { throw std::runtime_error("Unexpected test completion"); }
};

}

