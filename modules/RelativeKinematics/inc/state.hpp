#pragma once

#include "objecthandler.hpp"
#include <stdexcept>

class ObjectHandler;

class ObjectControlState {
public:
	//! The below transitions represent user commands and can be expected
	//! any time - thus any inheriting class must handle them all nicely
	virtual void initializeRequest(ObjectHandler&) = 0;
	virtual void disconnectRequest(ObjectHandler&) = 0;
	virtual void connectRequest(ObjectHandler&) = 0;
	virtual void armRequest(ObjectHandler&) = 0;
	virtual void disarmRequest(ObjectHandler&) = 0;
	virtual void startRequest(ObjectHandler&) = 0;
	virtual void stopRequest(ObjectHandler&) = 0;
	virtual void abortRequest(ObjectHandler&) = 0;
	virtual void allClearRequest(ObjectHandler&) = 0;

	//! The below transitions represent spontaneous actions uninitiated by
	//! the user - inheriting classes may throw exceptions if transitions
	//! are deemed unreasonable
	virtual void connectedToObject(ObjectHandler&) = 0;
	virtual void disconnectedFromObject(ObjectHandler&) = 0;
	virtual void connectedToLiveObject(ObjectHandler&) = 0;
	virtual void connectedToArmedObject(ObjectHandler&) = 0;
	virtual void allObjectsDisarmed(ObjectHandler&) = 0;
	virtual void allObjectsConnected(ObjectHandler&) = 0;
	virtual void testCompleted(ObjectHandler&) = 0;
	virtual void postProcessingCompleted(ObjectHandler&) = 0;

	virtual ~ObjectControlState();
protected:

	void setState(ObjectHandler& handler, ObjectControlState *st);
};

namespace RelativeKinematics {

class Idle : public ObjectControlState {
public:
	Idle();
	//! Handle initialization requests
	void initializeRequest(ObjectHandler&);

	//! Ignore other commands
	void disconnectRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! All spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ObjectHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Initialized : public ObjectControlState {
public:
	Initialized();
	//! Handle connect/disconnect requests
	void disconnectRequest(ObjectHandler&);
	void connectRequest(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ObjectHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Connecting : public ObjectControlState {
public:
	Connecting();
	//! Handle only connect/disconnect requests
	void disconnectRequest(ObjectHandler&);
	void connectRequest(ObjectHandler&);
	void connectedToObject(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);
	void connectedToLiveObject(ObjectHandler&);
	void connectedToArmedObject(ObjectHandler&);
	void allObjectsConnected(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Ready : public ObjectControlState {
public:
	Ready();
	//! Handle arm and disconnect requests
	void armRequest(ObjectHandler&);
	void disconnectRequest(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Aborting : public ObjectControlState {
public:
	Aborting();
	//! Only handle all clear signal
	void allClearRequest(ObjectHandler&);
	void connectedToObject(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);
	void connectedToLiveObject(ObjectHandler&);
	void connectedToArmedObject(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void disconnectRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class TestLive : public ObjectControlState {
public:
	TestLive();
	//! Only stop/abort allowed in live state
	void stopRequest(ObjectHandler&);
	void abortRequest(ObjectHandler&);
	void testCompleted(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void disconnectRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Disarming : public ObjectControlState {
public:
	Disarming();
	//! Only allow disconnect command
	void disconnectRequest(ObjectHandler&);
	void connectedToObject(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);
	void connectedToArmedObject(ObjectHandler&);
	void connectedToLiveObject(ObjectHandler&);
	void allObjectsDisarmed(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Armed : public ObjectControlState {
public:
	Armed();
	//! Only allow start/disarm
	void startRequest(ObjectHandler&);
	void disarmRequest(ObjectHandler&);
	void disconnectedFromObject(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void disconnectRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Done : public ObjectControlState {
public:
	Done();
	//! Completing postprocessing allows exiting this state
	void postProcessingCompleted(ObjectHandler&);

	//! Ignore other commands
	void initializeRequest(ObjectHandler&) {}
	void disconnectRequest(ObjectHandler&) {}
	void connectRequest(ObjectHandler&) {}
	void armRequest(ObjectHandler&) {}
	void disarmRequest(ObjectHandler&) {}
	void startRequest(ObjectHandler&) {}
	void stopRequest(ObjectHandler&) {}
	void abortRequest(ObjectHandler&) {}
	void allClearRequest(ObjectHandler&) {}

	//! Other spontaneous events unexpected
	void connectedToObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject(ObjectHandler&) { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject(ObjectHandler&) { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed(ObjectHandler&) { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected(ObjectHandler&) { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted(ObjectHandler&) { throw std::runtime_error("Unexpected test completion"); }
};

}

