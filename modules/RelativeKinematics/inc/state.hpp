#ifndef STATE_HPP
#define STATE_HPP

#include <stdexcept>

class ObjectHandler;

namespace RelativeKinematics {

class AbstractState {
public:
	//! The below transitions represent user commands and can be expected
	//! any time - thus any inheriting class must handle them all nicely
	virtual void initializeRequest() = 0;
	virtual void disconnectRequest() = 0;
	virtual void connectRequest() = 0;
	virtual void armRequest() = 0;
	virtual void disarmRequest() = 0;
	virtual void startRequest() = 0;
	virtual void stopRequest() = 0;
	virtual void abortRequest() = 0;
	virtual void allClearRequest() = 0;

	//! The below transitions represent spontaneous actions uninitiated by
	//! the user - inheriting classes may throw exceptions if transitions
	//! are deemed unreasonable
	virtual void initializationError() = 0;
	virtual void connectedToObject() = 0;
	virtual void disconnectedFromObject() = 0;
	virtual void connectedToLiveObject() = 0;
	virtual void connectedToArmedObject() = 0;
	virtual void allObjectsDisarmed() = 0;
	virtual void allObjectsConnected() = 0;
	virtual void testCompleted() = 0;
	virtual void postProcessingCompleted() = 0;

	virtual ~AbstractState();
protected:
	void setState(ObjectHandler &handler, AbstractState *st);
};

class Idle : public AbstractState {
public:
	Idle();
	//! Handle initialization requests
	void initializeRequest();

	//! Ignore other commands
	void disconnectRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! All spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject() { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Initialized : public AbstractState {
public:
	//! Handle connect/disconnect requests
	void disconnectRequest();
	void connectRequest();
	void initializationError();

	//! Ignore other commands
	void initializeRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject() { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Connecting : public AbstractState {
public:
	//! Handle only connect/disconnect requests
	void disconnectRequest();
	void connectRequest();
	void connectedToObject();
	void disconnectedFromObject();
	void connectedToLiveObject();
	void connectedToArmedObject();
	void allObjectsConnected();

	//! Ignore other commands
	void initializeRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Ready : public AbstractState {
public:
	//! Handle arm and disconnect requests
	void armRequest();
	void disconnectRequest();
	void disconnectedFromObject();

	//! Ignore other commands
	void initializeRequest() {}
	void connectRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Aborting : public AbstractState {
public:
	//! Only handle all clear signal
	void allClearRequest();
	void connectedToObject();
	void disconnectedFromObject();
	void connectedToLiveObject();
	void connectedToArmedObject();

	//! Ignore other commands
	void initializeRequest() {}
	void disconnectRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class TestLive : public AbstractState {
public:
	//! Only stop/abort allowed in live state
	void stopRequest();
	void abortRequest();
	void testCompleted();
	void disconnectedFromObject();

	//! Ignore other commands
	void initializeRequest() {}
	void disconnectRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Disarming : public AbstractState {
public:
	//! Only allow disconnect command
	void disconnectRequest();
	void connectedToObject();
	void disconnectedFromObject();
	void connectedToArmedObject();
	void connectedToLiveObject();
	void allObjectsDisarmed();

	//! Ignore other commands
	void initializeRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Armed : public AbstractState {
public:
	//! Only allow start/disarm
	void startRequest();
	void disarmRequest();
	void disconnectedFromObject();

	//! Ignore other commands
	void initializeRequest() {}
	void disconnectRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
	void postProcessingCompleted() { throw std::runtime_error("Unexpected postprocessing completion"); }
};

class Done : public AbstractState {
public:
	//! Completing postprocessing allows exiting this state
	void postProcessingCompleted();

	//! Ignore other commands
	void initializeRequest() {}
	void disconnectRequest() {}
	void connectRequest() {}
	void armRequest() {}
	void disarmRequest() {}
	void startRequest() {}
	void stopRequest() {}
	void abortRequest() {}
	void allClearRequest() {}

	//! Other spontaneous events unexpected
	void initializationError() { throw std::runtime_error("Unexpected initialization error"); }
	void connectedToObject() { throw std::runtime_error("Unexpected connection"); }
	void disconnectedFromObject() { throw std::runtime_error("Unexpected disconnection"); }
	void connectedToLiveObject() { throw std::runtime_error("Unexpected connection to live object"); }
	void connectedToArmedObject() { throw std::runtime_error("Unexpected connection to armed object"); }
	void allObjectsDisarmed() { throw std::runtime_error("Unexpected all objects disarmed"); }
	void allObjectsConnected() { throw std::runtime_error("Unexpected all objects connected"); }
	void testCompleted() { throw std::runtime_error("Unexpected test completion"); }
};

}

#endif
