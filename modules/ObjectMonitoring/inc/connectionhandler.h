#ifndef CONNECTIONHANDLER_H
#define CONNECTIONHANDLER_H

#include <pthread.h>
#include "protocoldata.h"

class ConnectionHandler
{
public:
	ConnectionHandler(int openSocketDescriptor, ProtocolData& data);
	ConnectionHandler(int openSocketDescriptor, ProtocolData& data, unsigned long readBufferSize);
	~ConnectionHandler();

	bool isTerminated() const { return terminated; }
private:
	int socketDescriptor = 0;
	ProtocolData &data;
	unsigned long readBufferSize;
	pthread_t readThread;
	bool terminated = false;
	constexpr static unsigned long DefaultReadBufferSize = 4096;

	void* threadRoutine(void*);
	static void* routineWrapper(void* context) {
		return static_cast<ConnectionHandler *>(context)->threadRoutine(nullptr);
	}
	[[ noreturn ]] void terminate(void* retval);
};

#endif // CONNECTIONHANDLER_H
