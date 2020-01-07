#ifndef CONNECTIONHANDLER_H
#define CONNECTIONHANDLER_H

#include <pthread.h>
#include "protocoldata.h"

class ConnectionHandler
{
public:
	ConnectionHandler(int openSocketDescriptor);
	ConnectionHandler(int openSocketDescriptor, unsigned long readBufferSize);

private:
	int socketDescriptor = 0;
	unsigned long readBufferSize;
	pthread_t readThread;
	constexpr static unsigned long DefaultReadBufferSize = 4096;
	ProtocolData data;

	void* threadRoutine(void*);
	static void* routineWrapper(void* context) {
		return static_cast<ConnectionHandler *>(context)->threadRoutine(nullptr);
	}
};

#endif // CONNECTIONHANDLER_H
