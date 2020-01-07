#ifndef PROTOCOLDATA_H
#define PROTOCOLDATA_H

#include <pthread.h>

class ProtocolData
{
public:
	ProtocolData();

	typedef enum {
		OK,
		DECODE_INCOMPLETE,
		DECODE_SUCCESSFUL,
		DECODE_ERROR
	} ReturnCode;

	virtual ReturnCode decodeFromData();

private:

};

#endif // PROTOCOLDATA_H
