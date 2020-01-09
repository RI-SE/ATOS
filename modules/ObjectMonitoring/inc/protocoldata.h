#ifndef PROTOCOLDATA_H
#define PROTOCOLDATA_H

#include <pthread.h>
#include <vector>
#include <functional>
#include <netinet/in.h>
#include <unordered_map>
#include <memory>

class ProtocolData {
public:
	ProtocolData() = default;
	ProtocolData(const ProtocolData&) = default;
	ProtocolData(ProtocolData&&) = default;
	ProtocolData& operator=(const ProtocolData&) = default;
	ProtocolData& operator=(ProtocolData&&) = default;
	virtual ~ProtocolData();

	typedef enum {
		DECODE_PARTIAL,
		DECODE_SUCCESSFUL,
		DECODE_ERROR
	} ReturnCode;

	virtual ReturnCode decodeFrom(const std::vector<char> &rawData) = 0;

private:

};

#endif // PROTOCOLDATA_H
