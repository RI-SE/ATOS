#ifndef ISO22133PROTOCOLDATA_H
#define ISO22133PROTOCOLDATA_H

#include "protocoldata.h"

class ISO22133ProtocolData : public ProtocolData {
public:
	static constexpr uint16_t TCP_PORT = 12345;

	ISO22133ProtocolData() {}
	~ISO22133ProtocolData() override;
	ReturnCode decodeFrom(const std::vector<char>& rawData) override;
};

#endif // PROTOCOLDATA_H
