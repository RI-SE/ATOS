#ifndef ISO22133PROTOCOLDATA_H
#define ISO22133PROTOCOLDATA_H

#include "protocoldata.h"

class ISO22133ProtocolData : public ProtocolData {
public:
	ISO22133ProtocolData() {}
	~ISO22133ProtocolData() override;
	ReturnCode decodeFrom(const std::vector<char>& rawData) override;
};

#endif // PROTOCOLDATA_H
