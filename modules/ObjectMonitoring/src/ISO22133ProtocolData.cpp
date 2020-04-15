#include "ISO22133ProtocolData.h"

ISO22133ProtocolData::~ISO22133ProtocolData() {

}

ProtocolData::ReturnCode ISO22133ProtocolData::decodeFrom(const std::vector<char> &rawData) {
	return DECODE_ERROR;
}
