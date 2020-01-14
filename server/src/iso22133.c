#include "iso22133.h"
static const uint8_t SupportedProtocolVersions[] = {2};
void getSupportedISOProtocolVersions(const uint8_t** supportedProtocolVersions, size_t* nProtocols) {
	*supportedProtocolVersions = SupportedProtocolVersions;
	*nProtocols = sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]);
}
