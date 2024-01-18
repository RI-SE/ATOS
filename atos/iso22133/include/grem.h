#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

#include "iso22133.h"
#include "header.h"
#include "footer.h"

#pragma pack(push, 1)
typedef struct{
    HeaderType header;
    uint16_t ReceivedHeaderTransmitterIDValueID;
    uint16_t ReceivedHeaderTransmitterIDContentLength;
    uint32_t ReceivedHeaderTransmitterID;

    uint16_t ReceivedHeaderMessageCounterValueID;
    uint16_t ReceivedHeaderMessageCounterContentLength;
    uint8_t ReceivedHeaderMessageCounter;

    uint16_t ReceivedHeaderMessageIDValueID;
    uint16_t ReceivedHeaderMessageIDContentLength;
    uint16_t ReceivedHeaderMessageID;

    uint16_t ResponseCodeValueID;
    uint16_t ResponseCodeContentLength;
    uint8_t ResponseCode;

    uint16_t PayloadLengthValueID;
    uint16_t PayloadLengthContentLength;
    uint16_t PayloadLength;

    uint16_t PayloadDataValueID;
    uint16_t PayloadDataContentLength;
    FooterType footer;
} GREMType;
#pragma pack(pop)

//! GREM value IDs

#define VALUE_ID_GREM_RECEIVED_HEADER_TRANSMITTER_ID    0x0200
#define VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_COUNTER	0x0201
#define VALUE_ID_GREM_RECEIVED_HEADER_MESSAGE_ID    	0x0202
#define VALUE_ID_GREM_RESPONSE_CODE	                0x0203
#define VALUE_ID_GREM_PAYLOAD_LENGTH                	0x0204
#define VALUE_ID_GREM_PAYLOAD_DATA  	                0x0205

enum ISOMessageReturnValue convertGREMoHostRepresentation(GREMType* GREMdata,
		GeneralResponseMessageType* gremData);


#ifdef __cplusplus
}
#endif