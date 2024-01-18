#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "header.h"
#include "footer.h"
#include <stdint.h>

#pragma pack(push, 1)

/*! OSTM message */
typedef struct {
	HeaderType header;
	uint16_t stateValueID;
	uint16_t stateContentLength;
	uint8_t state;
	FooterType footer;
} OSTMType;						//16 bytes

#pragma pack(pop)

//! OSTM value IDs
#define VALUE_ID_OSTM_STATE_CHANGE_REQUEST 0x0064
#ifdef __cplusplus
}
#endif