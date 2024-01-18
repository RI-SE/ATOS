#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "header.h"
#include "footer.h"
#include <stdint.h>

#pragma pack(push, 1)

/*! DREQ message */
typedef struct {
	HeaderType header;
	FooterType footer;
} DREQType;						//20 bytes

#pragma pack(pop)

#ifdef __cplusplus
}
#endif