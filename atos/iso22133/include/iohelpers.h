#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stddef.h>

typedef void (*DebugPrinter_t)(const void*);
// ************************* Debug printout helper data **********************************************************
/*! Debug formatting functions*/
void printU8(const void* val);
void printU16(const void* val);
void printU32(const void* val);
void printU64(const void* val);
void printI8(const void* val);
void printI16(const void* val);
void printI32(const void* val);
void printI64(const void* val);
void printString(const void* val);

/*! Debug struct */
typedef struct {
	char* name;
	char* unit;
	DebugPrinter_t printer;
} DebugStrings_t;

int encodeContent(uint16_t valueID, const void* src, char** dest, const size_t contentSize, size_t* bufferSpace, DebugStrings_t *debugStruct, const char debug);

void printContent(const uint16_t valueID, const uint16_t contentLength, const void* value, DebugStrings_t* deb);
#ifdef __cplusplus
}
#endif