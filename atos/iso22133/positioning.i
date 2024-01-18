/* File : positioning.i */
%module positioning

%{
#include "positioning.h"
#define SWIG_PYTHON_STRICT_BYTE_CHAR
%}
typedef double double_t;
typedef long int ssize_t;

struct timeval {
long int tv_sec;
long int tv_usec;
};

%include "stdint.i"
%include "cpointer.i"
%include "positioning.h"
%pointer_functions(uint32_t, uint32ptr);

