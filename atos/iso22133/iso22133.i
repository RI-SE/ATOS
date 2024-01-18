/* File : iso22133.i */
%module iso22133

%{
#include "iso22133.h"
#include "positioning.h"
#include "defines.h"
#include "dreq.h"
#include "dres.h"
#include "footer.h"
#include "grem.h"
#include "header.h"
#include "iohelpers.h"
#include "monr.h"
#include "osem.h"
#include "ostm.h"
#include "timeconversions.h"
// #include "traj.h"
#define SWIG_PYTHON_STRICT_BYTE_CHAR
%}



#%javaconst(1);

typedef double double_t;
typedef long int ssize_t;

%include "typemaps.i"
%include "stdint.i"
%include "cpointer.i"
%include "iso22133.h"
%include "positioning.h"
%include "defines.h"
%include "dreq.h"
%include "dres.h"
%include "footer.h"
%include "grem.h"
%include "header.h"
%include "iohelpers.h"
%include "monr.h"
%include "osem.h"
%include "ostm.h"
%include "timeconversions.h"
// %include "traj.h"
