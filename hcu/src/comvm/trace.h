/*
 * trace.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef COMVM_TRACE_H_
#define COMVM_TRACE_H_

#include "vmlayer.h"

/*
 * Trace and Debug Interface
 */
extern void TraceIdPrintf(char * fileName, UINT32 lineNum);
extern void TracePrintf(char * format, ...);
extern void ErrorTracePrintf(char * format, ...);
extern void TraceInit();

char *GetFileName( char *s);
void GetCurTimeWithMs( char* );

#define HcuDebugPrint \
TraceIdPrintf(__FILE__, __LINE__), \
TracePrintf

#define HcuErrorPrint \
TraceIdPrintf(__FILE__, __LINE__), \
ErrorTracePrintf


#endif /* COMVM_TRACE_H_ */
