/*
 * trace.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "trace.h"

#define FILE_NAME_LAN	18
/*********************************************************
** Local Declarations									**
*********************************************************/
const INT32  gpuTraceBufferSize = 1024;
static INT8   gpuTraceBuffer[1024];
static INT8  *gpuTraceBufferPtr;
//static INT8  *gpuTraceBufferPtrLimit = (gpuTraceBuffer+(gpuTraceBufferSize * 80 / 100));
static INT8  *gpuTraceBufferPtrLimit;

INT32  gpuTraceLineSize = 70; // 70 + \n
INT32  gpuTraceDumpLineSize = 16;
INT32  outputOnCrt = 1;

void TraceInit()
{
	gpuTraceBufferPtrLimit = (gpuTraceBuffer+(gpuTraceBufferSize * 80 / 100));
}

void TraceIdPrintf(char *pathname, UINT32 line)
{
	char timeBuf[50];

	GetCurTimeWithMs( timeBuf );

	/* init sprintf buffer pointer */
	gpuTraceBufferPtr = gpuTraceBuffer;

	sprintf( (char *)gpuTraceBuffer, "DBG : T: %s %-18.18s: %-4d: ", timeBuf,
		GetFileName((char*)pathname), line);

	/* move new begining pointer */
	gpuTraceBufferPtr += strlen((char *)gpuTraceBuffer);

}

void TracePrintf(char *format, ...)
{
	va_list marker;
	va_start( marker, format );

	/*
	** write stdout in gpu internal buffer
	*/
	vsprintf( (char *)gpuTraceBufferPtr, format, marker);

	/*
	** The trace is limited to 1000 characters.
	** If the string end is neither at 998 nor at 999 :
	*/
	if( (gpuTraceBuffer[998] != 0) &&
		(gpuTraceBuffer[999] != 0) )
	{
		gpuTraceBuffer[997] = '!';
		gpuTraceBuffer[998] = '\n';
		gpuTraceBuffer[999] = '\0';
	}

	if ( outputOnCrt )
	{
		printf("%s", gpuTraceBuffer);
	}

	gpuTraceBuffer[0] = '\0';

	va_end( marker );              /* Reset variable arguments.      */
}

void ErrorTracePrintf(char *format, ...)
{
	va_list marker;
	va_start( marker, format );

	/*
	** write stdout in gpu internal buffer
	*/
	vsprintf( (char *)gpuTraceBufferPtr, format, marker);

	/*
	** The trace is limited to 1000 characters.
	** If the string end is neither at 998 nor at 999 :
	*/
	if( (gpuTraceBuffer[998] != 0) &&
		(gpuTraceBuffer[999] != 0) )
	{
		gpuTraceBuffer[997] = '!';
		gpuTraceBuffer[998] = '\n';
		gpuTraceBuffer[999] = '\0';
	}

	//if ( outputOnCrt )
	//{
		printf("%s", gpuTraceBuffer);
	//}

#if 0 /* Print to File not implemented for the momnet */

	#ifdef MX_WIN32
		if(NULL != traceLogHandle)
		{
			fprintf( traceLogHandle, "%s", gpuTraceBuffer);
		}
		else
		{
			printf("Trace error, trace file not open.\n");
		}
	#else
		if ( 0 == traceShmHead )
		{
			printf("TracePrintf: Invalid shared memory info.\n");
			return;
		}
		else
		{
			AppendTraceToShm( traceShmHead, (char*)gpuTraceBuffer, strlen((char*)gpuTraceBuffer) );
			//int myLen = strlen( (char*)gpuTraceBuffer );

			//if( traceShmHead->tail + myLen > traceShmHead->head + TRACE_MEMORY_SIZE)
			//{
			//	printf( "Trace memory overflow\n" );
			//}
			//else
			//{
			//	memcpy( (void*)(traceShmHead->tail), gpuTraceBuffer, myLen);
			//	traceShmHead->tail += myLen;
			//}
		}
	#endif

#endif

	gpuTraceBuffer[0] = '\0';

	va_end( marker );              /* Reset variable arguments.      */
}

char *GetFileName( char *s)
{
	//char* c = s;	/* init iterator on beginning */

	int l = 0;

	l = strlen(s);

	if(l > FILE_NAME_LAN)
	{
		return s + (l - FILE_NAME_LAN);
	}
	else
	{
		return s;
	}

//	/* research the dot of xxx.c or xxx.cpp */
//	while ((*c != 0) && (*c != '.'))
//	{
//		c++;
//	}
//	if (*c == 0)
//	{
//		return s;	/* non recognized filename */
//	}
//
//	/* research the last slash or back slash or Z:\...\...\xxx.cpp */
//	while ((c != s) && (*c != '\\') && (*c != '/'))
//	{
//		c--;
//	}
//	if (c == s)
//	{
//		return s;	/* return full string */
//	}
//
//	return ++c;
}


void GetCurTimeWithMs( char* timeStr )
{
	time_t curSeconds;
	struct tm myTm;
	struct tm* myTmPtr = &myTm;
	INT32 curMilliSeconds;
	char dateBuf[16];
	char timeBuf[30];

	struct timeval tv;
	struct timezone tz;
	INT32 temp;
	gettimeofday(&tv, &tz);
	curSeconds = tv.tv_sec;
	curMilliSeconds = tv.tv_usec / 1000;
	temp = tv.tv_usec % 1000;

	if ( temp > 500 )
	{
		curMilliSeconds++;
	}

    myTmPtr = localtime ( &curSeconds );
	strftime( dateBuf, 16, "%x", myTmPtr );
	strftime( timeBuf, 30, "%X", myTmPtr );

	sprintf( timeStr, "%s %s:%-.3u:", dateBuf, timeBuf, curMilliSeconds);
}



