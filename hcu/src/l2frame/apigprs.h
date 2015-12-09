/*
 * apigprs.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef L2FRAME_APIGPRS_H_
#define L2FRAME_APIGPRS_H_

#include "../comvm/vmlayer.h"
#include "sps485.h"

#define MAX_AT_COMMOND_REPLY	8


/* GPRS module checking MAX time */

typedef struct GprsPppdConf
{
	UINT8 gUseGprsOrNot;
	UINT32 GprsPppdWaitSec;
	UINT32 MaxGprsCheckTimes;
	UINT32 MaxGprsResetTimes;
	char HwResetPortOp1[16];
	UINT32 HwResetPortOp1DelayMsec;
	char HwResetPortOp2[16];
	UINT32 HwResetPortOp2DelayMsec;
	UINT8 SyncSysTimeFromGprs;
	char PppConfigFileName[64];
}GprsPppdConf_t;

/* GPRS module checking MAX time */
#define 	HCU_MAX_GPRS_CHECK_TIMES		10
#define 	HCU_MAX_GPRS_RESET_TIMES		3
#define 	HCU_GPRS_PPPD_WAIT_SEC			30

/* System Information */
SysInfo_t gSysInfo;
SerialPort_t gSerialPortGprs;
GprsPppdConf_t gGprsPppdConf;
/*
 * Function API prototype for Serial Port
 */
UINT32 GprsInit(SerialPort_t *pGprsSerialPortConfig, GprsPppdConf_t *gpc);
UINT32 GprsPppdStart(GprsPppdConf_t *gpc);
void GprsPppdSwShutdown();
void GprsHwReset();
UINT32 GprsPppdSystemStartup(GprsPppdConf_t *gpc);
UINT32 AtCommand(SerialPort_t *pGprsSerialPortConfig, char *AtCmd, char *ReplyStr, int *ReplyCnt);
void SetSystemTime(char *t);


#endif /* L2FRAME_APIGPRS_H_ */
