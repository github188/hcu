/*
 * sysinfo.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef COMVM_SYSINFO_H_
#define COMVM_SYSINFO_H_

#include "vmlayer.h"

extern SysInfo_t gSysInfo;

UINT32 HcuGetSysInfo(SysInfo_t *si);
UINT32 IsGprsPppdOk();
UINT32 IsNetItfExist(char *itfname);
void KillPppdProcess();


#endif /* COMVM_SYSINFO_H_ */
