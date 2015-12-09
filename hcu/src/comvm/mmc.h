/*
 * mmc.h
 *
 *  Created on: 2015年11月17日
 *      Author: test
 */

#ifndef COMVM_MMC_H_
#define COMVM_MMC_H_

#include "commsg.h"
#include "vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_MMC
{
	FSM_STATE_MMC_INITED = 0x02,
	FSM_STATE_MMC_ACTIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_MMC_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmMmc[];

//API
extern OPSTAT fsm_mmc_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_mmc_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_mmc_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);


#endif /* COMVM_MMC_H_ */
