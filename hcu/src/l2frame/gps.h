/*
 * gps.h
 *
 *  Created on: 2015年11月23日
 *      Author: test
 */

#ifndef L2FRAME_GPS_H_
#define L2FRAME_GPS_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_GPS
{
	FSM_STATE_GPS_INITED = 0x02,
	FSM_STATE_GPS_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_GPS_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmGps[];

//API
extern OPSTAT fsm_gps_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_gps_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_gps_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);

//Local API
OPSTAT func_gps_int_init(void);



#endif /* L2FRAME_GPS_H_ */
