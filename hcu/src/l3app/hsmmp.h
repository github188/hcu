/*
 * hsmmp.h
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#ifndef L3APP_HSMMP_H_
#define L3APP_HSMMP_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_HSMMP
{
	FSM_STATE_HSMMP_INITED = 0x02,
	FSM_STATE_HSMMP_ACTIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_HSMMP_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmHsmmp[];

//API
extern OPSTAT fsm_hsmmp_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_time_out(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_audio_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_camera_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hsmmp_lcd_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);

//引用外部函数
extern OPSTAT hcu_save_to_storage_disc(UINT32 fId, void *dataBuffer, UINT32 dataLen);
extern OPSTAT hcu_read_from_storage_disc(UINT32 fId, void *dataBuffer, UINT32 dataLen);

#endif /* L3APP_HSMMP_H_ */
