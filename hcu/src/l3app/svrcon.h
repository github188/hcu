/*
 * svrcon.h
 *
 *  Created on: 2015年11月12日
 *      Author: test
 */

#ifndef L3APP_SVRCON_H_
#define L3APP_SVRCON_H_

#include "../comvm/vmlayer.h"
#include "../hcu.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_SVRCON
{
	FSM_STATE_SVRCON_INITED = 0x02,
	FSM_STATE_SVRCON_WAIT_FOR_FB,
	FSM_STATE_SVRCON_ACTIVED,
	FSM_STATE_SVRCON_INIT_FB_FAILURE,
	FSM_STATE_SVRCON_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmSvrCon[];

typedef struct HcuSvrConTimer
{
	UINT32 timerId;
	UINT32 timerDuration;
	UINT8 status;
}HcuSvrConTimer_t;
#define SVRCON_TIMER_START_OFF 1
#define SVRCON_TIMER_START_ON 2
#define SVRCON_TIMER_START_RESTART 3
#define SVRCON_TIMER_START_STOP 3

#define SVRCON_TIMER_DURATION_INIT_FB 3  //in second

//structure
typedef struct HcuSvrConTaskInitInfo
{
	UINT8 active;
	UINT8 state;
}HcuSvrConTaskInitInfo_t;
#define SVRCON_TASK_INIT_INVALID 0xFF
#define SVRCON_TASK_INIT_SEND 0x01
#define SVRCON_TASK_INIT_WAIT_FOR_BACK 0x02
#define SVRCON_TASK_INIT_FEEDBACK 0x03
#define SVRCON_TASK_ACTIVE 1
#define SVRCON_TASK_DEACTIVE 2

//API
extern OPSTAT fsm_svrcon_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_svrcon_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_svrcon_init_feed_back(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_svrcon_init_time_out(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_svrcon_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
BOOL func_svrcon_init_caculate_all_fb(void);

#endif /* L3APP_SVRCON_H_ */
