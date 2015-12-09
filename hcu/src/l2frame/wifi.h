/*
 * apiwifi.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef L2FRAME_WIFI_H_
#define L2FRAME_WIFI_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"
#include "../l2frame/l2comdef.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_WIFI
{
	FSM_STATE_WIFI_INITED = 0x02,
	FSM_STATE_WIFI_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_WIFI_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmWifi[];

//API
extern OPSTAT fsm_wifi_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_wifi_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_wifi_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT hcu_wifi_phy_link_setup(void);
extern OPSTAT hcu_wifi_phy_link_disconnect(void);

//Local API
OPSTAT func_wifi_int_init(void);

//对外引用的API
extern OPSTAT hcu_wifi_data_send(CloudDataSendBuf_t *buf);

#endif /* L2FRAME_WIFI_H_ */
