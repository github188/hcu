/*
 * usbnet.h
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#ifndef L2FRAME_USBNET_H_
#define L2FRAME_USBNET_H_


#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"
#include "../l2frame/l2comdef.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_USBNET
{
	FSM_STATE_USBNET_INITED = 0x02,
	FSM_STATE_USBNET_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_USBNET_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmUsbnet[];

//API
extern OPSTAT fsm_usbnet_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_usbnet_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_usbnet_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT hcu_usbnet_phy_link_setup(void);
extern OPSTAT hcu_usbnet_phy_link_disconnect(void);

//Local API
OPSTAT func_usbnet_int_init(void);

//对外引用的API
extern OPSTAT hcu_usbnet_data_send(CloudDataSendBuf_t *buf);


#endif /* L2FRAME_USBNET_H_ */
