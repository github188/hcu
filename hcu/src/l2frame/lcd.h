/*
 * lcd.h
 *
 *  Created on: 2015年11月25日
 *      Author: test
 */

#ifndef L2FRAME_LCD_H_
#define L2FRAME_LCD_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_LCD
{
	FSM_STATE_LCD_INITED = 0x02,
	FSM_STATE_LCD_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_LCD_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmLcd[];

//API
extern OPSTAT fsm_lcd_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_lcd_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_lcd_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);

//Local API
OPSTAT func_lcd_int_init(void);

#endif /* L2FRAME_LCD_H_ */
