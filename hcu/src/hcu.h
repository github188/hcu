/*
 * hcu.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef HCU_H_
#define HCU_H_

#include "./comvm/vmlayer.h"

void  hcu_app_system_init();
UINT32 hcu_system_task_init(void);
UINT32 hcu_system_task_init_call(UINT32 task_id, FsmStateItem_t *p);

#define 	TRUE 		1
#define 	FALSE 		0

#endif /* HCU_H_ */
