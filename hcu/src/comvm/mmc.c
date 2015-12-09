/*
 * mmc.c
 *
 *  Created on: 2015年11月17日
 *      Author: test
 */

#include "mmc.h"
#include "../comvm/trace.h"

/*
** FSM of the MMC
*/
FsmStateItem_t FsmMmc[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_mmc_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_mmc_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_mmc_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_MMC_ACTIVED,            		fsm_mmc_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Global variables


//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_mmc_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_MMC, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("MMC: Error Set FSM State at fsm_timer_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_mmc_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to HCU-MAIN
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_MMC, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("MMC: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MMC], zHcuTaskNameList[TASK_ID_HCUMAIN]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_MMC, FSM_STATE_MMC_INITED) == FAILURE){
		HcuErrorPrint("MMC: Error Set FSM State!\n");
		return FAILURE;
	}

	//INIT this task global variables

	//进入等待反馈状态
	if (FsmSetState(TASK_ID_MMC, FSM_STATE_MMC_ACTIVED) == FAILURE){
		HcuErrorPrint("MMC: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("MMC: Enter FSM_STATE_MMC_ACTIVED status, Keeping loop of signal sending here!\n");
	#endif //(TRACE_DEBUG_ON)

	return SUCCESS;
}

OPSTAT fsm_mmc_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

/*
 *   设想中的MMC人机命令模块
 *   1. 改变MSG_TRACE_ON的不同设置，可以使得消息TRACE动态可调整
 *   2. 全局变量都可以查阅，也可以修改。其中最重要的是HWINV中硬件信息，以及系统的配置信息
 *   3. 可以发送RESTART命令给任何模块，前提是所有模块必须实现RESTART这个消息接口。当然，实现的方式是消息发送
 *   其它的功能，待定
 *
 *   系统设计思路
 *   - HCU系统数据结构主要有以下几张表构成
 *   > 状态机任务表，掌控所有状态机转移和任务的列表构成，由VM控制
 *   > 任务列表，包括任务，线程，队列等信息，由VM控制
 *   > 定时器列表，包含了所有定时器资源的大表，由定时器任务模块控制
 *   > 硬件资源表表，定时刷新所有硬件资源的大表，由HWINV任务模块控制
 *   - 所有列表基本上都以TASK_ID作为主入口
 *
 */
