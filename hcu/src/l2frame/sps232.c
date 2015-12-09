/*
 * sps232.c
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#include "sps232.h"

#include "../comvm/trace.h"

/*
** FSM of the SPS232
*/
FsmStateItem_t FsmSps232[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_sps232_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_sps232_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_sps232_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_SPS232_RECEIVED,            fsm_sps232_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_sps232_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_SPS232, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("SPS232: Error Set FSM State at fsm_sps232_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_sps232_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_SPS232, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("SPS232: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SPS232], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_SPS232, FSM_STATE_SPS232_INITED) == FAILURE){
		HcuErrorPrint("SPS232: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化硬件接口
	if (func_sps232_int_init() == FAILURE){
		HcuErrorPrint("SPS232: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_SPS232, FSM_STATE_SPS232_RECEIVED) == FAILURE){
		HcuErrorPrint("SPS232: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("SPS232: Enter FSM_STATE_SPS232_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//进入阻塞式接收数据状态，然后继续发送
	while(1){
		//接收数据
		int dataLen=0;
		if (dataLen > 1){
			//发送数据给CLOUDCONT
			msg_struct_sps232_modbus_data_rx_t snd;
			memset(&snd, 0, sizeof(msg_struct_sps232_modbus_data_rx_t));
			snd.length = sizeof(msg_struct_sps232_modbus_data_rx_t);
			ret = hcu_message_send(MSG_ID_SPS232_MODBUS_DATA_RX, TASK_ID_MODBUS, TASK_ID_SPS232, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("SPS232: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SPS232], zHcuTaskNameList[TASK_ID_MODBUS]);
				return FAILURE;
			}
		}

		hcu_sleep(5);
	}

	return SUCCESS;
}

OPSTAT fsm_sps232_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_sps232_int_init(void)
{
	return SUCCESS;
}

