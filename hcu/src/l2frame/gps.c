/*
 * gps.c
 *
 *  Created on: 2015年11月23日
 *      Author: test
 */

#include "gps.h"

#include "../comvm/trace.h"

/*
** FSM of the GPS
*/
FsmStateItem_t FsmGps[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_gps_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_gps_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_gps_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_GPS_RECEIVED,            		fsm_gps_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_gps_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_GPS, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("GPS: Error Set FSM State at fsm_gps_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_gps_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_GPS, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("GPS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_GPS], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_GPS, FSM_STATE_GPS_INITED) == FAILURE){
		HcuErrorPrint("GPS: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化硬件接口
	if (func_gps_int_init() == FAILURE){
		HcuErrorPrint("GPS: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_GPS, FSM_STATE_GPS_RECEIVED) == FAILURE){
		HcuErrorPrint("GPS: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("GPS: Enter FSM_STATE_GPS_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//进入阻塞式接收数据状态，然后继续发送
	while(1){
		//接收数据
		int dataLen=0;
		if (dataLen > 1){
			//发送数据给CLOUDCONT
			//不建议使用消息机制，而直接采用全局变量，存入，以便其它任务模块随时读取
			//每次存入全局变量时，需要在全局变量中定义下刷新的时间，以便让业务模块自己判定，这个GPS更新的时间是太旧，还是正常


			//也可以如此实现：在全局变量中给出一个STATUS = NORMAL(正常)，INVAID(不可用)， YELLOW(不太政策，但数据具备一定的参考意义)
			//这样，只要数据不是INVAID，业务模块就使用，否则数据不应该填入并带回给后台



		}

		hcu_sleep(5);
	}

	return SUCCESS;
}

OPSTAT fsm_gps_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_gps_int_init(void)
{
	return SUCCESS;
}

