/*
 * svrcon.c
 *
 *  Created on: 2015年11月12日
 *      Author: test
 */

#include "svrcon.h"
#include "../comvm/trace.h"
#include "../comvm/configxml.h"
#include "../comvm/sysinfo.h"
#include "../comvm/timer.h"

/*
** FSM of the SvrCon
*/
FsmStateItem_t FsmSvrCon[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_svrcon_task_entry}, //Starting

	//System level initialization
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_svrcon_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_svrcon_restart},

	//Task level processing
	{MSG_ID_COM_INIT,       	FSM_STATE_SVRCON_INITED,            	fsm_svrcon_init},
    {MSG_ID_COM_RESTART,       	FSM_STATE_SVRCON_INITED,            	fsm_svrcon_restart},

    //Wait for feedback status processing
    {MSG_ID_COM_RESTART,        FSM_STATE_SVRCON_WAIT_FOR_FB,       	fsm_svrcon_restart},
    {MSG_ID_COM_INIT_FEEDBACK,  FSM_STATE_SVRCON_WAIT_FOR_FB,         	fsm_svrcon_init_feed_back},
    {MSG_ID_COM_TIME_OUT,  		FSM_STATE_SVRCON_WAIT_FOR_FB,         	fsm_svrcon_init_time_out},

	//Re-init all tasks after INIT_WAIT_FOR_FB failure
	{MSG_ID_COM_INIT,       	FSM_STATE_SVRCON_INIT_FB_FAILURE,       fsm_svrcon_init},
	{MSG_ID_COM_RESTART,        FSM_STATE_SVRCON_INIT_FB_FAILURE,       fsm_svrcon_restart},

    //Active status processing
    {MSG_ID_COM_RESTART,        FSM_STATE_SVRCON_ACTIVED,       		fsm_svrcon_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Local shared variables, to store all task inti status
HcuSvrConTaskInitInfo_t zHcuSvrConTaskInitInfo[MAX_TASK_NUM_IN_ONE_HCU]; //存储所有初始化任务的状态信息
//Not store local task timer anymore
//HcuSvrConTimer_t zSvrConTimer[SVRCON_TIMER_NUM_MAX];

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
//这个函数是被外部的主任务/VM所调用，初始化这里TASK里面的本地变量，是有问题的
OPSTAT fsm_svrcon_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret;
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	ret = FsmSetState(TASK_ID_SVRCON, FSM_STATE_IDLE);
	if (ret == FAILURE){
		HcuErrorPrint("SVRCON: Error Set FSM State at fsm_svrcon_task_entry\n");
	}
	return SUCCESS;
}

OPSTAT fsm_svrcon_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Checking source of INIT message, only accept INIT from HCUMAIN.
	//In future, maybe accept more cases, but right now this is the only allowable situation.
	if (src_id != TASK_ID_HCUMAIN){
		HcuErrorPrint("SVRCON: Error Init message get from task_id [%s]!\n", zHcuTaskNameList[src_id]);
		return FAILURE;
	}

	//Send back MSG_ID_COM_INIT_FEEDBACK to HCU-MAIN
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_HCUMAIN, TASK_ID_SVRCON, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("HCU-MAIN: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_HCUMAIN]);
	}

	//INIT this task global variables
	memset(zHcuSvrConTaskInitInfo, 0xFF, (sizeof(HcuSvrConTaskInitInfo_t) * MAX_TASK_NUM_IN_ONE_HCU));

	//收到初始化消息后，进入初始化状态
	ret = FsmSetState(TASK_ID_SVRCON, FSM_STATE_SVRCON_INITED);
	if (ret == FAILURE){
		HcuErrorPrint("SVRCON: Error Set FSM State at fsm_svrcon_init\n");
		return FAILURE;
	}
	HcuDebugPrint("SVRCON: Staring to init all task by distribute MSG_ID_COM_INIT message!\n");

	//发送初始化消息给所有活着的任务，以便让所有任务可控， wait for other task init accomplish
	msg_struct_com_init_t snd;
	memset(&snd, 0, sizeof(msg_struct_com_init_t));
	snd.length = sizeof(msg_struct_com_init_t);

	if (zHcuTaskInfo[TASK_ID_MMC].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_MMC, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_MMC]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_MMC].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_MMC].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_HWINV].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_HWINV, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_HWINV]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_HWINV].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_HWINV].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_MODBUS].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_MODBUS, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_MODBUS]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_MODBUS].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_MODBUS].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_CLOUD].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_CLOUD, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_CLOUD]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_CLOUD].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_CLOUD].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_EMC].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_EMC, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_EMC]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_EMC].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_EMC].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_PM25].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_PM25, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_PM25]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_PM25].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_PM25].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_TEMP].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_TEMP, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_TEMP]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_TEMP].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_TEMP].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_HUMID].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_HUMID, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_HUMID]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_HUMID].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_HUMID].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_WINDDIR].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_WINDDIR, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_WINDDIR]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_WINDDIR].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_WINDDIR].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_WINDSPD].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_WINDSPD, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_WINDSPD]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_WINDSPD].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_WINDSPD].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_HSMMP].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_HSMMP, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_HSMMP]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_HSMMP].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_HSMMP].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_ETHERNET].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_ETHERNET, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_ETHERNET]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_ETHERNET].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_ETHERNET].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_WIFI].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_WIFI, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_WIFI]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_WIFI].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_WIFI].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_USBNET].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_USBNET, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_USBNET]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_USBNET].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_USBNET].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_3G4G].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_3G4G, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_3G4G]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_3G4G].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_3G4G].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_SPS232].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_SPS232, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_SPS232]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_SPS232].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_SPS232].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_SPS485].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_SPS485, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_SPS485]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_SPS485].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_SPS485].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_AUDIO].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_AUDIO, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_AUDIO]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_AUDIO].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_AUDIO].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_CAMERA].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_CAMERA, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_CAMERA]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_CAMERA].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_CAMERA].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_BLE].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_BLE, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_BLE]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_BLE].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_BLE].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_GPS].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_GPS, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_GPS]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_GPS].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_GPS].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;

	if (zHcuTaskInfo[TASK_ID_LCD].swTaskActive == HCU_TASK_PNP_ON){
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_LCD, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){HcuErrorPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_LCD]);}}
	zHcuSvrConTaskInitInfo[TASK_ID_LCD].active = SVRCON_TASK_ACTIVE;
	zHcuSvrConTaskInitInfo[TASK_ID_LCD].state = SVRCON_TASK_INIT_WAIT_FOR_BACK;


	//设置定时器，如果定时器超时，所有任务都没有反馈，意味着失败，停止定时器，不然就是成功
	//采用静态表格控制Timer，避免了泄露的可能性
	//为了规避这种潜在过于动态的问题，所有TIMER_ID设计成静态配置固定对应式，新TIMER_ID必须通过timer.h中的人工定义才能使用
	//Start timer directly
	ret = hcu_timer_start(TASK_ID_SVRCON, TIMER_ID_1S_SVRCON_INIT_FB, SVRCON_TIMER_DURATION_INIT_FB, TIMER_TYPE_ONE_TIME, TIMER_RESOLUTION_1S);
	if (ret == FAILURE){
		HcuErrorPrint("SVRCON: Error start timer!\n");
		return FAILURE;
	}

	//进入等待反馈状态
	ret = FsmSetState(TASK_ID_SVRCON, FSM_STATE_SVRCON_WAIT_FOR_FB);
	if (ret == FAILURE){
		HcuErrorPrint("SVRCON: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("SVRCON: Enter FSM_STATE_SVRCON_WAIT_FOR_FB status, everything goes well!\n");
	#endif //(TRACE_DEBUG_ON)
	return SUCCESS;
}

OPSTAT fsm_svrcon_init_feed_back(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//检查src_id是否合法
	if ((dest_id < TASK_ID_MIN) || (dest_id > TASK_ID_MAX)){
		HcuErrorPrint("SVRCON: Error on msg received of src_id =%d [%s]!!!\n", src_id, zHcuTaskNameList[src_id]);
		return FAILURE;
	}

	if (zHcuSvrConTaskInitInfo[src_id].active != SVRCON_TASK_ACTIVE){
		HcuErrorPrint("SVRCON: Error init feedback msg received from, src_id=%d [%s]!!!\n", src_id, zHcuTaskNameList[src_id]);
		return FAILURE;
	}

	//存入到反馈表中
	zHcuSvrConTaskInitInfo[src_id].state = SVRCON_TASK_INIT_FEEDBACK;

	//检查所有的反馈是否都收到，不然维持状态不变
	if (func_svrcon_init_caculate_all_fb() == TRUE){
		ret = hcu_timer_stop(TASK_ID_SVRCON, TIMER_ID_1S_SVRCON_INIT_FB, TIMER_RESOLUTION_1S);
		if (ret == FAILURE){
			HcuErrorPrint("SVRCON: Error stop timer!\n");
			return FAILURE;
		}
		ret = FsmSetState(TASK_ID_SVRCON, FSM_STATE_SVRCON_ACTIVED);
		if (ret == FAILURE){
			HcuErrorPrint("SVRCON: Error stop timer!\n");
			return FAILURE;
		}
		//Enter active status, keep working
		#if (TRACE_DEBUG_ON)
			HcuDebugPrint("SVRCON: Enter FSM_STATE_SVRCON_ACTIVED status, Getting to SLEEP mode for a while!\n");
		#endif //(TRACE_DEBUG_ON)
	}

	//No need FSM status change
	return SUCCESS;
}

BOOL func_svrcon_init_caculate_all_fb(void)
{
	int i=0;
	for(i=0;i<MAX_TASK_NUM_IN_ONE_HCU;i++){
		if ((zHcuSvrConTaskInitInfo[i].active == SVRCON_TASK_ACTIVE) && (zHcuSvrConTaskInitInfo[i].state ==
			SVRCON_TASK_INIT_WAIT_FOR_BACK)){
			return FALSE;
		}
	}
	return TRUE;
}

OPSTAT fsm_svrcon_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT fsm_svrcon_init_time_out(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;
	msg_struct_com_time_out_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_com_time_out_t));
	if (param_len > sizeof(msg_struct_com_time_out_t)){
		HcuErrorPrint("SVRCON: Receive time out message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);

	if ((rcv.timeId == TIMER_ID_1S_SVRCON_INIT_FB) && (rcv.timeRes == TIMER_RESOLUTION_1S)){
		HcuErrorPrint("SVRCON: Error! Time out before receve all tasks feedback, fatal error!\n");
		//不用考虑STOP TIMER，底层TIMER任务在发出TIME_OUT时已经自动停止了该定时器，除非定时器错误，比如类型设置为PERIOD才会不断重报
		//考虑如何恢复吧，倒不是急着返回，这里先这么凑合着，未来再考虑完善的方案
		//先使用无限重复INIT所有任务的方式吧，给自己再发送一个INIT消息
		ret = FsmSetState(TASK_ID_SVRCON, FSM_STATE_SVRCON_INIT_FB_FAILURE);
		if (ret == FAILURE){
			HcuErrorPrint("SVRCON: Error set FSM state!\n");
			return FAILURE;
		}
		//hcu_sleep(rand(10));
		hcu_sleep(10);
		msg_struct_com_init_t snd;
		memset(&snd, 0, sizeof(msg_struct_com_init_t));
		snd.length = sizeof(msg_struct_com_init_t);
		ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_SVRCON, TASK_ID_SVRCON, &snd, snd.length);
		if (ret == FAILURE){
			HcuDebugPrint("SVRCON: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SVRCON], zHcuTaskNameList[TASK_ID_SVRCON]);
			return FAILURE;
		}
	}else{
		HcuErrorPrint("SVRCON: Error! TIME-OUT message received with wrong timerId or timeResolution!\n");
		return FAILURE;
	}

	return SUCCESS;
}




/*
//Main Entry old
OPSTAT fsm_svrcon_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret = 0;
	//test function area, to be removed afterwards
	HcuMsgSruct_t rcv;
	HcuDebugPrint("SVRCON: Task get control power!\n");

	//Pure debug test with HCUMAIN test part
	//本来打算使用Mutex/Sig等信号量方式进行通信，确保各个任务启动后不是提前干活，现在可以在差错中进行等待，从而确保主控程序拿到控制权，非常妙！
	//VM暂时不打算实现任何信号量与互斥量
	//Loop for ever
	while (1){
		memset(&rcv, 0, sizeof(HcuMsgSruct_t));
		ret = hcu_message_rcv(TASK_ID_SVRCON, &rcv);
		if (ret == FAILURE){
			HcuDebugPrint("SVRCON: Receive message error, hold for Main-Control action!\n");
			//continue;
			//To be defined here the error case, to recover everything
			hcu_sleep(1);  //wait for HCUMAIN clean all message queue and task, otherwise rubbish can not be clean.
			//exit(EXIT_FAILURE);  //do nothing, wait for main control clean myself.
		}else{
			msg_struct_com_debug_t debug1;
			memset(&debug1, 0, sizeof(msg_struct_com_debug_t));
			memcpy(&debug1, rcv.msgBody, rcv.msgLen);
			HcuDebugPrint("SVRCON: Receive message success, test id = %d\n", debug1.testid);
			//exit(EXIT_SUCCESS);  //make my job for ever, but not exit
			//Starting of STM processing
			UINT32 im=0, r_msgid=0;
			UINT8 r_state=0;
			r_msgid = rcv.msgType;
			r_state = FsmGetState(TASK_ID_SVRCON);
			//上限应该是number of FsmSvrCon[]，等待解决
			for (im=MSG_ID_COM_MIN; im<MSG_ID_COM_MAX; im++){
				if ((FsmSvrCon[im].msg_id == r_msgid) && (FsmSvrCon[im].state == r_state)){
					if (FsmSvrCon[im].stateFunc != NULL){
						ret = (FsmSvrCon[im].stateFunc)(rcv.dest_id, rcv.src_id, rcv.msgBody, rcv.msgLen);
					}//Not NULL
				}//Right message
			}//im
		}//Else
	}//While(1)
	return SUCCESS;
	//return ((void *)0);
}
*/
