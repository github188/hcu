/*
 ============================================================================
 Name        : hcu.c
 Author      : ZHANG
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include "hcu.h"
#include "./comvm/trace.h"
#include "./comvm/sysinfo.h"
#include "./comvm/configxml.h"
#include "./comvm/timer.h"
#include "./comvm/mmc.h"

#include "./l2frame/apigprs.h"
#include "./l2frame/hwinv.h"
#include "./l2frame/cloudcont.h"
#include "./l2frame/modbus.h"
#include "./l2frame/sps232.h"
#include "./l2frame/sps485.h"
#include "./l2frame/ble.h"
#include "./l2frame/ethernet.h"
#include "./l2frame/usbnet.h"
#include "./l2frame/wifi.h"
#include "./l2frame/audio.h"
#include "./l2frame/camera.h"
#include "./l2frame/con3g4g.h"
#include "./l2frame/gps.h"
#include "./l2frame/lcd.h"

#include "./l3app/svrcon.h"
#include "./l3app/sensoremc.h"
#include "./l3app/sensorhumid.h"
#include "./l3app/sensorpm25.h"
#include "./l3app/sensortemp.h"
#include "./l3app/sensorwinddir.h"
#include "./l3app/sensorwindspd.h"
#include "./l3app/hsmmp.h"


//Main Entry, Father process
int main(void) {
	int ret = 0;
	//HW Init
	hcu_vm_system_init();
	hcu_app_system_init();
	HcuDebugPrint("HCU-MAIN: System level initialization starting...\n");

	//Create all child task and relevant massage queue
	ret = hcu_system_task_init();
	if (ret == FAILURE){
		HcuErrorPrint("HCU-MAIN: Create task failure in the middle!\n");
	}else if (ret == SUCCESS){
		HcuDebugPrint("HCU-MAIN: Create all task successful, congratulation!\n");
	}

	//先初始化TIMER任务，不然后面的其它任务没法正常工作
	msg_struct_com_init_t snd;
	memset(&snd, 0, sizeof(msg_struct_com_init_t));
	snd.length = sizeof(msg_struct_com_init_t);
	ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_TIMER, TASK_ID_HCUMAIN, &snd, snd.length);
	if (ret == FAILURE){
		HcuDebugPrint("HCU-MAIN: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HCUMAIN], zHcuTaskNameList[TASK_ID_SVRCON]);
	}

	//Wait for TIMER task feedback
	HcuMsgSruct_t rcv;
	while (1){
		memset(&rcv, 0, sizeof(HcuMsgSruct_t));
		ret = hcu_message_rcv(TASK_ID_HCUMAIN, &rcv);
		if (ret == EINTR){
			continue;
		}
		else if (ret == FAILURE){
			HcuErrorPrint("HCU-MAIN: Receive message error, hold for Main-Control action!\n");
			continue;
			//hcu_sleep(1);  //wait for HCUMAIN clean all message queue and task, otherwise rubbish can not be clean.
			//exit(EXIT_FAILURE);  //do nothing, wait for main control clean myself.
		}else{
			if ((rcv.msgType == MSG_ID_COM_INIT_FEEDBACK) && (rcv.src_id == TASK_ID_TIMER)){
				HcuDebugPrint("HCU-MAIN: TIMER task init completed. \n");
				break;
			}else{
				HcuErrorPrint("HCU-MAIN: Error! Not right message received!\n");
				continue;
			}
		}
	}

	//发送初始化消息给主控任务SVRCON，以便初始化所有任务模块
	memset(&snd, 0, sizeof(msg_struct_com_init_t));
	snd.length = sizeof(msg_struct_com_init_t);
	ret = hcu_message_send(MSG_ID_COM_INIT, TASK_ID_SVRCON, TASK_ID_HCUMAIN, &snd, snd.length);
	if (ret == FAILURE){
		HcuDebugPrint("HCU-MAIN: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HCUMAIN], zHcuTaskNameList[TASK_ID_SVRCON]);
	}

	//Wait for SVRCON feedback
	while (1){
		memset(&rcv, 0, sizeof(HcuMsgSruct_t));
		ret = hcu_message_rcv(TASK_ID_HCUMAIN, &rcv);
		if (ret == EINTR){
			continue;
		}
		else if (ret == FAILURE){
			HcuErrorPrint("HCU-MAIN: Receive message error, hold for Main-Control action!\n");
			hcu_sleep(1);
			continue;
			//hcu_sleep(1);  //wait for HCUMAIN clean all message queue and task, otherwise rubbish can not be clean.
			//exit(EXIT_FAILURE);  //do nothing, wait for main control clean myself.
		}else{
			if ((rcv.msgType == MSG_ID_COM_INIT_FEEDBACK) && (rcv.src_id == TASK_ID_SVRCON)){
				HcuDebugPrint("HCU-MAIN: HCU-MAIN task init completed, enter into SLEEP mode forever! \n");
				break;
			}else{
				HcuErrorPrint("HCU-MAIN: Error! Not right message received!\n");
				continue;
			}
		}
	}
	//wait for ever
	while (1){
		hcu_sleep(1);
		hcu_usleep(100);
	}

	//Test purpose, remove all message queue, to be removed at formal situation
	//task_id => starting from TASK_NAME_ID_MIN to TASK_NAME_ID_MAX
	HcuDebugPrint("HCU-MAIN: Clean all running task and message queue for next time debug!\n");
	int task_id=0;
	for (task_id=TASK_ID_MIN; task_id<TASK_ID_MAX; task_id++){
		if (task_id != TASK_ID_HCUMAIN){hcu_task_delete(task_id);}
		hcu_msgque_delete(task_id);
	}
	//Main task end
	HcuDebugPrint("HCU-MAIN: Main task successfully accomplish and end!\n");
	hcu_sleep(1);
	return EXIT_SUCCESS;
}

//HCU local init for whole application
void hcu_app_system_init()
{
	//Checking sysconfig.h to see any task shall bet set active, or shall not start!
	int i=0;
	for (i=0; i<MAX_TASK_NUM_IN_ONE_HCU; i++){
		zHcuTaskInfo[i].hwPlugin = HCU_TASK_PNP_INVALID;
		zHcuTaskInfo[i].hwActive = HCU_TASK_PNP_INVALID;
		zHcuTaskInfo[i].swTaskActive = HCU_TASK_PNP_INVALID;
	}
	//肯定需要启动的任务模块
	zHcuTaskInfo[TASK_ID_HCUMAIN].swTaskActive = HCU_TASK_PNP_OFF;
	zHcuTaskInfo[TASK_ID_TIMER].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_CLOUD].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_SVRCON].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_MODBUS].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_MMC].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_HWINV].swTaskActive = HCU_TASK_PNP_ON;
	zHcuTaskInfo[TASK_ID_HSMMP].swTaskActive = HCU_TASK_PNP_ON;

	//Switch-case
	zHcuTaskInfo[TASK_ID_BLE].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_BLE == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_BLE].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_BLE].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_BLE].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_BLE].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_GPS].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_GPS == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_GPS].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_GPS].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_GPS].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_GPS].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_LCD].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_LCD == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_LCD].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_LCD].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_LCD].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_LCD].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_SPS232].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SPS232 == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_SPS232].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_SPS232].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_SPS232].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_SPS232].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_SPS485].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SPS485 == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_SPS485].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_SPS485].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_SPS485].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_SPS485].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_AUDIO].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_AUDIO == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_AUDIO].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_AUDIO].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_AUDIO].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_AUDIO].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_CAMERA].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_CAMERA == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_CAMERA].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_CAMERA].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_CAMERA].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_CAMERA].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_BLE].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_BLE == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_BLE].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_BLE].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_BLE].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_BLE].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_ETHERNET].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_ETHERNET == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_ETHERNET].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_ETHERNET].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_ETHERNET].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_ETHERNET].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_WIFI].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_WIFI == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_WIFI].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_WIFI].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_WIFI].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_WIFI].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_USBNET].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_USBNET == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_USBNET].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_USBNET].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_USBNET].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_USBNET].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_3G4G].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_HW_BOARD_3G4G == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_3G4G].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_3G4G].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_3G4G].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_3G4G].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_EMC].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_EMC == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_EMC].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_EMC].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_EMC].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_EMC].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_PM25].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_PM25 == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_PM25].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_PM25].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_PM25].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_PM25].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_TEMP].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_TEMP == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_TEMP].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_TEMP].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_TEMP].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_TEMP].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_HUMID].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_HUMID == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_HUMID].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_HUMID].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_HUMID].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_HUMID].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_WINDDIR].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_WINDDIR == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_WINDDIR].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_WINDDIR].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_WINDDIR].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_WINDDIR].hwActive = HCU_TASK_PNP_OFF;
	}
	zHcuTaskInfo[TASK_ID_WINDSPD].swTaskActive = HCU_TASK_PNP_ON;
	if (COMM_FRONT_SENSOR_WINDSPD == COMM_HW_BOARD_ON){
		zHcuTaskInfo[TASK_ID_WINDSPD].hwPlugin = HCU_TASK_PNP_ON;
		zHcuTaskInfo[TASK_ID_WINDSPD].hwActive = HCU_TASK_PNP_ON;
	}else{
		zHcuTaskInfo[TASK_ID_WINDSPD].hwPlugin = HCU_TASK_PNP_OFF;
		zHcuTaskInfo[TASK_ID_WINDSPD].hwActive = HCU_TASK_PNP_OFF;
	}
	//Nothing
}

//All child task and Queue creation
UINT32 hcu_system_task_init(void)
{
	int ret=0;
	int taskId = 0;
	//一个不成功，就应该返回，如果不提前返回，纯粹是为了测试
	//Create HCU-Main Queid /1  该队列创建，纯粹是为了测试，以后需要删掉
	taskId = TASK_ID_HCUMAIN;
	HcuDebugPrint("HCU-MAIN: Staring to create task [%s] related environments...\n", zHcuTaskNameList[taskId]);
   	ret = hcu_msgque_create(taskId);
    if (ret == FAILURE){
    	HcuErrorPrint("HCU-MAIN: create queue [%s] un-successfully, program exit.\n", zHcuTaskNameList[taskId]);
    	return FAILURE;
    }else{HcuDebugPrint("HCU-MAIN: create queue zHcuTaskNameList[%s] successfully.\n", zHcuTaskNameList[taskId]);}

    //Create task Timer environments /2
    hcu_system_task_init_call(TASK_ID_TIMER, FsmTimer);

    //Create task Timer environments /3
    hcu_system_task_init_call(TASK_ID_MMC, FsmMmc);

    //Create task Timer environments /4
    hcu_system_task_init_call(TASK_ID_HWINV, FsmHwinv);

    //Create task SvnCont environments /5
    hcu_system_task_init_call(TASK_ID_SVRCON, FsmSvrCon);

    //Create task CloudCont environments /6
    hcu_system_task_init_call(TASK_ID_CLOUD, FsmCloudCont);

    //Create task Modbus environments /7
    hcu_system_task_init_call(TASK_ID_MODBUS, FsmModbus);

    //Create task EMC environments/8
    hcu_system_task_init_call(TASK_ID_EMC, FsmEmc);

    //Create task PM25 environments/9
    hcu_system_task_init_call(TASK_ID_PM25, FsmPm25);

    //Create task WindDir environments/10
    hcu_system_task_init_call(TASK_ID_WINDDIR, FsmWinddir);

    //Create task WindSpeed environments/11
    hcu_system_task_init_call(TASK_ID_WINDSPD, FsmWindspd);

    //Create task Temperature environments/12
    hcu_system_task_init_call(TASK_ID_TEMP, FsmTemp);

    //Create task Humidity environments/13
    hcu_system_task_init_call(TASK_ID_HUMID, FsmHumid);

    //Create task Hsmmp environments/14
    hcu_system_task_init_call(TASK_ID_HSMMP, FsmHsmmp);

    //Create task Ethernet environments/16
    hcu_system_task_init_call(TASK_ID_ETHERNET, FsmEthernet);

    //Create task WIFI environments/17
    hcu_system_task_init_call(TASK_ID_WIFI, FsmWifi);

    //Create task USBNET environments/18
    hcu_system_task_init_call(TASK_ID_USBNET, FsmUsbnet);

    //Create task 3G4G environments/19
    hcu_system_task_init_call(TASK_ID_3G4G, Fsm3g4g);

    //Create task SPS232 environments/20
    hcu_system_task_init_call(TASK_ID_SPS232, FsmSps232);

    //Create task SPS485 environments/21
    hcu_system_task_init_call(TASK_ID_SPS485, FsmSps485);

    //Create task BLE environments/22
    hcu_system_task_init_call(TASK_ID_BLE, FsmBle);

    //Create task Audio environments/23
    hcu_system_task_init_call(TASK_ID_AUDIO, FsmAudio);

    //Create task Camera environments/24
    hcu_system_task_init_call(TASK_ID_CAMERA, FsmCamera);

    //Create task Camera environments/25
    hcu_system_task_init_call(TASK_ID_GPS, FsmGps);

    //Create task Camera environments/26
    hcu_system_task_init_call(TASK_ID_LCD, FsmLcd);


    return SUCCESS;
}

UINT32 hcu_system_task_init_call(UINT32 task_id, FsmStateItem_t *p)
{
	int ret = 0;
	if (zHcuTaskInfo[task_id].swTaskActive != HCU_TASK_PNP_ON){
		HcuErrorPrint("HCU-MAIN: no need create this task [%s]!\n", zHcuTaskNameList[task_id]);
		return FAILURE;
	}
	HcuDebugPrint("HCU-MAIN: Staring to create task [%s] related environments...\n", zHcuTaskNameList[task_id]);
	ret = hcu_task_create_and_run(task_id, p);
	if (ret == FAILURE){
		HcuErrorPrint("HCU-MAIN: create task env [%s] un-successfully, program exit.\n", zHcuTaskNameList[task_id]);
		return FAILURE;
	}
	return SUCCESS;
}


/*
//test function area, to be removed afterwards
int i=0;
msg_struct_com_debug_t debug1;
for (i=0;i<5;i++){
	memset(&debug1, 0, sizeof(msg_struct_com_debug_t));
	debug1.testid = i;
	strcpy(&(debug1.testinfo[0]),"My test!");
	//debug1.testinfo[] = "My test!";
	debug1.length = sizeof(msg_struct_com_debug_t);
	ret = hcu_message_send(MSG_ID_COM_DEBUG_TEST, TASK_ID_SVRCON, TASK_ID_HCUMAIN, &debug1, debug1.length);
	if (ret == FAILURE){
		HcuDebugPrint("HCU-MAIN: Send message error!\n");
	}else{
		HcuDebugPrint("HCU-MAIN: Send message successful, testid = %d\n", debug1.testid);
	}
	hcu_sleep(1);
}
*/
