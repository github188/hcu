/*
 * hwinv.c
 *
 *  Created on: 2015年11月17日
 *      Author: test
 */

#include "hwinv.h"
#include "../comvm/trace.h"
#include "../l2frame/l2comdef.h"
/*
** FSM of the HWINV
*/
FsmStateItem_t FsmHwinv[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_hwinv_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_hwinv_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_hwinv_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_HWINV_ACTIVED,            	fsm_hwinv_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

/*
 *  设备的标识符定义，统一放在了L2COMDEF.H文件中，需要根据不同设备进行同意修改
 *
 */

//Global variables
HcuHwinvCtrlTable_t zHcuHwinvTable;
zHcuTimeDateTable_t zCurTimeDate;

/* System Information */
SysInfo_t gSysInfo;

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_hwinv_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_HWINV, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("HWINV: Error Set FSM State at fsm_hwinv_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_hwinv_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_HWINV, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_HWINV, FSM_STATE_HWINV_INITED) == FAILURE){
		HcuErrorPrint("HWINV: Error Set FSM State!\n");
		return FAILURE;
	}

	//INIT this task global variables
	if (func_hwinv_global_par_init() == FAILURE){
		HcuErrorPrint("HWINV: Error init parameters!\n");
		return FAILURE;
	}

	//输出版本/调试/Copyright版权信息
	func_hwinv_copy_right();


	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_HWINV, FSM_STATE_HWINV_ACTIVED) == FAILURE){
		HcuErrorPrint("HWINV: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("HWINV: Enter FSM_STATE_HWINV_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//进入等待反馈状态
	while(1){
		func_hwinv_scan_all();
		hcu_sleep(5);  //休息5S后，继续重复SCAN所有的硬件状态
	}

	return SUCCESS;
}

OPSTAT fsm_hwinv_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_hwinv_global_par_init(void)
{
	time_t lt;
	struct tm *cu;
	DIR *dtmp;
	char stmp[HCU_DIR_LENGTH_MAX];

	//赋初值
	memset(&zHcuHwinvTable, 0xFF, sizeof(HcuHwinvCtrlTable_t));
	memset(&zCurTimeDate, 0xFF, sizeof(zHcuTimeDateTable_t));

	//初始本地时间
	lt=time(NULL);
	cu = localtime(&lt);
	zCurTimeDate.year = cu->tm_year;
	zCurTimeDate.mon = cu->tm_mon;
	zCurTimeDate.day = cu->tm_mday;
	zCurTimeDate.hour = cu->tm_hour;
	zCurTimeDate.min = cu->tm_min;
	//strftime(zCurDayString, 8, "%Y%m%d", cu);
	//strftime(zCurHourString,10, "%Y%m%d%H", cu);
	//strftime(zCurMinString, 12, "%Y%m%d%H%M", cu);
	sprintf(zCurTimeDate.sYear, "%04d", (UINT16)(1900+cu->tm_year));
	sprintf(zCurTimeDate.sMon,  "%04d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon);
	sprintf(zCurTimeDate.sDay,  "%04d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday);
	sprintf(zCurTimeDate.sHour, "%04d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour);
	sprintf(zCurTimeDate.sMin,  "%04d%02d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour, (UINT8)cu->tm_min);
	sprintf(zCurTimeDate.strYear, "./%04d", (UINT16)(1900+cu->tm_year));
	sprintf(zCurTimeDate.strMon,  "./%04d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon);
	sprintf(zCurTimeDate.strDay,  "./%04d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday);
	sprintf(zCurTimeDate.strHour, "./%04d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour);
	sprintf(zCurTimeDate.strMin,  "./%04d%02d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour, (UINT8)cu->tm_min);
	zCurTimeDate.grad = cu->tm_hour * 60 + cu->tm_min;

	//获取当前工作目录，并存入全局表
	if (getcwd(zCurTimeDate.curHcuDir, sizeof(zCurTimeDate.curHcuDir)-1) == NULL){
		HcuErrorPrint("HWINV: Error get current working directory!\n");
		return FAILURE;
	}

	//给curLogDir以及curYmDir赋值
	strcpy(stmp, zCurTimeDate.curHcuDir);
	strcat(stmp, HCU_RECORD_LOG_DIR_NAME_CLEAN);
	strcpy(zCurTimeDate.curLogDir, stmp);
	strcat(stmp, "/");
	strcat(stmp, zCurTimeDate.sMon);
	strcpy(zCurTimeDate.curYmDir, stmp);

	//创建备份记录的基本工作目录，如果存在，就直接使用
	dtmp = opendir(zCurTimeDate.curLogDir);
	if (dtmp == NULL){
		if (mkdir(HCU_RECORD_LOG_DIR_NAME_LOCAL, 0755) == -1){
			HcuErrorPrint("HWINV: Create dir error!\n");
			return FAILURE;
		}
		//成功就不打印提示了
	}else{
		//HcuDebugPrint("HWINV: Dir exist!\n");
		closedir(dtmp);
	}

	//创建YYMM子目录
	dtmp = opendir(zCurTimeDate.curYmDir);
	if (dtmp == NULL){
		chdir(zCurTimeDate.curLogDir);
		if (mkdir(zCurTimeDate.sMon, 0755) == -1){
			HcuErrorPrint("HWINV: Create directory error!\n");
			return FAILURE;
		}
		chdir(zCurTimeDate.curHcuDir);
		//成功就不打印提示了
	}else{
		//HcuDebugPrint("HWINV: Dir exist!\n");
		closedir(dtmp);
	}

	//创建文件名字
	//Sensor是天级别
	strcpy(stmp, zCurTimeDate.curYmDir);
	strcat(stmp, HCU_RECORD_FILE_NAME_SENSOR);
	strcat(stmp, zCurTimeDate.sDay);
	strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
	strcpy(zCurTimeDate.curSensorFd, stmp);
	//Audio是小时级别
	strcpy(stmp, zCurTimeDate.curYmDir);
	strcat(stmp, HCU_RECORD_FILE_NAME_AUDIO);
	strcat(stmp, zCurTimeDate.sHour);
	strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
	strcpy(zCurTimeDate.curAudioFd, stmp);
	//Camera是分钟级别
	strcpy(stmp, zCurTimeDate.curYmDir);
	strcat(stmp, HCU_RECORD_FILE_NAME_CAMERA);
	strcat(stmp, zCurTimeDate.sMin);
	strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
	strcpy(zCurTimeDate.curCameraFd, stmp);

	//HcuDebugPrint("%s\n", zCurTimeDate.curSensorFd);
	//HcuDebugPrint("%s\n", zCurTimeDate.curAudioFd);
	//HcuDebugPrint("%s\n", zCurTimeDate.curCameraFd);


	//初始化ETHERNET后台接口
	zHcuHwinvTable.ethernet.hwBase.taskId = TASK_ID_ETHERNET;
	zHcuHwinvTable.ethernet.hwBase.hwResistence = 0;
	if (COMM_HW_BOARD_ETHERNET == COMM_HW_BOARD_OFF)
		zHcuHwinvTable.ethernet.hwBase.hwStatus = HCU_HWINV_STATUS_NOT_INSTALL;
	if ((COMM_HW_BOARD_ETHERNET == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_ETHERNET == NULL))
		zHcuHwinvTable.ethernet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
	if ((COMM_HW_BOARD_ETHERNET == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_ETHERNET != NULL))
		zHcuHwinvTable.ethernet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;

	//初始化USBNET后台接口
	zHcuHwinvTable.usbnet.hwBase.taskId = TASK_ID_USBNET;
	zHcuHwinvTable.usbnet.hwBase.hwResistence = 0;
	if (COMM_HW_BOARD_USBNET == COMM_HW_BOARD_OFF)
		zHcuHwinvTable.usbnet.hwBase.hwStatus = HCU_HWINV_STATUS_NOT_INSTALL;
	if ((COMM_HW_BOARD_USBNET == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_NET == NULL))
		zHcuHwinvTable.usbnet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
	if ((COMM_HW_BOARD_USBNET == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_NET != NULL))
		zHcuHwinvTable.usbnet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;

	//初始化WIFI后台接口
	zHcuHwinvTable.wifi.hwBase.taskId = TASK_ID_WIFI;
	zHcuHwinvTable.wifi.hwBase.hwResistence = 0;
	if (COMM_HW_BOARD_WIFI == COMM_HW_BOARD_OFF)
		zHcuHwinvTable.wifi.hwBase.hwStatus = HCU_HWINV_STATUS_NOT_INSTALL;
	if ((COMM_HW_BOARD_WIFI == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_WIFI == NULL))
		zHcuHwinvTable.wifi.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
	if ((COMM_HW_BOARD_WIFI == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_WIFI != NULL))
		zHcuHwinvTable.wifi.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;

	//初始化3G4G后台接口
	zHcuHwinvTable.g3g4.hwBase.taskId = TASK_ID_3G4G;
	zHcuHwinvTable.g3g4.hwBase.hwResistence = 0;
	if (COMM_HW_BOARD_3G4G == COMM_HW_BOARD_OFF)
		zHcuHwinvTable.g3g4.hwBase.hwStatus = HCU_HWINV_STATUS_NOT_INSTALL;
	if ((COMM_HW_BOARD_3G4G == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_3G4G == NULL))
		zHcuHwinvTable.g3g4.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
	if ((COMM_HW_BOARD_3G4G == COMM_HW_BOARD_ON) && (HCU_DEFAULT_DEVICE_USB_3G4G != NULL))
		zHcuHwinvTable.g3g4.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;


	return SUCCESS;
}

void func_hwinv_copy_right(void)
{
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                                                                **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**              Home and Car Unisystem (HCU)                      **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**            Octopus Wireless Communications Co. Ltd.            **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                          Version 1.0                           **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                                                                **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint(" \n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("SW Build: %s, %s\n", __DATE__, __TIME__); /* prints !!!Hello World!!! */
	HcuDebugPrint("HW Build: P01\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint(" == SYSINFO ==\n"); /* prints !!!Hello World!!! */
}

void func_hwinv_scan_all(void)
{
	func_hwinv_scan_date();
	func_hwinv_scan_cpu();
	func_hwinv_scan_flash();
	func_hwinv_scan_rtc();
	func_hwinv_scan_memroy();
	func_hwinv_scan_main_board();
	func_hwinv_scan_hard_disc();
	func_hw_inv_scan_dir();
	func_hwinv_scan_gpio();
	func_hwinv_scan_sps232();
	func_hwinv_scan_sps485();
	func_hwinv_scan_ethernet();
	func_hwinv_scan_usbnet();
	func_hwinv_scan_wifi();
	func_hwinv_scan_3g4g();
	func_hwinv_scan_audio();
	func_hwinv_scan_camera();
	func_hwinv_scan_usb();
	func_hwinv_scan_emc();
	func_hwinv_scan_pm25();
	func_hwinv_scan_winddir();
	func_hwinv_scan_windspd();
	func_hwinv_scan_temp();
	func_hwinv_scan_humid();
	func_hwinv_scan_co2();
}

void func_hwinv_scan_date(void)
{
	time_t lt;
	struct tm *cu;
	char stmp[HCU_DIR_LENGTH_MAX];
	BOOL flagMon = FALSE;
	BOOL flagDay = FALSE;
	BOOL flagHour = FALSE;
	BOOL flagMin = FALSE;

	//获取日历时间
	lt=time(NULL);
	cu = localtime(&lt);

	//更新Current数据
	if (cu->tm_min != zCurTimeDate.min){
		if (cu->tm_mon != zCurTimeDate.mon) flagMon = TRUE;
		if (cu->tm_mday != zCurTimeDate.day) flagDay = TRUE;
		if (cu->tm_hour != zCurTimeDate.hour) flagHour = TRUE;
		if (cu->tm_min != zCurTimeDate.min) flagMin = TRUE;
		zCurTimeDate.year = cu->tm_year;
		zCurTimeDate.mon = cu->tm_mon;
		zCurTimeDate.day = cu->tm_mday;
		zCurTimeDate.hour = cu->tm_hour;
		zCurTimeDate.min = cu->tm_min;
		//strftime(zCurDayString, 8, "%Y%m%d", cu);
		//strftime(zCurHourString,10, "%Y%m%d%H", cu);
		//strftime(zCurMinString, 12, "%Y%m%d%H%M", cu);
		sprintf(zCurTimeDate.sYear, "%04d", (UINT16)(1900+cu->tm_year));
		sprintf(zCurTimeDate.sMon,  "%04d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon);
		sprintf(zCurTimeDate.sDay,  "%04d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday);
		sprintf(zCurTimeDate.sHour, "%04d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour);
		sprintf(zCurTimeDate.sMin,  "%04d%02d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour, (UINT8)cu->tm_min);
		sprintf(zCurTimeDate.strYear, "./%04d", (UINT16)(1900+cu->tm_year));
		sprintf(zCurTimeDate.strMon,  "./%04d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon);
		sprintf(zCurTimeDate.strDay,  "./%04d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday);
		sprintf(zCurTimeDate.strHour, "./%04d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour);
		sprintf(zCurTimeDate.strMin,  "./%04d%02d%02d%02d%02d", (UINT16)(1900+cu->tm_year), (UINT8)cu->tm_mon, (UINT8)cu->tm_mday, (UINT8)cu->tm_hour, (UINT8)cu->tm_min);
		zCurTimeDate.grad = cu->tm_hour * 60 + cu->tm_min;
	}

	/*
	 * 		检查目录是否存在，不存在，就建立目录
	 * 		./log/yyyymm/sensorlogyyyymmdd.dat
	 * 		./log/yyyymm/audiologyyyymmddhh.dat
	 * 		./log/yyyymm/cameralogyyyymmmmddhhmm.dat
	 */

	//创建YYMM子目录
	if (flagMon == TRUE){
		strcpy(stmp, zCurTimeDate.curLogDir);
		strcat(stmp, "/");
		strcat(stmp, zCurTimeDate.sMon);
		strcpy(zCurTimeDate.curYmDir, stmp);

		DIR *dtmp;
		dtmp = opendir(zCurTimeDate.curYmDir);
		if (dtmp == NULL){
			chdir(zCurTimeDate.curLogDir);
			if (mkdir(zCurTimeDate.sMon, 0755) == -1){
				HcuErrorPrint("HWINV: Create directory error!\n");
				return;
			}
			chdir(zCurTimeDate.curHcuDir);
			//成功就不打印提示了
		}else{
			//HcuDebugPrint("HWINV: Dir exist!\n");
			closedir(dtmp);
		}
	}//Flag=TRUE，如果FLASE，则进不来

	//创建文件名字
	//Sensor是天级别
	if (flagDay == TRUE){
		strcpy(stmp, zCurTimeDate.curYmDir);
		strcat(stmp, HCU_RECORD_FILE_NAME_SENSOR);
		strcat(stmp, zCurTimeDate.sDay);
		strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
		strcpy(zCurTimeDate.curSensorFd, stmp);
	}
	//Audio是小时级别
	if (flagHour == TRUE){
		strcpy(stmp, zCurTimeDate.curYmDir);
		strcat(stmp, HCU_RECORD_FILE_NAME_AUDIO);
		strcat(stmp, zCurTimeDate.sHour);
		strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
		strcpy(zCurTimeDate.curAudioFd, stmp);
	}
	//Camera是分钟级别
	if (flagMin == TRUE){
		strcpy(stmp, zCurTimeDate.curYmDir);
		strcat(stmp, HCU_RECORD_FILE_NAME_CAMERA);
		strcat(stmp, zCurTimeDate.sMin);
		strcat(stmp, HCU_RECORD_FILE_NAME_EXTEND);
		strcpy(zCurTimeDate.curCameraFd, stmp);
	}

	//HcuDebugPrint("%s\n", zCurTimeDate.curSensorFd);
	//HcuDebugPrint("%s\n", zCurTimeDate.curAudioFd);
	//HcuDebugPrint("%s\n", zCurTimeDate.curCameraFd);
	//No return
}

void func_hwinv_scan_cpu(void)
{
	//memset(&gSysInfo, 0xFF, sizeof(SysInfo_t));
	//HcuGetSysInfo(&gSysInfo);

}

void func_hwinv_scan_flash(void)
{}

void func_hwinv_scan_rtc(void)
{}

void func_hwinv_scan_memroy(void)
{}

void func_hwinv_scan_main_board(void)
{}

void func_hwinv_scan_hard_disc(void)
{}

void func_hw_inv_scan_dir(void)
{}

void func_hwinv_scan_gpio(void)
{}

void func_hwinv_scan_sps232(void)
{}

void func_hwinv_scan_sps485(void)
{}

//扫描相应的接口是否已经存在或者相应的PID还处于活跃状态
//检查的信息包括SYSCONFIG.H的配置/L2COMDEF.H的定义，以及相应DEVICE是否活跃
//然后，根据结果置相应状态
//注意，这里并不关心连接是否真的正常，而只是相应的硬件和驱动是否正常而已
void func_hwinv_scan_ethernet(void)
{
	int ret = SUCCESS; //先假设以太网是好使的
	msg_struct_hwinv_cloud_phy_status_chg_t snd;

	//scan接口的硬件状态，以便更新状态字
	if (ret == SUCCESS){
		if(zHcuHwinvTable.ethernet.hwBase.hwResistence > 0){zHcuHwinvTable.ethernet.hwBase.hwResistence = 0;}
		if (zHcuHwinvTable.ethernet.hwBase.hwStatus != HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.ethStatChg = HWINV_PHY_STATUS_DEACTIVE_TO_ACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.ethernet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;
		}
	}

	else if (ret == FAILURE){
		zHcuHwinvTable.ethernet.hwBase.hwResistence++;
		if (zHcuHwinvTable.ethernet.hwBase.hwStatus == HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.ethStatChg = HWINV_PHY_STATUS_ACTIVE_TO_DEACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.ethernet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
		}
	}
	//其它结果不管了
}

void func_hwinv_scan_usbnet(void)
{
	int ret = FAILURE;  //假设这个物理接口不好使
	msg_struct_hwinv_cloud_phy_status_chg_t snd;

	//scan接口的硬件状态，以便更新状态字
	if (ret == SUCCESS){
		if(zHcuHwinvTable.usbnet.hwBase.hwResistence > 0){zHcuHwinvTable.usbnet.hwBase.hwResistence = 0;}
		if (zHcuHwinvTable.usbnet.hwBase.hwStatus != HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.usbnetStatChg = HWINV_PHY_STATUS_DEACTIVE_TO_ACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.usbnet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;
		}
	}

	else if (ret == FAILURE){
		zHcuHwinvTable.usbnet.hwBase.hwResistence++;
		if (zHcuHwinvTable.usbnet.hwBase.hwStatus == HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.usbnetStatChg = HWINV_PHY_STATUS_ACTIVE_TO_DEACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.usbnet.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
		}
	}
	//其它结果不管了
}

void func_hwinv_scan_wifi(void)
{
	int ret = FAILURE; //假设这个物理接口不好使
	msg_struct_hwinv_cloud_phy_status_chg_t snd;

	//scan接口的硬件状态，以便更新状态字
	if (ret == SUCCESS){
		if(zHcuHwinvTable.wifi.hwBase.hwResistence > 0){zHcuHwinvTable.wifi.hwBase.hwResistence = 0;}
		if (zHcuHwinvTable.wifi.hwBase.hwStatus != HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.wifiStatChg = HWINV_PHY_STATUS_DEACTIVE_TO_ACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.wifi.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;
		}
	}

	else if (ret == FAILURE){
		zHcuHwinvTable.wifi.hwBase.hwResistence++;
		if (zHcuHwinvTable.wifi.hwBase.hwStatus == HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.wifiStatChg = HWINV_PHY_STATUS_ACTIVE_TO_DEACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.wifi.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
		}
	}
	//其它结果不管了
}

void func_hwinv_scan_3g4g(void)
{
	int ret = FAILURE; //假设这个物理接口不好使
	msg_struct_hwinv_cloud_phy_status_chg_t snd;

	//scan接口的硬件状态，以便更新状态字
	if (ret == SUCCESS){
		if(zHcuHwinvTable.g3g4.hwBase.hwResistence > 0){zHcuHwinvTable.g3g4.hwBase.hwResistence = 0;}
		if (zHcuHwinvTable.g3g4.hwBase.hwStatus != HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.g3g4StatChg = HWINV_PHY_STATUS_DEACTIVE_TO_ACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.g3g4.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_ACTIVE;
		}
	}

	else if (ret == FAILURE){
		zHcuHwinvTable.g3g4.hwBase.hwResistence++;
		if (zHcuHwinvTable.g3g4.hwBase.hwStatus == HCU_HWINV_STATUS_INSTALL_ACTIVE){
			//发送状态改变给CLOUD
			memset(&snd, 0, sizeof(msg_struct_hwinv_cloud_phy_status_chg_t));
			snd.length = sizeof(msg_struct_hwinv_cloud_phy_status_chg_t);
			snd.g3g4StatChg = HWINV_PHY_STATUS_ACTIVE_TO_DEACTIVE;
			ret = hcu_message_send(MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG, TASK_ID_CLOUD, TASK_ID_HWINV, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("HWINV: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_HWINV], zHcuTaskNameList[TASK_ID_CLOUD]);
				return;
			}
			//放在里面，为了提高效率：状态不改变，不用修改控制表的数据
			zHcuHwinvTable.g3g4.hwBase.hwStatus = HCU_HWINV_STATUS_INSTALL_DEACTIVE;
		}
	}
	//其它结果不管了
}

void func_hwinv_scan_usb(void)
{}

void func_hwinv_scan_audio(void)
{}

void func_hwinv_scan_camera(void)
{}

void func_hwinv_scan_emc(void)
{
	//其实，这里的难题是，如何获取MODBUS的操作，取得传感器上的硬件状态，待确定
	//其它传感器雷同

}

void func_hwinv_scan_pm25(void)
{}

void func_hwinv_scan_winddir(void)
{}

void func_hwinv_scan_windspd(void)
{}

void func_hwinv_scan_temp(void)
{}

void func_hwinv_scan_humid(void)
{}

void func_hwinv_scan_co2(void)
{}


//只是提供了硬件和FLASH访问的一种抽象
//一种最为简单的硬盘存储传感器记录的机制搞定，以后可以逐步丰富：碎片文件/内容按照时间轴/记录数量/定时清理等等，待完成
UINT32 hcu_disk_write(UINT32 fId, void *dataBuffer, UINT32 dataLen)
{
	int fHandler;
	char fname[HCU_DIR_LENGTH_MAX];

	//判断入参
	if ((dataBuffer == NULL) || (dataLen > FILE_LENGTH_RECORD_MAX)){
	  HcuErrorPrint("HWINV: Input parameter error!\n");
	  return FAILURE;
	}

	if (fId == FILE_OPERATION_TYPE_SENSOR){
		strcpy(fname, zCurTimeDate.curSensorFd);
	}else if (fId == FILE_OPERATION_TYPE_AUDIO){
		strcpy(fname, zCurTimeDate.curAudioFd);
	}else if (fId == FILE_OPERATION_TYPE_CAMERA){
		strcpy(fname, zCurTimeDate.curCameraFd);
	}else{
		HcuErrorPrint("HWINV: Error Oepration File type!\n");
		return FAILURE;
	}

	//打开源文件
	if((fHandler=open(fname, O_RDWR|O_CREAT))==-1){
	  HcuErrorPrint("HWINV: Open %s Error!\n", fname);
	  return FAILURE;
	}

	//测得文件大小
	UINT32 file_len=0;
	file_len= lseek(fHandler, 0L, SEEK_END);  //文件总长度
	lseek(fHandler, 0L, SEEK_SET); //移动文件到尾部开头
	lseek(fHandler, 0L, SEEK_END); //移动文件到尾部尾部

	//写入文件内容
	write(fHandler, dataBuffer, dataLen);
	close(fHandler);

	#if (MSG_DEBUG_ON)
	HcuDebugPrint("HWINV: Save record into hd-disk successful!\n");
	#endif
	return SUCCESS;
}


/*
 *
Linux时间操作(time、gettimeofday)
一、time函数

#include <time.h>
time_t time(time_t *calptr);
返回距计算机元年的秒数
一旦取得这种以秒计的很大的时间值后，通常要调用另一个时间函数将其变换为人们可读的时间和日期

#include <time.h>
//calendar time into a broken-down time expressed as UTC
struct tm *gmtime(const time_t *calptr);

//converts the calendar time to the local time, taking into account the local time zone and
//daylight saving time flag
struct tm *localtime(const time_t *calptr);

//converts it into a time_t value
time_t mktime(struct tm *tmptr);

  struct tm {         a broken-down time
     int  tm_sec;      seconds after the minute: [0 - 60]
     int  tm_min;      minutes after the hour: [0 - 59]
     int  tm_hour;     hours after midnight: [0 - 23]
     int  tm_mday;     day of the month: [1 - 31]
     int  tm_mon;      months since January: [0 - 11]
     int  tm_year;     years since 1900
     int  tm_wday;     days since Sunday: [0 - 6]
     int  tm_yday;     days since January 1: [0 - 365]
     int  tm_isdst;    daylight saving time flag: <0, 0, >0
　　// 以下两个字段在有些版本中是存在的，使用时需要查看对应的头文件确认
　　long int tm_gmtoff;  Seconds east of UTC.
　　const char *tm_zone;  Timezone abbreviation.
   };

char *asctime(const struct tm *tmptr);
char *ctime(const time_t *calptr);
asctime()和ctime()函数产生形式的26字节字符串，这与date命令的系统默认输出形式类似：
    Tue Feb 10 18:27:38 2004/n/0

二、gettimeofday函数得到更精确的时间
#include <sys/time.h>
int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
第二个形参是基于平台实现的，使用的时候最好用NULL
struct timeval{
    time_t tv_sec;        *** second ***
    susecond_t tv_usec;    *** microsecond 微妙***
}
1秒＝1000毫秒，
1毫秒＝1000微秒，
1微妙＝1000纳秒，
1纳秒＝1000皮秒。
秒用s表现,毫秒用ms,微秒用μs表示，纳秒用ns表示，皮秒用ps表示。

三、内核时间
内核有两个重要的全局变量:
unsigned long jiffies;
timeval xtime ;

jiffies 是记录着从电脑开机到现在总共的"时钟中断"的次数。
文件linux-2.6.24/kernel/timer.c
void do_timer(unsigned long ticks)
{
    jiffies_64 += ticks;
    update_times(ticks);
}


xtime 是从cmos电路或rtc芯片中取得的时间，一般是从某一历史时刻开始到现在的时间。 这个就是所谓的"墙上时钟walltimer"，通过它可计算得出操作系统需要的日期时间，它的精确度是微秒。
xtime第一次赋值是在系统启动时调用timekeeping_init或time_init进行的再调用read_persistent_clock进一步调用get_rtc_time得到的

PS：在/proc/uptime里面的两个数字分别表示:
the uptime of the system(seconds),
and the amount of time spent in idle process(seconds).

 注：
　 今天写一个时间管理模块，updateTime(time)  time为year month day hour minute second minlliseconds任意一个，包括正数 负数
    其中一个约定为  1970年之后的时间管理。 主要通过闰年来管理日期。对某个量进行设置后，对应调整日期，比较麻烦。
     如果利用 系统的  mktime 和 localtime/gmtime  在 tm size_t 之间进行转换，及其方便！
*/
