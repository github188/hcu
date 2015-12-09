/*
 * con3g4g.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "con3g4g.h"

#include "../comvm/trace.h"

/*
** FSM of the 3G4G
*/
FsmStateItem_t Fsm3g4g[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_3g4g_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_3g4g_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_3g4g_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_3G4G_RECEIVED,            	fsm_3g4g_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Global Variables
extern CURL *zHcuCloudCurlPtr;
extern CloudControlTable_t zHcuCloudConTable;

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_3g4g_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_3G4G, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("3G4G: Error Set FSM State at fsm_3g4g_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_3g4g_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_3G4G, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("3G4G: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_3G4G], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_3G4G, FSM_STATE_3G4G_INITED) == FAILURE){
		HcuErrorPrint("3G4G: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化硬件接口
	if (func_3g4g_int_init() == FAILURE){
		HcuErrorPrint("3G4G: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_3G4G, FSM_STATE_3G4G_RECEIVED) == FAILURE){
		HcuErrorPrint("3G4G: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("3G4G: Enter FSM_STATE_3G4G_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//进入阻塞式接收数据状态，然后继续发送
	while(1){
		//接收数据
		int dataLen=0;
		if (dataLen > 1){
			//发送数据给CLOUDCONT
			msg_struct_3g4g_cloud_data_rx_t snd;
			memset(&snd, 0, sizeof(msg_struct_3g4g_cloud_data_rx_t));
			snd.length = sizeof(msg_struct_3g4g_cloud_data_rx_t);
			ret = hcu_message_send(MSG_ID_3G4G_CLOUD_DATA_RX, TASK_ID_CLOUD, TASK_ID_3G4G, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("3G4G: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_3G4G], zHcuTaskNameList[TASK_ID_CLOUD]);
				return FAILURE;
			}
		}

		hcu_sleep(5);
	}

	return SUCCESS;
}

OPSTAT fsm_3g4g_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_3g4g_int_init(void)
{
	return SUCCESS;
}


OPSTAT hcu_3g4g_data_send(CloudDataSendBuf_t *buf)
{
	HcuDebugPrint("3G4G: Wireless 3G4G network is not working, FAKE data send back!\n");
	return SUCCESS;
}

//Add by MYC
OPSTAT hcu_3g_data_send()
{
	HcuDebugPrint("API3G: Wireless 3G network is not working, FAKE data send back!\n");
	return SUCCESS;
}


//MYC Add for Forlinx A8 Linux USB 3G Module
/* Comamnd Sequence */
/*
 * 3g_test start_um ZET-AC528
 * 3g_test start_ppp ZET-AC528
 *
 * Type Supported:
 *
 * ZET-MF190 (Not Test)
 * HW-E1750 (Not Tested
 * HW-E261 (Tested)
 * ZET-AC582 (Not Tested)
 * ZET-AC2787 (Not Tested)
 * ZET-A356 (Not Tested)
 * FL-AD3812 (Not Tested)
 *
 */
UsbUmtsPppControl_t gUsbUmtsPppControl =
{
	USB_UMTS_PPP_CARD_TYPE_MAX_INVALID,  /* MAX means not initialied, if not MAX, but a value >=0 < MAX, means active OK */
	{
		"ZET-MF190",
		"HW-E175",
		"HW-E261",
		"ZET-AC528",
		"ZET-AC2787",
		"ZET-A356",
		"FL-AD3812",
		"", "", "", "", "", "", "", "", "",   /* reserve for FFS */
	},
	{
		"3g_test start_ums ZET-MF190",
		"3g_test start_ums HW-E175",
		"3g_test start_ums HW-E261",
		"3g_test start_ums ZET-AC528",
		"3g_test start_ums ZET-AC2787",
		"3g_test start_ums ZET-A356",
		"3g_test start_ums FL-AD3812",
		"", "", "", "", "", "", "", "", "",   /* reserve for FFS */
	},
	{
		"3g_test start_ppp ZET-MF190",
		"3g_test start_ppp HW-E175",
		"3g_test start_ppp HW-E261",
		"3g_test start_ppp ZET-AC528",
		"3g_test start_ppp ZET-AC2787",
		"3g_test start_ppp ZET-A356",
		"3g_test start_ppp FL-AD3812",
		"", "", "", "", "", "", "", "", "",	   /* reserve for FFS */
	}
};

UINT16 UsbUmtsCardScan()
{
	/* Scan USB for known UMTS Card */

	UINT16 usbUmtsCardType = USB_UMTS_PPP_CARD_TYPE_MAX_INVALID;

	HcuDebugPrint("UsbUmtsCardScan: Scan USB for UMTS Card start ...\r\n");

	/* To be implementted in 2nd Step */
	usbUmtsCardType = USB_UMTS_PPP_CARD_TYPE_HW_E261;

	HcuDebugPrint("UsbUmtsCardScan: USB UMTS Card Type Found: [%s].\r\n", gUsbUmtsPppControl.usbUmtsTypeName[usbUmtsCardType]);

	gUsbUmtsPppControl.activeUsbUmtsType = usbUmtsCardType;
	return usbUmtsCardType;
}

/* Should be called at system startup */
UINT32 UsbUmtsPppInit()
{
	UsbUmtsCardScan();


	return SUCCESS;
}

UINT32 UsbUmtsPppdStart()
{
	UINT32 ret;
	int system_call_result, i, try;
	UINT16 usbUmtsCardType = gUsbUmtsPppControl.activeUsbUmtsType;

	if(USB_UMTS_PPP_CARD_TYPE_MAX_INVALID == usbUmtsCardType)
	{
		HcuErrorPrint("UsbUmtsPppdStart: UsbUmtsPppInit() not called before, device not initialized, return FAILURE.\n");
		return FAILURE;
	}

	/* Debug Trace */


	try = 0;
	while(1)
	{
		try++;
		HcuDebugPrint("UsbUmtsPppdStart: Start to bring up [%s] for [%d] times.\r\n", gUsbUmtsPppControl.usbUmtsTypeName[usbUmtsCardType], try);

		/* In USB Card Presented, start to run the command */
		/* First Check whether there is already GPRS and PPP setup */
		ret = IsGprsPppdOk();
		if (SUCCESS == ret)
		{
			HcuDebugPrint("UsbUmtsPppdStart: GPRS & PPP already start, nothing done, return SUCCESS.\n");
			return SUCCESS;
		}

		HcuDebugPrint("UsbUmtsPppdStart: Make a clean before startup, calling UsbUmtsPppdSwShutdown()...\n");
		UsbUmtsPppdSwShutdown();
		/* usleep for 5 sec */
		for (i = 0; i < 5; i++)
		{
			usleep(NB_MICROS_IN_ONE_SECOND);
			HcuDebugPrint("UsbUmtsPppdStart: wait for pppd call gprs& to complete, delay %02d sec ...\n", i+1);
		}

		/* Try each type of the UMTS USB CARD */
		HcuDebugPrint("UsbUmtsPppdStart: Step 1/2: call [%s] Begin. \r\n", gUsbUmtsPppControl._3g_test_start_ums[usbUmtsCardType]);
		system_call_result = system(gUsbUmtsPppControl._3g_test_start_ums[usbUmtsCardType]);
		if (-1 == system_call_result)
		{
			HcuErrorPrint("UsbUmtsPppdStart: [%s] command not found, can not start GPRS ...\n", gUsbUmtsPppControl._3g_test_start_ums[usbUmtsCardType]);

			//system("ifconfig eth0 up");
			//WipDebugPrint("GprsPppStart: [%s] up eth0 via [ifconfig eth0 up] ...\n", ret);
			return FAILURE;
		}
		else if (0 != system_call_result)
		{
			HcuErrorPrint("GprsPppStart: [%s] un-successful, ret = %d ...\n", gUsbUmtsPppControl._3g_test_start_ums[usbUmtsCardType], ret);

			//system("ifconfig eth0 up");
			//WipDebugPrint("GprsPppStart: Bring up eth0 via [ifconfig eth0 down] ...\n", ret);
			return FAILURE;
		}
		HcuDebugPrint("UsbUmtsPppdStart: Step 1/2: call [%s] End. \r\n", gUsbUmtsPppControl._3g_test_start_ums[usbUmtsCardType]);

		/* usleep for 8 sec, wait for ttyUSB has been setup, this is a must */
		for (i = 0; i < 8; i++)
		{
			usleep(NB_MICROS_IN_ONE_SECOND);
			HcuDebugPrint("UsbUmtsPppdStart: wait for pppd call gprs& to complete, delay %02d sec ...\n", i+1);
		}

		/* Check whether the UMTS Card Inserted, and whether it has been registered in network */
		ret = UsbUmtsCardInit("/dev/ttyUSB0");
		if (FAILURE == ret)
		{
			HcuErrorPrint("UsbUmtsPppdStart: GUsbUmtsCardInit() failed.\n");
			return SUCCESS;
		}

		HcuDebugPrint("UsbUmtsPppdStart: Step 2/2: call [%s] Begin. \r\n", gUsbUmtsPppControl._3g_test_start_ppp[usbUmtsCardType]);
		system_call_result = system(gUsbUmtsPppControl._3g_test_start_ppp[usbUmtsCardType]);
		if (-1 == system_call_result)
		{
			HcuErrorPrint("UsbUmtsPppdStart: [%s] command not found, can not start GPRS ...\n", gUsbUmtsPppControl._3g_test_start_ppp[usbUmtsCardType]);

			//system("ifconfig eth0 up");
			//WipDebugPrint("GprsPppStart: [%s] up eth0 via [ifconfig eth0 up] ...\n", ret);
			return FAILURE;
		}
		else if (0 != system_call_result)
		{
			HcuErrorPrint("GprsPppStart: [%s] un-successful, ret = %d ...\n", gUsbUmtsPppControl._3g_test_start_ppp[usbUmtsCardType], ret);

			//system("ifconfig eth0 up");
			//WipDebugPrint("GprsPppStart: Bring up eth0 via [ifconfig eth0 down] ...\n", ret);
			return FAILURE;
		}
		HcuDebugPrint("UsbUmtsPppdStart: Step 2/2: call [%s] End. \r\n", gUsbUmtsPppControl._3g_test_start_ppp[usbUmtsCardType]);

		/* usleep for 15 sec */
		for (i = 0; i < 12; i++)
		{
			usleep(NB_MICROS_IN_ONE_SECOND);
			HcuDebugPrint("UsbUmtsPppdStart: wait for pppd call gprs& to complete, delay %02d sec ...\n", i+1);
		}

		/* Check again for result */
		ret = IsGprsPppdOk();
		if (SUCCESS == ret)
		{
			HcuDebugPrint("UsbUmtsPppdStart: GPRS & PPP started successfully, return SUCCESS.\n");
			/* refresh system information for network interface checking */
			HcuGetSysInfo(&gSysInfo);
			HcuDebugPrint("UsbUmtsPppdStart: refresh system information complete.\n");
			IsNetItfExist("ppp0");

			/* usleep for 15 sec, to let console know the result */
			for (i = 0; i < 2; i++)
			{
				usleep(NB_MICROS_IN_ONE_SECOND);
			}

			return SUCCESS;
		}
		else
		{
			HcuDebugPrint("UsbUmtsPppdStart: GPRS & PPP started falure, return FAILURE.\n");
			/* refresh system information for network interface checking */
			HcuGetSysInfo(&gSysInfo);
			HcuDebugPrint("UsbUmtsPppdStart: refresh system information complete.\n");

			/* usleep for 15 sec, to let console know the result */
			for (i = 0; i < 2; i++)
			{
				usleep(NB_MICROS_IN_ONE_SECOND);
			}

			/* If failed for the 1st time, try for 2nd time */
			if(try >=2)
			{
				return FAILURE;
			}
		}

	}/* end if while */

	return SUCCESS;
}

UINT32 UsbUmtsPppdSwShutdown()
{
	UINT32 ret;

	/* Shutdown network interface */
	ret = IsNetItfExist("ppp0");
	if (SUCCESS == ret)
	{
		system("ifconfig ppp0 down");
		HcuDebugPrint("UsbUmtsPppdSwShutdown: Bring down ppp0 via [ifconfig ppp0 down]\n");
	}

	/* Kill PPPd Process if there is any, so that the serial port could be freed */
	KillPppdProcess();
	HcuDebugPrint("UsbUmtsPppdSwShutdown: pppd process cleaned to free serial port for GPRS model.\n");

	/* refresh system information for network interface checking */
	HcuGetSysInfo(&gSysInfo);
	HcuDebugPrint("UsbUmtsPppdSwShutdown: refresh system information complete.\n");

	return SUCCESS;
}

UINT32 UsbUmtsHwReset()
{
	return SUCCESS;
}

//UINT32 UsbUmtsPppdSystemStartup()
//{
//
//	UINT32 ret;
//	UINT32 TimesGprsCheck = 0;
//	UINT32 TimesGprsReset = 0;
//
//	/* First Check whether there is already GPRS and PPP setup */
//	ret = IsGprsPppdOk();
//	if (SUCCESS == ret)
//	{
//		HcuDebugPrint("UsbUmtsPppdSystemStartup: GPRS & PPP already start, nothing done.\n");
//		return SUCCESS;
//	}
//	else /* if the Gprs is not working, try to shut down both of the network interface and pppd pprocess */
//	{
//		HcuDebugPrint("UsbUmtsPppdSystemStartup: GPRS & PPP not started, reset GPRS HW and shutdown PPPd and ppp0 interface.\n");
//		//UmtsHwReset(gpc); /* Reset GPRS Module by PGIO */
//		UsbUmtsPppdSwShutdown();  /* Clean the SW status */
//	}
//	/* after this, the GPRS model and Network interface and PPP process should be cleaned */
//
//	/* If GPRS/PPP not active, prepare to strat, wait for GPRS model to startup */
//	/* Initial GPRS */
//	while(1) // STRATEGY TO BE DEFINED !!!!!!!!!
//	{
//		ret = GprsInit(&gSerialPortGprs, &gGprsPppdConf);
//		if (SUCCESS == ret)
//		{
//			WipDebugPrint("GprsPppdSystemStartup: GPRS module ok, launch ppd ...\n");
//			break;
//		}
//		else
//		{
//			/* Increase the Check counter */
//			TimesGprsCheck++;
//			WipErrorPrint("GprsPppdSystemStartup: GPRS module nok, checked %d times, will recheck in 10 seconds ...\n", TimesGprsCheck);
//			if(TimesGprsCheck > gpc->MaxGprsCheckTimes)
//			{
//
//				TimesGprsReset++;
//
//				WipErrorPrint("GprsPppdSystemStartup: GPRS module nok total for %d times, max time reach, reset GPRS HW for %d times...\n", TimesGprsCheck, TimesGprsReset);
//				/* HW reset if many times of checking failed !!!!!!!! */
//				if (TimesGprsReset > gpc->MaxGprsResetTimes)
//				{
//					WipErrorPrint("GprsPppdSystemStartup: GPRS module reset for %d times, max time reach, STOP ...\n", TimesGprsReset);
//					return FAILURE;
//				}
//
//				TimesGprsCheck = 0;
//
//				/* reset the GPRS module */
//				GprsHwReset(gpc);
//			}
//		}
//
//		usleep(NB_MICROS_IN_ONE_SECOND * 10);
//	}
//
//	/* decide whether we need to start GPRS */
//	TimesGprsCheck = 0;
//	/* Start PPPD over GPRS */
//	while(1) // STRATEGY TO BE DEFINED !!!!!!!!!
//	{
//		ret = GprsPppdStart(gpc);
//		if (SUCCESS == ret)
//		{
//			WipDebugPrint("GprsPppdSystemStartup: Init GPRS PPP Success ...\n");
//			break;
//		}
//		else
//		{
//			TimesGprsCheck++;
//			if(TimesGprsCheck > gpc->MaxGprsCheckTimes)
//			{
//				WipErrorPrint("GprsPppdSystemStartup: Init GPRS PPP failure %d times, Max time reach, STOP ...\n", TimesGprsCheck);
//				return FAILURE;
//			}
//			WipErrorPrint("GprsPppdSystemStartup: Init GPRS PPP failure %d times, will kill pppd process, and retry in 10 seconds ...\n", TimesGprsCheck);
//			/* Kill all SW related PPP process and also the ppp0 interface */
//			GprsPppdSwShutdown();
//			//return ret;
//		}
//
//		usleep(NB_MICROS_IN_ONE_SECOND * 10);
//	}
//
//	/* Check whether it is OK */
//	ret = IsGprsPppdOk();
//	if (SUCCESS == ret)
//	{
//		WipDebugPrint("GprsPppdSystemStartup: GPRS & PPP start successfully.\n");
//		return SUCCESS;
//	}
//
//	/* NORMALLY SHOULD NOT STOP AT HERE */
//	HcuDebugPrint("GprsPppdSystemStartup: GPRS & PPP STATUS NOK EVEN AFTER GPRS & PPP START, NEED TO CHECK, STOP.\n");
//	return FAILURE;
//
//}

/*
 * Initialize the GPRS parameters from local configuration
 */
UINT32 UsbUmtsCardInit(char *devPath)
{

	//UINT8 id = pGprsSerialPortConfig->id;				/* COM1=>0, COM1=>1, COM2=>2 ....  */
	//UINT32 nSpeed = pGprsSerialPortConfig->nSpeed;		/* 1200, 2400, 4800, 4800, 9600, 19200, 38400, 57600, 115200 */
	//UINT16 nBits = pGprsSerialPortConfig->nBits;		/* 7 or 8 */
	//UINT8 nEvent = pGprsSerialPortConfig->nEvent;		/* '0', 'N', 'E' */
	//UINT16 nStop = pGprsSerialPortConfig->nStop;		/* 0, or 1 */
	//UINT16 fd = pGprsSerialPortConfig->fd;

	//FILE *fp;
	UINT16 fd;
	//UINT32 cnt = 0;
	UINT32 ret;
	//UINT32 len = 0;
	//UINT32 readlen = 0;
	int NumOfLine = 0;
	char buf[1024];
	char out[8][1024];

	char CurTime[18]; /* +CCLK: "13/06/01,23:43:06" -> 13/06/01,23:43:06 */
	/*  start from 8:    01234567890123456789012345                      */
	/*  len is 17:                                     12345678901234567 */
	CurTime[17] = 0x00;

	HcuDebugPrint("UsbUmtsCardInit: GPRS and PPPD not actived, checking GPRS module ...\n");

	fd = open(devPath, O_RDWR|O_NOCTTY|O_NDELAY);
	if (HCU_INVALID_U16 == fd)
	{
		HcuErrorPrint("UsbUmtsCardInit: Can't Open usb port [%s]\n", devPath);
		return FAILURE;
	}

	HcuDebugPrint("UsbUmtsCardInit: usb port [%s] open successfully, fd saved as %d.\n", devPath, fd);

	/* Send first AT Command */
	/* Auto band rate is not implemented, use AT Command before Run HCU, Set: AT+IPR=<rate>, Get: AT+IPR? */
	ret = UsbUmtsAtCommand(fd, "AT\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, make sure BandRate is correct, via AT+IPR=<rate> firstly, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("UsbUmtsCardInit: GPRS Modem Ready.\n");
	}
	else
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	/* Send first Check the GPRS Model Version */
	ret = UsbUmtsAtCommand(fd, "AT+CGMR\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, Serial Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("UsbUmtsCardInit: GPRS Modem HW/SW verion read ok.\n");
	}
	else
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	ret = UsbUmtsAtCommand(fd, "AT+CPIN?\r", *out, &NumOfLine);
	if (ret == FAILURE)
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("UsbUmtsCardInit: GPRS SIM CARD OK\n");
	}
	else
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}


	ret = UsbUmtsAtCommand(fd, "AT+CREG?\r", *out, &NumOfLine);

	if (ret == FAILURE)
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		HcuDebugPrint("UsbUmtsCardInit: GPRS SIM CARD REGISTERED TO NETWORK\n");
	}
	else
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	/* Get GPRS system time */
	ret = UsbUmtsAtCommand(fd, "AT+CCLK?\r", *out, &NumOfLine);

	if (ret == FAILURE)
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS Modem is not ready, usb Port closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	if(0 == strcmp(out[NumOfLine-1], "OK"))
	{
		/* Get the time string from GPRS module */
		strncpy(CurTime, out[NumOfLine-3]+8, 17);
		HcuDebugPrint("UsbUmtsCardInit: GPRS READ Realtime Clock OK: %s\n", CurTime);
		if(HCU_LOGIC_TRUE)
		{
			SetSystemTime(CurTime);
		}
	}
	else
	{
		HcuErrorPrint("UsbUmtsCardInit: GPRS READ Realtime Clock failure, Usb Port COM%d closed ...\n");
		SerialPortClose(fd);
		return FAILURE;
	}

	/* return successfully */
	SerialPortClose(fd);
	HcuDebugPrint("UsbUmtsCardInit: GPRS status checking OK.\n");

	/* PPP Configuration File */
	/* "/etc/ppp/peers/gprs" */

//	fp = fopen(gpc->PppConfigFileName, "r");
//	if (NULL == fp)
//	{
//		HcuErrorPrint("UsbUmtsCardInit: GPRS scripts [%s] open error ...\n", gpc->PppConfigFileName);
//		return FAILURE;
//	}
//
//	while(!feof(fp))
//	{
//		fgets(buf, 1023, fp);
//		HcuDebugPrint("UsbUmtsCardInit: GPRS scripts [%s] line #%02d: %s", gpc->PppConfigFileName, cnt, buf);
//		cnt++;
//	}
//
//	HcuDebugPrint("UsbUmtsCardInit: GPRS scripts [%s] not changed for this version of the HCU ...\n", gpc->PppConfigFileName);

	/* PPP Dail Command */
	/* "/usr/bin/pppd" */
	/* Check local HCU configuration table for GPRS port */
	/* Apply these configuration in the OS's scripts */

	return SUCCESS;

}



/*
 * Send one AT Command, and wait for reply from the serails port
 *
 * In  : pGprsSerialPortConfig: Serial Port information, include fdl
 * In  : AtCmd: AT Command send to Modem
 * Out : ReplyStr: Each of the line of the AT command is save in ReplyStr[0], ReplyStr[1*1024], ReplyStr[(ReplayCnt-1)*1024]
 * Out : ReplyCnt: number of line expected from Modem (>=1)
 *
 */

UINT32 UsbUmtsAtCommand(UINT16 fd, char *AtCmd, char *ReplyStr, int *ReplyCnt)
{

	UINT32 cnt = 0;
	UINT32 i = 0;
	UINT32 len = 0;
	UINT32 readlen = 0;
	int NumOfLine = 0;
	UINT32 IdxOfCharInOneLine = 0;
	//UINT16 fd = pGprsSerialPortConfig->fd;
	char buf[1024];


	if(HCU_INVALID_U16 == fd)
	{
		HcuErrorPrint("UsbUmtsAtCommand: fd = 0xFFFF, usb port not openned, return.\n");
		return FAILURE;
	}

	if(NULL == AtCmd)
	{
		HcuErrorPrint("UsbUmtsAtCommand: Input pointer AtCmd = NULL, return.\n");
		return FAILURE;
	}

	if(NULL == ReplyStr)
	{
		HcuErrorPrint("UsbUmtsAtCommand: Output pointer ReplyStr = NULL, return.\n");
		return FAILURE;
	}

	/* Send second AT Command */
	bzero(buf, 1024);
	len = write(fd, AtCmd, strlen(AtCmd));
	HcuDebugPrint("UsbUmtsAtCommand: AT SEND: %s\n", AtCmd);

	usleep(NB_MICROS_IN_ONE_SECOND * 2);
	HcuDebugPrint("UsbUmtsAtCommand: Delayed 2 sec to get response ...\n");

	/* Read All Reply */
	readlen = read(fd, buf, 1023);
	if (-1 == readlen)
	{
		HcuDebugPrint("UsbUmtsAtCommand: read usb port (fd = %d) failure...\n", fd);
		return FAILURE;
	}
	buf[readlen] = 0;
	HcuDebugPrint("UsbUmtsAtCommand: RECV(HEX)(Len = %d): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", readlen, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], readlen);

	if(0 == readlen)
	{
		*ReplyCnt = NumOfLine;
		return SUCCESS;
	}

	for(cnt = 0; cnt < readlen; cnt++)
	{
		if( (0x0D == buf[cnt]) && (0x0A == buf[cnt+1]) )
		{

			/* End of the current line via \0 */
			ReplyStr[(NumOfLine * 1024) + IdxOfCharInOneLine] = 0x00;

			/* Point to next line */
			NumOfLine = NumOfLine + 1;

			if(8 == NumOfLine) break;

			/* reset index of char in line to 0 */
			IdxOfCharInOneLine = 0;

			/* skip to cha after 0x0D 0x0A */
			cnt = cnt + 1;
		}
		else
		{
			/* Copy each of the char */
			ReplyStr[(NumOfLine * 1024) + IdxOfCharInOneLine] = buf[cnt];
			IdxOfCharInOneLine = IdxOfCharInOneLine + 1;
		}

	}

	for(i = 0; i<NumOfLine; i++)
	{
		HcuDebugPrint("UsbUmtsAtCommand: AT RECV (Line #%02d): %s\n", i, (ReplyStr + (i * 1024)));
	}

	*ReplyCnt = NumOfLine;

	return SUCCESS;

}


OPSTAT hcu_3g4g_phy_link_setup(void)
{
	//MYC: Start to initialized USB UMTS CARD
	//暂时出现错误，需要MYC仔细调试，故而先注销，等待调试好了再打开
	//UsbUmtsPppInit();
	//UsbUmtsPppdStart();
	return SUCCESS;
}

OPSTAT hcu_3g4g_phy_link_disconnect(void)
{
	return SUCCESS;
}
