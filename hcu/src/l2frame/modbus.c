/*
 * modbus.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "modbus.h"
#include "../comvm/trace.h"
#include "../comvm/timer.h"
#include "../l2frame/l2comdef.h"

/*
** FSM of the MODBUS
*/
FsmStateItem_t FsmModbus[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_modbus_task_entry}, //Starting

	//System level initialization
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_modbus_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_modbus_restart},

	//Task level initialization
    {MSG_ID_COM_INIT,       	FSM_STATE_MODBUS_INITED,            	fsm_modbus_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_MODBUS_INITED,            	fsm_modbus_restart},

    //Active working mode, no state transfer
    {MSG_ID_COM_RESTART,        			FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_restart},
    {MSG_ID_EMC_MODBUS_DATA_READ,       	FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_emc_data_read},
    {MSG_ID_PM25_MODBUS_DATA_READ,       	FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_pm25_data_read},
    {MSG_ID_WINDDIR_MODBUS_DATA_READ,		FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_winddir_data_read},
    {MSG_ID_WINDSPD_MODBUS_DATA_READ,   	FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_windspd_data_read},
    {MSG_ID_TEMP_MODBUS_DATA_READ,   		FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_temp_data_read},
    {MSG_ID_HUMID_MODBUS_DATA_READ,      	FSM_STATE_MODBUS_ACTIVED,   fsm_modbus_humid_data_read},

	//Active-wait-for-feedback，该状态暂时只是为了状态协议的完整性，485还永不到
	//485接口采用了接收非阻塞式设计方式，232则采用了阻塞式。485模块的阻塞式任务模块只是装样子，由于MODBUS协议本身反馈的周期较短，这种方式暂时够用
	//232的RX程序任务模块框架采用了阻塞式，但由于RS232还没有距离应用，所以也没起作用
	//所有传感器接口（232/485/BLE/USB-CAMERA/I2S-AUDIO）均直接采用API发送，接受采用阻塞式任务框架
	//所有后台接口（ETHERNET/WIFI/USB-OTG/3G4G）也是发送TX直接API，但RX采用阻塞式任务框架
    {MSG_ID_COM_RESTART,        		FSM_STATE_MODBUS_ACTIVED_WFFB, 	fsm_modbus_restart},
	{MSG_ID_SPS232_MODBUS_DATA_RX,      FSM_STATE_MODBUS_ACTIVED_WFFB,  fsm_modbus_sps232_data_rx},
	{MSG_ID_SPS485_MODBUS_DATA_RX,      FSM_STATE_MODBUS_ACTIVED_WFFB,  fsm_modbus_sps485_data_rx},
	//BLE更有可能用在低速接口，用于被类似于MODBUS帧协议控制
	{MSG_ID_BLE_MODBUS_DATA_RX,      	FSM_STATE_MODBUS_ACTIVED_WFFB,  fsm_modbus_ble_data_rx},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Task level global variables，该任务是单入的，所以两个传感器同时操作是不可以的
UINT32 currentSensorEqpId;  //当前正在工作的传感器
SerialModbusMsgBuf_t currentModbusBuf;
SerialPort_t gSerialPort = {COM_PORT_2, 9600, 8, 'N', 1, HCU_INVALID_U16, 0, 1, 0};//initial config date for serial port
//MYC 2015/12/08, For 335xD, RS485 is ttyO2 !!!

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_modbus_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret;
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	ret = FsmSetState(TASK_ID_MODBUS, FSM_STATE_IDLE);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error Set FSM State at fsm_modbus_task_entry\n");
		return FAILURE;
	}
	return SUCCESS;
}

OPSTAT fsm_modbus_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;
	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd;
	memset(&snd, 0, sizeof(msg_struct_com_init_feedback_t));
	snd.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}
	ret = FsmSetState(TASK_ID_MODBUS, FSM_STATE_MODBUS_INITED);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error Set FSM State at fsm_modbus_init!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("MODBUS: Enter FSM_STATE_MODBUS_INITED status, everything goes well!\n");
	#endif //(TRACE_DEBUG_ON)

	//Init global variables
	currentSensorEqpId = 0;
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));

	//基本上不设置状态机，所有操作均为同步式，这样就不需要状态机了
	ret = FsmSetState(TASK_ID_MODBUS, FSM_STATE_MODBUS_ACTIVED);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error Set FSM State at fsm_modbus_init!\n");
		return FAILURE;
	}

	return SUCCESS;
}

OPSTAT fsm_modbus_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT fsm_modbus_emc_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_emc_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_emc_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_emc_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_emc))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_emc_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Preparing send modbus EMC req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON

	//串口不总是好的，而且也没有EMC外设，所以EMC保持为假数据，以便测试后台
	/*
	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 7);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}

	else
	{
	  HcuDebugPrint("MODBUS: Send EMC req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}
	*/

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	/*
	//从相应的从设备中读取数据
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, MAX_HCU_MSG_BODY_LENGTH);//获得的数据存在currentModbusBuf中
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Len %d \n ", ret);
	 HcuDebugPrint("MODBUS: Received EMC data succeed: %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d", ret);
	  return FAILURE;
	}
	*/

	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_emc_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_emc_data_report_t));

	//放点假数据进行测试
	currentModbusBuf.curLen = 7;
	UINT8 sample[] = {0x05,0x03,0x02,0x12,0x34,0x44,0xF3};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	if (func_modbus_emc_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}
	/*
	currentModbusBuf.curLen = ret;
	if (func_modbus_emc_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}*/

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_emc_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_emc_data_req:
		snd.useroptid = L3PO_emc_data_report;
		break;
	case L3PO_emc_set_switch:
		snd.useroptid = L3PO_emc_read_switch;
		break;
	case L3PO_emc_set_modbus_address:
		snd.useroptid = L3PO_emc_read_modbus_address;
		break;
	case L3PO_emc_set_work_cycle:
		snd.useroptid = L3PO_emc_read_work_cycle;
		break;
	case L3PO_emc_set_sample_cycle:
		snd.useroptid = L3PO_emc_read_sample_cycle;
		break;
	case L3PO_emc_set_sample_number:
		snd.useroptid = L3PO_emc_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_EMC_DATA_REPORT, TASK_ID_EMC, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_EMC]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}

//需要一大堆标准化的函数：便解码，CRC16等等
//这个整体上可以做成一个大函数，供所有的不同传感器公用，而不用每一种单独搞一个不同的函数
OPSTAT fsm_modbus_pm25_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_pm25_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_pm25_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_pm25_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_pm25))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_pm25_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Preparing send modbus PM25 req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON
	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 17);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}

	else
	{
	  HcuDebugPrint("MODBUS: Send PM25 data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	//从相应的从设备中读取数据
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, MAX_HCU_MSG_BODY_LENGTH);//获得的数据存在currentModbusBuf中
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Len %d  \n", ret);
	 HcuDebugPrint("MODBUS: Received PM2.5 data succeed: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7],currentModbusBuf.curBuf[8],currentModbusBuf.curBuf[9],currentModbusBuf.curBuf[10],currentModbusBuf.curBuf[11],currentModbusBuf.curBuf[12],currentModbusBuf.curBuf[13],currentModbusBuf.curBuf[14],currentModbusBuf.curBuf[15],currentModbusBuf.curBuf[16]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d", ret);
	  return FAILURE;
	}
	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_pm25_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_pm25_data_report_t));

	/*
	//放点假数据进行测试
	currentModbusBuf.curLen = 17;
	UINT8 sample[] = {0x01,0x03,0x0C,0x01,0x02,0x03,0x04,0x11,0x12,0x13,0x14,
			0x21,0x22,0x23,0x24,0xD4,0xB8};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	*/

	currentModbusBuf.curLen = ret;
	if (func_modbus_pm25_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_pm25_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_pm25_data_req:
		snd.useroptid = L3PO_pm25_data_report;
		break;
	case L3PO_pm25_set_switch:
		snd.useroptid = L3PO_pm25_read_switch;
		break;
	case L3PO_pm25_set_modbus_address:
		snd.useroptid = L3PO_pm25_read_modbus_address;
		break;
	case L3PO_pm25_set_work_cycle:
		snd.useroptid = L3PO_pm25_read_work_cycle;
		break;
	case L3PO_pm25_set_sample_cycle:
		snd.useroptid = L3PO_pm25_read_sample_cycle;
		break;
	case L3PO_pm25_set_sample_number:
		snd.useroptid = L3PO_pm25_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_PM25_DATA_REPORT, TASK_ID_PM25, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_PM25]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}
OPSTAT fsm_modbus_winddir_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_winddir_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_winddir_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_winddir_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_winddir))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_winddir_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Prepareing send modbus winddir req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON

	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 7);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}

	else
	{
	  HcuDebugPrint("MODBUS: Send  winddir req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	//从相应的从设备中读取数据
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, MAX_HCU_MSG_BODY_LENGTH); //获得的数据存在currentModbusBuf中
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Len %d  \n", ret);
	 HcuDebugPrint("MODBUS: Received windir data succeed: %02X %02X %02X %02X %02X %02X %02X \n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d \n", ret);
	  return FAILURE;
	}

	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_winddir_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_winddir_data_report_t));

	/*
	//放点假数据进行测试
	currentModbusBuf.curLen = 7;
	UINT8 sample[] = {0x03,0x03,0x02,0x12,0x34,0xCC,0xF3};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	*/

	currentModbusBuf.curLen =ret;
	if (func_modbus_winddir_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_winddir_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_winddir_data_req:
		snd.useroptid = L3PO_winddir_data_report;
		break;
	case L3PO_winddir_set_switch:
		snd.useroptid = L3PO_winddir_read_switch;
		break;
	case L3PO_winddir_set_modbus_address:
		snd.useroptid = L3PO_winddir_read_modbus_address;
		break;
	case L3PO_winddir_set_work_cycle:
		snd.useroptid = L3PO_winddir_read_work_cycle;
		break;
	case L3PO_winddir_set_sample_cycle:
		snd.useroptid = L3PO_winddir_read_sample_cycle;
		break;
	case L3PO_winddir_set_sample_number:
		snd.useroptid = L3PO_winddir_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_WINDDIR_DATA_REPORT, TASK_ID_WINDDIR, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_WINDDIR]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}
OPSTAT fsm_modbus_windspd_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_windspd_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_windspd_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_windspd_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_windspd))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_windspd_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Preparing send modbus windspd req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON

	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 7);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}
	else
	{
	  HcuDebugPrint("MODBUS: Send windspd req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	//从相应的从设备中读取数据
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, MAX_HCU_MSG_BODY_LENGTH);
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Len %d  \n", ret);
	 HcuDebugPrint("MODBUS: Received windspd data succeed %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d \n",  ret);
	  return FAILURE;
	}

	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_windspd_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_windspd_data_report_t));

	/*
	//放点假数据进行测试
	currentModbusBuf.curLen = 7;
	UINT8 sample[] = {0x02,0x03,0x02,0x12,0x34,0xF1,0x33};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	*/

	currentModbusBuf.curLen = ret;
	if (func_modbus_windspd_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_windspd_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_windspd_data_req:
		snd.useroptid = L3PO_windspd_data_report;
		break;
	case L3PO_windspd_set_switch:
		snd.useroptid = L3PO_windspd_read_switch;
		break;
	case L3PO_windspd_set_modbus_address:
		snd.useroptid = L3PO_windspd_read_modbus_address;
		break;
	case L3PO_windspd_set_work_cycle:
		snd.useroptid = L3PO_windspd_read_work_cycle;
		break;
	case L3PO_windspd_set_sample_cycle:
		snd.useroptid = L3PO_windspd_read_sample_cycle;
		break;
	case L3PO_windspd_set_sample_number:
		snd.useroptid = L3PO_windspd_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_WINDSPD_DATA_REPORT, TASK_ID_WINDSPD, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_WINDSPD]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}
OPSTAT fsm_modbus_temp_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_temp_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_temp_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_temp_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_temp))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_temp_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Preparing send modbus temp req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON

	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 9);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}

	else
	{
	  HcuDebugPrint("MODBUS: Send temp req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	//从相应的从设备中读取数据

	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, 9); //获得的数据存在currentModbusBuf中
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Received temp data length: %d \n ", ret);
	 HcuDebugPrint("MODBUS: Received temp data content: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7],currentModbusBuf.curBuf[8]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d \n", ret);
	  return FAILURE;
	}

	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_temp_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_temp_data_report_t));

	/*
	//放点假数据进行测试
	currentModbusBuf.curLen = 9;
	UINT8 sample[] = {0x06,0x03,0x04,0x12,0x34,0x56,0x78,0xF7,0xC7};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	*/

	currentModbusBuf.curLen = ret;
	if (func_modbus_temp_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_temp_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_temp_data_req:
		snd.useroptid = L3PO_temp_data_report;
		break;
	case L3PO_temp_set_switch:
		snd.useroptid = L3PO_temp_read_switch;
		break;
	case L3PO_temp_set_modbus_address:
		snd.useroptid = L3PO_temp_read_modbus_address;
		break;
	case L3PO_temp_set_work_cycle:
		snd.useroptid = L3PO_temp_read_work_cycle;
		break;
	case L3PO_temp_set_sample_cycle:
		snd.useroptid = L3PO_temp_read_sample_cycle;
		break;
	case L3PO_temp_set_sample_number:
		snd.useroptid = L3PO_temp_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_TEMP_DATA_REPORT, TASK_ID_TEMP, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_TEMP]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}
OPSTAT fsm_modbus_humid_data_read(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//消息参数检查
	msg_struct_humid_modbus_data_read_t rcv;
	memset(&rcv, 0, sizeof(msg_struct_humid_modbus_data_read_t));
	if ((param_ptr == NULL || param_len > sizeof(msg_struct_humid_modbus_data_read_t))){
		HcuErrorPrint("MODBUS: Receive message error!\n");
		return FAILURE;
	}
	memcpy(&rcv, param_ptr, param_len);
	currentSensorEqpId = rcv.equId;

	//Equipment Id can not be 0
	if ((currentSensorEqpId <=0) || (rcv.cmdId != L3CI_humid))
		return FAILURE;

	//只从MODBUS语法角度检查收到的命令的正确性，逻辑正确性由上层任务保证

	//对信息进行MODBUS协议的编码，包括CRC16的生成
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = func_modbus_humid_msg_pack(&rcv, &currentModbusBuf);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Error pack message!\n");
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	HcuDebugPrint("MODBUS: Preparing send modbus humid req data = %02X %02x %02X %02X %02X %02X %02X %02X\n", currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	#endif //TRACE_DEBUG_ON

	//Init Serial Port
	ret = hcu_sps485_serial_init(&gSerialPort);
	if (FAILURE == ret)
	{
	HcuErrorPrint("MODBUS: Init Serial Port Failure, Exit.\n");
	return ret;
	}
	else
	{
	HcuDebugPrint("MODBUS: Init Serial Port Success ...\n");
	}

	SerialPortSetVtimeVmin(&gSerialPort, 0, 9);
	HcuDebugPrint("MODBUS: COM port flags: VTIME = 0x%d, TMIN = 0x%d\n",  gSerialPort.vTime, gSerialPort.vMin);

	ret = hcu_sps485_serial_port_send(&gSerialPort, currentModbusBuf.curBuf, currentModbusBuf.curLen);

	if (FAILURE == ret)
	{
	  HcuErrorPrint("MODBUS: Error send command to serials port!\n");
	}
	else
	{
	  HcuDebugPrint("MODBUS: Send humuid req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7]);
	}

	hcu_usleep(50); //经典的操作，需要50ms的延迟，确保安全，该休眠不会被打断

	//从相应的从设备中读取数据
	memset(&currentModbusBuf, 0, sizeof(SerialModbusMsgBuf_t));
	ret = hcu_sps485_serial_port_get(&gSerialPort, currentModbusBuf.curBuf, MAX_HCU_MSG_BODY_LENGTH);//获得的数据存在currentModbusBuf中
	if (ret > 0)
	{
	 HcuDebugPrint("MODBUS: Len %d\n", ret);
	 HcuDebugPrint("MODBUS: Received humuid req data succeed: %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",currentModbusBuf.curBuf[0],currentModbusBuf.curBuf[1],currentModbusBuf.curBuf[2],currentModbusBuf.curBuf[3],currentModbusBuf.curBuf[4],currentModbusBuf.curBuf[5],currentModbusBuf.curBuf[6],currentModbusBuf.curBuf[7],currentModbusBuf.curBuf[8]);
	}
	else
	{
	  HcuDebugPrint("MODBUS: Can not read data from serial port, return of read %d \n", ret);
	  return FAILURE;
	}

	//对信息进行MODBUS协议的解码，包括CRC16的判断
	msg_struct_modbus_humid_data_report_t snd;
	memset(&snd, 0, sizeof(msg_struct_modbus_humid_data_report_t));

	/*
	//放点假数据进行测试
	currentModbusBuf.curLen = 9;
	UINT8 sample[] = {0x06,0x03,0x04,0x12,0x34,0x56,0x78,0xF7,0xC7};
	memcpy(currentModbusBuf.curBuf, sample, currentModbusBuf.curLen);
	*/

	currentModbusBuf.curLen = ret;
	if (func_modbus_humid_msg_unpack(&currentModbusBuf, &rcv, &snd) == FAILURE){
		HcuErrorPrint("MODBUS: Error unpack message!\n");
		return FAILURE;
	}

	//检查下equipmentId，确保没重入
	if (snd.equipid != currentSensorEqpId){
		HcuErrorPrint("MODBUS: Re-enter modbus operation by equpId used!\n");
		return FAILURE;
	}
	//将读取的数据回送给传感器控制器
	snd.length = sizeof(msg_struct_modbus_humid_data_report_t);
	snd.usercmdid = rcv.cmdId;
	snd.cmdIdBackType = rcv.cmdIdBackType;
	snd.timeStamp = time(0);

	switch(rcv.optId){
	case L3PO_humid_data_req:
		snd.useroptid = L3PO_humid_data_report;
		break;
	case L3PO_humid_set_switch:
		snd.useroptid = L3PO_humid_read_switch;
		break;
	case L3PO_humid_set_modbus_address:
		snd.useroptid = L3PO_humid_read_modbus_address;
		break;
	case L3PO_humid_set_work_cycle:
		snd.useroptid = L3PO_humid_read_work_cycle;
		break;
	case L3PO_humid_set_sample_cycle:
		snd.useroptid = L3PO_humid_read_sample_cycle;
		break;
	case L3PO_humid_set_sample_number:
		snd.useroptid = L3PO_humid_read_sample_number;
		break;
	default:
		HcuErrorPrint("MODBUS: Error operation code received!\n");
		return FAILURE;
		break;
	}

	snd.gps_x = 444;
	snd.gps_y = 555;
	snd.gps_z = 666;

	//Remaining data to be filled
	ret = hcu_message_send(MSG_ID_MODBUS_HUMID_DATA_REPORT, TASK_ID_HUMID, TASK_ID_MODBUS, &snd, snd.length);
	if (ret == FAILURE){
		HcuErrorPrint("MODBUS: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_MODBUS], zHcuTaskNameList[TASK_ID_HUMID]);
		return FAILURE;
	}

	//恢复当前传感器ID的初始值
	currentSensorEqpId = 0;
	return SUCCESS;
}



OPSTAT func_modbus_emc_msg_pack(msg_struct_emc_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_emc_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((EMC_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(EMC_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((EMC_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(EMC_LENGTH_OF_REG & 0x0FF); //长度低位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_emc_set_switch:
		break;

	case L3PO_emc_set_modbus_address:
		break;

	case L3PO_emc_set_work_cycle:
		break;

	case L3PO_emc_set_sample_cycle:
		break;

	case L3PO_emc_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_emc_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_emc_modbus_data_read_t *rcv, msg_struct_modbus_emc_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_emc_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != EMC_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		snd->emcValue = t0 + t1;
		break;

	case L3PO_emc_set_switch:
		break;

	case L3PO_emc_set_modbus_address:
		break;

	case L3PO_emc_set_work_cycle:
		break;

	case L3PO_emc_set_sample_cycle:
		break;

	case L3PO_emc_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}

OPSTAT func_modbus_pm25_msg_pack(msg_struct_pm25_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_pm25_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((PM25_REG_DATA_PM1D0  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(PM25_REG_DATA_PM1D0 & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((PM25_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 3个寄存器，6B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(PM25_LENGTH_OF_REG & 0x0FF); //长度低位 = 3个寄存器，6B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_pm25_set_switch:
		break;

	case L3PO_pm25_set_modbus_address:
		break;

	case L3PO_pm25_set_work_cycle:
		break;

	case L3PO_pm25_set_sample_cycle:
		break;

	case L3PO_pm25_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;

	return SUCCESS;
}

OPSTAT func_modbus_pm25_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_pm25_modbus_data_read_t *rcv, msg_struct_modbus_pm25_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0, t2=0, t3=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_pm25_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != PM25_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t2 = buf->curBuf[index++];
		t3 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		t2 = (t2 <<24) & 0xFF000000;
		t3 = (t3 << 16) & 0xFF0000;
		snd->pm1d0Value = t0 + t1 + t2 + t3;
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t2 = buf->curBuf[index++];
		t3 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		t2 = (t2 <<24) & 0xFF000000;
		t3 = (t3 << 16) & 0xFF0000;
		snd->pm2d5Value = t0 + t1 + t2 + t3;
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t2 = buf->curBuf[index++];
		t3 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		t2 = (t2 <<24) & 0xFF000000;
		t3 = (t3 << 16) & 0xFF0000;
		snd->pm10Value = t0 + t1 + t2 + t3;
		break;

	case L3PO_pm25_set_switch:
		break;

	case L3PO_pm25_set_modbus_address:
		break;

	case L3PO_pm25_set_work_cycle:
		break;

	case L3PO_pm25_set_sample_cycle:
		break;

	case L3PO_pm25_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}

OPSTAT func_modbus_winddir_msg_pack(msg_struct_winddir_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_winddir_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((WINDDIR_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(WINDDIR_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((WINDDIR_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(WINDDIR_LENGTH_OF_REG & 0x0FF); //长度低位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_winddir_set_switch:
		break;

	case L3PO_winddir_set_modbus_address:
		break;

	case L3PO_winddir_set_work_cycle:
		break;

	case L3PO_winddir_set_sample_cycle:
		break;

	case L3PO_winddir_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_winddir_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_winddir_modbus_data_read_t *rcv, msg_struct_modbus_winddir_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_winddir_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != WINDDIR_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		snd->winddirValue = t0 + t1;
		break;

	case L3PO_winddir_set_switch:
		break;

	case L3PO_winddir_set_modbus_address:
		break;

	case L3PO_winddir_set_work_cycle:
		break;

	case L3PO_winddir_set_sample_cycle:
		break;

	case L3PO_winddir_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}

OPSTAT func_modbus_windspd_msg_pack(msg_struct_windspd_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_windspd_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((WINDSPD_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(WINDSPD_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((WINDSPD_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(WINDSPD_LENGTH_OF_REG & 0x0FF); //长度低位 = 1个寄存器，2B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_windspd_set_switch:
		break;

	case L3PO_windspd_set_modbus_address:
		break;

	case L3PO_windspd_set_work_cycle:
		break;

	case L3PO_windspd_set_sample_cycle:
		break;

	case L3PO_windspd_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_windspd_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_windspd_modbus_data_read_t *rcv, msg_struct_modbus_windspd_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_windspd_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != WINDSPD_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		snd->windspdValue = t0 + t1;
		break;

	case L3PO_windspd_set_switch:
		break;

	case L3PO_windspd_set_modbus_address:
		break;

	case L3PO_windspd_set_work_cycle:
		break;

	case L3PO_windspd_set_sample_cycle:
		break;

	case L3PO_windspd_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}

OPSTAT func_modbus_temp_msg_pack(msg_struct_temp_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_temp_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((TEMP_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(TEMP_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((TEMP_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(TEMP_LENGTH_OF_REG & 0x0FF); //长度低位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_temp_set_switch:
		break;

	case L3PO_temp_set_modbus_address:
		break;

	case L3PO_temp_set_work_cycle:
		break;

	case L3PO_temp_set_sample_cycle:
		break;

	case L3PO_temp_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_temp_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_temp_modbus_data_read_t *rcv, msg_struct_modbus_temp_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_temp_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != TEMP_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		t0 = buf->curBuf[index++];  //前两个数据是温度，后面的2个数据是湿度
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		snd->tempValue = t0 + t1;
		break;

	case L3PO_temp_set_switch:
		break;

	case L3PO_temp_set_modbus_address:
		break;

	case L3PO_temp_set_work_cycle:
		break;

	case L3PO_temp_set_sample_cycle:
		break;

	case L3PO_temp_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}

OPSTAT func_modbus_humid_msg_pack(msg_struct_humid_modbus_data_read_t *inMsg, SerialModbusMsgBuf_t *outMsg)
{
	//参数不再做详细的检查，因为上层调用者已经做过严格的检查了
	//取得设备地址
	if (inMsg->equId > 0xFF) return FAILURE;
	outMsg->curBuf[outMsg->curLen] = (UINT8)(inMsg->equId & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//取得功能码字，目前这是唯一支持的操作命令码字
	outMsg->curBuf[outMsg->curLen] = (UINT8)(MODBUS_GENERIC_FUNC_DATA_INQUERY & 0x0FF);
	outMsg->curLen = outMsg->curLen + 1;

	//根据不同的操作码字OPT，进行分支操作
	switch(inMsg->optId){
	case L3PO_humid_data_req:
		//取得寄存器地址
		outMsg->curBuf[outMsg->curLen] = (UINT8)((HUMID_REG_DATA_READ  >> 8) & 0x0FF); //高位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(HUMID_REG_DATA_READ & 0x0FF); //低位
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)((HUMID_LENGTH_OF_REG >> 8) & 0x0FF) ; //长度高位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		outMsg->curBuf[outMsg->curLen] = (UINT8)(HUMID_LENGTH_OF_REG & 0x0FF); //长度低位 = 2个寄存器，4B长度
		outMsg->curLen = outMsg->curLen + 1;
		break;

	case L3PO_humid_set_switch:
		break;

	case L3PO_humid_set_modbus_address:
		break;

	case L3PO_humid_set_work_cycle:
		break;

	case L3PO_humid_set_sample_cycle:
		break;

	case L3PO_humid_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg pack!\n");
		return FAILURE;
		break;
	}

	//增加CRC16-2B，高位在前，低位在后，但下面的函数本身得出的结果是低位在前，高位在后，晕乎吧
	UINT16 crc16=0;
	CheckCRCModBus(outMsg->curBuf, outMsg->curLen, &crc16);
	outMsg->curBuf[outMsg->curLen] = (UINT8)(crc16 & 0x0FF); //高位字节
	outMsg->curLen = outMsg->curLen + 1;
	outMsg->curBuf[outMsg->curLen] = (UINT8)((crc16 >> 8) & 0x0FF); //低位字节
	outMsg->curLen = outMsg->curLen + 1;
	return SUCCESS;
}

OPSTAT func_modbus_humid_msg_unpack(SerialModbusMsgBuf_t *buf, msg_struct_humid_modbus_data_read_t *rcv, msg_struct_modbus_humid_data_report_t *snd)
{
	UINT32 index=0;
	UINT16 crc16_orin=0, crc16_gen=0;
	UINT32 len=0, t0=0, t1=0;

	//检查长度
	if ((buf->curLen<=0) || (buf->curLen>MAX_HCU_MSG_BODY_LENGTH)){
		HcuErrorPrint("MODBUS: Receive Modbus data error with length = %d\n", buf->curLen);
		return FAILURE;
	}

	//检查地址码
	if (buf->curBuf[index] != rcv->equId){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	snd->equipid = buf->curBuf[index];
	index++;

	//检查功能码=03
	if (buf->curBuf[index] != MODBUS_GENERIC_FUNC_DATA_INQUERY){
		HcuErrorPrint("MODBUS: Receive Modbus data error with EquId = %d\n", buf->curBuf[index]);
		return FAILURE;
	}
	index++;

	//检查CRC16
	crc16_orin = buf->curBuf[buf->curLen-1];
	crc16_orin = (crc16_orin <<8)+ buf->curBuf[buf->curLen-2];
	CheckCRCModBus(buf->curBuf, buf->curLen-2, &crc16_gen);
	if (crc16_orin != crc16_gen){
		HcuErrorPrint("MODBUS: Receive Modbus data error with CRC16 check!\n");
		return FAILURE;
	}

	//根据不同的操作码字OPT，进行解码分支操作
	switch(rcv->optId){
	case L3PO_humid_data_req:
		len = buf->curBuf[index];
		index++;
		if (len != HUMID_LENGTH_OF_REG *2){
			HcuErrorPrint("MODBUS: Receive Modbus data error with data length!\n");
			return FAILURE;
		}
		index = index+2;  //前两个数据是温度，后面的2个数据是湿度
		t0 = buf->curBuf[index++];
		t1 = buf->curBuf[index++];
		t0 = (t0 <<8) & 0xFF00;
		t1 = t1 & 0xFF;
		snd->humidValue = t0 + t1;
		break;

	case L3PO_humid_set_switch:
		break;

	case L3PO_humid_set_modbus_address:
		break;

	case L3PO_humid_set_work_cycle:
		break;

	case L3PO_humid_set_sample_cycle:
		break;

	case L3PO_humid_set_sample_number:
		break;

	default:
		HcuErrorPrint("MODBUS: Error cmId par received during msg unpack!\n");
		return FAILURE;
		break;
	}

	return SUCCESS;
}



//CRC16计算方法
static UINT16 wCRC16Table[256] = {
		0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
		0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
		0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
		0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
		0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
		0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
		0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
		0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
		0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
		0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
		0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
		0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
		0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
		0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
		0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
		0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
		0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
		0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
		0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
		0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
		0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
		0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
		0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
		0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
		0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
		0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
		0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
		0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
		0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
		0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
		0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
		0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

//////////////////////////////////////////////////////////////////////////
// CRC MODBUS 效验
// 输入参数: pDataIn: 数据地址
//           iLenIn: 数据长度
// 输出参数: pCRCOut: 2字节校验值
void CheckCRCModBus(const UINT8* pDataIn, int iLenIn, UINT16* pCRCOut)
{
	UINT16 wHi = 0;
	UINT16 wLo = 0;
	UINT16 wCRC;
    wCRC = 0xFFFF;
    int i=0;

    for (i=0;i<iLenIn;i++)
    {
        wCRC = CalcCRCModBus((UINT8)*pDataIn, wCRC);
        pDataIn++;
    }

    wHi = wCRC / 256;
    wLo = wCRC % 256;
    wCRC = (wHi << 8) | wLo;

    *pCRCOut = wCRC;
}

UINT16 CalcCRCModBus(UINT8 cDataIn, UINT16 wCRCIn)
{
    UINT16 wCheck = 0;
    wCRCIn = wCRCIn ^ cDataIn;
    int i=0;

    for(i=0;i<8;i++)
    {
        wCheck = wCRCIn & 1;
        wCRCIn = wCRCIn >> 1;
        wCRCIn = wCRCIn & 0x7fff;

        if(wCheck == 1)
        {
            wCRCIn = wCRCIn ^ 0xa001;
        }
        wCRCIn = wCRCIn & 0xffff;
    }

    return wCRCIn;
}

UINT16 crc_ccitt(unsigned char *q, int len)
{
	unsigned short crc = 0;
	while (len-- > 0)
	crc = wCRC16Table[(crc >> 8 ^ *q++) & 0xff] ^ (crc << 8);
	return crc;
}

OPSTAT fsm_modbus_sps232_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT fsm_modbus_sps485_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT fsm_modbus_ble_data_rx(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}
