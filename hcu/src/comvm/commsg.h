/*
 * commsg.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef COMVM_COMMSG_H_
#define COMVM_COMMSG_H_

#include <stdio.h>
#include <stdlib.h>
#include <netdb.h>
#include <stdarg.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <dirent.h>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <getopt.h>
#include <limits.h>
#include <locale.h>

#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ipc.h>
#include <sys/time.h>

#include <libxml/xmlmemory.h>
#include <libxml/tree.h>
#include <libxml/parser.h>
#include <libxml/xmlstring.h>
#include <linux/netdevice.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <asm/types.h>

//视频捕获
#include <linux/videodev2.h>

//CURL
#include <curl/curl.h>

/*1. 公共定义结构体
** Date types used in this project.
*/
typedef unsigned char   		UINT8;
typedef signed char     		INT8;
typedef char            		CHAR;           //only for character array
typedef unsigned short  		UINT16;
typedef signed short    		INT16;
typedef unsigned int    		UINT32;
typedef	signed int      		INT32;
typedef unsigned long int 		UINT64;
typedef signed long int 		INT64;
typedef unsigned long   		ULONG;
typedef signed long     		SLONG;
typedef UINT8           		BOOLEAN;          //defined for RP
typedef void            		VOID;
typedef BOOLEAN					bool;
typedef BOOLEAN					BOOL;
typedef uint16_t 				ke_task_id_t;
typedef uint16_t 				ke_msg_id_t;
typedef uint16_t 				ke_state_t;


/*
** Function return value used in the project.
** Only two possible values for OPSTAT return type:
** 1. SUCCESS
** 0. FAILURE
*/
typedef UINT32         OPSTAT;
#define SUCCESS        0
#define FAILURE        -1
#define true   			1
#define false			0


/*
** Definition of invalid values used in the project.
*/
#define  	HCU_INVALID_U32					0xffffffff
#define  	HCU_INVALID_U16					0xffff
#define  	HCU_INVALID_U8					0xff
#define 	HCU_LOGIC_TRUE					0x1
#define 	HCU_LOGIC_FALSE					0x0
#define 	HCU_INVALID_PARAMETER			0xffffffff
#define 	HCU_MEASURE_RECEIVED			0x0
#define 	HCU_COMMAND_RECEIVED			0x1
#define 	HCU_MEASURE_COMMAND_RECEIVED	0x2

//2. 公共消息结构体定义
//Under normal case, 1024Byte shall be enough for internal message communciation purpose.
//If not enough, need modify here to enlarge
#define MAX_HCU_MSG_BODY_LENGTH 512
#define MAX_HCU_MSG_BUF_LENGTH MAX_HCU_MSG_BODY_LENGTH-16
typedef struct HcuMsgSruct
{
	UINT32 msgType;
	UINT32 dest_id;
	UINT32 src_id;
	UINT32 msgLen;
	INT8 msgBody[MAX_HCU_MSG_BODY_LENGTH];
}HcuMsgSruct_t;


//3. 原有程序的消息及数据结构体定义
/*
 * Type define for Socket
 */
typedef struct SocketPort
{
	char ServerIpAddress[16];
	UINT16 ServerPort;
	UINT16 SocketFd;
	struct hostent *pHostName;
	struct sockaddr_in ServerSocketAddress;
	struct sockaddr_in ClientSocketAddress;
} SocketPort_t;

#define		HCU_MAX_NUM_OF_WED	 			16
#define		WED_MAX_NUM_OF_MEASURE			7
#define		HCU_MAX_LENGTH_CMD_SERIAL 		(256 + 4 + 1)*4 /* 4 for MsgHeader, 1 for FCS */
#define		HCU_MAX_LENGTH_CMD_TCP 			1024

#define		HCU_MAX_NUM_POLLING_LOOP		100
#define		HCU_MAX_NUM_RECORD				(60*60*24*2)  /* two days of record, if we have 1 sec per measurement */


/* Sensor Type Definition */
#define		WED_SENSOR_TYPE_PM10			0x00000001
#define		WED_SENSOR_TYPE_PM25			0x00000002
#define		WED_SENSOR_TYPE_SO2				0x00000004
#define		WED_SENSOR_TYPE_NOX				0x00000008
#define		WED_SENSOR_TYPE_OO3				0x00000010
#define		WED_SENSOR_TYPE_CO				0x00000020
#define		WED_SENSOR_TYPE_TEMP			0x00000040

//#define 	SENSOR_ACTIVE	 				1
//#define		SENSOR_INACTIVE		 			0

#define		NUM_OF_SMT_COPY					2

/*
 * WED Wireless End Point Properties
 */
typedef struct WedSensorProperty
{
	UINT16 NodeId;
	UINT16 NodeZbDynAddress;
	UINT32 NodeZbIeeeAddress[2]; /* ZigBee IEEE address */
	UINT16 NodeStatus;
	UINT32 NodeSensorType;
} WedSensorProperty_t;

/*
 * WED All Sensors
 */
typedef struct WedSensors
{
	UINT32 NumOfActiveSensors;
	WedSensorProperty_t Sensor[HCU_MAX_NUM_OF_WED];
} WedSensors_t;

typedef struct MeasurePerSensor
{
	INT32 pm10; 	/* unit: mg/m3 devided by 1000 */
	INT32 pm25;		/* unit: mg/m3 devided by 1000 */
	INT32 so2;		/* unit: tbd */
	INT32 nox;		/* unit: tbd */
	INT32 oo3;		/* unit: tbd */
	INT32 co;		/* unit: tbd */
	INT32 temp;
} MeasurePerSensor_t;

typedef struct MeasureItem
{
	double 	value;
	time_t	time;
	UINT32 	sensorid;
	UINT32	sensortype;
}MeasureItem_t;
/*
 * WED All Sensors' Measurement Results
 *
 * For each of the sensor (which can measure several variables), there are total HCU_MAX_NUM_POLLING_LOOP records could be saved in the table.
 * This does not mean that for each of the loop, all HCU_MAX_NUM_POLLING_LOOP will be poll, teh actual number of loop for each of the Polling is defined in
 * NumOfPollingLoop in HCUMeasureBehavior_t
 */
typedef struct SensorsMeasurementsTable
{
	UINT32	activeTable;   /* copy 0 or copy 1 */
	UINT32	curRecord;     /* 0 to HCU_MAX_NUM_RECORD - 1*/
	MeasurePerSensor_t smt[NUM_OF_SMT_COPY][HCU_MAX_NUM_POLLING_LOOP][HCU_MAX_NUM_OF_WED];
	MeasureItem_t mi[NUM_OF_SMT_COPY][HCU_MAX_NUM_RECORD];
} SensorsMeasurementsTable_t;

typedef struct HCUMeasureBehavior
{
	UINT8  SerialVtime;				/* Timeout if no char received, timeout = Vtime * 100ms */
	UINT8  SerialVmin;				/* Number of char received before block read return */
	UINT32 DelayBeforeRead;		 	/* unit: ms */
	UINT32 DelayAfterRead; 			/* unit: ms */
	time_t SerialPollItvlSec;		/* unit: s */
	UINT32 SerialPollItvlnSec;		/* unit: nsec */
	UINT32 NumOfPollingLoop;   		/*  */

	UINT32 SocketInterval;
	UINT32 socketCnt;/*  */

	/* System Counter */
	ULONG NumberOfFrameTotalRx;
	ULONG NumberOfFrameValidRx;
	ULONG NumberOfFrameTotalTx;
	ULONG NumberOfTicksTriggerred;

	/* max counter when network is down before another socket connection tried */
	UINT32 MaxCounterTicksNetworkDown;

} HcuMeasureBehavior_t;

/*
 * HCU ZB interface
 *
 */


typedef struct HcuZbMsgHeader
{
	UINT8 sop;
	UINT8 len;
	UINT8 cmd0;
	UINT8 cmd1;
} HcuZbMsgHeader_t;

typedef struct PollingLoopBehavior
{
	UINT32 NumOfPoll;
	UINT32 NumOfSensor;
	UINT32 NumOfMeasure;
	UINT32 CurrentTick;
}PollingLoopBehavior_t;

typedef struct AnalyzeDataBehavior
{
	UINT32 NumOfType;
	UINT32 NumOfPolling;
	UINT32 NumOfWed;
}AnalyzeDataBehavior_t;

/* data header for msg send to sever */
typedef struct MsgSeverDataSegHeader
{
	UINT32 LenOfData;   /* ST=31;CN=2011;PW=123456;MN=12345678901002;CP=&&DataTime=2012101700100.....;CO-Rtd=0.00,CO-Flag=N;PMB-Rtd=0.00,PMB-Flag=N&& */
	char ST[16];       /* ST: 31 */
	char CN[16];       /* CN: 2011 */
	char PW[16];       /* PW: 123456 */
	char MN[16];       /* MN: 12345677654321 */
	char DataTime[64];
}MsgSeverDataSegHeader_t;

/* System Information */
#define     MAX_SYS_INFO_STRING_LENGTH			128
#define     MAX_SYS_INFO_INTERFACE_NUMBER		16

typedef struct SysHwInfo
{
	char cpu_processor[MAX_SYS_INFO_STRING_LENGTH];
	char cpu_vendor[MAX_SYS_INFO_STRING_LENGTH];
	char cpu_modelname[MAX_SYS_INFO_STRING_LENGTH];
	char cpu_mhz[MAX_SYS_INFO_STRING_LENGTH];
}SysHwInfo_t;

typedef struct SysOsInfo
{
	char version_signature[MAX_SYS_INFO_STRING_LENGTH];
}SysOsInfo_t;

typedef struct SysNetworkInfo
{

	int itfnum;
	char itfname[MAX_SYS_INFO_INTERFACE_NUMBER][MAX_SYS_INFO_STRING_LENGTH];
	char itfip[MAX_SYS_INFO_INTERFACE_NUMBER][MAX_SYS_INFO_STRING_LENGTH];
	/* whether there is PPP */
	pid_t ppppid;

}SysNetworkInfo_t;

typedef struct SysInfo
{
	SysHwInfo_t hw;
	SysOsInfo_t os;
	SysNetworkInfo_t net;
}SysInfo_t;

/* For thread creation */
typedef void *(*CALLBACK)(void *);

#define		MSG_SOP 						0xFE
#define 	HCU_SOP 						0xFE

#define 	MSG_COORD_START_IND_LEN 		0x00
#define 	MSG_COORD_START_IND_CMD1		0x80
#define		MSG_COORD_START_IND_CMD2		0x16

#define 	MSG_NODE_STATUS_IND_LEN    		0x0C
#define 	MSG_NODE_STATUS_IND_CMD1		0x80
#define 	MSG_NODE_STATUS_IND_CMD2		0x17

#define 	MSG_SENSOR_REG_IND_LEN			0x02
#define 	MSG_SENSOR_REG_IND_CMD1			0x80
#define 	MSG_SENSOR_REG_IND_CMD2			0x18

#define 	MSG_SENSOR_DEREG_IND_LEN		0x02
#define 	MSG_SENSOR_DEREG_IND_CMD1		0x80
#define 	MSG_SENSOR_DEREG_IND_CMD2		0x19

#define		MSG_APP_DATA_REG_CMD1			0x01
#define 	MSG_APP_DATA_REG_CMD2			0x20

#define 	MSG_APP_DATA_IND_CMD1			0x80
#define 	MSG_APP_DATA_IND_CMD2			0x20


#define		SENSOR_STATUS_INIT				0x0000
#define		SENSOR_STATUS_NODE_CONN			0x0001
#define		SENSOR_STATUS_DATA_CONN			0x0002

#define 	CLOCKID 						CLOCK_REALTIME
#define 	THREAD_COM_TIMER_ID 			111
#define		NB_NANOS_IN_ONE_SECOND			1000000000	//number of nanos in one second
#define 	NB_NANOS_IN_ONE_MS          	1000000     //number of nanos secondes in 1 ms
#define 	NB_MICROS_IN_ONE_SECOND        	1000000     //number of micros secondes in 1 second
#define 	NB_MICROS_IN_ONE_MS    	     	1000	    //number of micros secondes in 1 ms
#define 	START_DELAY                 	4           //delay(in second) before the launcher starts (to have time for clock synchro)

#define     HCU_THREAD_PRIO             	10          //priority of the main loop de 1 a 99 max



/* HCU MAIN STATE MACHINE BIT MAP */
#define		HCU_STATE_ALL_INIT				0x00000000
#define		HCU_STATE_SENSOR_OK				0x00000001
#define		HCU_STATE_NET_SERVICE_OK		0x00010000
#define		HCU_STATE_NET_OAM_OK			0x01000000
#define		HCU_STATE_AUTO_POLLING			0x00000100

#define 	HCU_MAX_TICKS_NETWORKDOWN		10

/* OAM CONTROL MESSAGE */
#define		MAX_OAM_CONTROL_MSG_LEN			64
#define		MAX_OAM_CONTROL_MSG_BUFFER		16

#define		MSG_ID_SENSOR_POLLING			0x00000001
#define		MSG_ID_DEBUG_TRACE_TOGGLE		0x00000002
#define		MSG_ID_STOP_AUTO_POLLING		0x00000003
#define		MSG_ID_START_AUTO_POLLING		0x00000004

typedef struct OamControlMsg
{
	UINT32 MsgId;
	UINT32 MsgLen;
	char MsgBody[MAX_OAM_CONTROL_MSG_LEN];
}OamControlMsg_t;

typedef struct OamControlMsgBuf
{
	UINT32 First;
	UINT32 Last;
	OamControlMsg_t MsgBuf[MAX_OAM_CONTROL_MSG_BUFFER];
}OamControlMsgBuf_t;

extern INT32  outputOnCrt;

//4. 新消息新程序结构体定义部分
//复用下位机的消息定义
/// Message API of the AIRSYNC task
#define MSG_ID_ENTRY 0
enum HCU_INTER_TASK_MSG_ID
{
	//COMMOM message, KE_FIRST_MSG(TASK_AIRSYNC) as starting
	MSG_ID_COM_MIN = 0x00, //Starting point
	MSG_ID_COM_INIT,
	MSG_ID_COM_INIT_FEEDBACK,
	MSG_ID_COM_RESTART,
	MSG_ID_COM_STOP,
	MSG_ID_COM_COMPLETE,
	MSG_ID_COM_TIME_OUT,
	MSG_ID_COM_DEBUG_TEST,
	//Service Control message

	//ETHERNET
	MSG_ID_ETHERNET_CLOUD_DATA_RX,

	//WIFI
	MSG_ID_WIFI_CLOUD_DATA_RX,

	//USBNET
	MSG_ID_USBNET_CLOUD_DATA_RX,

	//3G4G
	MSG_ID_3G4G_CLOUD_DATA_RX,

	//RS232
	MSG_ID_SPS232_MODBUS_DATA_RX,

	//RS485
	MSG_ID_SPS485_MODBUS_DATA_RX,

	//AUDIO
	MSG_ID_AUDIO_HSMMP_DATA_RX,

	//CAMERA
	MSG_ID_CAMERA_HSMMP_DATA_RX,

	//BLE
	MSG_ID_BLE_HSMMP_DATA_RX,
	MSG_ID_BLE_MODBUS_DATA_RX,

	//GPS

	//LCD
	MSG_ID_LCD_HSMMP_DATA_RX,

	//HWINV
	MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG,

	//CloudConnection message
	MSG_ID_CLOUD_EMC_DATA_REQ,
	MSG_ID_CLOUD_PM25_DATA_REQ,
	MSG_ID_CLOUD_WIND_DIR_DATA_REQ,
	MSG_ID_CLOUD_WIND_SPEED_DATA_REQ,
	MSG_ID_CLOUD_TEMPERATURE_DATA_REQ,
	MSG_ID_CLOUD_HUMIDITY_DATA_REQ,

	//EMC
	MSG_ID_EMC_MODBUS_DATA_READ,
	MSG_ID_EMC_CLOUD_DATA_RESP,

	//PM25 message
	MSG_ID_PM25_MODBUS_DATA_READ,
	MSG_ID_PM25_CLOUD_DATA_RESP,

	//WIND_DIRECTION message
	MSG_ID_WINDDIR_MODBUS_DATA_READ,
	MSG_ID_WINDDIR_CLOUD_DATA_RESP,

	//WIND_SPEED message
	MSG_ID_WINDSPD_MODBUS_DATA_READ,
	MSG_ID_WINDSPD_CLOUD_DATA_RESP,

	//TEMPERATURE message
	MSG_ID_TEMP_MODBUS_DATA_READ,
	MSG_ID_TEMP_CLOUD_DATA_RESP,

	//HUMIDITY message
	MSG_ID_HUMID_MODBUS_DATA_READ,
	MSG_ID_HUMID_CLOUD_DATA_RESP,

	//Modbus report
	MSG_ID_MODBUS_EMC_DATA_REPORT,
	MSG_ID_MODBUS_PM25_DATA_REPORT,
	MSG_ID_MODBUS_WINDDIR_DATA_REPORT,
	MSG_ID_MODBUS_WINDSPD_DATA_REPORT,
	MSG_ID_MODBUS_TEMP_DATA_REPORT,
	MSG_ID_MODBUS_HUMID_DATA_REPORT,

	//AirSync message
	MSG_ID_AIRSYNC_INIT,
	MSG_ID_AIRSYNC_BLE_CONNNECTD,
	MSG_ID_AIRSYNC_AUTH_REQ,
	MSG_ID_AIRSYNC_AUTH_RESP,
	MSG_ID_AIRSYNC_INIT_REQ,
	MSG_ID_AIRSYNC_INIT_RESP,
	MSG_ID_AIRSYNC_CONNECT_READY,
	MSG_ID_AIRSYNC_DATA_PUSH,
	MSG_ID_AIRSYNC_DATA_REQ,
	MSG_ID_AIRSYNC_DATA_RESP,
	MSG_ID_AIRSYNC_AUTH_WAIT_TIMER,
	MSG_ID_AIRSYNC_DISCONNECT_TIGGER_L3,
	MSG_ID_AIRSYNC_SEND_BLE_DATA_TIMER,
	//WxApp message
	MSG_ID_WXAPP_3MIN_TIMEOUT,
	MSG_ID_WXAPP_5SECOND_TIMEOUT,
	MSG_ID_WXAPP_DATA_REQ_EMC,
	MSG_ID_WXAPP_PERIOD_REPORT_EMC_TRIGGER,
	MSG_ID_WXAPP_DATA_REQ_TIME_SYNC,
	MSG_ID_WXAPP_EQUIPMENT_INFO_SYNC,
	MSG_ID_WXAPP_MODBUS_CONNECT_READY, //trigger for MODBUS connection ready
	MSG_ID_WXAPP_MODBUS_DATA_PUSH, //send push command to MODBUS
	MSG_ID_WXAPP_MODBUS_DISCONNECT, //send for modbus
	//Modbus message
	MSG_ID_MODBUS_10MIN_TIMEOUT,
	MSG_ID_MODBUS_5SECOND_TIMEOUT, //
	MSG_ID_MODBUS_PERIOD_REPORT_PM1025_TRIGGER,  //TIMR FOR internal process
	MSG_ID_MODBUS_PERIOD_REPORT_WIND_SPEED_TRIGGER,  //TIMR FOR internal process
	MSG_ID_MODBUS_PERIOD_REPORT_WIND_DIR_TRIGGER,  //TIMR FOR internal process
	MSG_ID_MODBUS_PERIOD_REPORT_TEMPERATURE_TRIGGER,  //TIMR FOR internal process
	MSG_ID_MODBUS_PERIOD_REPORT_HUMIDITY_TRIGGER,  //TIMR FOR internal process
	MSG_ID_MODBUS_DATA_REQ_PM1025_REPORT,  //Send to AIRSYNC directly
	MSG_ID_MODBUS_DATA_REQ_WIND_SPEED_REPORT,  //Send to AIRSYNC directly
	MSG_ID_MODBUS_DATA_REQ_WIND_DIR_REPORT,  //Send to AIRSYNC directly
	MSG_ID_MODBUS_DATA_REQ_TEMPERATURE_REPORT,  //Send to AIRSYNC directly
	MSG_ID_MODBUS_DATA_REQ_HUMIDITY_REPORT,  //Send to AIRSYNC directly
	MSG_ID_MODBUS_UART1_FRAME_TIMEOUT,  //TIMR FOR UART1 Frame construction
	MSG_ID_COM_MAX, //Ending point

}; //end of HCU_INTER_TASK_MSG_ID
#define MSG_ID_END 0xFF  //跟MASK_MSGID_NUM_IN_ONE_TASK设置息息相关，不能随便改动
#define MSG_ID_INVALID 0xFFFFFFFF

//先定义个ARRAY_EMPTY的数值，以便对付过去编译，后面再仔细研究和定义这个消息机制
#define MSG_NAME_MAX_LENGTH 70

//缺省消息都使用UINT32进行定义，在内存不是最重要的制约因素下，这种统一的方式，是为了更加不容易出错，不用在不同
//长度的字型之间进行转换。如果遇到INT类型，直接强制转换即可，符号单独处理
//message definition
typedef struct msg_struct_com_init //
{
	UINT32 length;
}msg_struct_com_init_t;

typedef struct msg_struct_com_init_feedback //
{
	UINT32 length;
}msg_struct_com_init_feedback_t;

typedef struct  msg_struct_com_restart //
{
	UINT32 length;
}msg_struct_com_restart_t;

typedef struct  msg_struct_com_stop //
{
	UINT32 length;
}msg_struct_com_stop_t;

typedef struct  msg_struct_com_complete //
{
	UINT32 length;
}msg_struct_com_complete_t;

typedef struct  msg_struct_com_time_out //
{
	UINT32 timeId;
	UINT8 timeRes;
	UINT32 length;
}msg_struct_com_time_out_t;

typedef struct  msg_struct_com_debug //
{
	UINT32 testid;
	char testinfo[10];
	UINT32 length;
}msg_struct_com_debug_t;

#define HWINV_PHY_STATUS_NULL 0
#define HWINV_PHY_STATUS_DEACTIVE_TO_ACTIVE 1
#define HWINV_PHY_STATUS_ACTIVE_TO_DEACTIVE 2
#define HWINV_PHY_STATUS_INVALID 0xFF
typedef struct  msg_struct_hwinv_cloud_phy_status_chg //
{
	UINT8 ethStatChg;
	UINT8 usbnetStatChg;
	UINT8 wifiStatChg;
	UINT8 g3g4StatChg;
	UINT32 length;
}msg_struct_hwinv_cloud_phy_status_chg_t;

//Rx data coming from CLOUD direction, shall be controller data, no need too much data traffic
typedef struct  msg_struct_ethernet_cloud_data_rx //
{
	UINT32 length;
	char buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_ethernet_cloud_data_rx_t;
typedef struct  msg_struct_wifi_cloud_data_rx //
{
	UINT32 length;
	char buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_wifi_cloud_data_rx_t;
typedef struct  msg_struct_usbnet_cloud_data_rx //
{
	UINT32 length;
	char buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_usbnet_cloud_data_rx_t;
typedef struct  msg_struct_3g4g_cloud_data_rx //
{
	UINT32 length;
	char buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_3g4g_cloud_data_rx_t;
//Used to common all different received data buffer
typedef struct  msg_struct_com_cloud_data_rx //
{
	UINT32 length;
	char buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_com_cloud_data_rx_t;

typedef struct  msg_struct_sps232_modbus_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_sps232_modbus_data_rx_t;
typedef struct msg_struct_sps485_modbus_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_sps485_modbus_data_rx_t;
typedef struct msg_struct_audio_hsmmp_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_audio_hsmmp_data_rx_t;
typedef struct msg_struct_camera_hsmmp_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_camera_hsmmp_data_rx_t;
typedef struct msg_struct_ble_hsmmp_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_ble_hsmmp_data_rx_t;
typedef struct msg_struct_ble_modbus_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_ble_modbus_data_rx_t;
typedef struct msg_struct_lcd_hsmmp_data_rx //
{
	UINT32 length;
	UINT8 buf[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_lcd_hsmmp_data_rx_t;


//Data request Cloud to Sensors
typedef struct  msg_struct_cloud_emc_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_emc_data_req_t;
typedef struct  msg_struct_cloud_pm25_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_pm25_data_req_t;
typedef struct  msg_struct_cloud_winddir_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_winddir_data_req_t;
typedef struct  msg_struct_cloud_windspd_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_windspd_data_req_t;
typedef struct  msg_struct_cloud_temp_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_temp_data_req_t;
typedef struct  msg_struct_cloud_humid_data_req //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_cloud_humid_data_req_t;

//SENSOR Data request to MODBUS
typedef struct  msg_struct_emc_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_emc_modbus_data_read_t;
typedef struct  msg_struct_pm25_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_pm25_modbus_data_read_t;
typedef struct  msg_struct_winddir_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_winddir_modbus_data_read_t;
typedef struct  msg_struct_windspd_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_windspd_modbus_data_read_t;
typedef struct  msg_struct_temp_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_temp_modbus_data_read_t;
typedef struct  msg_struct_humid_modbus_data_read //
{
	UINT32 equId;
	UINT8  cmdId;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  optId;
	UINT32 length;
}msg_struct_humid_modbus_data_read_t;

//Data Report message from Modbus
typedef struct  msg_struct_modbus_emc_data_report //
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 emcValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_emc_data_report_t;
typedef struct  msg_struct_modbus_pm25_data_report //
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 pm1d0Value;
	UINT32 pm2d5Value;
	UINT32 pm10Value;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_pm25_data_report_t;
typedef struct msg_struct_modbus_winddir_data_report
{
	UINT32 length;
	UINT8 usercmdid;
	UINT8 cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8 useroptid;
	UINT32 winddirValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_winddir_data_report_t;
typedef struct msg_struct_modbus_windspd_data_report
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 windspdValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_windspd_data_report_t;
typedef struct msg_struct_modbus_temp_data_report
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 tempValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_temp_data_report_t;
typedef struct msg_struct_modbus_humid_data_report
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 humidValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_modbus_humid_data_report_t;

//Data response message to Cloud
typedef struct  msg_struct_emc_cloud_data_resp //
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 emcValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_emc_cloud_data_resp_t;
typedef struct  msg_struct_pm25_cloud_data_resp_t //
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 pm1d0Value;
	UINT32 pm2d5Value;
	UINT32 pm10Value;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_pm25_cloud_data_resp_t;
typedef struct msg_struct_winddir_cloud_data_resp
{
	UINT32 length;
	UINT8 usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8 useroptid;
	UINT32 winddirValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_winddir_cloud_data_resp_t;
typedef struct msg_struct_windspd_cloud_data_resp
{
	UINT32 length;
	UINT8  usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8  useroptid;
	UINT32 windspdValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_windspd_cloud_data_resp_t;
typedef struct msg_struct_temp_cloud_data_resp
{
	UINT32 length;
	UINT8 usercmdid;
	UINT8  cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8 useroptid;
	UINT32 tempValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_temp_cloud_data_resp_t;
typedef struct msg_struct_humid_cloud_data_resp
{
	UINT32 length;
	UINT8 usercmdid;
	UINT8 cmdIdBackType; //指明是瞬时，还是周期性读数
	UINT8 useroptid;
	UINT32 humidValue;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT32 gps_z;
	UINT32 equipid;
	UINT32 timeStamp;
}msg_struct_humid_cloud_data_resp_t;





//Old world message structure, for removal
typedef struct  msg_struct_airsync_ble_connected //
{
	UINT32 length;
}msg_struct_airsync_ble_connected_t;

typedef struct  msg_struct_airsync_auth_resp
{
	/// Handle of the attribute that has to be written
	UINT32 handle;
	/// Data length to be written
	UINT32 length;
	/// offset at which the data has to be written
	UINT32 offset;
	/// Destination task shall send back a write response command if true.
	BOOL response;
	///Informs that it's last request of a multiple prepare write request.
	bool last;
	/// Data to be written in attribute database
	UINT8  value[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_airsync_auth_resp_t;

typedef struct  msg_struct_airsync_init_resp
{
	/// Handle of the attribute that has to be written
	UINT32 handle;
	/// Data length to be written
	UINT32 length;
	/// offset at which the data has to be written
	UINT32 offset;
	/// Destination task shall send back a write response command if true.
	bool response;
	///Informs that it's last request of a multiple prepare write request.
	bool last;
	/// Data to be written in attribute database
	UINT8  value[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_airsync_init_resp_t;

typedef struct  msg_struct_airsync_connect_ready
{
	UINT32 length;
}msg_struct_airsync_connect_ready_t;

#define ASY_DATA_PUSH_LENGTH_MAX 20
typedef struct  msg_struct_airsync_data_push
{
	UINT8 UserCmdId;
	UINT16 EmDeviceDataType;
	UINT32 length;
	UINT8 data[ASY_DATA_PUSH_LENGTH_MAX];
}msg_struct_airsync_data_push_t;
//
typedef struct  msg_struct_airsync_data_resp
{
	/// Handle of the attribute that has to be written
	UINT32 handle;
	/// Data length to be written
	UINT32 length;
	/// offset at which the data has to be written
	UINT32 offset;
	/// Destination task shall send back a write response command if true.
	bool response;
	///Informs that it's last request of a multiple prepare write request.
	bool last;
	/// Data to be written in attribute database
	UINT8  value[MAX_HCU_MSG_BUF_LENGTH];
}msg_struct_airsync_data_resp_t;

typedef struct  msg_struct_airsync_disconnect_tigger_l3
{
	UINT32 length;
}msg_struct_airsync_disconnect_tigger_l3_t;

typedef struct  msg_struct_airsync_send_ble_data_timer
{
	UINT32 length;
}msg_struct_airsync_send_ble_data_timer_t;

//All L3 message structure defined here

typedef struct  msg_struct_3min_timtout
{
	UINT32 length;
}msg_struct_3min_timtout_t;

typedef struct  msg_struct_5second_timtout
{
	UINT32 length;
}msg_struct_5second_timtout_t;

typedef struct  msg_struct_period_report_emc_trigger
{
	UINT32 length;
}msg_struct_period_report_emc_trigger_t;
//
typedef struct  msg_struct_wxapp_data_req_emc
{
	UINT8 usercmdid;
	UINT32 length;
	UINT16 emc_value;
	UINT8 year;
	UINT8 month;
	UINT8 day;
	UINT8 hour;
	UINT8 minute;
	UINT8 second;
	UINT32 gps_x;
	UINT32 gps_y;
	UINT16 ntimes;
	UINT16 EmDeviceDataType;
}msg_struct_wxapp_data_req_emc_t;

typedef struct  msg_struct_wxapp_data_req_time_sync
{
	UINT32 length;
}msg_struct_wxapp_data_req_time_sync_t;

typedef struct  msg_struct_wxapp_equipment_info_sync
{
	UINT32 length;
	UINT8 hw_uuid[6];
	UINT8 hw_type;
	UINT16 hw_version;
	UINT8 sw_release;
	UINT16 sw_delivery;
}msg_struct_wxapp_equipment_info_sync_t;

typedef struct  msg_struct_wxapp_modbus_connect_ready
{
	UINT32 length;
}msg_struct_wxapp_modbus_connect_ready_t;

#define WXAPP_DATA_PUSH_LENGTH_MAX 20
typedef struct  msg_struct_wxapp_modbus_data_push
{
	UINT8 UserCmdId;
	UINT16 EmDeviceDataType;
	UINT32 length;
	UINT8 data[WXAPP_DATA_PUSH_LENGTH_MAX];
}msg_struct_wxapp_modbus_data_push_t;

typedef struct  msg_struct_wxapp_modbus_disconnect
{
	UINT32 length;
}msg_struct_wxapp_modbus_disconnect_t;

//All L3 message structure defined here

typedef struct  msg_struct_mobus_10min_timtout
{
	UINT32 length;
}msg_struct_mobus_10min_timtout_t;

typedef struct  msg_struct_modbus_5second_timtout
{
	UINT32 length;
}msg_struct_modbus_5second_timtout_t;

typedef struct  msg_struct_modbus_period_report_pm1025_trigger
{
	UINT32 length;
}msg_struct_modbus_period_report_pm1025_trigger_t;

typedef struct  msg_struct_modbus_period_report_windspd_trigger
{
	UINT32 length;
}msg_struct_modbus_period_report_windspd_trigger_t;

typedef struct  msg_struct_modbus_period_report_winddir_trigger
{
	UINT32 length;
}msg_struct_modbus_period_report_winddir_trigger_t;

typedef struct  msg_struct_modbus_period_report_temp_trigger
{
	UINT32 length;
}msg_struct_modbus_period_report_temp_trigger_t;

typedef struct  msg_struct_modbus_period_report_humid_trigger
{
	UINT32 length;
}msg_struct_modbus_period_report_humid_trigger_t;


typedef struct  msg_struct_modbus_uart1_frame_timtout
{
	UINT32 length;
}msg_struct_modbus_uart1_frame_timtout_t;


#endif /* COMVM_COMMSG_H_ */
