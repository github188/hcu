/*
 * hwinv.h
 *
 *  Created on: 2015年11月17日
 *      Author: test
 */

#ifndef L2FRAME_HWINV_H_
#define L2FRAME_HWINV_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

#include "../hcu.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_HWINV
{
	FSM_STATE_HWINV_INITED = 0x02,
	FSM_STATE_HWINV_ACTIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_HWINV_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//System level hardware inventory
#define HCU_HWINV_RES_NAME_MAX_LEN 10
#define HCU_HWINV_TYPE_NAME_MAX_LEN 20
#define HCU_HWINV_MANU_NAME_MAX_LEN 20
typedef struct HcuHwinvBaseElement
{
	UINT32 hwLogicId; //给每一个硬件资源命令一个独特的系统内部ID
	UINT32 taskId; //如果有对应的单独任务，则为任务ID标识，否则设置为0或者0xFFFFFFFF
	char hwResName[HCU_HWINV_RES_NAME_MAX_LEN];  //Description in sentence
	UINT8 hwStatus;
	UINT32 hwResistence;  //扫描其离线的周期数
	UINT32 hwType;
	char hwTypeName[HCU_HWINV_TYPE_NAME_MAX_LEN];  //Description in sentence
	UINT32 hwManu;
	char hwManufacture[HCU_HWINV_MANU_NAME_MAX_LEN]; //Description in sentence
}HcuHwinvBaseElement_t;

typedef struct HcuHwinvCpuElement
{
	HcuHwinvBaseElement_t hwBase;
	char cpuType[20];
}HcuHwinvCpuElement_t;
typedef struct HcuHwinvFlashElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 hwVolume;  //in MB
}HcuHwinvFlashElement_t;
typedef struct HcuHwinvRtcElement
{
	HcuHwinvBaseElement_t hwBase;
}HcuHwinvRtcElement_t;
typedef struct HcuHwinvMemoryElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 hwVolume;  //in MB
}HcuHwinvMemoryElement_t;
typedef struct HcuHwinvMainBoardElement
{
	HcuHwinvBaseElement_t hwBase;
}HcuHwinvMainBoardElement_t;
typedef struct HcuHwinvHardDiscElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 hwVolume;  //in MB
}HcuHwinvHardDiscElement_t;
typedef struct HcuHwinvGpioElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 number; //多少路GPIO
}HcuHwinvGpioElement_t;
typedef struct HcuHwinvSps232Element
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 portNbr;
}HcuHwinvSps232Element_t;
typedef struct HcuHwinvSps485Element
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 portNbr;
}HcuHwinvSps485Element_t;
typedef struct HcuHwinvEthernetElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 speed;  //1-10M, 2-100M, 3-10/100M, 4-1G, 5-100/1G, 6-Others
	UINT8 macAddress[12];
	UINT8 ipv4Address[4];
	UINT8 ipv6Address[8];
}HcuHwinvEthernetElement_t;
typedef struct HcuHwinvUsbElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 supportUTG2;  //YES-1, NO-0
	UINT8 speed; //1-USB2.0, 2-USB3.0, 3-USB2/3, 4-others
}HcuHwinvUsbElement_t;
typedef struct HcuHwinv3g4gElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 speed;
	UINT8 operator; //1-CMCC, 2-CTC, 4-CU, 4-No, 5-others
	UINT8 simCardNbr[16];
	UINT8 macAddress[12];
	UINT8 ipv4Address[4];
	UINT8 ipv6Address[8];
}HcuHwinv3g4gElement_t;
typedef struct HcuHwinvWifiElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 speed;
	UINT8 operator; //1-CMCC, 2-CTC, 4-CU, 4-No, 5-others
	UINT8 macAddress[12];
	UINT8 ipv4Address[4];
	UINT8 ipv6Address[8];
}HcuHwinvWifiElement_t;
typedef struct HcuHwinvGpsElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 tech; //1-GPS, 2-GLONOSS, 3-BEIDU, 4-GALINIO, 5-others
}HcuHwinvGpsElement_t;
typedef struct HcuHwinvBleElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 speed;
	UINT8 tech; //1-BLE4, 2-LowPower, 3-others
	UINT8 macAddress[12];
	UINT8 ipv4Address[4];
	UINT8 ipv6Address[8];
}HcuHwinvBleElement_t;
typedef struct HcuHwinvAudioElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 tech;
}HcuHwinvAudioElement_t;
typedef struct HcuHwinvCameraElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT8 tech;
	UINT8 resolution;  //1-50W, 2-60W, 3-80W, 4-100W, 5-200W, 6-300W, 7-400W, 8-500W, 9-600W, 10-800W, 11-1000W, 12-Others
}HcuHwinvCameraElement_t;
typedef struct HcuHwinvEmcElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 emcEqupId;
}HcuHwinvEmcElement_t;
typedef struct HcuHwinvPm25Element
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 pm25EqupId;
}HcuHwinvPm25Element_t;
typedef struct HcuHwinvWindDirElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 windDirEqupId;
}HcuHwinvWindDirElement_t;
typedef struct HcuHwinvWindSpdElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 windSpdEqupId;
}HcuHwinvWindSpdElement_t;
typedef struct HcuHwinvTempElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 tempEqupId;
}HcuHwinvTempElement_t;
typedef struct HcuHwinvHumidElement
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 humidEqupId;
}HcuHwinvHumidElement_t;
typedef struct HcuHwinvCo2Element
{
	HcuHwinvBaseElement_t hwBase;
	UINT32 co2EqupId;
}HcuHwinvCo2Element_t;
//All in one
typedef struct HcuHwinvCtrlTable
{
	HcuHwinvCpuElement_t 			cpu;
	HcuHwinvFlashElement_t 			flash;
	HcuHwinvRtcElement_t 			rtc;
	HcuHwinvMemoryElement_t 		memroy;
	HcuHwinvMainBoardElement_t 		mainBoard;
	HcuHwinvHardDiscElement_t 		hardDisc;
	HcuHwinvGpioElement_t 			gpio;
	HcuHwinvSps232Element_t 		sps232[4];
	HcuHwinvSps485Element_t			sps485;
	HcuHwinvEthernetElement_t		ethernet;
	HcuHwinvUsbElement_t			usbnet;
	HcuHwinv3g4gElement_t			g3g4;
	HcuHwinvWifiElement_t			wifi;
	HcuHwinvGpsElement_t			gps;
	HcuHwinvBleElement_t			ble;
	HcuHwinvAudioElement_t			audio;
	HcuHwinvCameraElement_t			camera;
	HcuHwinvUsbElement_t			usb;
	HcuHwinvEmcElement_t			emc;
	HcuHwinvPm25Element_t			pm25;
	HcuHwinvWindDirElement_t		windDir;
	HcuHwinvWindSpdElement_t		windSpd;
	HcuHwinvTempElement_t			temp;
	HcuHwinvHumidElement_t			humid;
	HcuHwinvCo2Element_t			co2;
}HcuHwinvCtrlTable_t;

#define FILE_LENGTH_RECORD_MAX 1024  //一条存入文件的记录最长长度
#define HCU_DIR_LENGTH_MAX 256
#define HCU_RECORD_LOG_DIR_NAME_LOCAL	"./log"
#define HCU_RECORD_LOG_DIR_NAME_CLEAN 	"/log"
#define HCU_RECORD_FILE_NAME_SENSOR    	"/sensor"
#define HCU_RECORD_FILE_NAME_AUDIO    	"/audio"
#define HCU_RECORD_FILE_NAME_CAMERA    	"/camera"
#define HCU_RECORD_FILE_NAME_EXTEND		".dat"

//时间全局变量
typedef struct zHcuTimeDateTable
{
	UINT16 year;
	UINT8 mon;
	UINT8 day;
	UINT8 hour;
	UINT8 min;
	UINT16 grad;  //网格in Minutes
	char sYear[5];		//"2015"
	char strYear[7]; 	//"./2015"
	char sMon[7]; 		//"201510"
	char strMon[9]; 	//"./201510"
	char sDay[9]; 		//"20151021"
	char strDay[11]; 	//"./20151021"
	char sHour[11]; 	//"2015102119"
	char strHour[13]; 	//"./2015102119"
	char sMin[13]; 		//"201510211955";
	char strMin[15]; 	//"./201510211955";
	char curHcuDir[HCU_DIR_LENGTH_MAX];
	char curLogDir[HCU_DIR_LENGTH_MAX];
	char curYmDir[HCU_DIR_LENGTH_MAX];
	char curSensorFd[HCU_DIR_LENGTH_MAX];
	char curAudioFd[HCU_DIR_LENGTH_MAX];
	char curCameraFd[HCU_DIR_LENGTH_MAX];
}zHcuTimeDateTable_t;

//Global variables
extern FsmStateItem_t FsmHwinv[];
extern HcuHwinvCtrlTable_t zHcuHwinvTable;
extern zHcuTimeDateTable_t zCurTimeDate;

//API
extern OPSTAT fsm_hwinv_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hwinv_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_hwinv_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
OPSTAT func_hwinv_global_par_init(void);
void func_hwinv_copy_right(void);
void func_hwinv_scan_all(void);
void func_hwinv_scan_date();
void func_hwinv_scan_cpu(void);
void func_hwinv_scan_flash(void);
void func_hwinv_scan_rtc(void);
void func_hwinv_scan_memroy(void);
void func_hwinv_scan_main_board(void);
void func_hwinv_scan_hard_disc(void);
void func_hw_inv_scan_dir(void);
void func_hwinv_scan_gpio(void);
void func_hwinv_scan_sps232(void);
void func_hwinv_scan_sps485(void);
void func_hwinv_scan_ethernet(void);
void func_hwinv_scan_usbnet(void);
void func_hwinv_scan_3g4g(void);
void func_hwinv_scan_wifi(void);
void func_hwinv_scan_usb(void);
void func_hwinv_scan_audio(void);
void func_hwinv_scan_camera(void);
void func_hwinv_scan_emc(void);
void func_hwinv_scan_pm25(void);
void func_hwinv_scan_winddir(void);
void func_hwinv_scan_windspd(void);
void func_hwinv_scan_temp(void);
void func_hwinv_scan_humid(void);
void func_hwinv_scan_co2(void);

//External APIs
extern UINT32 hcu_disk_write(UINT32 fId, void *dataBuffer, UINT32 dataLen);

#endif /* L2FRAME_HWINV_H_ */
