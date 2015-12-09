/*
 * vm_layer.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef COMVM_VMLAYER_H_
#define COMVM_VMLAYER_H_

#include "sysconfig.h"
#include "commsg.h"

#define MAX_TASK_NUM_IN_ONE_HCU 64
//定义TASK对应的名字
//!!!!!!!!!!!!!!!!ATTENTION!!!!!!!!!!!!!!!!
//Whenever the Task Id is changed, QueID and vmlayer.c/zHcuTaskNameList array should be updated, otherwise error will happen!!!
//不是任务的任务，比如TRACE/CONFIG/3G/GPIO等等，留待清理，简化任务列表
enum HCU_TASK_NAME_ID
{
	TASK_ID_MIN = 0,
	TASK_ID_HCUMAIN,
	TASK_ID_HCUVM,
	TASK_ID_TRACE,
	TASK_ID_CONFIG,
	TASK_ID_TIMER,
	TASK_ID_MMC, //Man Machine Communication
	TASK_ID_GPIO,
	TASK_ID_GRPS,
	TASK_ID_SPS232,
	TASK_ID_SPS485,
	TASK_ID_BLE,
	TASK_ID_ETHERNET,  //ETHERNET
	TASK_ID_WIFI,
	TASK_ID_USBNET,  //USB used for network connection
	TASK_ID_3G4G,
	TASK_ID_HARDDISK,
	TASK_ID_CAMERA,
	TASK_ID_AUDIO,
	TASK_ID_FLASH,
	TASK_ID_GPS,
	TASK_ID_LCD,
	TASK_ID_HWINV, //Hardware Inventory
	TASK_ID_CLOUD,
	TASK_ID_MODBUS,
	TASK_ID_HSMMP,  //High Speed MultiMedia processing
	TASK_ID_EMC,
	TASK_ID_HUMID,
	TASK_ID_PM25,
	TASK_ID_TEMP,
	TASK_ID_WINDDIR,
	TASK_ID_WINDSPD,
	TASK_ID_SVRCON,
	TASK_ID_MAX,
	TASK_ID_INVALID = 0xFFFFFFFF,
}; //end of HCU_TASK_NAME_ID
//定义TASK对应的MESSAGE_QUEUE的ID
#define HCU_TASK_QUEUE_ID_START 1024
enum HCU_TASK_QUEUE_ID
{
	TASK_QUE_ID_MIN = HCU_TASK_QUEUE_ID_START,
	TASK_QUE_ID_HCUMAIN,
	TASK_QUE_ID_HCUVM,
	TASK_QUE_ID_TRACE,
	TASK_QUE_ID_CONFIG,
	TASK_QUE_ID_TIMER,
	TASK_QUE_ID_MMC, //Man Machine Communication
	TASK_QUE_ID_GPIO,
	TASK_QUE_ID_GRPS,
	TASK_QUE_ID_SPS232,
	TASK_QUE_ID_SPS485,
	TASK_QUE_ID_BLE,
	TASK_QUE_ID_ETHERNET,
	TASK_QUE_ID_WIFI,
	TASK_QUE_ID_USBNET,
	TASK_QUE_ID_3G4G,
	TASK_QUE_ID_HARDDISK,
	TASK_QUE_ID_FLASH,
	TASK_QUE_ID_GPS,
	TASK_QUE_ID_LCD,
	TASK_QUE_ID_HWINV,
	TASK_QUE_ID_CAMERA,
	TASK_QUE_ID_AUDIO,
	TASK_QUE_ID_CLOUD,
	TASK_QUE_ID_MODBUS,
	TASK_QUE_ID_HSMMP,
	TASK_QUE_ID_EMC,
	TASK_QUE_ID_HUMID,
	TASK_QUE_ID_PM25,
	TASK_QUE_ID_TEMP,
	TASK_QUE_ID_WINDDIR,
	TASK_QUE_ID_WINDSPD,
	TASK_QUE_ID_SVRCON,
	TASK_QUE_ID_MAX,
	TASK_QUE_ID_NULL = 0xFFFFFFFF,
}; //end of HCU_TASK_QUEUE_ID

#define TASK_NAME_MAX_LENGTH 10
typedef struct HcuTaskTag
{
	UINT32 TaskId;
	UINT8 hwPlugin;
	UINT8 hwActive;
	UINT8 swTaskActive;
	pthread_t ThrId;
	INT32 QueId;
	UINT8 state;
	char TaskName[TASK_NAME_MAX_LENGTH];
}HcuTaskTag_t;
#define HCU_TASK_PNP_ON 2
#define HCU_TASK_PNP_OFF 1
#define HCU_TASK_PNP_INVALID 0xFF

/*
** Fsm INFORMATION structure.
*/
#define FSM_STATE_ENTRY  0x00
#define FSM_STATE_IDLE  0x01
#define FSM_STATE_END   0xFE
#define FSM_STATE_INVALID 0xFF
/*maxmum state number and msg number in one task, no instance concept*/
#define MAX_MSGID_NUM_IN_ONE_TASK 254 //一个任务中最多定义的消息数量
#define MASK_MSGID_NUM_IN_ONE_TASK 255 //消息号段在同一个任务中必须连续到这个范围内
#define MAX_STATE_NUM_IN_ONE_TASK 16  //一个任务之中最多定义的状态数量
#define MAX_FSM_STATE_ENTRY_NUM_IN_ONE_TASK 32   //一个任务之中，STATE-MSGID成对处理函数最多数量

typedef struct FsmStateItem
{
	UINT32 msg_id;
	UINT8 state;
	OPSTAT (*stateFunc)(UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len);
}FsmStateItem_t;

typedef struct FsmArrayElement
{
	OPSTAT (*stateFunc)(UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len);
}FsmArrayElement_t;

typedef struct FsmCtrlTable
{
	UINT32 numOfFsmArrayElement;  //每一个具体任务TASK中，定义了多少个STATE-MSGID映射表单
	UINT32 taskId;
	FsmArrayElement_t pFsmArray[MAX_STATE_NUM_IN_ONE_TASK][MAX_MSGID_NUM_IN_ONE_TASK];
}FsmCtrlTable_t;

typedef struct FsmTable
{
	UINT32 numOfFsmCtrlTable;  //Number of running (Task + Instance)
	UINT32 currentTaskId;  //transfer task_id to launched FSM machine, then useless
	FsmCtrlTable_t  pFsmCtrlTable[MAX_TASK_NUM_IN_ONE_HCU];
}FsmTable_t;

//Global variables 全局变量声明的好例子，不能加Static/Const等等修饰性，除非的确不改动
extern HcuTaskTag_t zHcuTaskInfo[];
extern char *zHcuTaskNameList[];

//Global VM layer basic API and functions
extern void hcu_vm_system_init(void);  //系统级别的初始化
extern void hcu_sw_restart(void);  //软件状态机初始化, TBD
extern UINT32 hcu_task_create(UINT32 task_id, void *(*task_func)(void *), void *arg, int prio);
extern UINT32 hcu_task_delete(UINT32 task_id);
extern UINT32 hcu_msgque_create(UINT32 task_id);
extern UINT32 hcu_msgque_delete(UINT32 task_id);
extern UINT32 hcu_task_create_and_run(UINT32 task_id, FsmStateItem_t* pFsmStateItem);
extern UINT32 hcu_message_send(UINT32 msg_id, UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len); //message send
extern UINT32 hcu_message_rcv(UINT32 dest_id, HcuMsgSruct_t *msg);
extern UINT32 taskid_to_string(UINT32 id, char *string);
extern UINT32 msgid_to_string(UINT32 id, char *string);
extern UINT32 hcu_timer_polling(time_t sec, UINT32 nsec, void *handler());
extern UINT32 hcu_timer_set(UINT32 timerid, UINT32 taskid, UINT32 delay);
extern UINT32 hcu_timer_clear(UINT32 timerid, UINT32 taskid);
extern void hcu_sleep(UINT32 second);
extern void hcu_usleep(UINT32 usecond);  //resulution 10^(-6)s = 1 microsecond
//UNIX下时钟函数非常丰富，这里不再做任何抽象化，上层应用可以直接调用系统库函数进行使用和处理

/*FSM related APIs */
extern UINT32 FsmInit(void);
extern UINT32 FsmAddNew(UINT32 task_id, FsmStateItem_t* pFsmStateItem);
extern UINT32 FsmRunEngine(UINT32 msg_id, UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len);
extern UINT32 FsmProcessingLaunch(void);
extern OPSTAT FsmSetState(UINT32 task_id, UINT8 newState);
extern UINT8  FsmGetState(UINT32 task_id);


#endif /* COMVM_VMLAYER_H_*/


