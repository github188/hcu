/*
 * timer.c
 *
 *  Created on: 2015年11月15日
 *      Author: test
 */
#include "timer.h"
#include "../comvm/trace.h"

/*
** FSM of the TIMER
*/
FsmStateItem_t FsmTimer[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_timer_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_timer_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_timer_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_TIMER_ACTIVED,            	fsm_timer_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Global variables
HcuTimerTable_t zHcuTimerTable;

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_timer_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret;
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	ret = FsmSetState(TASK_ID_TIMER, FSM_STATE_IDLE);
	if (ret == FAILURE){
		HcuErrorPrint("TIMER: Error Set FSM State at fsm_timer_task_entry\n");
	}
	return SUCCESS;
}

OPSTAT fsm_timer_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Checking source of INIT message, only accept INIT from HCUMAIN.
	//In future, maybe accept more cases, but right now this is the only allowable situation.
	if (src_id != TASK_ID_HCUMAIN){
		HcuErrorPrint("TIMER: Error Init message get from task_id [%s]!\n", zHcuTaskNameList[src_id]);
		return FAILURE;
	}

	//Send back MSG_ID_COM_INIT_FEEDBACK to HCU-MAIN
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_HCUMAIN, TASK_ID_TIMER, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("TIMER: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_TIMER], zHcuTaskNameList[TASK_ID_HCUMAIN]);
		return FAILURE;
	}

	//INIT this task global variables zHcuTimerTable
	memset(&zHcuTimerTable, 0, sizeof(HcuTimerTable_t));

	//收到初始化消息后，进入初始化状态
	ret = FsmSetState(TASK_ID_TIMER, FSM_STATE_TIMER_INITED);
	if (ret == FAILURE){
		HcuErrorPrint("TIMER: Error Set FSM State at fsm_timer_init\n");
		return FAILURE;
	}

	//启动信号量级的永恒定时器，颗粒度只支持两种，1S和10MS的
	/*if (signal(SIGPROF, func_timer_routine) == SIG_ERR){
		HcuErrorPrint("TIMER: Error set signal = SIGPROF for 1S resolution timer!\n");
	}
	if (signal(SIGVTALRM, func_timer_routine) == SIG_ERR){
		HcuErrorPrint("TIMER: Error set signal = SIGVTALRM for 1S resolution timer!\n");
	}
	//signal(SIGALRM, func_timer_routine);
	//常用定时器
	struct itimerval t1s, t10ms;
	zHcuTimerSet1s.iniValue.tvSec = HCU_TIMER_TICK_1_SEC;
	zHcuTimerSet1s.iniValue.tvUsec = 0;
	zHcuTimerSet1s.intValue.tvSec = HCU_TIMER_TICK_1_SEC;
	zHcuTimerSet1s.intValue.tvUsec = 0;
	t1s.it_value.tv_sec = 1;
	t1s.it_value.tv_usec = 0;
	t1s.it_interval.tv_sec = 1;
	t1s.it_interval.tv_usec = 0;
	if (setitimer(ITIMER_PROF, &t1s, NULL) == -1){
		HcuErrorPrint("TIMER: Error set ITIMER_PROF!\n");
		return FAILURE;
	}
	//L2帧协议定时器
	zHcuTimerSet10ms.iniValue.tvSec = 0;
	zHcuTimerSet10ms.iniValue.tvUsec = HCU_TIMER_TICK_10_MS;
	zHcuTimerSet10ms.intValue.tvSec = 0;
	zHcuTimerSet10ms.intValue.tvUsec = HCU_TIMER_TICK_10_MS;
	t10ms.it_value.tv_sec = 0;
	t10ms.it_value.tv_usec = 100000;
	t10ms.it_interval.tv_sec = 0;
	t10ms.it_interval.tv_usec = 100000;
	if (setitimer(ITIMER_VIRTUAL, &t10ms, NULL) == -1){
		HcuErrorPrint("TIMER: Error set ITIMER_VIRTUAL!\n");
		return FAILURE;
	}
	//实时数据流定时器
	zHcuTimerSet1ms.iniValue.tvSec = 0;
	zHcuTimerSet1ms.iniValue.tvUsec = HCU_TIMER_TICK_1_MS;
	zHcuTimerSet1ms.intValue.tvSec = 0;
	zHcuTimerSet1ms.intValue.tvUsec = HCU_TIMER_TICK_1_MS;
	if (setitimer(ITIMER_REAL, &zHcuTimerSet1ms, NULL) == -1){
		HcuErrorPrint("TIMER: Error set ITIMER_REAL\n");
		return FAILURE;
	}*/

	//1s级别常用定时器
    timer_t timer1;
    struct sigevent evp1;
    struct sigaction act;

    //Init figure
    memset(&act, 0, sizeof(act));
    act.sa_handler = func_timer_routine;
    act.sa_flags = 0;
    sigemptyset(&act.sa_mask);
    if (sigaction(SIGUSR1, &act, NULL) == -1) {
    	HcuErrorPrint("TIMER: Error set SIGUSR1 sigaction()!\n");
    	return FAILURE;
    }
    memset(&evp1, 0, sizeof(struct sigevent));
    evp1.sigev_signo = SIGUSR1;
    evp1.sigev_notify = SIGEV_SIGNAL;
    if (timer_create(CLOCK_REALTIME, &evp1, &timer1) == -1) {
    	HcuErrorPrint("TIMER: Error set SIGUSR1 timer_create()!\n");
    	return FAILURE;
    }
    struct itimerspec it;
    it.it_interval.tv_sec = HCU_TIMER_TICK_1_SEC;
    it.it_interval.tv_nsec = 0;
    it.it_value.tv_sec = HCU_TIMER_TICK_1_SEC;
    it.it_value.tv_nsec = 0;
    if (timer_settime(timer1, 0, &it, 0) == -1) {
    	HcuErrorPrint("TIMER: Error set SIGUSR1 timer_settime()!\n");
    	return FAILURE;
    }

    //L2 10ms 帧协议定时器, too much impact on sleep function, not yet start!
    struct sigevent evp;
    struct itimerspec ts;
    timer_t timer;

    memset (&evp, 0, sizeof (evp));
    evp.sigev_value.sival_ptr = &timer;
    evp.sigev_notify = SIGEV_THREAD;
    evp.sigev_notify_function = func_timer_routine_handler_10ms;
    evp.sigev_value.sival_int = 3; //作为func_timer_routine_handler_10ms的参数

    if (timer_create(CLOCK_REALTIME, &evp, &timer) == -1) {
    	HcuErrorPrint("TIMER: Error create timer timer_create()!\n");
    	return FAILURE;
    }

    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = HCU_TIMER_TICK_10_MS_IN_NS;
    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = HCU_TIMER_TICK_10_MS_IN_NS;

    if (timer_settime(timer, TIMER_ABSTIME, &ts, NULL) == -1) {
    	HcuErrorPrint("TIMER: Error set timer timer_settime()!\n");
    	return FAILURE;
    }

	//进入等待反馈状态
	ret = FsmSetState(TASK_ID_TIMER, FSM_STATE_TIMER_ACTIVED);
	if (ret == FAILURE){
		HcuErrorPrint("TIMER: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("TIMER: Enter FSM_STATE_TIMER_ACTIVED status, Keeping loop of signal sending here!\n");
	#endif //(TRACE_DEBUG_ON)

	while(1) {
		pause();
	  }
	return SUCCESS;
}

OPSTAT fsm_timer_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

void func_timer_routine(int signo)
{
	int i=0, ret=0;
	msg_struct_com_time_out_t snd;

	switch (signo){
	case SIGUSR1:
	{
		//HcuDebugPrint("TIMER: SIGPROF 1S resolution timer enter one time!\n");
		for (i=0;i<MAX_TIMER_NUM_IN_ONE_HCU_1S;i++)
		{
			//Active status
			if (zHcuTimerTable.timer1s[i].status == TIMER_STATUS_ACTIVE){
				if (zHcuTimerTable.timer1s[i].elapse > zHcuTimerTable.timer1s[i].tDuration){
					HcuErrorPrint("TIMER: Error elapse value meet!\n");
					zHcuTimerTable.timer1s[i].elapse = zHcuTimerTable.timer1s[i].elapse % zHcuTimerTable.timer1s[i].tDuration;
				}
				if (zHcuTimerTable.timer1s[i].elapse >= 1){
					//防止出现潜在问题，求余再减1
					zHcuTimerTable.timer1s[i].elapse--;
				}else{
					//change ACTIVE to STOP, no
					if (zHcuTimerTable.timer1s[i].timerType == TIMER_TYPE_ONE_TIME){
						zHcuTimerTable.timer1s[i].status = TIMER_STATUS_STOP;
						zHcuTimerTable.timer1s[i].elapse = 0;
					}
					//No change for PERIOD, re-start this timer
					//本轮也算1秒，所以需要在周期的基础上减1
					else if (zHcuTimerTable.timer1s[i].timerType == TIMER_TYPE_PERIOD){
						zHcuTimerTable.timer1s[i].elapse = zHcuTimerTable.timer1s[i].tDuration-1;
					}
					//Send message time_out
					memset(&snd, 0, sizeof(msg_struct_com_time_out_t));
					snd.length = sizeof(msg_struct_com_time_out_t);
					snd.timeId = zHcuTimerTable.timer1s[i].timerId;
					snd.timeRes = zHcuTimerTable.timer1s[i].timerRes;
					ret = hcu_message_send(MSG_ID_COM_TIME_OUT, zHcuTimerTable.timer1s[i].taskId, TASK_ID_TIMER, &snd, snd.length);
					if (ret == FAILURE){
						HcuErrorPrint("TIMER: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_TIMER], zHcuTaskNameList[zHcuTimerTable.timer1s[i].taskId]);
						return;}
				}//Elapse <= 0, timeout reach
			}
			//START status
			else if(zHcuTimerTable.timer1s[i].status == TIMER_STATUS_START){
				zHcuTimerTable.timer1s[i].elapse = zHcuTimerTable.timer1s[i].tDuration;
				zHcuTimerTable.timer1s[i].status = TIMER_STATUS_ACTIVE;
			}//Not meet ACTIVE or START timer
		}//for-MAX_TIMER_NUM_IN_ONE_HCU_1S
	   break;
	}//SIGUSR1
	/*case SIGUSR2:
	{
		HcuDebugPrint("TIMER: SIGVTALRM 10MS resolution timer enter one time!");
		for (i=0;i<MAX_TIMER_NUM_IN_ONE_HCU_10MS;i++){
			if (zHcuTimerTable.timer10ms[i].status == TIMER_STATUS_ACTIVE){
				if (zHcuTimerTable.timer10ms[i].elapse >= 1){
					//防止出现潜在问题，求余再减1
					zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].elapse % zHcuTimerTable.timer10ms[i].tDuration - 1;
				}else{
					zHcuTimerTable.timer10ms[i].elapse = 0;
					//change ACTIVE to STOP, no
					if (zHcuTimerTable.timer10ms[i].timerType == TIMER_TYPE_ONE_TIME){
						zHcuTimerTable.timer10ms[i].status = TIMER_STATUS_STOP;
						zHcuTimerTable.timer10ms[i].elapse = 0;
					}
					//No change for PERIOD, re-start this timer
					else if (zHcuTimerTable.timer10ms[i].timerType == TIMER_TYPE_PERIOD){
						zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].tDuration;
					}
					//Send message time_out
					memset(&snd, 0, sizeof(msg_struct_com_time_out_t));
					snd.length = sizeof(msg_struct_com_time_out_t);
					snd.timeId = zHcuTimerTable.timer10ms[i].taskId;
					snd.timeRes = zHcuTimerTable.timer10ms[i].timerRes;
					ret = hcu_message_send(MSG_ID_COM_TIME_OUT, zHcuTimerTable.timer10ms[i].taskId, TASK_ID_TIMER, &snd, snd.length);
					if (ret == FAILURE){
						HcuErrorPrint("TIMER: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_TIMER], zHcuTaskNameList[zHcuTimerTable.timer10ms[i].taskId]);
						return;}
				}//Elapse <= 0, timeout reach
			//START status
			}else if(zHcuTimerTable.timer10ms[i].status == TIMER_STATUS_START){
				zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].tDuration;
				zHcuTimerTable.timer10ms[i].status = TIMER_STATUS_ACTIVE;
			}//Not meet ACTIVE or START timer
		}//for-MAX_TIMER_NUM_IN_ONE_HCU_10MS
		break;
	}//SIGUSR2*/
	/*case SIGALRM:
	{
		HcuDebugPrint("TIMER: SIGALRM 1MS resolution timer enter one time!");
		signal(SIGALRM, func_timer_routine);
		for (i=0;i<MAX_TIMER_NUM_IN_ONE_HCU_1MS;i++){
			if (zHcuTimerTable.timer1ms[i].status == TIMER_STATUS_ACTIVE){
				if (zHcuTimerTable.timer1ms[i].elapse >= 1){
					//防止出现潜在问题，求余再减1
					zHcuTimerTable.timer1ms[i].elapse = zHcuTimerTable.timer1ms[i].elapse % zHcuTimerTable.timer1ms[i].tDuration - 1;
				}else{
					zHcuTimerTable.timer1ms[i].elapse = 0;
					//change ACTIVE to STOP, no
					if (zHcuTimerTable.timer1ms[i].timerType == TIMER_TYPE_ONE_TIME){
						zHcuTimerTable.timer1ms[i].status = TIMER_STATUS_STOP;
						zHcuTimerTable.timer1ms[i].elapse = 0;
					}
					//No change for PERIOD, re-start this timer
					else if (zHcuTimerTable.timer1ms[i].timerType == TIMER_TYPE_PERIOD){
						zHcuTimerTable.timer1ms[i].elapse = zHcuTimerTable.timer1ms[i].tDuration;
					}
					//Send message time_out
					memset(&snd, 0, sizeof(msg_struct_com_time_out_t));
					snd.length = sizeof(msg_struct_com_time_out_t);
					snd.timeId = zHcuTimerTable.timer1ms[i].taskId;
					snd.timeRes = zHcuTimerTable.timer1ms[i].timerRes;
					ret = hcu_message_send(MSG_ID_COM_TIME_OUT, zHcuTimerTable.timer1ms[i].taskId, TASK_ID_TIMER, &snd, snd.length);
					if (ret == FAILURE){
						HcuErrorPrint("TIMER: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_TIMER], zHcuTaskNameList[zHcuTimerTable.timer1ms[i].taskId]);
						return;}
				}//Elapse <= 0, timeout reach
			//START status
			}else if(zHcuTimerTable.timer1ms[i].status == TIMER_STATUS_START){
				zHcuTimerTable.timer1ms[i].elapse = zHcuTimerTable.timer1ms[i].tDuration;
				zHcuTimerTable.timer1ms[i].status = TIMER_STATUS_ACTIVE;
			}//Not meet ACTIVE or START ti -l
			mer
		}//for-MAX_TIMER_NUM_IN_ONE_HCU_1MS
		break;
	}//SIGALRM*/
	default:
	{
		break;
	}//default
	return;
	}//switch
}//end of func_timer_routine

//线程机制，搞定时器
void func_timer_routine_handler_10ms(union sigval v)
{
	int i=0, ret=0;
	msg_struct_com_time_out_t snd;

	if (v.sival_int !=3) return;

	//HcuDebugPrint("TIMER: 10ms resolution timer enter one time!\n");
	for (i=0;i<MAX_TIMER_NUM_IN_ONE_HCU_10MS;i++)
	{
		//Active status
		if (zHcuTimerTable.timer10ms[i].status == TIMER_STATUS_ACTIVE){
			if (zHcuTimerTable.timer10ms[i].elapse > zHcuTimerTable.timer10ms[i].tDuration){
				HcuErrorPrint("TIMER: Error elapse value meet!\n");
				zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].elapse % zHcuTimerTable.timer10ms[i].tDuration;
			}
			if (zHcuTimerTable.timer10ms[i].elapse >= 1){
				//防止出现潜在问题，求余再减1
				zHcuTimerTable.timer10ms[i].elapse--;
			}else{
				//change ACTIVE to STOP, no
				if (zHcuTimerTable.timer10ms[i].timerType == TIMER_TYPE_ONE_TIME){
					zHcuTimerTable.timer10ms[i].status = TIMER_STATUS_STOP;
					zHcuTimerTable.timer10ms[i].elapse = 0;
				}
				//No change for PERIOD, re-start this timer
				//本轮也算1次，所以需要在周期的基础上减1，不然会出现问题
				else if (zHcuTimerTable.timer10ms[i].timerType == TIMER_TYPE_PERIOD){
					zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].tDuration-1;
				}
				//Send message time_out
				memset(&snd, 0, sizeof(msg_struct_com_time_out_t));
				snd.length = sizeof(msg_struct_com_time_out_t);
				snd.timeId = zHcuTimerTable.timer10ms[i].timerId;
				snd.timeRes = zHcuTimerTable.timer10ms[i].timerRes;
				ret = hcu_message_send(MSG_ID_COM_TIME_OUT, zHcuTimerTable.timer10ms[i].taskId, TASK_ID_TIMER, &snd, snd.length);
				if (ret == FAILURE){
					HcuErrorPrint("TIMER: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_TIMER], zHcuTaskNameList[zHcuTimerTable.timer10ms[i].taskId]);
					return;}
			}//Elapse <= 0, timeout reach
		}
		//START status
		else if(zHcuTimerTable.timer10ms[i].status == TIMER_STATUS_START){
			zHcuTimerTable.timer10ms[i].elapse = zHcuTimerTable.timer10ms[i].tDuration;
			zHcuTimerTable.timer10ms[i].status = TIMER_STATUS_ACTIVE;
		}//Not meet ACTIVE or START timer
	}//for-MAX_TIMER_NUM_IN_ONE_HCU_10MS

}


//由于SIGALRM信号量太强大，没法中断，故而先不实现1MS精度的时钟，未来再考虑方式
OPSTAT hcu_timer_start(UINT32 task_id, UINT32 timer_id, UINT32 t_dur, UINT8 t_type, UINT8 t_res)
{
	//检查task_id是否合法
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("TIMER: Error on timer start src_id =%d [%s]!!!\n", task_id, zHcuTaskNameList[task_id]);
		return FAILURE;
	}

	//检查timer_duartion是否合法
	if ((t_dur > MAX_TIMER_SET_DURATION) || (t_dur <=0)){
		HcuErrorPrint("TIMER: Error on timer start timer duration!!!\n");
		return FAILURE;
	}

	//检查t_type是否合法
	if ((t_type != TIMER_TYPE_ONE_TIME) && (t_type != TIMER_TYPE_PERIOD)){
		HcuErrorPrint("TIMER: Error on timer start type!!!\n");
		return FAILURE;
	}

	//检查t_resolution & timer_id 是否合法
	if (t_res == TIMER_RESOLUTION_1S){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_1S) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer start 1S timerId!!!\n");
			return FAILURE;
		}
	}else if (t_res == TIMER_RESOLUTION_10MS){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_10MS) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer start 10MS timerId!!!\n");
			return FAILURE;
		}
	}
	/*else if (t_res == TIMER_RESOLUTION_1MS){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_1MS) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer start 1MS timerId!!!\n");
			return FAILURE;
		}
	}*/
	else{
		HcuErrorPrint("TIMER: Error on timer start timer resolution!!!\n");
		return FAILURE;
	}

	//设置进全局数据表单
	if (t_res == TIMER_RESOLUTION_1S){
		zHcuTimerTable.timer1s[timer_id].taskId = task_id;
		zHcuTimerTable.timer1s[timer_id].timerId = timer_id;
		zHcuTimerTable.timer1s[timer_id].tDuration = t_dur;
		zHcuTimerTable.timer1s[timer_id].timerType = t_type;
		zHcuTimerTable.timer1s[timer_id].timerRes = TIMER_RESOLUTION_1S;
		zHcuTimerTable.timer1s[timer_id].status = TIMER_STATUS_START;
	}
	else if (t_res == TIMER_RESOLUTION_10MS){
		zHcuTimerTable.timer10ms[timer_id].taskId = task_id;
		zHcuTimerTable.timer10ms[timer_id].timerId = timer_id;
		zHcuTimerTable.timer10ms[timer_id].tDuration = t_dur;
		zHcuTimerTable.timer10ms[timer_id].timerType = t_type;
		zHcuTimerTable.timer10ms[timer_id].timerRes = TIMER_RESOLUTION_10MS;
		zHcuTimerTable.timer10ms[timer_id].status = TIMER_STATUS_START;
	}/*else if (t_res == TIMER_RESOLUTION_1MS){
		zHcuTimerTable.timer1ms[timer_id].taskId = task_id;
		zHcuTimerTable.timer1ms[timer_id].timerId = timer_id;
		zHcuTimerTable.timer1ms[timer_id].tDuration = t_dur;
		zHcuTimerTable.timer1ms[timer_id].timerType = t_type;
		zHcuTimerTable.timer1ms[timer_id].timerRes = TIMER_RESOLUTION_1MS;
		zHcuTimerTable.timer1ms[timer_id].status = TIMER_STATUS_START;
	}*/

	return SUCCESS;
}

OPSTAT hcu_timer_stop(UINT32 task_id, UINT32 timer_id, UINT8 t_res)
{
	//检查task_id是否合法
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("TIMER: Error on timer stop src_id =%d [%s]!!!\n", task_id, zHcuTaskNameList[task_id]);
		return FAILURE;
	}

	//检查t_resolution & timer_id 是否合法
	if (t_res == TIMER_RESOLUTION_1S){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_1S) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer stop 1S timerId!!!\n");
			return FAILURE;
		}
	}else if (t_res == TIMER_RESOLUTION_10MS){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_10MS) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer stop 10MS timerId!!!\n");
			return FAILURE;
		}
	}else if (t_res == TIMER_RESOLUTION_1MS){
		if ((timer_id > MAX_TIMER_NUM_IN_ONE_HCU_1MS) || (timer_id <0)){
			HcuErrorPrint("TIMER: Error on timer stop 1MS timerId!!!\n");
			return FAILURE;
		}
	}else{
		HcuErrorPrint("TIMER: Error on timer stop timer resolution!!!\n");
		return FAILURE;
	}

	//设置进全局数据表单
	if (t_res == TIMER_RESOLUTION_1S){
		if ((zHcuTimerTable.timer1s[timer_id].taskId == task_id) && (zHcuTimerTable.timer1s[timer_id].timerId == timer_id)){
			zHcuTimerTable.timer1s[timer_id].status = TIMER_STATUS_STOP;
		}else{
			zHcuTimerTable.timer1s[timer_id].status = TIMER_STATUS_DEACTIVE;
		}
	}else if (t_res == TIMER_RESOLUTION_10MS){
		if ((zHcuTimerTable.timer10ms[timer_id].taskId == task_id) && (zHcuTimerTable.timer10ms[timer_id].timerId == timer_id)){
			zHcuTimerTable.timer10ms[timer_id].status = TIMER_STATUS_STOP;
		}else{
			zHcuTimerTable.timer10ms[timer_id].status = TIMER_STATUS_DEACTIVE;
		}
	}else if (t_res == TIMER_RESOLUTION_1MS){
		if ((zHcuTimerTable.timer1ms[timer_id].taskId == task_id) && (zHcuTimerTable.timer1ms[timer_id].timerId == timer_id)){
			zHcuTimerTable.timer1ms[timer_id].status = TIMER_STATUS_STOP;
		}else{
			zHcuTimerTable.timer1ms[timer_id].status = TIMER_STATUS_DEACTIVE;
		}
	}
	return SUCCESS;
}
