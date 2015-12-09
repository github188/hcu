/*
 * vm_layer.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "vmlayer.h"
#include "trace.h"

//全局变量，存储所有任务的状态信息，以便后面后面使用

HcuTaskTag_t zHcuTaskInfo[MAX_TASK_NUM_IN_ONE_HCU];
//请确保，该全局字符串的定义跟Task_Id的顺序保持完全一致，不然后面的显示内容会出现差错
//请服从最长长度TASK_NAME_MAX_LENGTH的定义，不然Debug/Trace打印出的信息也会出错

char *zHcuTaskNameList[MAX_TASK_NUM_IN_ONE_HCU] =
	{"MIN", "HCUMAIN", "HCUVM", "TRACE", "CONFIG", "TIMER", "MMC", "GPIO", "GRPS", "SPS232", "SPS485", "BLE", \
	"ETHERNET", "WIFI", "USBNET", "3G4G", "HARDDISK", "CAMERA", "AUDIO", "FLASH", "GPS", "LCD", "HWINV", "CLOUD", \
	"MODBUS", "HSMMP", "EMC", "HUMID", "PM25", "TEMP", "WINDDIR", "WINDSPD", "SVRCON", "MAX", "NULL"};
//任务状态机FSM全局控制表，占用内存的大户！！！
FsmTable_t zHcuFsmTable;

//消息ID的定义全局表，方便TRACE函数使用
//请服从MSG_NAME_MAX_LENGTH的最长定义，不然出错
//该全局数组暂时没用，后期优化，TBD
char *zHcuMsgNameList[MAX_MSGID_NUM_IN_ONE_TASK] ={};


//INIT the whole system
void hcu_vm_system_init(void)
{
	//INIT HCU itself
	HcuDebugPrint("HCU-VM: Application Layer start and initialized! Build: %s, %s\n", __DATE__, __TIME__);
	HcuDebugPrint("HCU-VM: Current Hardware Type = %d, Hardware Module = %d\n", CURRENT_HW_TYPE, CURRENT_HW_MODULE);
	HcuDebugPrint("HCU-VM: Current Software Release = %d, Software Delivery = %d\n", CURRENT_SW_RELEASE, CURRENT_SW_DELIVERY);

    //Print MAC address, to show the hardware physical identity
    //需要进一步确定MAC地址和相应硬件标识的来源
	UINT8 MacAddress[]={0,0,0,0,0,0};
	HcuDebugPrint("HCU-VM: Hardware MAC Address = %02X %02X %02X %02X %02X %02X\n", MacAddress[5], MacAddress[4], MacAddress[3], MacAddress[2], MacAddress[1], MacAddress[0]);

	//初始化全局变量TASK_ID/QUE_ID/TASK_STAT
	memset(&(zHcuTaskInfo[0].TaskId), 0, sizeof(zHcuTaskInfo)*(TASK_ID_MAX-TASK_ID_MIN+1));
	int i=0;
	for (i=TASK_ID_MIN; i<TASK_ID_MAX; i++){
		zHcuTaskInfo[i].TaskId = i;
		strcpy(zHcuTaskInfo[i].TaskName, zHcuTaskNameList[i]);
	}

	//TrACE INIT
	TraceInit();

	//Init Fsm
	FsmInit();



}

/**************************************************************************************
 * 创建任务，使用的是线程模拟，各个任务之间可以共享内存
 * To Create a thread
 * void  hcu_task_create( UINT32 task_id,          		out: pointer to zHcuTaskId[task_id]
 *                   void *(*task_func)(void *),        in: task's entry point
 *                   void *arg,                         in: parameter of the task's function
 *                   int prio )                         in: priority of the task*
 ***************************************************************************************/
UINT32 hcu_task_create(UINT32 task_id, void *(*task_func)(void *), void *arg, int prio)
{
	int                     err;
	pthread_attr_t          attr;
	struct  sched_param     sched;

	// The thread is initialized with attributes object default values
	if(0!= (err = pthread_attr_init(&attr)))
	{
		HcuDebugPrint("HCU-VM: pthread_attr_init() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	// Change the inherit scheduling attribute
	// of the specified thread attribute object,because PTHREAD_INHERIT_SCHED is default value
	if (0!= (err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)))
	{
		HcuDebugPrint("HCU-VM: pthread_attr_setinheritsched() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	// Set the contention scope attribute of
	// the specified thread attributes object, the thread compete with all threads on the station
	if (0 != (err = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM)))
	{
		HcuDebugPrint("HCU-VM: pthread_attr_setscope() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	// Set the scheduling policy,SCHED_OTHER is the default
	if (0 != (err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO)))
	{
		HcuDebugPrint("HCU-VM: pthread_attr_setschedpolicy() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	// Set the priority of the task
	sched.sched_priority = prio;
	if (0 != (err = pthread_attr_setschedparam(&attr, &sched)))
	{
		HcuErrorPrint("HCU-VM: pthread_attr_setschedparam() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	// Checking task_id
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id!! err=%d, errno=%d, %s, task_id=%d\n", err, errno, strerror(err), task_id);
		return FAILURE;
	}

	//As FsmProcessLaunch has to transfer task_id in, we will use global variance of zHcuFsmTable.currentTaskId
	//So task_id has to store into there
	//Wonderful mechanism!!!
	if (zHcuFsmTable.currentTaskId != TASK_ID_INVALID){
		hcu_sleep(1);
	}
	if (zHcuFsmTable.currentTaskId != TASK_ID_INVALID){
		HcuErrorPrint("HCU-VM: Task_id not yet read by running process, new task create failure!\n");
		return FAILURE;
	}
	zHcuFsmTable.currentTaskId = task_id;

	// creation of the task
	err=pthread_create(&(zHcuTaskInfo[task_id].ThrId), &attr, (void *(*)(void*))(task_func), (void*)arg);
	if(err != 0)
	{
		HcuDebugPrint("HCU-VM: pthread_create() nok!! err=%d, errno=%d, %s\n", err, errno, strerror(err));
		return FAILURE;
	}

	//zHcuTaskInfo[task_id].TaskName to be added in another way, TBD
	zHcuTaskInfo[task_id].TaskId = task_id;
	HcuDebugPrint("HCU-VM: pthread_create() OK ...\n");
	/* ERRORS
	       EAGAIN Insufficient resources to create another thread, or a system-imposed limit on the number of threads was encountered.  The latter case may occur in two ways: the RLIMIT_NPROC soft  resource  limit
	              (set via setrlimit(2)), which limits the number of process for a real user ID, was reached; or the kernel's system-wide limit on the number of threads, /proc/sys/kernel/threads-max, was reached.

	       EINVAL Invalid settings in attr.

	       EPERM  No permission to set the scheduling policy and parameters specified in attr.
	 */
	return SUCCESS;
}

//kill task
UINT32 hcu_task_delete(UINT32 task_id)
{
	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	//Not exist
	if (zHcuTaskInfo[task_id].ThrId ==0){return FAILURE;}

	//SIGKILL用于杀死整个进程, 返回为ESRCH则表示线程不存在
	if (pthread_kill(zHcuTaskInfo[task_id].ThrId, 0) != (EINVAL || ESRCH)) {
		pthread_kill(zHcuTaskInfo[task_id].ThrId, SIGQUIT);
		return SUCCESS;
	}else{
		return FAILURE;
	}
}

/**************************************************************************************
 * 创建任务队列，以便用于任务之间消息通信, 每个任务创建唯一一个队列，所以使用TASKID作为索引
 * void  Hcu_MsgQue_Create( UINT task_id)
 ***************************************************************************************/
UINT32 hcu_msgque_create(UINT32 task_id)
{
	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	//Generate msgKey
	int msgKey=0, msgQid=0;
	msgKey = task_id + HCU_TASK_QUEUE_ID_START;
	if ((msgKey < TASK_QUE_ID_MIN) || (msgKey > TASK_QUE_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, msgKey=%d!!!\n", msgKey);
		return FAILURE;
	}
	//Checking msgQid exiting or not, if YES, just DELETE.
	msgQid=msgget(msgKey,IPC_EXCL);  /*检查消息队列是否存在*/
	if (msgQid>=0){
		//HcuErrorPrint("HCU-VM: Existing Queue id error, input Key = %d\n", msgKey);
		//删除消息队列
		if (msgctl(msgQid, IPC_RMID, NULL) == FAILURE){
			HcuErrorPrint("HCU-VM: Remove Queue error!\n");
			return FAILURE;
		}
		//HcuErrorPrint("HCU-VM: Already remove Queue Id, input Key = %d\n", msgKey);
	}
	//Re-create the relevant message queue Id
	msgQid = msgget(msgKey,IPC_CREAT|0666);/*创建消息队列*/
	if(msgQid <0){
		HcuErrorPrint("HCU-VM: Failed to create msq que | errno=%d [%s]\n", errno, strerror(errno));
		return FAILURE;
	}
	zHcuTaskInfo[task_id].QueId = msgQid;
	return SUCCESS;
}

//kill message queue
UINT32 hcu_msgque_delete(UINT32 task_id)
{
	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	//Not exist
	//特殊情况下，msgqid=0也是可能的，这种情形，需要再仔细考虑
	if (zHcuTaskInfo[task_id].QueId == 0) {return FAILURE;}

	//Delete queue
	msgctl(zHcuTaskInfo[task_id].QueId,IPC_RMID,NULL); //删除消息队列

	zHcuTaskInfo[task_id].QueId =0;
	return SUCCESS;
}

//聚合创建任务，消息队列，并直接让其开始运行
UINT32 hcu_task_create_and_run(UINT32 task_id, FsmStateItem_t* pFsmStateItem)
{
	OPSTAT ret = 0;

	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Input Error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	//Init Fsm table
	ret = FsmAddNew(task_id, pFsmStateItem);
	if (ret == FAILURE){
		HcuErrorPrint("HCU-VM: Init state machine FsmAddNew error, taskid = %d\n", task_id);
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("HCU-VM: FsmAddNew Successful, taskId = 0x%x [%s].\n", task_id, zHcuTaskNameList[task_id]);
	#endif //(TRACE_DEBUG_ON)

    //Create Queid
   	ret = hcu_msgque_create(task_id);
    if (ret == FAILURE)
    {
	#if (TRACE_DEBUG_ON)
    	HcuErrorPrint("HCU-VM: Create queue unsuccessfully, taskId = %d\n", task_id);
	#endif //(TRACE_DEBUG_ON)
    	return FAILURE;
    }
	#if (TRACE_DEBUG_ON)
    HcuDebugPrint("HCU-VM: hcu_msgque_create Successful, taskId = 0x%x [%s].\n", task_id, zHcuTaskNameList[task_id]);
	#endif //(TRACE_DEBUG_ON)

    //Create task and make it running for ever
    ret = hcu_task_create(task_id, (CALLBACK)FsmProcessingLaunch, (void *)NULL, HCU_THREAD_PRIO);
    if (ret == FAILURE)
    {
    	HcuErrorPrint("HCU-VM: Create task un-successfully, taskid = %d\n", task_id);
    	return FAILURE;
    }
	#if (TRACE_DEBUG_ON)
    HcuDebugPrint("HCU-VM: hcu_task_create Successful, taskId = 0x%x [%s].\n", task_id, zHcuTaskNameList[task_id]);
	#endif //(TRACE_DEBUG_ON)

	#if (TRACE_DEBUG_ON)
	HcuDebugPrint("HCU-VM: Whole task environment setup successful, taskId = 0x%x [%s].\n", task_id, zHcuTaskNameList[task_id]);
	#endif //(TRACE_DEBUG_ON)
	return SUCCESS;
}


//message send basic processing
//All in parameters
UINT32 hcu_message_send(UINT32 msg_id, UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len)
{
	//Checking task_id range
	if ((dest_id < TASK_ID_MIN) || (dest_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, dest_id=%d!!!\n", dest_id);
		return FAILURE;
	}
	if ((src_id < TASK_ID_MIN) || (src_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, src_id=%d!!!\n", src_id);
		return FAILURE;
	}
	if (param_len>MAX_HCU_MSG_BODY_LENGTH){
		HcuErrorPrint("HCU-VM: Too large message length than HCU set capability, param_len=%d!!!\n", param_len);
		return FAILURE;
	}

	//Starting to process message send
	//Not use malloc/free function, as it is a little bit complex to manage, to be checked here the memory conflict
	int ret=0;
	HcuMsgSruct_t *msg;
	msg = malloc(sizeof(HcuMsgSruct_t));
	if (msg == NULL){
		HcuErrorPrint("HCU-VM: Message send allocate memory error, exit!\n");
		return FAILURE;
	}
	//Init to clean this memory area
	//初始化消息内容，是为了稳定可靠安全
	memset(msg, 0, sizeof(HcuMsgSruct_t));
	msg->msgType = msg_id;
	msg->dest_id = dest_id;
	msg->src_id = src_id;
	msg->msgLen = param_len;
	memcpy(&(msg->msgBody[0]), param_ptr, param_len);
	//msg->msgBody是否可以直接当指针？

	/*没有采用精确的长度，也是为了程序简单稳定可靠，所有队列长度都是一致相等的
	msgflg
	0：当消息队列满时，msgsnd将会阻塞，直到消息能写进消息队列
	IPC_NOWAIT：当消息队列已满的时候，msgsnd函数不等待立即返回
	IPC_NOERROR：若发送的消息大于size字节，则把该消息截断，截断部分将被丢弃，且不通知发送进程。*/
	/*MsgSend Return 错误代码
	EAGAIN：参数msgflg设为IPC_NOWAIT，而消息队列已满
	EIDRM：标识符为msqid的消息队列已被删除
	EACCESS：无权限写入消息队列
	EFAULT：参数msgp指向无效的内存地址
	EINTR：队列已满而处于等待情况下被信号中断
	EINVAL：无效的参数msqid、msgsz或参数消息类型type小于0  */
	ret = msgsnd(zHcuTaskInfo[dest_id].QueId, msg, (sizeof(HcuMsgSruct_t)-sizeof(long)), IPC_NOWAIT);
	free(msg);
	if ( ret < 0 ) {
		HcuErrorPrint("HCU-VM: msgsnd() write msg failed, errno=%d[%s]\n",errno,strerror(errno));
		return FAILURE;
	}

	//Message Trace proccessing
	//注意字符串长度，太小会出现内存飞掉的情形
	//未来如果有机会，这部分最好能做成内存动态申请，以便确保安全
#if (TRACE_MSG_ON)
	char s1[TASK_NAME_MAX_LENGTH+2]="", s2[TASK_NAME_MAX_LENGTH+2]="", s3[MSG_NAME_MAX_LENGTH]="";
	taskid_to_string(dest_id, s1);
	taskid_to_string(src_id, s2);
	msgid_to_string(msg_id, s3);
	if ((TRACE_MSG_ON == 1) || ((TRACE_MSG_ON == 2) && (msg_id != MSG_ID_COM_TIME_OUT))){
		HcuDebugPrint("TRACE: MSGID=%02X%s,DID=%02X%s,SID=%02X%s,LEN=%d\n", msg_id, s3, dest_id, s1, src_id, s2, param_len);
	}
#endif //TRACE_MSG_ON

	return SUCCESS;
}

/*
Linux的进程间通讯方式有多种：如通过网卡转发的UDP或TCP方式；消息队列；共享内存；具体怎么编写程序我就不写了，大家在网上google一下什么都有了。在这里我主要说一下用linux的消息队列如何实现可靠的进程间通讯。这里的“进程间”指的是同一个PC上的，跨PC的只能用网络方式实时传递消息。
IPC之消息队列的优势：
为什么要用消息队列呢？根据我所见的，大部分人都是用udp或tcp 进行本机进程间通讯，但一旦网卡失效，程序就歇菜了。我在负责开发一个firmware时就要考虑这个问题，从而写了一个可靠的基于消息队列的进程间通讯。
为什么说要可靠的呢？实验表明，简单的一个消息队列通讯是会有一些问题的：
1.     程序异常退出后，队列不会自动删除。
2.     当通讯双方的一方退出后重新链接的问题。
3.     软件升级时，消息体长度发生变化时，新老版本兼容问题。
4.     1对多或多对多通讯问题。

下面以我的程序IPC模块为例介绍一下，本程序实现了可靠的，1对多，自动重连功能的IPC通讯。IPC模块分为server和cline两部分。 Server部分运行在我们的平台上，cline部分已静态库的方式提供API给应用程序。

Server线程的执行流程是：
1.     编写程序时为server及各个cline分配一个ipc id，如：
typedef enum t_fwipc_id{
IPC_SVR_ID = 1,        //ipc server id
APP1_IPC_ID,           //ipc cline id
APP2_IPC_ID,           //ipc cline id
MAX_IPC_ID             //IPC id count, always the last one
}FW_IPC_ID;
2.     初始化消息队列：创建消息队列，如果已存在，要先删除后再创建。
3.     循环接收消息：消息接受buf清0，msgrcv采用阻塞方式接收消息，如果msgrcv返回失败或消息体长度为0，重新创建消息队列，重复step 1。如果接收成功，则处理消息，返回消息处理结果。IPC通道状态标志置位。然后继续等待消息。

Cline初始化流程：
1.     检测server进程是否存在。（如何检测及如果防止多个server同时运行，我会在另外一篇文章介绍），如果不在，退出。
2.     初始化消息队列
3.     发送注册消息给server。把由server事先定义的ipc cline id发送给server。Server接到消息后会核对IPC cline版本信息。置IPC通道状态标志。
4.     cline设置自己的IPC通道状态标志。
5.     APP可以调用IPC cline API与firmware平台交互信息了。

Cline API发送消息的流程：
1.     发生消息不能用阻塞方式，而是IPC_NOWAIT方式。
2.     每个IPC cline的API在被调用时，都应先检查IPC通道标志，如果标志为“0”，则需要重新执行初始化流程。
3.     检查互斥锁。一个IPC cline同一时间只允许一个API被调用，一个API被调用后，收到server的答复后，释放锁并返回后，下一次API调用才能成功。
4.     每个API有一个特定的超时重试值，这个值要根据server在处理这个消息所费时间来决定的。如果在指定的时间（try次数×try间隔延时）里，server没有返回处理结果，则API返回失败，清IPC通道标志。如果消息发送失败或接收server应答失败，则也清除通道标准，并返回。
*/


/*
 * 慢系统调用的问题
Android平台下程序中的一个线程使用setitimer，而主进程则作为套接字接收服务器，但是却经常出现 accept返回Interrupted system call错误，
追寻原因及解决方法：

出自unix网络编程（第三版）第五章 P115页。

我们用术语慢系统调用（slow system call）描述accept函数，该术语也适用于那些可能永远阻塞的系统调用。永远阻塞的系统调用有可能永远无法返回，多数网络支持函数都属于这一类。举例来说，如果没有客户连接到服务器上，那么服务器的accept调用就没有返回的保证，类似的如果客户端从未发送过数据，那么read调用将永不返回。其他慢系统调用的例子是对管道和终端设备的读和写。一个值得注意的例外是磁盘IO，它们一般都会返回到调用者（假设没有灾难性的硬件事故）。
  适用于慢系统调用的基本规则是：当阻塞于某个慢系统调用的一个进程捕获某个信号且相应处理函数返回时，该系统调用可能返回一个EINTR错误。所以，我们必须对慢系统调用返回的EINTR有所准备。
  为了处理被中断的accept,可以包装证：
 for (;;) {
    if ((connfd = accept(listenfd, NULL, NULL)) < 0) {
  if (errno == EINTR)
    continue;
  else
    err_sys("accept error");
    }
        }
  这段代码所做的事情就是自己重启被中断的系统调用。对于accept以及诸如read、write、select和open之类的函数，这是适合的。不过有一个函数我们不能重启:connect。如果该函数返回EINTR，我们就不能再次调用它，否则将立即返回一个错误。当connect被一个捕获信号中断而且不自动重启时，我们必须调用select来等待连接完成。
http://www.cs.moravian.edu/pipermail/cs320/2008-February/000040.html
   在本人的程序中，为了能够使用一种机制，使accept不会被SIGALRM信号中断，尝试了使用pthread_sigmask函数，但pthread_sigmask 总是返回失败。无奈尝试了下再起一个线程，一个线程作为timer，另一个线程进行套接字数据接收处理，主线程通过等待互斥量来进行阻塞（不能使用sleep，Timer和sleep也会存在冲突，timer到时后的SIGALRM信号会导致sleep被唤醒），测试问题没有出现。。。
    多个线程的SIGALRM信号不会影响？ 原因没有想明白，怀疑和Android内核处理有关。
 *
 */
//Reveive message
//dest_id: in
//*msg: out
// ret: 接受消息中的errno=4/EINTR需要单独处理，不然会出现被SIGALRM打断的情形
UINT32 hcu_message_rcv(UINT32 dest_id, HcuMsgSruct_t *msg)
{
	//Checking task_id range
	if ((dest_id < TASK_ID_MIN) || (dest_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, dest_id=%d!!!\n", dest_id);
		return FAILURE;
	}
	int ret = 0;
	UINT32 size;
	//没仔细研究过这里msgrcv的参数，需要后期进一步打磨推敲
	//No need use msgget to retrieve new message
    //msgtyp 0 接收第一个消息
    //msgtyp <0 接收类型等于或者小于msgtyp绝对值的第一个消息
    //msgtyp >0 接收类型等于msgtyp的第一个消息
    //msgflg 0 阻塞式接收消息，没有该类型的消息msgrcv函数一直阻塞等待
    //msgflg IPC_NOWAIT 如果没有返回条件的消息调用立即返回，此时错误码为ENOMSG
    //msgflg IPC_EXCEPT 与msgtype配合使用返回队列中第一个类型不为msgtype的消息
    //msgflg IPC_NOERROR 如果队列中满足条件的消息内容大于所请求的size字节，则把该消息截断，截断部分将被丢弃
	/*返回值：
		成功时返回所获取信息的长度，失败返回-1，错误信息存于error
		Number of bytes copied into message buffer -1 on error: errno =
			E2BIG (Message length is greater than msgsz, no MSG_NOERROR)
			EACCES (No read permission)
			EFAULT (Address pointed to by msgp is invalid)
			EIDRM (Queue was removed during retrieval)
			EINTR (Interrupted by arriving signal)
			EINVAL (msgqid invalid, or msgsz less than 0)
			ENOMSG (IPC_NOWAIT asserted, and no message exists in the queue to satisfy the request)
	*/
	ret = msgrcv(zHcuTaskInfo[dest_id].QueId, msg, (sizeof(HcuMsgSruct_t) - sizeof(long)), 0, 0);
	if (errno == EINTR) //EINTR, system interrupt call error
		return EINTR;
	if ( ret < 0 ) {
		HcuErrorPrint("HCU-VM: msgrcv() receive msg failed, Qid=%d, msg=%08X, errno=%d[%s]\n",
				zHcuTaskInfo[dest_id].QueId, msg, errno, strerror(errno));
		return FAILURE;
	}
	return SUCCESS;
}

//TaskId transfer to string
//待调试的函数，是否需要使用动态内存
//正确的做法也许应该使用zHcuTaskInfo[id].TaskName这个信息来表达
UINT32 taskid_to_string(UINT32 id, char *string)
{
	if ((id<TASK_ID_MIN) || (id>=TASK_ID_MAX)){
		return FAILURE;
	}
	strcat(string, "[");
	strcat(string, zHcuTaskNameList[id]);
	strcat(string, "]");
	return SUCCESS;
}

//MsgId transfer to string
//输入的string参数，其内存地址空间和长度预留，是否足够
UINT32 msgid_to_string(UINT32 id, char *string)
{
	switch(id)
	{
		case MSG_ID_COM_INIT:
			strcpy(string, "[MSG_ID_COM_INIT]");
			break;
		case MSG_ID_COM_INIT_FEEDBACK:
			strcpy(string, "[MSG_ID_COM_INIT_FB]");
			break;
		case MSG_ID_COM_RESTART:
			strcpy(string, "[MSG_ID_COM_RESTART]");
			break;
		case MSG_ID_COM_STOP:
			strcpy(string, "[MSG_ID_COM_STOP]");
			break;
		case MSG_ID_COM_COMPLETE:
			strcpy(string, "[MSG_ID_COM_COMPLETE]");
			break;
		case MSG_ID_COM_TIME_OUT:
			strcpy(string, "[MSG_ID_COM_TIME_OUT]");
			break;
		case MSG_ID_COM_DEBUG_TEST:
			strcpy(string, "[MSG_ID_COM_DEBUG_TEST]");
			break;
		case MSG_ID_EMC_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_EMC_MODBUS_DATA_READ]");
			break;
		case MSG_ID_EMC_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_EMC_MODBUS_DATA_READ]");
			break;
		case MSG_ID_PM25_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_PM25_MODBUS_DATA_READ]");
			break;
		case MSG_ID_PM25_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_PM25_CLOUD_DATA_RESP]");
			break;
		case MSG_ID_WINDDIR_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_WINDDIR_MODBUS_DATA_READ]");
			break;
		case MSG_ID_WINDDIR_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_WINDDIR_CLOUD_DATA_RESP]");
			break;
		case MSG_ID_WINDSPD_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_WINDSPD_MODBUS_DATA_READ]");
			break;
		case MSG_ID_WINDSPD_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_WINDSPD_CLOUD_DATA_RESP]");
			break;
		case MSG_ID_TEMP_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_TEMP_MODBUS_DATA_READ]");
			break;
		case MSG_ID_TEMP_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_TEMP_CLOUD_DATA_RESP]");
			break;
		case MSG_ID_HUMID_MODBUS_DATA_READ:
			strcpy(string, "[MSG_ID_HUMID_MODBUS_DATA_READ]");
			break;
		case MSG_ID_HUMID_CLOUD_DATA_RESP:
			strcpy(string, "[MSG_ID_HUMIDITY_CLOUD_DATA_RESP]");
			break;
		case MSG_ID_MODBUS_EMC_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_EMC_DATA_REPORT]");
			break;
		case MSG_ID_MODBUS_PM25_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_PM25_DATA_REPORT]");
			break;
		case MSG_ID_MODBUS_WINDDIR_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_WINDDIR_DATA_REPORT]");
			break;
		case MSG_ID_MODBUS_WINDSPD_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_WINDSPD_DATA_REPORT]");
			break;
		case MSG_ID_MODBUS_TEMP_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_TEMP_DATA_REPORT]");
			break;
		case MSG_ID_MODBUS_HUMID_DATA_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_HUMID_DATA_REPORT]");
			break;
		case MSG_ID_AIRSYNC_INIT:
			strcpy(string, "[MSG_ID_AIRSYNC_INIT]");
			break;
		case MSG_ID_AIRSYNC_BLE_CONNNECTD:
			strcpy(string, "[MSG_ID_AIRSYNC_BLE_CONNNECTD]");
			break;
		case MSG_ID_AIRSYNC_AUTH_REQ:
			strcpy(string, "[MSG_ID_AIRSYNC_AUTH_REQ]");
			break;
		case MSG_ID_AIRSYNC_AUTH_RESP:
			strcpy(string, "[MSG_ID_AIRSYNC_AUTH_RESP]");
			break;
		case MSG_ID_AIRSYNC_INIT_REQ:
			strcpy(string, "[MSG_ID_AIRSYNC_INIT_REQ]");
			break;
		case MSG_ID_AIRSYNC_INIT_RESP:
			strcpy(string, "[MSG_ID_AIRSYNC_INIT_RESP]");
			break;
		case MSG_ID_AIRSYNC_CONNECT_READY:
			strcpy(string, "[MSG_ID_AIRSYNC_CONNECT_READY]");
			break;
		case MSG_ID_AIRSYNC_DATA_PUSH:
			strcpy(string, "[MSG_ID_AIRSYNC_DATA_PUSH]");
			break;
		case MSG_ID_AIRSYNC_DATA_REQ:
			strcpy(string, "[MSG_ID_AIRSYNC_DATA_REQ]");
			break;
		case MSG_ID_AIRSYNC_DATA_RESP:
			strcpy(string, "[MSG_ID_AIRSYNC_DATA_RESP]");
			break;
		case MSG_ID_AIRSYNC_AUTH_WAIT_TIMER:
			strcpy(string, "[MSG_ID_AIRSYNC_AUTH_WAIT_TIMER]");
			break;
		case MSG_ID_AIRSYNC_DISCONNECT_TIGGER_L3:
			strcpy(string, "[MSG_ID_AIRSYNC_DISCONNECT_TIGGER_L3]");
			break;
		case MSG_ID_AIRSYNC_SEND_BLE_DATA_TIMER:
			strcpy(string, "[MSG_ID_AIRSYNC_SEND_BLE_DATA_TIMER]");
			break;
		case MSG_ID_WXAPP_3MIN_TIMEOUT:
			strcpy(string, "[MSG_ID_WXAPP_3MIN_TIMEOUT]");
			break;
		case MSG_ID_WXAPP_5SECOND_TIMEOUT:
			strcpy(string, "[MSG_ID_WXAPP_5SECOND_TIMEOUT]");
			break;
		case MSG_ID_WXAPP_DATA_REQ_EMC:
			strcpy(string, "[MSG_ID_WXAPP_DATA_REQ_EMC]");
			break;
		case MSG_ID_WXAPP_PERIOD_REPORT_EMC_TRIGGER:
			strcpy(string, "[MSG_ID_WXAPP_PERIOD_REPORT_EMC_TRIGGER]");
			break;
		case MSG_ID_WXAPP_DATA_REQ_TIME_SYNC:
			strcpy(string, "[MSG_ID_WXAPP_DATA_REQ_TIME_SYNC]");
			break;
		case MSG_ID_WXAPP_MODBUS_CONNECT_READY:
			strcpy(string, "[MSG_ID_WXAPP_MODBUS_CONNECT_READY]");
			break;
		case MSG_ID_WXAPP_MODBUS_DATA_PUSH:
			strcpy(string, "[MSG_ID_WXAPP_MODBUS_DATA_PUSH]");
			break;
		case MSG_ID_WXAPP_MODBUS_DISCONNECT:
			strcpy(string, "[MSG_ID_WXAPP_MODBUS_DISCONNECT]");
			break;
		case MSG_ID_MODBUS_10MIN_TIMEOUT:
			strcpy(string, "[MSG_ID_MODBUS_10MIN_TIMEOUT]");
			break;
		case MSG_ID_MODBUS_5SECOND_TIMEOUT:
			strcpy(string, "[MSG_ID_MODBUS_5SECOND_TIMEOUT]");
			break;
		case MSG_ID_MODBUS_DATA_REQ_PM1025_REPORT:
			strcpy(string, "[MSG_ID_MODBUS_DATA_REQ_PM1025_REPORT]");
			break;
		case MSG_ID_MODBUS_PERIOD_REPORT_PM1025_TRIGGER:
			strcpy(string, "[MSG_ID_MODBUS_PERIOD_REPORT_PM1025_TRIGGER]");
			break;
		case MSG_ID_MODBUS_UART1_FRAME_TIMEOUT:
			strcpy(string, "[MSG_ID_MODBUS_UART1_FRAME_TIMEOUT]");
			break;
		case MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG:
			strcpy(string, "[MSG_ID_HWINV_CLOUD_PHY_STATUS_CHG]");
			break;
		case MSG_ID_ETHERNET_CLOUD_DATA_RX:
			strcpy(string, "[MSG_ID_ETHERNET_CLOUD_DATA_RX]");
			break;

		default:
			strcpy(string, "[MSG_ID_XXXX]");
			break;
	}
	return SUCCESS;
}

//API abstract
void hcu_sleep(UINT32 second)
{
	if (second <0) second =0;
	second = sleep(second);
	while (second>0)
	{
#if (TRACE_DEBUG_ON)
		HcuErrorPrint("HCU-VM: Sleep interrupt by other higher level system call, remaining %d second to be executed\n", second);
#endif
		second = sleep(second);
	}
}

void hcu_usleep(UINT32 usecond)
{
	if (usecond <0) usecond =0;
	usecond = usleep(usecond);
	while (usecond>0)
	{
		HcuErrorPrint("HCU-VM: uSleep interrupt by other higher level system call, remaining %d usecond to be executed\n", usecond);
		usecond = usleep(usecond);
	}
}

//API set timer and back to handler function
UINT32 hcu_timer_polling(time_t sec, UINT32 nsec, void *handler())
{
	return SUCCESS;
}


/*******************************************************************************
** METHOD :FsmInit
**------------------------------------------------------------------------------
** Description :
**    Initialize the module of the fsm.
**------------------------------------------------------------------------------
** Arguments :
**    IN:	 none
**    OUT:   none
**    INOUT: none
**------------------------------------------------------------------------------
** Return value : SUCCESS OR FAILURE
*******************************************************************************/
UINT32 FsmInit(void)
{
	UINT32 i;
	HcuDebugPrint("HCU-VM: >>Start init FSM.\n");
	zHcuFsmTable.numOfFsmCtrlTable = 0;
	for(i=0; i<MAX_TASK_NUM_IN_ONE_HCU; i++)
	{
		zHcuFsmTable.pFsmCtrlTable[i].taskId = TASK_ID_INVALID;
		zHcuFsmTable.pFsmCtrlTable[i].numOfFsmArrayElement = 0;
	}
	zHcuFsmTable.currentTaskId = TASK_ID_INVALID;

	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("HCU-VM: Maxium (%d) process supported.\n", MAX_TASK_NUM_IN_ONE_HCU);
	#endif //TRACE_DEBUG_ON

    return SUCCESS;
}

/*******************************************************************************
** METHOD : FsmAddNew
**------------------------------------------------------------------------------
** Description :
**    fill the FSM table information of entity
**------------------------------------------------------------------------------
** Arguments :
**    IN:	 Taskid : entity identification
**			 pFsmStateItem: pointer to message table of entity
**    OUT:   none
**    INOUT: none
**------------------------------------------------------------------------------
** Return value : SUCCESS OR FAILURE
*******************************************************************************/
UINT32 FsmAddNew(UINT32 task_id, FsmStateItem_t* pFsmStateItem )
{
	OPSTAT ret;
	UINT32 state;
	UINT32 msgid;
	UINT32 item, itemNo, i, j;

	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("HCU-VM: >>Register new FSM. TaskId:(%d), pFsm(0x%x)\n", task_id, pFsmStateItem);
	#endif //TRACE_DEBUG_ON
	/*
	** Check the array of the state machine.
	*/
	if( NULL== pFsmStateItem)
	{
		HcuErrorPrint("HCU-VM: Can not init FSM machine.\n");
		return FAILURE;
	}
	if((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX))
	{
		HcuErrorPrint("HCU-VM: The task_ID is invalid.\n");
		return FAILURE;
	}
	if( zHcuFsmTable.pFsmCtrlTable[task_id].taskId != TASK_ID_INVALID )
	{
		HcuErrorPrint("HCU-VM: This task_id has been already inited.\n");
		return FAILURE;
	}

	/*
	** Check the first entry. 最初一个FSM表单必须以FSM_STATE_ENTRY开始
	*/
	if( pFsmStateItem[FSM_STATE_ENTRY].state != FSM_STATE_ENTRY )
	{
		HcuErrorPrint("HCU-VM: The format of the FSM table is invalid.\n");
		return FAILURE;
	}

    /*
    ** Check each item of the state entry. 最后一个FSM表单使用FSM_STATE_END结束
    */
	for(item=1; item<MAX_FSM_STATE_ENTRY_NUM_IN_ONE_TASK; item++)
	{
		if( FSM_STATE_END == pFsmStateItem[item].state )
		{
			break;
		}
		if( FSM_STATE_ENTRY == pFsmStateItem[item].state )
		{
			HcuErrorPrint("HCU-VM: Invalid FSM machine.\n");
			return FAILURE;
		}
		if( NULL == pFsmStateItem[item].stateFunc)
		{
			HcuErrorPrint("HCU-VM: Invalid state function pointer.\n");
			return FAILURE;
		}
	}

	/*
	** Check the number of the item.
	** If we can not find the ENDING flag of the state machine after more than
	** MAX_FSM_STATE_ENTRY_NUM_IN_ONE_TASK entry, raise fatal alarm.
	*/
	if( MAX_FSM_STATE_ENTRY_NUM_IN_ONE_TASK == item)
	{
		HcuErrorPrint("HCU-VM: Invalid FSM machine -- Can not find the end of the FSM.\n");
		return FAILURE;
	}
	zHcuFsmTable.pFsmCtrlTable[task_id].numOfFsmArrayElement = item-1; //有效STATE-MSG条目数，不包括START/END两条

	/*
	**  Insert this new fsm item into fsm table.
	**  The position in the fsm table is based on the task_id.
	*/
	zHcuFsmTable.numOfFsmCtrlTable ++;

	#if (TRACE_DEBUG_ON)
	{
		HcuDebugPrint("HCU-VM: FsmAddNew: task_id = 0x%x [%s], src_id= %x, dest_id= %X\n", task_id, zHcuTaskNameList[task_id], 0, 0);
		HcuDebugPrint("HCU-VM: After add this one, Total (%d) FSM in the table.\n", zHcuFsmTable.numOfFsmCtrlTable);
	}
	#endif //TRACE_DEBUG_ON

	/*
	** Save the state machine info.
	*/
	zHcuFsmTable.pFsmCtrlTable[task_id].taskId = task_id;
	for(i=0; i<MAX_STATE_NUM_IN_ONE_TASK; i++)
	{
		for(j=0; j<MAX_MSGID_NUM_IN_ONE_TASK; j++)
		{
			zHcuFsmTable.pFsmCtrlTable[task_id].pFsmArray[i][j].stateFunc = NULL;
		}
	}

	//检查Msg_id的合法性，不能超过FF，这里很有技巧，全靠消息ID不超过FF来设定，小心消息过多的情况
	//内部消息也不能使用任意定义的数值，因为这里将采用MsgId进行索引，以提高效率
	for(itemNo=0; itemNo < item; itemNo ++ )
	{
		state = pFsmStateItem[itemNo].state;
		if(state >= MAX_STATE_NUM_IN_ONE_TASK)
		{
			HcuErrorPrint("HCU-VM: The State number > %d\n", MAX_STATE_NUM_IN_ONE_TASK);
			return FAILURE;
		}
		msgid = pFsmStateItem[itemNo].msg_id & MASK_MSGID_NUM_IN_ONE_TASK;
		zHcuFsmTable.pFsmCtrlTable[task_id].pFsmArray[state][msgid].stateFunc = pFsmStateItem[itemNo].stateFunc;
	}

	/*
	** Start the task entry init api/function
	*/
	//Input message id = 0, param_ptr = NULL, param_len = 0.
    /*
    ** call the entry
    ** 这一块代码不能复用，因为这个任务和被执行任务并不再同一个空间之内，它只能被目标任务的入口函数进行执行
    ** 该函数API必须在每个任务中进行表达，否则一定会出错， 该代码需要移动到任务创建成功后的入口函数中进行调用
    */
	//(zHcuFsmTable.pFsmCtrlTable[task_id].pFsmArray[FSM_STATE_ENTRY][MSG_ID_COM_MIN].stateFunc)(task_id, 0, NULL, 0);

	/*
	** Set the state to init state(FSM_STATE_IDLE).
	** 一旦任务创建，自动转移到IDLE状态，因而每个任务独特的初始化不能处理状态，也不能被FsmRunEngine调度，而必须被任务创建的入口函数所执行
	*/
	ret = FsmSetState(task_id, FSM_STATE_IDLE);
	if (ret == FAILURE){
		HcuErrorPrint("HCU-VM: Error set FSM State!\n");
	}

    return SUCCESS;
}


/*******************************************************************************
** METHOD : FsmProcessingLaunch
**------------------------------------------------------------------------------
** Description :
**    check and handle message loop as each task entry
**------------------------------------------------------------------------------
** Arguments :
**    IN:	 none
**    OUT:   none
**    INOUT: none
**------------------------------------------------------------------------------
** Return value : SUCCESS OR FAILURE
*******************************************************************************/
UINT32 FsmProcessingLaunch(void)
{
	OPSTAT ret;
	HcuMsgSruct_t rcv;
	UINT32 task_id = 0; //Get current working task_id

	task_id = zHcuFsmTable.currentTaskId;
	zHcuFsmTable.currentTaskId = TASK_ID_INVALID;
	/*
	** Check the task_id
	*/
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	/*
	** Run each task entry API, give each task a chance to init any specific information
	*/
	if (zHcuFsmTable.pFsmCtrlTable[task_id].pFsmArray[FSM_STATE_ENTRY][MSG_ID_ENTRY].stateFunc != NULL){
		(zHcuFsmTable.pFsmCtrlTable[task_id].pFsmArray[FSM_STATE_ENTRY][MSG_ID_ENTRY].stateFunc)(task_id, 0, NULL, 0);
	}else{
		HcuDebugPrint("HCU-VM: Task (ID=%d) get no init entry fetched!\n", task_id);
	}

	//Loop for ever
	while (1){
		memset(&rcv, 0, sizeof(HcuMsgSruct_t));
		ret = hcu_message_rcv(task_id, &rcv);
		if (ret == EINTR){
			continue;
		}
		else if (ret == FAILURE){
			HcuDebugPrint("HCU-VM: STM kernal receive message error, hold for further action!\n");
			//continue;
			//To be defined here the error case, to recover everything
			hcu_sleep(1);  //wait for HCUMAIN clean all message queue and task, otherwise rubbish can not be clean.
			//exit(EXIT_FAILURE);  //do nothing, wait for main control clean myself.
		}else{
			//hcu_sleep(1); //to be removed
			ret = FsmRunEngine(rcv.msgType, rcv.dest_id, rcv.src_id, rcv.msgBody, rcv.msgLen);
			if (ret == FAILURE){
				HcuErrorPrint("HCU-VM: Error execute FsmRun state machine!\n");
			}
		}
	}//While(1)
	return SUCCESS;
}

/*******************************************************************************
** METHOD : FsmApiRun
**------------------------------------------------------------------------------
** Description :
**    supply action function entry of task that receive current Event in current state
**------------------------------------------------------------------------------
** Arguments :
**    IN:	 src_id: identification of task that send out current message
** 		     dest_id: identification of task that receive current message
** 		     commandId: identification of current message
** 		     param_ptr: pointer to current message structure
** 		     param_len: message length
**    OUT:   none
**    INOUT: none
**------------------------------------------------------------------------------
** Return value : SUCCESS OR FAILURE
*******************************************************************************/
UINT32 FsmRunEngine(UINT32 msg_id, UINT32 dest_id, UINT32 src_id, void *param_ptr, UINT32 param_len)
{
	UINT8  state;
	OPSTAT ret;
	UINT32 mid;

	/*
	** Check the task_id, message_id and par_length
	*/
	if ((dest_id < TASK_ID_MIN) || (dest_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, dest_id=%d!!!\n", dest_id);
		return FAILURE;
	}
	if ((src_id < TASK_ID_MIN) || (src_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, src_id=%d!!!\n", src_id);
		return FAILURE;
	}
	if (param_len>MAX_HCU_MSG_BODY_LENGTH){
		HcuErrorPrint("HCU-VM: Too large message length than HCU set capability, param_len=%d!!!\n", param_len);
		return FAILURE;
	}
	if ((msg_id < MSG_ID_COM_MIN) || (msg_id > MSG_ID_COM_MAX)){
		HcuErrorPrint("HCU-VM: Error on Msg_Id, msg_id=%d!!!\n", msg_id);
		return FAILURE;
	}

	/*
	** Search for the dest_id task state
	*/

	if( zHcuFsmTable.pFsmCtrlTable[dest_id].taskId != dest_id )
	{
		HcuErrorPrint("HCU-VM: The destination process does not exist.\n");
		return FAILURE;
	}

	/*
	** Get current FSM state
	*/
	state = FsmGetState(dest_id);
	mid = msg_id & MASK_MSGID_NUM_IN_ONE_TASK;

	//check the state and messageId of task
	if((state >= MAX_STATE_NUM_IN_ONE_TASK)||(mid >= MAX_MSGID_NUM_IN_ONE_TASK))
	{
		HcuErrorPrint("HCU-VM: The state(%d) or msgId(0x%x) of task(0x%x) is error\n", 	state, mid, dest_id);
		return FAILURE;
	}
	#ifdef TRACE_DEBUG_ON
	{
		HcuDebugPrint("HCU-VM: Call state function(0x%x) in state(%d) of task(0x%x) for msg(0x%x)\n",
				zHcuFsmTable.pFsmCtrlTable[dest_id].pFsmArray[state][mid].stateFunc, state, dest_id, mid);
	}
	#endif //TRACE_DEBUG_ON

	/*
	** Call the state function.
	*/
	if(zHcuFsmTable.pFsmCtrlTable[dest_id].pFsmArray[state][mid].stateFunc != NULL)
	{
		ret = (zHcuFsmTable.pFsmCtrlTable[dest_id].pFsmArray[state][mid].stateFunc)
			(dest_id, src_id, param_ptr, param_len);
		if( FAILURE == ret)
		{
			HcuErrorPrint("HCU-VM: Internal error is found in the state function.\n");
			return FAILURE;
		}
	}
	else
	{
		if(NULL != param_ptr)
		{
			//Free memory, here do nothing.
		}
		HcuErrorPrint("HCU-VM: Receive invalid msg(%x) in state(%d) of task(0x%x)\n",
			mid, state, dest_id);
		return FAILURE;
	}

    return SUCCESS;
}




OPSTAT FsmSetState(UINT32 task_id, UINT8 newState)
{
	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: FsmSetState error on task_id, task_id=%d!!!\n", task_id);
		return FAILURE;
	}

	//Checking newState range
	if (newState > MAX_STATE_NUM_IN_ONE_TASK){
		HcuErrorPrint("HCU-VM: FsmSetState error on state, State=%d!!!\n", newState);
		return FAILURE;
	}

	zHcuTaskInfo[task_id].state = newState;
	return SUCCESS;
}

UINT8  FsmGetState(UINT32 task_id)
{
	/*
	** Check the task_id
	*/
	//Checking task_id range
	if ((task_id < TASK_ID_MIN) || (task_id > TASK_ID_MAX)){
		HcuErrorPrint("HCU-VM: Error on task_id, task_id=%d!!!\n", task_id);
		return FSM_STATE_INVALID;
	}
	/*
	** Check the state info
	*/
	if (zHcuTaskInfo[task_id].state <= MAX_STATE_NUM_IN_ONE_TASK){
		return zHcuTaskInfo[task_id].state;
	}else{
		return FSM_STATE_INVALID;
	}
}

