/*
 * Hcuref.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "./comvm/vmlayer.h"
#include "./comvm/trace.h"
#include "./comvm/configxml.h"
#include "./comvm/sysinfo.h"
#include "./l2frame/apigprs.h"
#include "l2frame/sps485.h"

/*
 * Global Varible for Serial Port
 */
SerialPort_t gSerialPortZb = {COM_PORT_1, 38400, 8, 'N', 1, HCU_INVALID_U16, 0, 0, 0};
SerialPort_t gSerialPortGprs = {COM_PORT_1, 115200, 8, 'N', 1, HCU_INVALID_U16, 0, 0, 0};
/*
 * Global Variable for Socket
 */
SocketPort_t gSocketPort = {"10.0.0.19", 6000, 0, NULL, };



/*
 * Global Variable for Hcu and Sensor
 */

/* Zigbee Sensors Nodes */
WedSensors_t gZbSensors;

/* Sensor Measurement Result */
SensorsMeasurementsTable_t gSMT;

/* Measuremnet Bahavior */
/* Default 0.2s, 1 char, 100ms, 800ms, 2, 10 */
HcuMeasureBehavior_t gHcuMeasureBehavior = {2, 0, 100, 1500, 2, 0, 2, 30, 0, 0, 0, 0, 0};

/*Init PollingLoop Behavior*/
PollingLoopBehavior_t PollingLoopBehavior = {0, 0, 0, 1};

/*Global  HcuZbMsgHeader_t*/
HcuZbMsgHeader_t  gHcuZbMsgHeader[5] = {
		{MSG_SOP, MSG_NODE_STATUS_IND_LEN, MSG_NODE_STATUS_IND_CMD1, MSG_NODE_STATUS_IND_CMD2},
		{MSG_SOP, MSG_SENSOR_REG_IND_LEN, MSG_SENSOR_REG_IND_CMD1, MSG_SENSOR_REG_IND_CMD2},
		{MSG_SOP, MSG_SENSOR_DEREG_IND_LEN, MSG_SENSOR_DEREG_IND_CMD1, MSG_SENSOR_DEREG_IND_CMD2},
		{MSG_SOP, 0x00, MSG_APP_DATA_IND_CMD1, MSG_APP_DATA_IND_CMD2},
		{MSG_SOP, 0x00, MSG_APP_DATA_REG_CMD1, MSG_APP_DATA_REG_CMD2}
};

AnalyzeDataBehavior_t gADB = {0, 0, 0};

#define LEN_SEND_DATA		0xFF

UINT8 gFcsEnable = HCU_LOGIC_FALSE;
/*define the lenth is NULL?*/
/* Interface between Hcu and Sensor */
/* pm10chr$ so2chr$ noxchr$ oo3chr cochr$ d$ */
static char gSensorCmd[WED_MAX_NUM_OF_MEASURE][16] = {{"pm10chr$"}, {"pm25chr$"}, {"so2chr$"}, {"noxchr$"}, {"oo3chr$"}, {"cochr$"}, {"tempchr$"}};
static char gSensorMeasure[WED_MAX_NUM_OF_MEASURE][16] = {{"pm10="}, {"pm25="}, {"so2="}, {"nox="}, {"oo3="}, {"co="}, {"temp="}};
#define SENSOR_CMD_PM10		0x0
#define SENSOR_CMD_PM25		0x1
#define SENSOR_CMD_SO2		0x2
#define SENSOR_CMD_NOX		0x3
#define SENSOR_CMD_OO3		0x4
#define SENSOR_CMD_CO		0x5
#define SENSOR_CMD_TEMP		0x6
/* Make Sure this sequence is inline with the BIT definition of the following */
/*
#define WED_SENSOR_TYPE_PM10		0x00000001
#define WED_SENSOR_TYPE_PM25		0x00000002
#define WED_SENSOR_TYPE_SO2			0x00000004
#define WED_SENSOR_TYPE_NOX			0x00000008
#define WED_SENSOR_TYPE_OO3			0x00000010
#define WED_SENSOR_TYPE_CO			0x00000020
#define WED_SENSOR_TYPE_TEMP		0x00000040
*/

/*define the num of the message of received*/

UINT32 LenCmdReceived = 0;

UINT8 gVerboseMode = HCU_LOGIC_TRUE;

UINT8 TraceWillBeDisplayed = HCU_LOGIC_TRUE;

static pthread_t HcuMainLoopThreadId;

/* System Information */
extern SysInfo_t gSysInfo;

/* GPRS module checking MAX time */
GprsPppdConf_t gGprsPppdConf;

/* Report to server header */
MsgSeverDataSegHeader_t gMsgSeverDataSegHeader;

/* Main state for Hcu */
UINT32 gHcuState = HCU_STATE_ALL_INIT;

/* OAM Control Message Buffer */
OamControlMsgBuf_t gOamControlMsgBuf;

static char testString[] = "Hello, This is sent by HCU for test purpose ...";

/*
 * Interface between CPU and ZB
 *
 * 1. BIND STATUS INDICATOR, When system start up, HCU will wait for BIND STATUS INDICATOR, forever
 *
 * | SOP | LEN | CMD0 | CMD1 | DATA  				        | FCS |
 * |  1  |  1  |   1  |   1  |  8				            |  1  |
 * | FE  | 08  |  41  |  80  |  02 03 00 00 00 00 00 00 	| C5  |		;SUCCESSFUL
 * | FE  | 08  |  41  |  80  |  FF FF FF FF FF FF FF FF 	| C5  |		;SUCCESSFUL
 *
 * Or:
 *
 *                            device1|device2
 *   FE  | 08  |  41  |  80  |  00 00|29 DE FF FF FF FF     | C5  |
 *
 */

/*
 * Message Interaction with ZB and IP Server
 */
static char CmdToSendOverSerial[HCU_MAX_LENGTH_CMD_SERIAL];
static char CmdReceivedOverSerial[HCU_MAX_LENGTH_CMD_SERIAL];
static char CmdToSendOverTcp[HCU_MAX_LENGTH_CMD_TCP];
static char CmdReceivedOverTcp[HCU_MAX_LENGTH_CMD_TCP];

static char SumCmdReceivedOverSerial[HCU_MAX_LENGTH_CMD_SERIAL];

/*
##0258ST=31;CN=2011;PW=123456;MN=12345678901002;CP=&&DataTime=20121017001000;
PMD-Rtd=0.11,PMD-Flag=N;
SO2-Rtd=0.00,SO2-Flag=N;
NO-Rtd=0.00,NO-Flag=N;
NO2-Rtd=0.00,NO2-Flag=N;
NOX-Rtd=0.00,NOX-Flag=N;
O3-Rtd=0.00,O3-Flag=N;
CO-Rtd=0.00,CO-Flag=N;
PMB-Rtd=0.00,PMB-Flag=N
&&9481

*/

/*
** Local static variables
*/
//static unsigned char traceFileName[] = __FILE__;

/*
 * Sub Functions Declararions
 */
UINT32 ThreadCreate(pthread_t *thrdId, void *(*start_func)(void *), void *arg, int prio);
UINT32 HcuMainLoop();
UINT32 ReadCommandLineParameter(int argc, char *argv[]);
UINT32 InitGlobleVarialbles();
UINT32 InitSerialPort(SerialPort_t *sp);
UINT32 InitSocketPort(SocketPort_t *sp);
UINT32 IsFrameFcsOk(UINT8 *f);
UINT32 IsBindStatusInd(UINT8 *f);
void MsgProcBindStatusInd(UINT8 *f);
void HcuResetZigbee();
void WaitZbStartUp();
UINT32 FindNextPoolingIndex(WedSensors_t *wss, PollingLoopBehavior_t *plb);
void SensorPolling(SerialPort_t *sp, WedSensors_t *wss, SensorsMeasurementsTable_t *smt, HcuMeasureBehavior_t *wmb, PollingLoopBehavior_t *plb);
void BuildDataReq(UINT8 *zbCmd, char *sensorCmd,UINT32 NumOfSensor);
//UINT32 AnalyzePollingResult( UINT32 idxPollingLoop, UINT32 idxSensor, UINT32 idxMeasure, char *CmdReceivedOverSerial, UINT32 len, char **validCmdString);
UINT32 ZbCmdProssing(UINT8 *CmdString, UINT32 LenOfCmd);
void ReportSensorsResultToServer(SocketPort_t *sp, SensorsMeasurementsTable_t *smt);
void PrintHwSwInfo();
UINT32 HcuStartTimerPollingReport(time_t sec, long nsec, void *handler());
void HcuPollingReportHandler();
void InitCmdNodeStatusInd(HcuZbMsgHeader_t gWt, UINT8 *f);
void InitCmdSensorRegInd(HcuZbMsgHeader_t gWt, UINT8 *f);
void InitCmdSensorDeregInd(HcuZbMsgHeader_t gWt, UINT8 *f);
void InitCmdAppDateInd(HcuZbMsgHeader_t gWt, UINT8 *f);
void AnalyzeDataOfReceive(char *f, UINT32 DataNumOfWep, UINT32 NumberOfPoll);
UINT32 SetHcuState(UINT32 state);
UINT32 GetHcuState();
UINT32 DeSetHcuState(UINT32 state);
UINT32 InitNetWorkService();
void GetTimeOfReport(char *TimeString);

void InitOamMsgBuf(OamControlMsgBuf_t *pocmb);
OamControlMsg_t *NextMsgToSave(OamControlMsgBuf_t *pocmb);
OamControlMsg_t *NextMsgToRead(OamControlMsgBuf_t *pocmb);
void OamMsgProcess(OamControlMsg_t *pOamMsg);
void OamMsgProcessTraceToggle(void);
void OamMsgProcessSensorPolling(OamControlMsg_t *pOamMsg);



/*
 * Main Entry for HCU
 *
 * Command Line: hcu <x> <ip> <port> [fcs_enable]
 *
 * E.g #hcu 1 10.0.0.9 6000 1
 * 1 for COM1
 * 10.0.0.9, IP address of the TCP server
 * 6000, port of the TCP Server *
 * 1, FCS enabled, if want to disabled, just ommit the "1"
 *
 */
int wip_Main(int argc, char *argv[])
{
	UINT32 ret = SUCCESS;
	UINT32 len = 0;
	struct rlimit		rlimit;
	OamControlMsg_t		*pOamMsg;

	/* received signal */
	/*int 		sig;*/
	/* set of signals */
	sigset_t	set;
	/* sigset_t	setx;*/

	pid_t	pid;            // pid of the main thread

	/* Initilize Trace */
	TraceInit();

	/* Initialize GPIO */
	ret = GpioInit();

	if (FAILURE == ret)
	{
		HcuErrorPrint("GPIO initialize failure, Hcu Exit.\n");
		return ret;
	}

	/* Make Zb RST PIN to HIGH, ZB RST Cable Connect to GPQ3 */
	GpioPortOp("Q3=1");
	GpioPortOp("M1=1");

	/* Make GPRS PWON to HIGH, GPRS PWON Cable Connect to GPQ3 */
	GpioPortOp("Q4=1");
	GpioPortOp("M0=1");

	/* Print HW and SW version info */
	PrintHwSwInfo();
	HcuDebugPrint("Hcu Started ...\n"); /* prints !!!Hello World!!! */

	/* initialization to generate core dump */
	if(0 != getrlimit(RLIMIT_CORE, &rlimit))
	{
		HcuErrorPrint("Hcu getrlimit() nok!!: errno = %d\n", errno);
		exit(0);
	}
	rlimit.rlim_cur = RLIM_INFINITY;
	rlimit.rlim_max = RLIM_INFINITY;
	if(0 != setrlimit(RLIMIT_CORE, &rlimit))
	{
		HcuErrorPrint("Hcu setrlimit() nok!!: errno = %d\n",errno);
		exit(0);
	}

	/* For future implementation */
	/* Initial Other Global Variables to default value, Later get from config file */
	ret = InitGlobleVarialbles();
	if (FAILURE == ret)
	{
		HcuErrorPrint("Invalid configuration parameters, Hcu Exit.\n");
		return ret;
	}

	/* First get all global parameters from configxml.xml */
	HcuDebugPrint("Hcu start to read parameters from configxml.xml ...\n");
	ret = AnalysisOfXml("configxml.xml");
	if (FAILURE == ret)
	{
		HcuErrorPrint("Hcu get configxml.xml failure, Hcu Exit.\n");
		return ret;
	}
	/* Always open trace at system start, and save the configuration */

	HcuDebugPrint("Hcu reads parameters from configxml.xml complete ...\n");

	/* Read Command Line Parameters, overwritten what we have in the xml */
	ret = ReadCommandLineParameter(argc, argv);
	if (FAILURE == ret)
	{
		HcuErrorPrint("Invalid command line parameters, Hcu Exit.\n");
		return ret;
	}

	/* SET FISRT STATE */
	SetHcuState(HCU_STATE_ALL_INIT);

	/* Initial Serial Port */
	ret = InitSerialPort(&gSerialPortZb);
	if (FAILURE == ret)
	{
		HcuErrorPrint("Init Serial Port Failure, Hcu Exit.\n");
		return ret;
	}
	else
	{
		HcuDebugPrint("Init Serial Port Success ...\n");
	}

	/* Initialize TCPIP Network */
	ret = InitNetWorkService();
	if (FAILURE == ret)
	{
		HcuErrorPrint("Init Network Failure, Hcu Exit.\n");
		return ret;
	}
	else
	{
		HcuDebugPrint("Init Network Success ...\n");
	}

	/* CONSIDER THE SERVICE NETWORK IS OK STATE, TBD for OAM INTERFACE */
	SetHcuState(HCU_STATE_NET_SERVICE_OK);

	/* Test Serial Port via a Send */
	len = write(gSerialPortZb.fd, testString, strlen(testString));
	HcuDebugPrint("Hcu has just sent via Serial: %s with lenth of %d bytes.\n", testString, len);

	/* Test Socket TCP Link via Send */
	len = send(gSocketPort.SocketFd, testString, strlen(testString), 0);
	HcuDebugPrint("Hcu has just sent via Socket: %s with lenth of %d bytes.\n", testString, len);

	/* Wait for Zigbee Startup, Infinite loop runs, if no correct BINS STATUS IND receive ... */
	HcuDebugPrint("Hcu is waiting for Zigbee startup, expecting COORD_START_IND over Serial\n");
	HcuResetZigbee();
	WaitZbStartUp();
	SetHcuState(HCU_STATE_SENSOR_OK); /* CONSIDER ZIGBEE NETWORK OK */
	HcuDebugPrint("Hcu Zigbee started, COORD_START_IND over Serial received.\n");

	/* Chang the serial from 0, 1 to 2, 1, enable serial time out*/
	SerialPortSetVtimeVmin(&gSerialPortZb, gHcuMeasureBehavior.SerialVtime, gHcuMeasureBehavior.SerialVmin);
	HcuDebugPrint("Hcu is preparing to start timers for sensor pooling and report reading, Serial timeout set to VTIME = %d*0.1 s, VMIN = %d\n", gHcuMeasureBehavior.SerialVtime, gHcuMeasureBehavior.SerialVmin);

	/* empty signals from the set */
	if (0 != sigemptyset(&set))
	{
		HcuDebugPrint("Hcu sigemptyset() nok! errno = %d\n",errno);
		return FAILURE;
	}

	/* add all signals to the set */
	if (0 != sigfillset(&set))
	{
		HcuDebugPrint("Hcu sigfillset() nok! errno = %d\n", errno);
		return FAILURE;
	}

	/* block all the signals */
	if(0 != pthread_sigmask(SIG_SETMASK, &set, NULL))
	{
		HcuDebugPrint("Hcu pthread_sigmask() nok! errno = %d\n",errno);
		return FAILURE;
	}

	/* Test LED */
	/*
    fd = open("/dev/leds0", 0);
    if (fd < 0) {
            fd = open("/dev/leds", 0);
    }
    if (fd < 0) {
            perror("open device leds");
            exit(1);
    }
    ioctl(fd, 1, 1);
	*/
	/* Get PID of the main thread */
	pid = getpid();
	HcuDebugPrint("Hcu pid of main entry = %d\n", pid);

    /* Create the Hcu Main Loop thread */
    ret = ThreadCreate(&HcuMainLoopThreadId, (CALLBACK)HcuMainLoop, (void *)NULL, HCU_THREAD_PRIO);
    if (FAILURE == ret)
    {
    	/* The End */
    	HcuDebugPrint("Hcu create HcuMainLoop un-successfully, program exit.\n");
    	return FAILURE;
    }
    else
    {
    	HcuDebugPrint("Hcu create HcuMainLoop successfully.\n");
    }

    /* Infinite Loop for Main thread */
    while(1)
    {

    	if (gHcuState & HCU_STATE_NET_SERVICE_OK)
    	{
    		/* Allocate one massage */
			pOamMsg = (OamControlMsg_t *)NextMsgToSave( &gOamControlMsgBuf );

			if(NULL == pOamMsg)
			{
				HcuDebugPrint("Hcu OAM message buffer is full.\n");
			}
			else
			{
				/* Get message from socket, as it is blocked read */
				if(0 >= recv(gSocketPort.SocketFd, (char *)pOamMsg, sizeof(OamControlMsg_t), 0))
				{
					HcuErrorPrint("Hcu Error read OAM message from socket.\n");
					/* To be added to restore the ring buffer */
					pOamMsg = (OamControlMsg_t *)NextMsgToRead( &gOamControlMsgBuf );

					/* Send SIGPIPE to hcu main loop to DeSetHcuState(HCU_STATE_NET_SERVICE_OK) */
					pthread_kill(HcuMainLoopThreadId, SIGPIPE);
					DeSetHcuState(HCU_STATE_NET_SERVICE_OK);
				}
				else
				{
					HcuDebugPrint("Hcu read OAM message from socket, MsgId = 0x%08X, MsgLen = (0x%08X)%d, MsgBody[0-7] = %02X %02X %02X %02X %02X %02X %02X %02X\n", pOamMsg->MsgId, pOamMsg->MsgLen, pOamMsg->MsgLen, pOamMsg->MsgBody[0], pOamMsg->MsgBody[1], pOamMsg->MsgBody[2], pOamMsg->MsgBody[3], pOamMsg->MsgBody[4], pOamMsg->MsgBody[5], pOamMsg->MsgBody[6], pOamMsg->MsgBody[7]);

					/* PULL A SIGNAL SIGUSR1 TO HcuMainLoopThread */
					pthread_kill(HcuMainLoopThreadId, SIGUSR1);
				}
			}
    	}
    }

	/* The End */
	HcuDebugPrint("Hcu run to the end of the program, normal exit.\n");
	return SUCCESS;
}

/*
 * To Create a thread
 *
 * void  ThreadCreate( pthread_t  *thrdId,              out: thread id
 *                   void *(*start_func)(void *),       in: task's entry point
 *                   void *arg,                         in: parameter of the task's function
 *                   int prio )                         in: priority of the task*
 *
 */
UINT32 ThreadCreate(pthread_t *thrdId, void *(*start_func)(void *), void *arg, int prio)
{
	int                     err;
	pthread_attr_t          attr;
	struct  sched_param     sched;

	// The thread is initialized with attributes object default values
	if(0!= (err = pthread_attr_init(&attr)))
	{
		HcuDebugPrint("ThreadCreate: pthread_attr_init() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	// Change the inherit scheduling attribute
	// of the specified thread attribute object,because PTHREAD_INHERIT_SCHED is default value
	if (0!= (err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED)))
	{
		HcuDebugPrint( "ThreadCreate: pthread_attr_setinheritsched() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	// Set the contention scope attribute of
	// the specified thread attributes object, the thread compete with all threads on the station
	if (0 != (err = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM)))
	{
		HcuDebugPrint( "ThreadCreate: pthread_attr_setscope() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	// Set the scheduling policy,SCHED_OTHER is the default
	if (0 != (err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO)))
	{
		HcuDebugPrint( "ThreadCreate: pthread_attr_setschedpolicy() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	// Set the priority of the task
	sched.sched_priority = prio;
	if (0 != (err = pthread_attr_setschedparam(&attr, &sched)))
	{
		HcuDebugPrint( "ThreadCreate: pthread_attr_setschedparam() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	// creation of the task
	if(0 != (err=pthread_create(thrdId, &attr, (void *(*)(void*))(start_func), (void*)arg)))
	{
		HcuDebugPrint("ThreadCreate: pthread_create() nok!! err=%d, errno=%d\n", err, errno);
		return FAILURE;
	}

	HcuDebugPrint("ThreadCreate: pthread_create() OK ...\n");
	/* ERRORS
	       EAGAIN Insufficient resources to create another thread, or a system-imposed limit on the number of threads was encountered.  The latter case may occur in two ways: the RLIMIT_NPROC soft  resource  limit
	              (set via setrlimit(2)), which limits the number of process for a real user ID, was reached; or the kernel's system-wide limit on the number of threads, /proc/sys/kernel/threads-max, was reached.

	       EINVAL Invalid settings in attr.

	       EPERM  No permission to set the scheduling policy and parameters specified in attr.
	 */
	return SUCCESS;
}


/*
 * Hcu Main Loop
 */

UINT32 HcuMainLoop()
{

	UINT32 				ret = SUCCESS;
	//UINT32 len = 0;
	struct timespec		currentTime;
	struct timespec		timeres;

	/* received signal */
	int 				sig;
	/* set of signals */
	sigset_t			set;
	//sigset_t	setx;

	timer_t 			timerid;
	struct sigevent 	evp;
	pthread_t 			tid;
	pid_t 				pid;
	struct itimerspec 	new_value;
	struct itimerspec 	stop_value;
	struct itimerspec	startTimer2;

	OamControlMsg_t		*pOamMsg;

	/* Counter for ticks when Network failure, used to trigger the restore the network */
	UINT32 				CounterTicksNetworkDown = 0;
	UINT32 				MaxCounterTicksNetworkDown = gHcuMeasureBehavior.MaxCounterTicksNetworkDown;

	/* get pid of the current process */
	pid = getpid();
	tid = pthread_self();
	HcuDebugPrint("HcuMainLoop: process# = %d, thread# = %d: received event and start processing...\n", pid, tid);

	/* intialize high resolution timer */
	if((ret = clock_getres(CLOCK_REALTIME, &timeres)) != 0)
	{
	    HcuDebugPrint("HcuMainLoop: clock_getres() nok!! ret=%d,errno=%d\n", ret, errno);
	    exit(0);
	}
	HcuDebugPrint("HcuMainLoop: clock_getres() ok, ret = %d, granularity: secs = %ld, nanos = %ld\n", ret, timeres.tv_sec, timeres.tv_nsec);

	if((ret = clock_gettime(CLOCK_REALTIME, &currentTime)) != 0)
	{
	    HcuDebugPrint("HcuMainLoop: clock_gettime() nok!! ret = %d\n",ret);
	    exit(0);
	}
	HcuDebugPrint("HcuMainLoop: clock_gettime() ok, ret = %d, current time: secs = %ld, nanos = %ld\n", ret, currentTime.tv_sec, currentTime.tv_nsec);

	/* Create Timer */
	memset(&evp, 0, sizeof(struct sigevent));
	//evp.sigev_value.sival_int = SIGUSR2;  /* via signact method, not by signal */
	evp.sigev_signo = SIGUSR2;
	evp.sigev_notify = SIGEV_SIGNAL;
	//evp.sigev_notify_function = handler;  /* via signact method, not by signal */
	evp.sigev_value.sival_ptr = &timerid;

	if ((ret = timer_create(CLOCK_REALTIME, &evp, &timerid)) == -1)
	{
		HcuErrorPrint("HcuMainLoop: timer_create() failed, ret = %d. errno = %d\n", ret, errno);
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: timer_create() ok, ret = %d.\n", ret);

	/* Start to MASK signal */
	/* add all signals to the set */
	if (0 != sigfillset(&set))
	{
	    HcuDebugPrint("HcuMainLoop: sigfillset() nok!!, errno = %d\n", errno);
	    exit(0);
	}

	/* block all the signals */
	if(0 != pthread_sigmask(SIG_SETMASK, &set, NULL))
	{
	    HcuDebugPrint("HcuMainLoop: sigprocmask() nok!!, errno = %d\n", errno);
	    exit(0);
	}

	if (0 != sigemptyset(&set))
	{
		HcuDebugPrint("HcuMainLoop: sigemptyset() nok! errno = %d\n",errno);
		exit(0);
	}

	/* add the SIGINT to catch CTRL C and flush trace buffer */
	if (0 != sigaddset(&set, SIGINT))
	{
		HcuDebugPrint("HcuMainLoop: sigaddset(SIGINT) nok! errno = %d\n",errno);
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: has blocked SIGINT, ready to catch SIGINT ...\n");

	/* add the SIGPIPE to catch socket disconnect */
	if (0 != sigaddset(&set, SIGPIPE))
	{
		HcuDebugPrint("HcuMainLoop sigaddset(SIGPIPE) nok! errno = %d\n",errno);
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: has blocked SIGPIPE, ready to catch SIGPIPE ...\n");

	/* add the THREAD_COM_TIMER_ID to catch timers */
	if (0 != sigaddset(&set, SIGUSR2))
	{
		HcuDebugPrint("HcuMainLoop: sigaddset(SIGUSR2) nok! errno = %d\n",errno);
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: has blocked SIGUSR2, ready to catch SIGUSR2 for timer ...\n");

	/* add the OAM MSG to catch timers */
	if (0 != sigaddset(&set, SIGUSR1))
	{
		HcuDebugPrint("HcuMainLoop: sigaddset(SIGUSR1) nok! errno = %d\n",errno);
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: has blocked SIGUSR1, ready to catch SIGUSR1 for parameter update ...\n");

	/* block all the signals */
	if(0 != pthread_sigmask(SIG_SETMASK, &set, NULL))
	{
		HcuDebugPrint("HcuMainLoop: pthread_sigmask() nok! errno = %d\n",errno);
		exit(0);
	}

	/* set the timer synchronous to the pc clock, on a multiple of ms */
	new_value.it_interval.tv_sec  = gHcuMeasureBehavior.SerialPollItvlSec;
	new_value.it_interval.tv_nsec = gHcuMeasureBehavior.SerialPollItvlnSec;
	new_value.it_value.tv_sec     = currentTime.tv_sec + START_DELAY - 1;
	new_value.it_value.tv_nsec    = 990000000;  //start from xx:yy:zz.000s for ARM, already -10ms in advance

	if ((ret = timer_settime(timerid, 1, &new_value, NULL)) == -1)
	{
		HcuErrorPrint("HcuMainLoop: timer_settime() failed.\n");
		exit(0);
	}
	HcuDebugPrint("HcuMainLoop: timer_settime() ok, Timer started with %u(s): %l(ns).\n", new_value.it_interval.tv_sec, new_value.it_interval.tv_nsec);

	if((ret = timer_gettime(timerid, &startTimer2)) != 0)
	{
		HcuDebugPrint("HcuMainLoop: timer_gettime() nok!! ret = %d\n",ret);
		exit(0);
	}
	//timerPeriodInNano = startTimer2.it_interval.tv_nsec;
    //timerPeriodInNano = startTimer2.it_interval.tv_sec;
    HcuDebugPrint("HcuMainLoop: it_interval.tv_sec = %d, it_interval.tv_nsec = %d\n", startTimer2.it_interval.tv_sec, startTimer2.it_interval.tv_nsec);

	/* Will close debug trace, if configuration file tell not to display */
	if( HCU_LOGIC_TRUE == TraceWillBeDisplayed)
	{
		outputOnCrt = HCU_LOGIC_TRUE;
	}
	else
	{
		HcuDebugPrint("HcuMainLoop: all debug traces will be closed ...\n");
		outputOnCrt = HCU_LOGIC_FALSE;
	}

	/* Hcu infinite loop, waiting for signals */
	HcuDebugPrint("HcuMainLoop: Everying prepared is OK, entering the infinite loop ...\n");
	/* Always open trace at system start, and save the configuration */

	SetHcuState(HCU_STATE_AUTO_POLLING);

	while(1)
	{
		sigwait(&set, &sig);
		HcuDebugPrint("HcuMainLoop: main thread just received signal = %d ... \n", sig);

		if(SIGINT == sig)
		{
			HcuDebugPrint("HcuMainLoop: received CTRL-C(SIGINT = %d), flush trace buffer and exit.\n", sig);
			//TraceClose();
			//ioctl(fd, 0, 1);
			//close(fd);
			exit(0);
		}
		else if(SIGPIPE == sig)
		{
			HcuErrorPrint("HcuMainLoop: received SIGPIPE = %d, NETWORK is brokon, start to restore ...\n", sig);

			DeSetHcuState(HCU_STATE_NET_SERVICE_OK);

			HcuErrorPrint("HcuMainLoop: STOP SENSOR TIMER.\n");

			/* Stop Pooling Timer */
			/* for stop timer */
			stop_value.it_value.tv_sec     = 0;
			stop_value.it_value.tv_nsec    = 0;
			if ((ret = timer_settime(timerid, 1, &stop_value, NULL)) == -1)
			{
				HcuErrorPrint("HcuMainLoop: timer_settime() failed.\n");
				exit(0);
			}

			HcuErrorPrint("HcuMainLoop: REINITIALIZE NETWORK.\n", sig);
			/* Initialize TCPIP Network */
			ret = InitNetWorkService();
			if (FAILURE == ret)
			{
				HcuErrorPrint("ReInit Network Failure, WILL RESTART SENSOR TIMER.\n");
			}
			else
			{
				HcuErrorPrint("HcuMainLoop: REINITIALIZE NETWORK OK.\n", sig);
				SetHcuState(HCU_STATE_NET_SERVICE_OK);
			}

			/* restart the timer */
			/* intialize high resolution timer */
			HcuErrorPrint("HcuMainLoop: RESTART SENSOR TIEMR.\n", sig);
			if((ret = clock_gettime(CLOCK_REALTIME, &currentTime)) != 0)
			{
			    HcuDebugPrint("HcuMainLoop: clock_gettime() nok!! ret = %d\n",ret);
			    exit(0);
			}

			/* set the timer synchronous to the pc clock, on a multiple of ms */
			new_value.it_interval.tv_sec  = gHcuMeasureBehavior.SerialPollItvlSec;
			new_value.it_interval.tv_nsec = gHcuMeasureBehavior.SerialPollItvlnSec;
			new_value.it_value.tv_sec     = currentTime.tv_sec + START_DELAY - 1;
			new_value.it_value.tv_nsec    = 990000000;  //start from xx:yy:zz.000s for ARM, already -10ms in advance

			if ((ret = timer_settime(timerid, 1, &new_value, NULL)) == -1)
			{
				HcuErrorPrint("HcuMainLoop: timer_settime() failed.\n");
				exit(0);
			}

			/* reset the counter, so that another trial will happend if network is down */
			CounterTicksNetworkDown = 0;
			HcuDebugPrint("HcuMainLoop: CounterTicksNetworkDown reset to %d.\n", CounterTicksNetworkDown);

		}
		else if(SIGUSR2 == sig)
		{
			HcuDebugPrint("HcuMainLoop: received SIGUSR2 = %d for timer, Call timer Function.\n", sig);
			HcuPollingReportHandler();

			/* increament the counter when the network is down */
			if ( 0 == (GetHcuState() & HCU_STATE_NET_SERVICE_OK))
			{
				CounterTicksNetworkDown++;
				HcuDebugPrint("HcuMainLoop: CounterTicksNetworkDown increased to %d.\n", CounterTicksNetworkDown);
				if (CounterTicksNetworkDown >= MaxCounterTicksNetworkDown)
				{
					/* send the SIGPIPE to myself */
					char killcmd[64];
					bzero(killcmd, 64);
					sprintf(killcmd, "kill -13 %d", pid); /* Send to myself with SIGPIPE */
					HcuDebugPrint("HcuMainLoop: CounterTicksNetworkDown increased to %d, max %d reach, send SIGPIPE to myself.\n", CounterTicksNetworkDown, MaxCounterTicksNetworkDown);
					system(killcmd);
				}
			}

		}

		else if(SIGUSR1 == sig)
		{
			HcuDebugPrint("HcuMainLoop: received SIGUSR1 = %d to process OAM trace.\n", sig);

			pOamMsg = NextMsgToRead(&gOamControlMsgBuf);
			if(NULL == pOamMsg)
			{
				HcuErrorPrint("HcuMainLoop: Ring Buffer already empty, wait for next ...");
			}
			else
			{
				HcuDebugPrint("HcuMainLoop get OAM message from, MsgId = %04X, MsgLen = %d, MsgBody[0-7] = %02X %02X %02X %02X %02X %02X %02X %02X\n", pOamMsg->MsgId, pOamMsg->MsgLen, pOamMsg->MsgBody[0], pOamMsg->MsgBody[1], pOamMsg->MsgBody[2], pOamMsg->MsgBody[3], pOamMsg->MsgBody[4], pOamMsg->MsgBody[5], pOamMsg->MsgBody[6], pOamMsg->MsgBody[7]);
				OamMsgProcess(pOamMsg);
			}

		}
		else
		{
			HcuDebugPrint("HcuMainLoop: received unkown signal (sig = %d), ignored ...\n", sig);
		}
	}
}

/*
 * Initialize Serial Port
 *
 */
UINT32 InitSerialPort(SerialPort_t *sp)
{
	UINT32 ret = SUCCESS;

	HcuDebugPrint("InitSerialPort Start ...\n");

	ret = SerialPortOpen(sp->id, (UINT16 *)(&(sp->fd)));
	if (FAILURE == ret)
	{
		HcuErrorPrint("InitSerialPort serial port COM%d open failure.\n", sp->id);
		return ret;
	}
	HcuDebugPrint("InitSerialPort serial port COM%d open successfully.\n", sp->id);

	ret = SerialPortSet(sp);
	if (FAILURE == ret)
	{
		HcuErrorPrint("Open Serial Port %d failure.\n", sp->id);
		return ret;
	}
	HcuDebugPrint("InitSerialPort serial port COM%d properties has been set successfully.\n", sp->id);

	/* return successfully */
	HcuDebugPrint("InitSerialPort completed.\n");
	return ret;
}

UINT32 InitSocketPort(SocketPort_t *sp)
{

	UINT32 ret = SUCCESS;
	//struct sockaddr localaddr;
	char ipaddress[15];
	socklen_t len;

	bzero(ipaddress, 16);

	HcuDebugPrint("InitSocketPort Start ...\n");

	/* Get Host Name */
	sp->pHostName = gethostbyname(sp->ServerIpAddress);
	if (NULL == sp->pHostName)
	{
		HcuErrorPrint("InitSocketPort: gethostbyname failed with server ip address. %s\n", sp->ServerIpAddress);
		return FAILURE;
	}
	HcuDebugPrint("InitSocketPort: gethostbyname completed ...\n");

	/* Init Socket */
	sp->SocketFd = socket(AF_INET, SOCK_STREAM, 0);
	if (HCU_INVALID_U16 == sp->SocketFd)
	{
		HcuErrorPrint("InitSocketPort: Init socket failed.\n");
		return FAILURE;
	}
	HcuDebugPrint("InitSocketPort: socket initialized ...\n");

	(sp->ServerSocketAddress).sin_family = AF_INET;
	(sp->ServerSocketAddress).sin_port = htons(sp->ServerPort);
	(sp->ServerSocketAddress).sin_addr = *((struct in_addr *)(sp->pHostName)->h_addr);
	bzero(&((sp->ServerSocketAddress).sin_zero), 6);  /* 6 is the sin_zero's size !!!!! */

	ret = connect(sp->SocketFd, (struct sockaddr *)&(sp->ServerSocketAddress), sizeof(struct sockaddr));
	if (FAILURE == ret)
	{
		HcuErrorPrint("InitSocketPort: Socket can not connect to Server, make sure server is listening on port %d.\n", sp->ServerPort);
		return FAILURE;
	}
	HcuDebugPrint("InitSocketPort: socket connect to server successfully ...\n");

	/* check which local IP address is used */
	len = sizeof(struct sockaddr);
	ret = getsockname(sp->SocketFd, (struct sockaddr *)&(sp->ClientSocketAddress), &len);
	if (FAILURE == ret)
	{
		HcuErrorPrint("InitSocketPort: getsockname(...) failure, errno = %d;\n", errno);
		return FAILURE;
	}
	HcuDebugPrint("InitSocketPort: got client socket address ...\n");

	if (HCU_LOGIC_TRUE == gVerboseMode)
	{
		//HcuDebugPrint("InitSocketPort: Server IP address, sin_addr: %s\n", inet_ntoa((sp->ServerSocketAddress.sin_addr).s_addr));
		HcuDebugPrint("InitSocketPort: Server TCP Port, sin_port: %d\n", ntohs(sp->ServerSocketAddress.sin_port));

		//strncpy(ipaddress, (sp->ClientSocketAddress.sa_data), 15);
		//HcuDebugPrint("InitSocketPort: Client IP address, sin_addr: %s\n", inet_ntoa((sp->ClientSocketAddress.sin_addr).s_addr));
		HcuDebugPrint("InitSocketPort: Client TCP Port, sin_port: %d\n", ntohs(sp->ClientSocketAddress.sin_port));
	}

	HcuDebugPrint("InitSocketPort: completed.\n");

	return ret;
}

UINT32 ReadCommandLineParameter(int argc, char *argv[])
{

	HcuDebugPrint("ReadCommandLineParameter start ...\n");

	/* there is no command line input, all get from configxml.xml */
	if(1 == argc)
	{
		HcuDebugPrint("ReadCommandLineParameter: no command line input, default value loaded from configxml.xml\n");
		return SUCCESS;
	}

	if( !((4 == argc) || (5 == argc)) )
	{
		HcuDebugPrint("Usage: hcu <x> <ip> <port> [fcs_enable], Use COMx with TCP Server ip:port, Optional fcs_enable=1 to enable FCS sending & checking.\n");
		return FAILURE;
	}

	if (0 == strcmp(argv[1],"0"))
	{
		gSerialPortZb.id = COM_PORT_0;
		HcuDebugPrint("ReadCommandLineParameter: COM0 port will be used ...\n");
	}
	else if (0 == strcmp(argv[1],"1"))
	{
		gSerialPortZb.id = COM_PORT_1;
		HcuDebugPrint("ReadCommandLineParameter: COM1 port will be used ...\n");
	}
	else if (0 == strcmp(argv[1],"2"))
	{
		gSerialPortZb.id = COM_PORT_2;
		HcuDebugPrint("ReadCommandLineParameter: COM2 port will be used ...\n");
	}
	else if (0 == strcmp(argv[1],"3"))
	{
		gSerialPortZb.id = COM_PORT_3;
		HcuDebugPrint("ReadCommandLineParameter: COM3 port will be used ...\n");
	}
	else if (0 == strcmp(argv[1],"4"))
	{
		gSerialPortZb.id = COM_PORT_4;
		HcuDebugPrint("ReadCommandLineParameter: COM4 port will be used ...\n");
	}
	else
	{
		HcuDebugPrint("Usage: hcu <x> <ip> <port> [fcs_enable], Use COMx with TCP Server ip:port, Optional fcs_enable=1 to enable FCS sending & checking.\n");
		return FAILURE;
	}

	/* Read IP address */
	if(strlen(argv[2]) > 15) /* xxx.yyy.zzz.sss total 15 byts */
	{
		HcuDebugPrint("Usage: hcu <x> <ip> <port> [fcs_enable], Use COMx with TCP Server ip:port, Optional fcs_enable=1 to enable FCS sending & checking.\n");
		return FAILURE;
	}
	else
	{
		strcpy(gSocketPort.ServerIpAddress, argv[2]);

		/* TO DO: To check whether it is a valid IP address */

		HcuDebugPrint("ReadCommandLineParameter: Server IP %s port will be used ...\n", gSocketPort.ServerIpAddress);
	}

	/* Read port */
	gSocketPort.ServerPort = atoi(argv[3]);
	if((gSocketPort.ServerPort<=0) & (gSocketPort.ServerPort>=65535))
	{
		HcuDebugPrint("Usage: hcu <x> <ip> <port> [fcs_enable], Use COMx with TCP Server ip:port, Optional fcs_enable=1 to enable FCS sending & checking.\n");
		return FAILURE;
	}
	HcuDebugPrint("ReadCommandLineParameter: Server TCP port %d port will be used ...\n", gSocketPort.ServerPort);

	/* Get FCS enable or not paramter */
	if (5 == argc)
	{
		if (0 == strcmp(argv[4],"1"))
		{
			gFcsEnable = HCU_LOGIC_TRUE;
			HcuDebugPrint("ReadCommandLineParameter: FCS encoding for TX and FCS checking for RX has been enabled ...\n");
		}
		else
		{
			gFcsEnable = HCU_LOGIC_FALSE;
			HcuDebugPrint("ReadCommandLineParameter: FCS fixed as [0xC5] for TX, FCS checking for RX has been disabled ...\n");
		}
	}
	else
	{
		gFcsEnable = HCU_LOGIC_FALSE;
		HcuDebugPrint("ReadCommandLineParameter: FCS fixed as [0xC5] for TX, FCS checking for RX has been disabled ...\n");
	}

	HcuDebugPrint("ReadCommandLineParameter completed.\n");

	return SUCCESS;
}
/*
 * To Check whether the Frame FCS is OK
 */
UINT32 IsFrameFcsOk(UINT8 *f)
{
	UINT8 msgPayloadLen;
	UINT8 i;
	UINT8 temp = 0x00;

	if (NULL == f)
	{
		HcuErrorPrint("IsFrameFcsOk: Invalid parameter for IsFrameFcsOk(): NULL == CmdSerical\n");
		return FAILURE;
	}

	if(HCU_LOGIC_FALSE == gFcsEnable)
	{
		HcuDebugPrint("IsFrameFcsOk: The FCS checking is disabled, always return FCS OK.\n");
		return SUCCESS;
	}

	/* Get Payload Lenth */
	msgPayloadLen = f[1];
	HcuDebugPrint("IsFrameFcsOk: Payload Length(LEN) = %d.\n", msgPayloadLen);

	for(i = 1; i < msgPayloadLen + 4; i++)
	{
		temp = temp ^ f[i]; //XOR
	}

	if(temp == f[msgPayloadLen + 4])
	{
		HcuDebugPrint("IsFrameFcsOk: The FCS calculated equals to FCS in frame (%2X).\n", temp);
		return SUCCESS;
	}
	else
	{
		HcuDebugPrint("IsFrameFcsOk: The FCS calculated (%2X) does not equal to FCS in frame (%2X).\n", temp, f[msgPayloadLen+4]);
		return FAILURE;
	}

	//if(Fcs == f[msgPayloadLen+4])
	/* TO DO */
}

/*
 * To Check whether the Frame is BIND_STATUS_IND
 */
UINT32 IsBindStatusInd(UINT8 *f)
{

	HcuZbMsgHeader_t p;

	if (NULL == f)
	{
		HcuErrorPrint("Invalid frame address.\n");
		return FAILURE;
	}

	p = *((HcuZbMsgHeader_t *)f);

	if ((MSG_SOP == p.sop) && (MSG_COORD_START_IND_LEN  == p.len) && (MSG_COORD_START_IND_CMD1 == p.cmd0) && (MSG_COORD_START_IND_CMD2 == p.cmd1))
	{
		return SUCCESS;
	}

	return FAILURE;
}

/*
 * To process the BIND_STATUS_IND msg
 */
void MsgProcBindStatusInd(UINT8 *f)
{
	//HcuZbMsgHeader_t p;

	//UINT8 i = 0;

	if (NULL == f)
	{
		HcuErrorPrint("MsgProcBindStatusInd: Invalid frame starting address of NULL.\n");
		return;
	}

	/* print out the msg */
	HcuDebugPrint("MsgProcBindStatusInd: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n", f[0], f[1], f[2], f[3]);

	/* print out the msg */
	HcuDebugPrint("MsgProcBindStatusInd: PAYLOAD(8)...|FCS(HEX): %02X %02X %02X %02X %02X %02X %02X %02X | %02X\n", f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[f[1]+4]);

	/* Save the content of the message */
	/* TO DO */
}

/*
 * Infinite wait for BIND_STATUS_IND when Hcu is startup.
 */
void WaitZbStartUp()
{

	UINT32 readlen = 0;
	UINT32 i = 0;
	UINT8 CmdStrTemp[HCU_MAX_LENGTH_CMD_SERIAL];
	LenCmdReceived = 0;
	while(1)
	{
		bzero(CmdStrTemp, HCU_MAX_LENGTH_CMD_SERIAL);
		if((readlen = read(gSerialPortZb.fd, CmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL)) > 0)
		{
			HcuDebugPrint("WaitZbStartUp: Read %d bytes from Serial.\n",readlen);
			HcuDebugPrint("WaitZbStartUp: There is %d bytes(LenCmdReceived) received.\n", LenCmdReceived);

			if(LenCmdReceived + readlen > HCU_MAX_LENGTH_CMD_SERIAL)
			{
				bzero(SumCmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
				LenCmdReceived = 0;
				readlen = 0;
				HcuDebugPrint("WaitZbStartUp: SumCmdReceivedOverSerial is full, clear all contents to ZERO.\n");
			}
			else
			{
				memcpy(SumCmdReceivedOverSerial + LenCmdReceived, CmdReceivedOverSerial, readlen);
				LenCmdReceived = LenCmdReceived + readlen;
			}

			for(i = 0; i < LenCmdReceived; i++)
			{
				if((MSG_SOP == SumCmdReceivedOverSerial[i]) && (0x00 == SumCmdReceivedOverSerial[i+1]) && (0x80 == SumCmdReceivedOverSerial[i+2]) && (0x16 == SumCmdReceivedOverSerial[i+3]))//check the fcs is right
				{
					if(SUCCESS == IsFrameFcsOk((UINT8 *)(SumCmdReceivedOverSerial + i)) )
					{

						gHcuMeasureBehavior.NumberOfFrameTotalRx++;

						if(SUCCESS == IsBindStatusInd((UINT8 *)(SumCmdReceivedOverSerial + i)))
						{
							HcuDebugPrint("WaitZbStartUp: COORD_START_IND Found\n");
							/* Process the message of COORD_START_IND */
							MsgProcBindStatusInd((UINT8 *)(SumCmdReceivedOverSerial+i));
							/* Exit the infinity wait for COORD_START_IND */
							LenCmdReceived = LenCmdReceived - i - 5;

							HcuDebugPrint("WaitZbStartUp: Save contents(%d bytes) after COORD_START_IND Msg\n", LenCmdReceived);
							memcpy(CmdStrTemp, SumCmdReceivedOverSerial + i + 5, LenCmdReceived);
							bzero(SumCmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
							memcpy(SumCmdReceivedOverSerial, CmdStrTemp, LenCmdReceived);

							HcuDebugPrint("WaitZbStartUp: Complete.\n");
							return;
						}/* end of if(SUCCESS == IsBindStatusInd((UINT8 *)CmdReceivedOverSerial)) */
					}
				}
				//HcuDebugPrint("WaitZbStartUp: %d byte has been looked, inside buffer size of %d.\n", i, LenCmdReceived);

		    }
			//HcuDebugPrint("WaitZbStartUp: %d bytes in buffer has been searched.\n", LenCmdReceived);
		}
	}	/* Read every charactor, block read for infinite time, till there is >=1 charactor in the buffer */
} /* end of function WaitZbStartUp() */

UINT32 InitGlobleVarialbles()
{
	/* Zigbee Sensor Nodes */
	/* WedSensorProperty_t gZbSensorNodes[HCU_MAX_NUM_OF_WED]; */
	bzero(&gZbSensors, sizeof(WedSensors_t));

	/* Sensor Measurement Result */
	/* SensorMeasurement_t gSensorMeasureResult[HCU_MAX_NUM_OF_WED][WED_MAX_NUM_OF_MEASURE]; */
	memset(&gSMT, 0xFF, sizeof(SensorsMeasurementsTable_t));
	gSMT.activeTable = 0;  /* Initialize pointer to the active Table to 0 */
	gSMT.curRecord = 0;    /*  Initialize pointer to current Record to 0 */

	/* Measuremnet Bahavior */
	gHcuMeasureBehavior.NumberOfFrameTotalRx = 0;
	gHcuMeasureBehavior.NumberOfFrameTotalRx = 0;
	gHcuMeasureBehavior.NumberOfFrameValidRx = 0;
	gHcuMeasureBehavior.NumberOfFrameTotalTx = 0;
	gHcuMeasureBehavior.NumberOfTicksTriggerred = 0;
	/* HcuMeasureBehavior_t gHcuMeasureBehavior = {1000, 2}; */

	gFcsEnable = HCU_LOGIC_FALSE;
	gGprsPppdConf.gUseGprsOrNot = HCU_LOGIC_TRUE;

	/* This is an example for there is only 2 sensors connected, and this sensor can report PM10 and PM2.5 */
	gZbSensors.NumOfActiveSensors = 0;
	gZbSensors.Sensor[0].NodeId = 0;
	gZbSensors.Sensor[0].NodeSensorType = WED_SENSOR_TYPE_PM10 | WED_SENSOR_TYPE_NOX;
	gZbSensors.Sensor[0].NodeStatus = SENSOR_STATUS_INIT;
	gZbSensors.Sensor[0].NodeZbDynAddress = 0xFFFF;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[0] = 0xABF9F301;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[1] = 0x004B1200;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[0] = 0x22FCF301;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[1] = 0x004B1200;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[0] = 0x00124B00;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[1] = 0x01F3F9CB;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[0] = 0xEF9E3E01;
	gZbSensors.Sensor[0].NodeZbIeeeAddress[1] = 0x004B1200;


	/* EF9E3E01 004B1200 */

	/* 2nd sensor, and this sensor can report PM2.5 only */
	gZbSensors.Sensor[1].NodeId = 1;
	gZbSensors.Sensor[1].NodeSensorType = WED_SENSOR_TYPE_PM25;
	gZbSensors.Sensor[1].NodeStatus = SENSOR_STATUS_INIT;
	gZbSensors.Sensor[1].NodeZbDynAddress = 0xFFFF;
	gZbSensors.Sensor[1].NodeZbIeeeAddress[0] = 0x00124B00;
	gZbSensors.Sensor[1].NodeZbIeeeAddress[1] = 0x013E9E1F;
	gZbSensors.Sensor[1].NodeZbIeeeAddress[0] = 0xCBF9F301;
	gZbSensors.Sensor[1].NodeZbIeeeAddress[1] = 0x004B1200;

	/* 00124B00 024F0D37 */
	/* CBF9F301 004B1200 */

	bzero(CmdToSendOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdToSendOverTcp, HCU_MAX_LENGTH_CMD_TCP);
	bzero(CmdReceivedOverTcp, HCU_MAX_LENGTH_CMD_TCP);

	bzero(SumCmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);

	/* TO DO */
	/* In future, there maybe configuartion file to read */
	/* Combined also the report after Zigbee Coordinator has setup the network */
	bzero(CmdToSendOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdToSendOverTcp, HCU_MAX_LENGTH_CMD_TCP);
	bzero(CmdReceivedOverTcp, HCU_MAX_LENGTH_CMD_TCP);

	InitOamMsgBuf(&gOamControlMsgBuf);

	return SUCCESS;
}

/*
 * Polling Sensors
 */
void SensorPolling(SerialPort_t *sp, WedSensors_t *wss, SensorsMeasurementsTable_t *smt, HcuMeasureBehavior_t *wmb, PollingLoopBehavior_t *plb)
{

	UINT16	fd = sp->fd;
	UINT32  delayBeforeRead = wmb->DelayBeforeRead; 			/* unit: ms */
	//UINT32  delayAfterRead = wmb->DelayAfterRead; 			/* unit: ms */
	UINT32	ReadLen;
	UINT8	CmdStr[HCU_MAX_LENGTH_CMD_SERIAL];
	UINT8	CmdStrTemp[HCU_MAX_LENGTH_CMD_SERIAL];
	UINT32 i;
	UINT32 len = 0;
	UINT8 *f = NULL;

	/************************************************************************************************************/
	/**** PART 1: Search for next pooling index and send command ************************************************/
	/************************************************************************************************************/
	bzero(CmdToSendOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);

	if( 0!= wss->NumOfActiveSensors)
	{
		/* Find the next valid sensor and meaure */
		FindNextPoolingIndex(wss, plb);
		HcuDebugPrint("SensorPolling: Pool = %3d, Sensor = %3d, Measure = %3d, Polling Command = %s\n", plb->NumOfPoll, plb->NumOfSensor, plb->NumOfMeasure, gSensorCmd[plb->NumOfMeasure]);

		/* Build the Data Req message */
		BuildDataReq( (UINT8 *)CmdToSendOverSerial, gSensorCmd[plb->NumOfMeasure], plb->NumOfSensor);

		/* Caculate the msg Lenth, later when DataReg Frame is added, Length of the message header will be included */
		len = strlen(gSensorCmd[plb->NumOfMeasure]) + 7; /* 0 will changed to header and FCS length */

		/* Send the command over Serials */
		len = write(fd, CmdToSendOverSerial, len);
		gHcuMeasureBehavior.NumberOfFrameTotalTx++;

		f = CmdToSendOverSerial;
		HcuDebugPrint("SensorPolling: SEND: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n",f[0], f[1], f[2], f[3]);
		HcuDebugPrint("SensorPolling: SEND: PAYLOAD(12)|FCS(HEX): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | %02X\n",f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15], f[f[1]+4]);
		/* Error Handling to be added */

	}

	/************************************************************************************************************/
	/**** PART 2: Get Sensor Feedback *************************************************************************/
	/************************************************************************************************************/
	/*add the process to received cmd*/
	/*zero the CmdROS's string*/
	bzero(CmdStr, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdStrTemp, HCU_MAX_LENGTH_CMD_SERIAL);
	bzero(CmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);

	/* Delay for a short period of time, in order to let the sensor to response */
	usleep(delayBeforeRead * 1000);
	HcuDebugPrint("SensorPolling: Just been delayed for %d(ms), in order to wait for sensor to response.\n", (delayBeforeRead));

	if((ReadLen = read(fd, CmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL)) > 0)
	{
		HcuDebugPrint("SensorPolling: Get %d bytes from serial, and before this, already %d bytes saved.\n", ReadLen, LenCmdReceived);

		/* !!!! Protection is needed, to AVOID EXCEED the max length of Buffer !!!!!! */
		if(LenCmdReceived + ReadLen > HCU_MAX_LENGTH_CMD_SERIAL)
		{
			bzero(SumCmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
			LenCmdReceived = 0;
			ReadLen = 0;
			HcuDebugPrint("SensorPolling: SumCmdReceivedOverSerial is full, clear all contents to ZERO.\n");
		}
		else
		{
			memcpy(SumCmdReceivedOverSerial + LenCmdReceived, CmdReceivedOverSerial, ReadLen);
			LenCmdReceived = LenCmdReceived + ReadLen;
		}
	}
	else
	{
		HcuDebugPrint("SensorPolling: Get %d bytes from serial for this tick, and before this, already %d bytes saved.\n", ReadLen, LenCmdReceived);
	}

	for(i = 0; i < LenCmdReceived; i++)
	{
		//if((MSG_SOP == SumCmdReceivedOverSerial[i]) && (0x00 != SumCmdReceivedOverSerial[i+SumCmdReceivedOverSerial[i+1]+4]))//check the fcs is right
		if(MSG_SOP == SumCmdReceivedOverSerial[i])// SOP Found
		{
			if (SUCCESS == IsFrameFcsOk((UINT8 *)(SumCmdReceivedOverSerial+i))) // FCS Checking OK
			{
				HcuDebugPrint("SensorPolling: A valid (SOP|LEN|....|FCS) frame found when offset i = %d.\n", i);//

				/* Not able to read the Frame Lenght */
				if (i == LenCmdReceived - 1)
				{
					HcuDebugPrint("SensorPolling: Offset i = %d, reach the last bytes (LenCmdReceived - 1 = %d) of buffer, wait for next bytes.\n", i, LenCmdReceived - 1);
					return;
				}

				/* able to read frame length, but frame has not been received completely, just return and wait for next coming data */
				if ((LenCmdReceived - i) < (SumCmdReceivedOverSerial[i+1] + 5))
				{
					HcuDebugPrint("SensorPolling: (LenCmdReceived - i) = %d, less than frame length = %d, whem offset i = %d, wait for next bytes.\n", (LenCmdReceived - i), (SumCmdReceivedOverSerial[i+1] + 5), i);
					return;
				}

				/* for debug */
				f = SumCmdReceivedOverSerial + i;
				HcuDebugPrint("SensorPolling: RECEIVE: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n",f[0], f[1], f[2], f[3]);
				HcuDebugPrint("SensorPolling: RECEIVE: PAYLOAD(12)|FCS(HEX): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | %02X\n",f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15], f[f[1]+4]);

				LenCmdReceived = LenCmdReceived - i - (SumCmdReceivedOverSerial[i+1] + 5);

				memcpy(CmdStr, SumCmdReceivedOverSerial+i, SumCmdReceivedOverSerial[i+1] + 5);
				HcuDebugPrint("SensorPolling: The bytes left for next process is (LenCmdReceived = %d).\n", LenCmdReceived);

				//HcuDebugPrint("SensorPolling: start to process the Frame.\n",SumCmdReceivedOverSerial[i+1] + 5);

				gHcuMeasureBehavior.NumberOfFrameTotalRx++;
				if(SUCCESS == ZbCmdProssing(CmdStr, (UINT32)SumCmdReceivedOverSerial[i+1] + 5))
				{
					//HcuDebugPrint("SensorPolling: Frame process sucessfully, will discard the frame.\n");
				}
				else
				{
					//HcuDebugPrint("SensorPolling: Frame process failed, but will discard the frame.\n");
				}

				memcpy(CmdStrTemp, SumCmdReceivedOverSerial + i + (SumCmdReceivedOverSerial[i+1] + 5), LenCmdReceived);
				bzero(SumCmdReceivedOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);
				memcpy(SumCmdReceivedOverSerial, CmdStrTemp, LenCmdReceived);
				HcuDebugPrint("SensorPolling: There is still %d bytes left to scan in the receive buffer.\n", LenCmdReceived);

				/* restart the scan */
				i = 0xFFFFFFFF;
			}
		}
	}

}
/*
 *        7  6  5  4  3  2  1  0   <- MEASURE
 *     0           1     1
 *     1
 *     2
 *     .
 *     .
 *    15
 *     ^
 *     |____ SENSOR
 *
 * To find the next Sensor and Measure to Pool *
 *
 */
UINT32 FindNextPoolingIndex(WedSensors_t *wss, PollingLoopBehavior_t *plb)
{
	//#define		HCU_MAX_NUM_OF_WED	 			16
	//#define		WED_MAX_NUM_OF_MEASURE			7
	//UINT32 NumOfPoll = plb->NumOfPoll;
	/* Current Measure */
	UINT32 NumOfSensor = plb->NumOfSensor;
	UINT32 NumOfMeasure = plb->NumOfMeasure;
	UINT32 curMeasureOneD = 0;

	/* Next Measure to search */
	UINT32 nextMeasureOneD = 0;
	UINT32 nextSensor = 0;
	UINT32 nextMeasure = 0;

	UINT32 nextValidMeasureFound = FAILURE;
	if(0 == wss->NumOfActiveSensors )
	{
		/* DO NOTHING IF THERE IS NO ACTIVE SENSOR */
		return nextValidMeasureFound;
	}

	/* Get one dimension index of Measure */
	curMeasureOneD = NumOfSensor * WED_MAX_NUM_OF_MEASURE + NumOfMeasure;

	/* Use One dimension Index to search, note that in the for, there is stop condition */
	for(nextMeasureOneD = curMeasureOneD + 1;  ; nextMeasureOneD++)
	{
		/* Search back to Sensor 0, and Measure 0, Loop will be increased */
		if (WED_MAX_NUM_OF_MEASURE * HCU_MAX_NUM_OF_WED == nextMeasureOneD)
		{
			nextMeasureOneD = 0;
			plb->NumOfPoll++;
			plb->NumOfPoll = plb->NumOfPoll % HCU_MAX_NUM_POLLING_LOOP;
			/* loop increased */
		}

		//if (nextMeasureOneD == curMeasureOneD) /* as we have judge the number of active sensor, so should never enter here */
		//{
		//	nextValidMeasureFound = FAILURE;
		//	HcuErrorPrint("FindNextPoolingIndex: as already judge the number of active sensor, so should never enter here.\n");
		//	break; /* not found for one loop, this is only when there is no sensor active in SENSOR_STATUS_DATA_CONN status */
		//}

		/* Get Sensor and Measure Index */
		nextSensor = nextMeasureOneD / WED_MAX_NUM_OF_MEASURE;
		nextMeasure = nextMeasureOneD % WED_MAX_NUM_OF_MEASURE;
		if( (SENSOR_STATUS_DATA_CONN == wss->Sensor[nextSensor].NodeStatus) && ( (1<<nextMeasure) & (wss->Sensor[nextSensor].NodeSensorType) ) )
		{
			/* A valid measure found, break the search, nextSensor and nextMeasure save the next possible pooling pointer */
			nextValidMeasureFound = SUCCESS;
			HcuDebugPrint("FindNextPoolingIndex: CurSensor=%d, CurMeasure=%d, NextSensor=%d, NextMeasure=%d, NextLoop=%d.\n", NumOfSensor, NumOfMeasure, nextSensor, nextMeasure, plb->NumOfPoll);
			break;
		}
	} /* search complete */

	/* save the next valid pointer (Sensor + Measure) */
	if (SUCCESS == nextValidMeasureFound)
	{
		plb->NumOfSensor = nextSensor;
		plb->NumOfMeasure = nextMeasure;
	}

	/* return whether we find the valid pointer */
	return nextValidMeasureFound;

}

void BuildDataReq(UINT8 *zbCmd, char *sensorCmd, UINT32 NumOfSensor)
{
	UINT32 LengthOfSendData = 0;
	UINT8 i = 0;
	UINT8 Fcs = 0;
	LengthOfSendData = strlen(sensorCmd);
	//HcuDebugPrint("The LengthOfSendData is %d, The sensor Cmd is %s.\n", LengthOfSendData, sensorCmd);
	/* Build Msg According to DATA_REQ header */
	if (NULL == zbCmd)
	{
		HcuErrorPrint("Invalid parameter for BuildDataReq(): NULL == zbCmd\n");
		return;
	}
	bzero(zbCmd, HCU_MAX_LENGTH_CMD_SERIAL);//define the macro
	if(LengthOfSendData > (HCU_MAX_LENGTH_CMD_SERIAL-2))
	{
		HcuErrorPrint("The CMD we send is too long\n");
		return;
	}
	if (NULL == sensorCmd)
	{
		HcuErrorPrint("Invalid parameter for BuildDataReq(): NULL == sensorCmd\n");
		return;
	}
	/* TO DO: Duild DataReg Message according to the frame structure */
	HcuDebugPrint("The LengthOfSendData is %d, The sensor Cmd is %s.\n", LengthOfSendData, sensorCmd);
	zbCmd[0] = gHcuZbMsgHeader[4].sop;
	zbCmd[1] = 2 + LengthOfSendData;
	zbCmd[2] = gHcuZbMsgHeader[4].cmd0;
	zbCmd[3] = gHcuZbMsgHeader[4].cmd1;
	zbCmd[4] = (gZbSensors.Sensor[NumOfSensor].NodeZbDynAddress >> 8) & 0xFF;
	zbCmd[5] = gZbSensors.Sensor[NumOfSensor].NodeZbDynAddress & 0xFF;
	memcpy(zbCmd+6, sensorCmd, strlen(sensorCmd));

	Fcs = zbCmd[1];
	for(i=2; i< LengthOfSendData+6; i++)
	{
		Fcs = Fcs ^ zbCmd[i];
	}
	if(HCU_LOGIC_TRUE == gFcsEnable)
	{
		zbCmd[6 + LengthOfSendData] = Fcs;
	}
	else
	{
		zbCmd[6 + LengthOfSendData] = 0xC5;
	}
	return;
}


/*
 * After a valid Frame is found, start to process the frame according to the command ID
 */
UINT32 ZbCmdProssing(UINT8 *CmdString, UINT32 LenOfCmd)
{
	/* TO DO */
	//UINT8 Fcs;
	HcuZbMsgHeader_t ReadMsgHeader;
	if (NULL == CmdString)
	{
		HcuErrorPrint("ZbCmdProssing: CmdString is NULL, return FAILURE.\n");
		return FAILURE;
	}
	/* save the sop cmd0 1 and the length*/
	ReadMsgHeader.sop = CmdString[0];
	ReadMsgHeader.len = CmdString[1];
	ReadMsgHeader.cmd0 = CmdString[2];
	ReadMsgHeader.cmd1 = CmdString[3];
	HcuDebugPrint("ZbCmdProssing: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n", CmdString[0], CmdString[1], CmdString[2], CmdString[3]);

	if(FAILURE == (IsFrameFcsOk(CmdString)))
	{
		HcuErrorPrint("ZbCmdProssing: Frame FCS check NOK, return FAILURE.\n");
		return FAILURE;
	}

	if((ReadMsgHeader.cmd1 == gHcuZbMsgHeader[0].cmd1) && (ReadMsgHeader.cmd0 == gHcuZbMsgHeader[0].cmd0))
	{
		HcuDebugPrint("ZbCmdProssing: NODE_STATUS_IND received, start process ...\n");
		InitCmdNodeStatusInd(ReadMsgHeader, (UINT8 *)CmdString);
	}
	else if((ReadMsgHeader.cmd1 == gHcuZbMsgHeader[1].cmd1) && (ReadMsgHeader.cmd0 == gHcuZbMsgHeader[1].cmd0))
	{
		HcuDebugPrint("ZbCmdProssing: SENSOR_REG_IND received, start process ...\n");
		InitCmdSensorRegInd(ReadMsgHeader, (UINT8 *)CmdString);
	}
	else if((ReadMsgHeader.cmd1 == gHcuZbMsgHeader[2].cmd1) && (ReadMsgHeader.cmd0 == gHcuZbMsgHeader[2].cmd0))
	{
		HcuDebugPrint("ZbCmdProssing: SENSOR_DEREG_IND received, start process ...\n");
		InitCmdSensorDeregInd(ReadMsgHeader, (UINT8 *)CmdString);
	}
	else if((ReadMsgHeader.cmd1 == gHcuZbMsgHeader[3].cmd1) && (ReadMsgHeader.cmd0 == gHcuZbMsgHeader[3].cmd0))
	{
		HcuDebugPrint("ZbCmdProssing: APP_DATA_IND received, start process ...\n");
		InitCmdAppDateInd(ReadMsgHeader, (UINT8 *)CmdString);
	}
	else
	{
		HcuDebugPrint("ZbCmdProssing: Unknown CMD0=%2X, CMD1=%2X, return.\n", CmdString[2], CmdString[3]);
	}

	return SUCCESS;
}


void ReportSensorsResultToServer(SocketPort_t *sp, SensorsMeasurementsTable_t *smt)
{

	/* This is only a demo */
	/* No real machnisn done according to the protocal */

	/*
	##0258ST=31;CN=2011;PW=123456;MN=12345678901002;CP=&&DataTime=20121017091600;PMD-Rtd=0.11,PMD-Flag=N;SO2-Rtd=0.00,SO2-Flag=N;NO-Rtd=0.00,NO-Flag=N;NO2-Rtd=0.00,NO2-Flag=N;NOX-Rtd=0.00,NOX-Flag=N;O3-Rtd=0.00,O3-Flag=N;CO-Rtd=0.00,CO-Flag=N;PMB-Rtd=0.00,PMB-Flag=N&&9B81
	*/

	//UINT16 socketfd = sp->SocketFd;
	UINT32 len = 0;
	//INT32 pm10_int32 = smt->smt[0][0][0].pm10;
	double pm10_double =  0;//(double)gMeasurement / 1000;
	//char SensorCmd[16];
	/*make the idxSensorCmd PM10 SO2 ...*/
	//strcpy(SensorCmd, gSensorReport[idxSensorCmd]);
	char msgSegment[1024];
	char msgHeader[1024];
	char msgCrc[5];

	GetTimeOfReport((char *)gMsgSeverDataSegHeader.DataTime);
	HcuDebugPrint("ReportSensorsResultToServer: Get current time: [%s]\n", gMsgSeverDataSegHeader.DataTime);

	/* Generate the command string */
	sprintf(msgSegment, "ST=%s;CN=%s;PW=%d;MN=%s;CP=&&DataTime=%s;PMD-Rtd=%.2f,PMD-Flag=N&&", gMsgSeverDataSegHeader.ST, gMsgSeverDataSegHeader.CN, gMsgSeverDataSegHeader.PW, gMsgSeverDataSegHeader.MN, gMsgSeverDataSegHeader.DataTime, pm10_double);

	/* Calculate the length */
	gMsgSeverDataSegHeader.LenOfData = strlen(msgSegment);

	/* Build the header */
	sprintf(msgHeader, "##%04d",gMsgSeverDataSegHeader.LenOfData);

	HcuDebugPrint("ReportSensorsResultToServer: The msgHeader is %s\n", msgHeader);
	HcuDebugPrint("ReportSensorsResultToServer: The msgSegment is %s\n", msgSegment);
	/*add the function for CRC*/

	sprintf(CmdToSendOverTcp, "%s%s%s", msgHeader, msgSegment, msgCrc);

	len = send(sp->SocketFd, CmdToSendOverTcp, strlen(CmdToSendOverTcp), 0);
	HcuDebugPrint("Len = %d, Sent: %s\n", len, CmdToSendOverTcp);
	return;

}

void PrintHwSwInfo()
{
	/* !!! !!! !!! !!! */
	UINT32 ret = 0;

	memset(&gSysInfo, 0xFF, sizeof(SysInfo_t));

	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                                                                **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**              Home and Car Unisystem (HCU)              **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**            Octopus Wireless Communications Co. Ltd.            **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                          Version 1.0                           **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("**                                                                **\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("********************************************************************\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint(" \n"); /* prints !!!Hello World!!! */
	HcuDebugPrint("SW Build: %s, %s\n", __DATE__, __TIME__); /* prints !!!Hello World!!! */
	HcuDebugPrint("HW Build: P01\n"); /* prints !!!Hello World!!! */
	HcuDebugPrint(" \n"); /* prints !!!Hello World!!! */

	HcuDebugPrint(" == SYSINFO ==\n"); /* prints !!!Hello World!!! */

	ret = HcuGetSysInfo(&gSysInfo);
	/* !!! !!! !!! !!! */
}

/*
 *  Start the Main Loop Timer for Hcu, the main tick is initialized here !!!
 */
UINT32 HcuStartTimerPollingReport(time_t sec, long nsec, void *handler())
{

	return(0);

}

/*
 * This is the main entry for every tick, the ISP for timer initialized.
 */
void HcuPollingReportHandler()
{

	UINT32 bitRem = 0xFFFFFFFF - (0xFFFFFFFF % gHcuMeasureBehavior.SocketInterval);
	UINT32 HcuState;

	gHcuMeasureBehavior.NumberOfTicksTriggerred++;
	HcuDebugPrint("HcuPollingReportHandler is triggered for %lu times ...\n", gHcuMeasureBehavior.NumberOfTicksTriggerred);

	/* Step1 & 2: Pool Sensors over Serials*/
	//HcuDebugPrint("HcuPollingReportHandler, Step 1 Start ... (socketCnt = %9d): Sensor Pooling and Get Result and Saving ...\n", gHcuMeasureBehavior.socketCnt);
	/* please be noted that, for each of the loop, HcuMeasureBehavior_t.NumOfPollingLoop will be polled */
	/* In the current implementation, in next loop, all previous measurement will be overwritten */
	/* In future, data could be processed between the polling loop */
	HcuState = GetHcuState();
	if (HcuState & HCU_STATE_AUTO_POLLING)
	{
		HcuDebugPrint("HcuPollingReportHandler: HCU_STATE_AUTO_POLLING is on, start polling ...\n");
		SensorPolling(&gSerialPortZb, &gZbSensors, &gSMT, &gHcuMeasureBehavior, &PollingLoopBehavior);
	}
	else
	{
		HcuDebugPrint("HcuPollingReportHandler: HCU_STATE_AUTO_POLLING is off, do nothing\n");
	}
	//HcuDebugPrint("HcuPollingReportHandler, Step 1 Completed (socketCnt = %9d): Sensor Pooling and Get Result and Saving ...\n", gHcuMeasureBehavior.socketCnt);

	/* Get Network State */
	//HcuState = GetHcuState();

	/* Check whether need to send to the server */
	if ( (0 == (PollingLoopBehavior.CurrentTick % gHcuMeasureBehavior.SocketInterval)) && (HcuState & HCU_STATE_NET_SERVICE_OK) )
	{
		HcuDebugPrint("HcuPollingReportHandler, (CurrentTick = %9d): Report Sensor Measurement Result to server ...\n", PollingLoopBehavior.CurrentTick);
		ReportSensorsResultToServer(&gSocketPort, &gSMT);
	}

	/* Counter the number of report */
	PollingLoopBehavior.CurrentTick++;

	if (bitRem == PollingLoopBehavior.CurrentTick )
		PollingLoopBehavior.CurrentTick = bitRem;

	HcuDebugPrint("HcuPollingReportHandler is completed for this tick, TickTrigger: %lu, FrameTX: %lu, FrameRX: %lu ...\n", gHcuMeasureBehavior.NumberOfTicksTriggerred, gHcuMeasureBehavior.NumberOfFrameTotalTx, gHcuMeasureBehavior.NumberOfFrameTotalRx);
	return;
}

/*
 * To Process the NODE_STATUS_IND
 */
void InitCmdNodeStatusInd(HcuZbMsgHeader_t gWt, UINT8 *f)
{
	UINT16 Read16Address;
	UINT32 Read64Address[2]; /* ZigBee IEEE address */
	UINT32 i;
	UINT32 AddressFindOrNot = HCU_LOGIC_FALSE;

	if (NULL == f)
	{
		HcuErrorPrint("InitCmdNodeStatusInd: Invalid parameter, f == NULL.\n");
		return;
	}

	/* Save the 64 bit IEEE Address */
	Read64Address[0] =  f[7] + (f[6]<<8)  + (f[5]<<16) + (f[4]<<24);
	Read64Address[1] = f[11] + (f[10]<<8) + (f[9]<<16) + (f[8]<<24);

	/* Save the 16 bit IEEE Address */
	Read16Address = f[13] + (f[12]<<8);

	HcuDebugPrint("InitCmdNodeStatusInd: 64 bits IEEE address = %08X %08X, start to search.\n", Read64Address[0], Read64Address[1]);
	HcuDebugPrint("InitCmdNodeStatusInd: 16 bits Dynamic Zigbee Address = %04X \n", Read16Address);
	//HcuDebugPrint("InitCmdNodeStatusInd: The gZbSensorAddress is %08X %08X\n",gZbSensors.Sensor[0].NodeZbIeeeAddress[0],gZbSensors.Sensor[0].NodeZbIeeeAddress[1]);
	//HcuDebugPrint("InitCmdNodeStatusInd: Start to search the local sensor table for the 64 bit address \n", Read16Address);

	/* To ensure there is no duplicated 16bit address */
	for(i = 0; i < HCU_MAX_NUM_OF_WED; i++)
	{
		if(gZbSensors.Sensor[i].NodeZbDynAddress == Read16Address)
		{
			HcuErrorPrint("InitCmdNodeStatusInd: already 16 bits address exist, ingore.\n");
			return;
		}
	}

	/* start to search for 64bit address */
	for(i = 0; i < HCU_MAX_NUM_OF_WED; i++)
	{
		if((gZbSensors.Sensor[i].NodeZbIeeeAddress[0] == Read64Address[0]) && (gZbSensors.Sensor[i].NodeZbIeeeAddress[1] == Read64Address[1]))
		{
			HcuDebugPrint("InitCmdNodeStatusInd: 64 bits IEEE address [%08X %08X] found, (SensorID/Index = %d).\n", Read64Address[0], Read64Address[1], i);
			AddressFindOrNot = HCU_LOGIC_TRUE;
			switch(gZbSensors.Sensor[i].NodeStatus)
			{
				case SENSOR_STATUS_INIT:
					gZbSensors.Sensor[i].NodeZbDynAddress = Read16Address;
					gZbSensors.Sensor[i].NodeStatus = SENSOR_STATUS_NODE_CONN;
					HcuDebugPrint("InitCmdNodeStatusInd: SensorID = %d, SENSOR_STATUS_INIT to SENSOR_STATUS_NODE_CONN, Dynamic Address = %04X saved.\n", i, Read16Address);
					break;

				case SENSOR_STATUS_NODE_CONN:
					gZbSensors.Sensor[i].NodeZbDynAddress = Read16Address;
					HcuDebugPrint("InitCmdNodeStatusInd: SensorID = %d, stays in SENSOR_STATUS_NODE_CONN, Dynamic Address updated to %04X.\n", i, Read16Address);
					break;

				case SENSOR_STATUS_DATA_CONN:
					HcuDebugPrint("InitCmdNodeStatusInd: SensorID = %d stays in SENSOR_STATUS_DATA_CONN, Dynamic Address %04X ignored.\n", i, Read16Address);
					break;

				default:
					HcuErrorPrint("InitCmdNodeStatusInd: Should not enter there, ingore.\n");
					break;
			}
		}

		/* stop search, if already one 64 address has been found */
		if(HCU_LOGIC_TRUE == AddressFindOrNot)
		{
			HcuDebugPrint("InitCmdNodeStatusInd, stop search, already one 64 address has been found.\n");
			break;
		}
	}

	if(HCU_MAX_NUM_OF_WED == i)
		HcuDebugPrint("InitCmdNodeStatusInd: 64 bits IEEE address [%08X %08X] not found in local Zigbee table, return.\n", Read64Address[0], Read64Address[1]);
}
/*
 * Process SENSOR_REG_IND
 */
void InitCmdSensorRegInd(HcuZbMsgHeader_t gWt, UINT8 *f)
{
	UINT16 Read16Address;
	UINT32 i;
	UINT32 AddressFindOrNot = HCU_LOGIC_FALSE;

	if (NULL == f)
	{
		HcuErrorPrint("InitCmdSensorRegInd: Invalid parameter, f == NULL\n");
		return;
	}

    Read16Address = f[5] + (f[4]<<8);
	HcuDebugPrint("InitCmdSensorRegInd: SENSOR_REG_IND received 16 bits Dynamic Zigbee Address = %04X\n", Read16Address);

	for(i=0; i < HCU_MAX_NUM_OF_WED; i++)
	{
		if(gZbSensors.Sensor[i].NodeZbDynAddress == Read16Address)
		{
			HcuDebugPrint("InitCmdSensorRegInd: 16 bits Dynamic Zigbee Address = %04X found (SensorID/Index = %d).\n", Read16Address, i);
			AddressFindOrNot = HCU_LOGIC_TRUE;

			switch(gZbSensors.Sensor[i].NodeStatus)
			{
				case SENSOR_STATUS_INIT:
					HcuDebugPrint("InitCmdSensorRegInd: SensorID(%d) currently in SENSOR_STATUS_INIT, ignore SENSOR_REG_IND.\n", i);
					break;

				case SENSOR_STATUS_NODE_CONN:
					gZbSensors.Sensor[i].NodeStatus = SENSOR_STATUS_DATA_CONN;
					gZbSensors.NumOfActiveSensors++;
					HcuDebugPrint("InitCmdSensorRegInd: SensorID(%d) state SENSOR_STATUS_NODE_CONN->SENSOR_STATUS_DATA_CONN, Total %d active.\n", i, gZbSensors.NumOfActiveSensors);
					break;

				case SENSOR_STATUS_DATA_CONN:
					HcuDebugPrint("InitCmdSensorRegInd: SensorID(%d) currently in SENSOR_STATUS_DATA_CONN, ignore SENSOR_REG_IND.\n", i);
					break;

				default:
					HcuDebugPrint("InitCmdSensorRegInd: should not enter here, ignored.\n");
					break;
			}
		}

		if(HCU_LOGIC_TRUE == AddressFindOrNot)
		{
			HcuDebugPrint("InitCmdSensorRegInd: stop search, one 16 address found.\n");//add trace
			break;
		}
	}

	if(HCU_MAX_NUM_OF_WED == i)
		HcuDebugPrint("InitCmdSensorRegInd: 16 bits address [%04X] not found, return.\n", Read16Address);
}
/*
 * Process SENSOR_DEREG_IND
 */
void InitCmdSensorDeregInd(HcuZbMsgHeader_t gWt, UINT8 *f)
{
	UINT16 Read16Address;
	UINT32 i;
	UINT32 AddressFindOrNot = HCU_LOGIC_FALSE;

	if (NULL == f)
	{
		HcuErrorPrint("InitCmdSensorDeregInd: Invalid parameter, f == NULL\n");
		return;
	}

	Read16Address = f[5] + (f[4]<<8);
	HcuDebugPrint("InitCmdSensorDeregInd: SENSOR_DEREG_IND received with 16 bits Dynamic Zigbee Address = %04X, start to search in local Zigbee table.\n", Read16Address);

	for(i=0; i < HCU_MAX_NUM_OF_WED; i++)
	{
		if(gZbSensors.Sensor[i].NodeZbDynAddress == Read16Address)
		{
			HcuDebugPrint("InitCmdSensorDeregInd: 16 bits Dynamic Zigbee Address = %04X found in local Zigbee table (SensorID/Index = %d).\n", Read16Address, i);
			AddressFindOrNot = HCU_LOGIC_TRUE;

			switch(gZbSensors.Sensor[i].NodeStatus)
			{
				case SENSOR_STATUS_INIT:
					HcuDebugPrint("InitCmdSensorDeregInd: SensorID(%d) currently stays in SENSOR_STATUS_INIT, ignore SENSOR_REG_IND.\n", i);
					break;

				case SENSOR_STATUS_NODE_CONN:
					HcuDebugPrint("InitCmdSensorDeregInd: SensorID(%d) currently stays in SENSOR_STATUS_NODE_CONN, ignore SENSOR_REG_IND.\n", i);
					break;

				case SENSOR_STATUS_DATA_CONN:
					gZbSensors.Sensor[i].NodeZbDynAddress = HCU_INVALID_U16; /* delete the 16 bit address */
					gZbSensors.Sensor[i].NodeStatus = SENSOR_STATUS_NODE_CONN;
					gZbSensors.NumOfActiveSensors--;
					HcuDebugPrint("InitCmdSensorDeregInd: SensorID(%d) changed from SENSOR_STATUS_DATA_CONN to SENSOR_STATUS_NODE_CONN, total %d active sensors.\n", i, gZbSensors.NumOfActiveSensors);
					break;

				default:
					HcuDebugPrint("InitCmdSensorDeregInd: should not enter here, ignored.\n");

					break;
			}
		}

		if(HCU_LOGIC_TRUE == AddressFindOrNot)
		{
			HcuDebugPrint("InitCmdSensorDeregInd: stop search, one 16 address found in local Zigbee table.\n");//add trace
			break;
		}
	}

	if(i == HCU_MAX_NUM_OF_WED)
		HcuDebugPrint("InitCmdSensorDeregInd: 16 bits address [%04X] not found in local Zigbee table, return.\n", Read16Address);

}
/*
 * process APP_DATA_REG
 */
void InitCmdAppDateInd(HcuZbMsgHeader_t gWt, UINT8 *f)
{
	UINT16 Read16Address;
	UINT8 Data[HCU_MAX_LENGTH_CMD_SERIAL];
	UINT32 i;

	if (NULL == f)
	{
		HcuErrorPrint("InitCmdAppDateInd: Invalid parameter, f == NULL\n");
		return;
	}

	//if(f[1] > HCU_MAX_LENGTH_CMD_SERIAL)
	//{
		//HcuErrorPrint("The data's length is too long.\n");
		//return;
	//}

	Read16Address = f[5] + (f[4]<<8);
	HcuDebugPrint("InitCmdAppDateInd: APP_DATA_IND with 16 bits Zigbee Address = %04X.\n", Read16Address);

	for(i = 0; i < HCU_MAX_NUM_OF_WED; i++)
	{
		if((gZbSensors.Sensor[i].NodeZbDynAddress == Read16Address) && (gZbSensors.Sensor[i].NodeStatus == SENSOR_STATUS_DATA_CONN))
		{
			HcuDebugPrint("InitCmdAppDateInd: 16 bits Zigbee Address = %04X found (SensorID/Index = %d).\n", Read16Address, i);

			/*    Save the string in the payload             */
			/*  FRAME: SOP|LEN|CMD0|CMD1|ADDRESS|COMMNAD|FCS */
			/* length:  1   1    1    1     2     LEN-2   1  */
			strncpy((char *)Data, (char *)(f+6), f[1]-2);
			Data[f[1]- 2] = 0x00;

			HcuDebugPrint("InitCmdAppDateInd: report [%s] recevied for SensorID/Index [%d].\n", Data, i);
			AnalyzeDataOfReceive((char *)Data, i, PollingLoopBehavior.NumOfPoll); /* Little bit dangerouse, access to global variable directly */

			break;
		}
	}

	if(HCU_MAX_NUM_OF_WED == i)
		HcuDebugPrint("InitCmdAppDateInd: 16 bits address [%04X] not found in local Zigbee table, ignore.\n", Read16Address);
	return;
}
/*
 * Save the Sensor Measurement report
 */
void AnalyzeDataOfReceive(char *f, UINT32 DataNumOfWep, UINT32 NumberOfPoll)
{
	char m[HCU_MAX_LENGTH_CMD_SERIAL];
	char *p = NULL;
	char *startPoint = NULL;
	char *endPoint = NULL;
	double measureDouble = 0;

	INT32 measureInt = 0;
	UINT32 i = 0;

	UINT32 activeTable = gSMT.activeTable;
	UINT32 curRecord = gSMT.curRecord;

	if (NULL == f)
	{
		HcuErrorPrint("AnalyzeDataOfReceive: Invalid parameter for AnalyzeDataOfReceive(): NULL == f\n");
		return;
	}

	if (NumberOfPoll >= (HCU_MAX_NUM_POLLING_LOOP-1))
	{
		HcuErrorPrint("AnalyzeDataOfReceive: Invalid parameter for AnalyzeDataOfReceive(): NumberOfPoll >= (HCU_MAX_NUM_POLLING_LOOP-1)\n");
		return;
	}

	for(i=0; i<WED_MAX_NUM_OF_MEASURE; i++)
	{
		startPoint = strstr( f, gSensorMeasure[i] );
		if(NULL != startPoint)
		{
			/* Find End Char '$' */
			startPoint = startPoint + strlen (gSensorMeasure[i]);

			HcuDebugPrint("AnalyzeDataOfReceive: The measure start is [%s] found.\n", gSensorMeasure[i]);
			endPoint = strchr(startPoint, '$');
			if (NULL != endPoint)
			{
				HcuDebugPrint("AnalyzeDataOfReceive: The Measure end [$] found.\n");
			}
			else
			{
				HcuDebugPrint("AnalyzeDataOfReceive: The measure end [$] not found.\n");
			}
		}
		else
		{
			HcuDebugPrint("AnalyzeDataOfReceive: The measure start is [%s] not found.\n", gSensorMeasure[i]);
		}

		if ( (NULL != startPoint) && (NULL != endPoint) )
		{

			HcuDebugPrint("AnalyzeDataOfReceive: startPoint = %x, endPoint = %x, [%s] Found\n", startPoint, endPoint, gSensorMeasure[i]);

			/* Get out the data */
			bzero(m, 256);
			strncpy(m, startPoint, (endPoint - startPoint));

			HcuDebugPrint("AnalyzeDataOfReceive: Result of [%s] extracted\n", m);

			/* Convert the string to double */
			measureDouble = strtod( m, &p );

			/* Error handling to be added */
			if((0 == measureDouble) && (p == m) )//*p->p?
			{
				HcuDebugPrint("AnalyzeDataOfReceive: result [%s] Convert to floating point varialble failed.\n", m);
			}
			else
			{
				HcuDebugPrint("AnalyzeDataOfReceive: result [%s] Convert to %f.\n", m, measureDouble);
			}

			/* Convert in INT32 format*/
			measureInt = (INT32)(measureDouble * 1000);
			//HcuDebugPrint("AnalyzeDataOfReceive: result [%s] convert to %f, and then convert to %d, will be saved in SMT\n", m, measureDouble, measureInt);
			//
			/* Save the result in SMT, Sesor Measurement Table */
			switch (i)
			{
				case SENSOR_CMD_PM10:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].pm10 = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in PM10 of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_PM25:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].pm25 = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in PM2.5 of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_SO2:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].so2 = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in SO2 of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_NOX:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].nox = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in NOX of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_OO3:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].oo3 = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in OO3 of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_CO:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].co = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in CO of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				case SENSOR_CMD_TEMP:
					gSMT.smt[0][NumberOfPoll][DataNumOfWep].temp = measureInt;
					HcuDebugPrint("AnalyzeDataOfReceive: result [%s] -> [%f] -> [%d], saved in Temperature of SensorID %d\n", m, measureDouble, measureInt, DataNumOfWep);
					break;

				default:
					HcuDebugPrint("AnalyzeDataOfReceive: Measurement ID %d not identified for SensorID %d, ignored\n", i, DataNumOfWep);
					break;
			}


			/* Update in another format */
			gSMT.mi[activeTable][curRecord].value = measureDouble;
			time(&(gSMT.mi[activeTable][curRecord].time));
			gSMT.mi[activeTable][curRecord].sensorid = DataNumOfWep;
			gSMT.mi[activeTable][curRecord].sensortype = (1 << i);

			HcuDebugPrint("AnalyzeDataOfReceive: gSMT.mi[%d][%d].value = %f, .sensorid = %d, .sensortype = %X, .time = %s.\n", activeTable, curRecord, measureDouble, DataNumOfWep, gSMT.mi[activeTable][curRecord].sensortype, localtime(&(gSMT.mi[activeTable][curRecord].time)));

			/* Point to the next record for next measure measurement result */
			gSMT.curRecord++;
			if (HCU_MAX_NUM_RECORD == gSMT.curRecord)
			{
				/* Return to 0 */
				gSMT.curRecord = 0;

				/* switch the active table to the other one */
				if(0 == gSMT.activeTable)
				{
					gSMT.activeTable = 1;
				}
				else if (1 == gSMT.activeTable)
				{
					gSMT.activeTable = 0;
				}
				/* TO DO: Triggr a new thread to save all record to a file */
				/* TO DO: To be added in future */
				HcuDebugPrint("AnalyzeDataOfReceive: gSMT.activeTable = %d (Just switch table), gSMT.curRecord = %d (back to 0).\n", gSMT.activeTable, gSMT.curRecord);

			}

			HcuDebugPrint("AnalyzeDataOfReceive: AnalyzeData Result saved in SMT completed.\n");

		}
	}
}

/*
 * State Machaine for HCU, to set the state
 */
UINT32 SetHcuState(UINT32 state)
{
	UINT32 temp = gHcuState;
	gHcuState = gHcuState | state;
	HcuDebugPrint("SetHcuState: gHcuState changed from %08X to %08X\n", temp, gHcuState);
	return gHcuState;
}

UINT32 DeSetHcuState(UINT32 state)
{
	UINT32 temp = gHcuState;
	gHcuState = gHcuState & (~state);
	HcuDebugPrint("SetHcuState: gHcuState changed from %08X to %08X\n", temp, gHcuState);
	return gHcuState;
}

/*
 * State Machaine for Hcu, to get the state
 */
UINT32 GetHcuState()
{
	return gHcuState;
}

UINT32 InitNetWorkService()
{
	UINT32 ret;

	/* Start GPRS and PPP */
	if (HCU_LOGIC_TRUE == gGprsPppdConf.gUseGprsOrNot)
	{
		ret = GprsPppdSystemStartup(&gGprsPppdConf);
		if (FAILURE == ret)
		{
			HcuErrorPrint("InitNetWorkService: Init GPRS and PPP failure, Hcu Exit.\n");
			return ret;
		}
		else
		{
			HcuDebugPrint("InitNetWorkService: Init  GPRS and PPP Success ...\n");
		}
	}

	/* Initial Socket */
	ret = InitSocketPort(&gSocketPort);
	if (FAILURE == ret)
	{
		HcuDebugPrint("InitNetWorkService: Init Socket Failure, Hcu Exit.\n");
		return ret;
	}
	else
	{
		HcuDebugPrint("InitNetWorkService: Init Socket Success ...\n");
	}

	/* TO DO: ADD OAM SEVER */
	return SUCCESS;
}

void GetTimeOfReport(char *TimeString)
{
	struct tm *t;
	time_t tt;
	time(&tt);
	t = localtime(&tt);
	if(NULL == TimeString)
		return;

	sprintf(TimeString, "%04d%02d%02d%02d%02d%02d", t->tm_year+1900, t->tm_mon+1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
}

void MsgCrc(char *msgSeg, char *msgCrc)
{
	if((NULL == msgCrc)||(NULL == msgSeg ))
		return;

	msgCrc[0] = 'A';
	msgCrc[1] = 'B';
	msgCrc[2] = 'C';
	msgCrc[3] = 'D';
	msgCrc[4] = 0x00;

	unsigned short Crc = 0xFFFF;
	unsigned char Tmp = 0;
	int i, j, nLength;

	nLength = strlen(msgSeg);

	for(i = 0; i < nLength ; i++)
	{
		Crc = Crc ^ (( (unsigned short)msgSeg[i] ) << 8);
		for (j = 0; j < 8; j++)
		{
			Tmp = Crc & 0x0001;
			Crc = Crc >> 1;

			if(0x0001 == Tmp)
				Crc = (Crc ^ 0xA001);
		}
	}
	sprintf(msgCrc, "%04X", Crc);
}

void HcuResetZigbee()
{
	/* Make Zb RST PIN to LOW, ZB RST Cable Connect to GPQ3 */
	GpioPortOp("Q3=0");
	GpioPortOp("M1=0");
	usleep(NB_MICROS_IN_ONE_SECOND);
	/* Make Zb RST PIN to HIGH, ZB RST Cable Connect to GPQ3 */
	GpioPortOp("Q3=1");
	GpioPortOp("M1=1");
}

void InitOamMsgBuf(OamControlMsgBuf_t *pocmb)
{
	pocmb->First = 0;
	pocmb->Last = 0;
	bzero( &(pocmb->MsgBuf[0]), sizeof(OamControlMsg_t) * MAX_OAM_CONTROL_MSG_BUFFER);
}

OamControlMsg_t *NextMsgToSave(OamControlMsgBuf_t *pocmb)
{
	char *MsgPtr = NULL;
	UINT32 PrevFirst = 0;

	MsgPtr = (OamControlMsg_t *)&(pocmb->MsgBuf[pocmb->First]);
	PrevFirst = pocmb->First;

	HcuDebugPrint("NextMsgToSave: Start: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);

	pocmb->First++;
	if(MAX_OAM_CONTROL_MSG_BUFFER == pocmb->First)
	{
		pocmb->First = 0;
	}

	/* Check if buffer is full */
	if (pocmb->Last == pocmb->First)
	{
		HcuErrorPrint("NextMsgToSave: Ring buffer full ...\n");
		pocmb->First = PrevFirst;
		HcuDebugPrint("NextMsgToSave: End: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);
		return NULL;
	}
	else
	{
		HcuDebugPrint("NextMsgToSave: End: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);
		return *MsgPtr;
	}
}

OamControlMsg_t *NextMsgToRead(OamControlMsgBuf_t *pocmb)
{

	char *MsgPtr = NULL;

	HcuDebugPrint("NextMsgToSave: Start: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);

	/* Check if buffer is empty */
	if (pocmb->Last == pocmb->First)
	{
		HcuErrorPrint("NextMsgToRead: Ring buffer empty ...\n");
		HcuDebugPrint("NextMsgToRead: End: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);
		return NULL;
	}

	MsgPtr = (OamControlMsg_t *)&(pocmb->MsgBuf[pocmb->Last]);

	pocmb->Last++;
	if(MAX_OAM_CONTROL_MSG_BUFFER == pocmb->Last)
	{
		pocmb->Last = 0;
	}

	HcuDebugPrint("NextMsgToRead: End: First = %d, Last = %d.\n", pocmb->First, pocmb->Last);
	return MsgPtr;
}

void OamMsgProcess(OamControlMsg_t *pOamMsg)
{
	if(NULL == pOamMsg)
	{
		HcuErrorPrint("OamMsgProcess: NULL == pOamMsg.\n");
		return;
	}

	switch(pOamMsg->MsgId)
	{
		case MSG_ID_DEBUG_TRACE_TOGGLE:
			HcuDebugPrint("OamMsgProcess: MSG_ID_DEBUG_TRACE_TOGGLE received ...\n");
			OamMsgProcessTraceToggle();
			break;

		case MSG_ID_SENSOR_POLLING:
			HcuDebugPrint("OamMsgProcess: MSG_ID_SENSOR_POLLING received ...\n");
			OamMsgProcessSensorPolling(pOamMsg);
			break;

		case MSG_ID_STOP_AUTO_POLLING:
			HcuDebugPrint("OamMsgProcess: MSG_ID_STOP_AUTO_POLLING received ...\n");
			DeSetHcuState(HCU_STATE_AUTO_POLLING);
			break;

		case MSG_ID_START_AUTO_POLLING:
			HcuDebugPrint("OamMsgProcess: MSG_ID_START_AUTO_POLLING received ...\n");
			SetHcuState(HCU_STATE_AUTO_POLLING);
			break;

		default:
			HcuDebugPrint("OamMsgProcess: unkown message received ...\n");
			break;
	}
}

void OamMsgProcessTraceToggle(void)
{
	if( HCU_LOGIC_TRUE == outputOnCrt)
	{
		outputOnCrt = HCU_LOGIC_FALSE;
		GpioPortOp("M3=1");
		GpioPortOp("Q4=1");
	}
	else
	{
		outputOnCrt = HCU_LOGIC_TRUE;
		GpioPortOp("M3=0");
		GpioPortOp("Q4=1");
	}
}

void OamMsgProcessSensorPolling(OamControlMsg_t *pOamMsg)
{

	UINT32 NumOfMeasure = 0;
	UINT32 NumOfSensor = 0;

	//UINT32 i = 0;
	UINT32 len = 0;
	char *f = NULL;

	if(pOamMsg->MsgLen != 2)
	{
		HcuDebugPrint("OamMsgProcessSensorPolling: MsgLen <> 3, do nothing.\n");
		return;
	}

	if(pOamMsg->MsgBody[0] < HCU_MAX_NUM_OF_WED)
	{
		NumOfSensor = pOamMsg->MsgBody[0];
	}
	else
	{
		HcuDebugPrint("OamMsgProcessSensorPolling: NumOfSensor > HCU_MAX_NUM_OF_WED, do nothing.\n");
		return;
	}

	if(pOamMsg->MsgBody[1] < WED_MAX_NUM_OF_MEASURE)
	{
		NumOfMeasure = pOamMsg->MsgBody[1];
	}
	else
	{
		HcuDebugPrint("OamMsgProcessSensorPolling: NumOfMeasure > WED_MAX_NUM_OF_MEASURE, do nothing.\n");
		return;
	}

	/* Start to send command */
	bzero(CmdToSendOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);

	/* Build the Data Req message */
	BuildDataReq( (UINT8 *)CmdToSendOverSerial, gSensorCmd[NumOfMeasure], NumOfSensor);

	/* Caculate the msg Lenth, later when DataReg Frame is added, Length of the message header will be included */
	len = strlen(gSensorCmd[NumOfMeasure]) + 7; /* 0 will changed to header and FCS length */

	/* Send the command over Serials */
	len = write(gSerialPortZb.fd, CmdToSendOverSerial, len);
	//gHcuMeasureBehavior.NumberOfFrameTotalTx++;

	f = CmdToSendOverSerial;
	HcuDebugPrint("OamMsgProcessSensorPolling: SEND: [%s] to sensor [%d]:\n", gSensorCmd[NumOfMeasure], NumOfSensor);
	HcuDebugPrint("OamMsgProcessSensorPolling: SEND: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n",f[0], f[1], f[2], f[3]);
	HcuDebugPrint("OamMsgProcessSensorPolling: SEND: PAYLOAD(12)|FCS(HEX): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | %02X\n", f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15], f[f[1]+4]);
}
#if 0
	for(NumOfSensor = 0; NumOfSensor<pOamMsg->MsgLen; NumOfSensor++)
	{

		NumOfMeasure = pOamMsg->MsgBody[NumOfSensor];

		if(0xFF != NumOfMeasure)
		{
			bzero(CmdToSendOverSerial, HCU_MAX_LENGTH_CMD_SERIAL);

			/* Build the Data Req message */
			BuildDataReq( (UINT8 *)CmdToSendOverSerial, gSensorCmd[NumOfMeasure], NumOfSensor);

			/* Caculate the msg Lenth, later when DataReg Frame is added, Length of the message header will be included */
			len = strlen(gSensorCmd[NumOfMeasure]) + 7; /* 0 will changed to header and FCS length */

			/* Send the command over Serials */
			len = write(gSerialPortZb.fd, CmdToSendOverSerial, len);
			//gHcuMeasureBehavior.NumberOfFrameTotalTx++;

			f = CmdToSendOverSerial;
			HcuDebugPrint("OamMsgProcessSensorPolling: SEND: [%s] to sensor [%d]:\n", gSensorCmd[NumOfMeasure], NumOfSensor);
			HcuDebugPrint("OamMsgProcessSensorPolling: SEND: SOP|LEN|CMD0|CMD1(HEX): %02X %02X %02X %02X\n",f[0], f[1], f[2], f[3]);
			HcuDebugPrint("OamMsgProcessSensorPolling: SEND: PAYLOAD(12)|FCS(HEX): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X | %02X\n",f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15], f[f[1]+4]);
		}
	}
#endif

