/*
 * serialapi.c
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#include "sps485.h"

#include "../comvm/trace.h"

/*
** FSM of the SPS485
*/
FsmStateItem_t FsmSps485[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_sps485_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_sps485_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_sps485_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_SPS485_RECEIVED,            fsm_sps485_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_sps485_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_SPS485, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("SPS485: Error Set FSM State at fsm_sps485_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_sps485_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_SPS485, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("SPS485: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SPS485], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_SPS485, FSM_STATE_SPS485_INITED) == FAILURE){
		HcuErrorPrint("SPS485: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化硬件接口
	if (func_sps485_int_init() == FAILURE){
		HcuErrorPrint("SPS485: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_SPS485, FSM_STATE_SPS485_RECEIVED) == FAILURE){
		HcuErrorPrint("SPS485: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("SPS485: Enter FSM_STATE_SPS485_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//进入阻塞式接收数据状态，然后继续发送
	while(1){
		//接收数据
		int dataLen=0;
		if (dataLen > 1){
			//发送数据给CLOUDCONT
			msg_struct_sps485_modbus_data_rx_t snd;
			memset(&snd, 0, sizeof(msg_struct_sps485_modbus_data_rx_t));
			snd.length = sizeof(msg_struct_sps485_modbus_data_rx_t);
			ret = hcu_message_send(MSG_ID_SPS485_MODBUS_DATA_RX, TASK_ID_MODBUS, TASK_ID_SPS485, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("SPS485: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_SPS485], zHcuTaskNameList[TASK_ID_MODBUS]);
				return FAILURE;
			}
		}

		hcu_sleep(5);
	}

	return SUCCESS;
}

OPSTAT fsm_sps485_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_sps485_int_init(void)
{
	return SUCCESS;
}


////////////////////////////////////////////第一种串口函数方式//////////////////////////////////////////////////////
/*
 * Global Variables
 */

#ifdef TARGET_LINUX_ARM
//	static char com_port_0_path[] = "/dev/ttySAC0";
//	static char com_port_1_path[] = "/dev/ttySAC1";
//	static char com_port_2_path[] = "/dev/ttySAC2";
//	static char com_port_3_path[] = "/dev/ttySAC3";
//	static char com_port_4_path[] = "/dev/ttySAC4";
	static char com_port_0_path[] = "/dev/ttyO0";
	static char com_port_1_path[] = "/dev/ttyO1";
	static char com_port_2_path[] = "/dev/ttyO2";
	static char com_port_3_path[] = "/dev/ttyO3";
	static char com_port_4_path[] = "/dev/ttyO4";
#endif

/*
 */
#ifdef TARGET_LINUX_X86
	static char com_port_0_path[] = "/dev/ttyS0";
	static char com_port_1_path[] = "/dev/ttyS1";
	static char com_port_2_path[] = "/dev/ttyS2";
	//static char com_port_3_path[] = "/dev/ttyS3";
	//static char com_port_4_path[] = "/dev/ttyS4";
  static char com_port_3_path[] = "/dev/ttyUSB0";
  static char com_port_4_path[] = "/dev/ttyUSB1";
#endif
/*
** Local static variables
*/
//static unsigned char traceFileName[] = __FILE__;


UINT32 SerialPortOpen(UINT8 com_port_to_open, UINT16 *fd)
{

	/* Port Open Result */
	if (NULL == fd)
	{
		HcuErrorPrint("SPS485: Invalid fd, *fd is pointing to address NULL.\n");
		return FAILURE;
	}

	if (COM_PORT_0 == com_port_to_open)
	{
		*fd = open( com_port_0_path, O_RDWR|O_NOCTTY|O_NDELAY);

		if (HCU_INVALID_U16 == *fd)
		{
			HcuErrorPrint("SPS485: Can't Open Serial Port COM0\n");
			return FAILURE;
		}
	}
	else if(COM_PORT_1 == com_port_to_open)
	{
		*fd = open( com_port_1_path, O_RDWR|O_NOCTTY|O_NDELAY);

		if (HCU_INVALID_U16 == *fd)
		{
			HcuErrorPrint("SPS485: Can't Open Serial Port COM1\n");
			return FAILURE;
		}
	}
	else if(COM_PORT_2 == com_port_to_open)
	{
		*fd = open( com_port_2_path, O_RDWR|O_NOCTTY|O_NDELAY);

		if (HCU_INVALID_U16 == *fd)
		{
			HcuErrorPrint("SPS485: Can't Open Serial Port COM2\n");
			return FAILURE;
		}
	}

	else if(COM_PORT_3 == com_port_to_open)
	{
		*fd = open( com_port_3_path, O_RDWR|O_NOCTTY|O_NDELAY);

		if (HCU_INVALID_U16 == *fd)
		{
			HcuErrorPrint("SPS485: Can't Open Serial Port COM3\n");
			return FAILURE;
		}
	}

	else if(COM_PORT_4 == com_port_to_open)
	{
		*fd = open( com_port_4_path, O_RDWR|O_NOCTTY|O_NDELAY);

		if (HCU_INVALID_U16 == *fd)
		{
			HcuErrorPrint("SPS485: Can't Open Serial Port COM4\n");
			return FAILURE;
		}
	}
	else
	{
		HcuErrorPrint("SPS485: Input COM port ID = %d, should be between 0 and 4.\n", com_port_to_open);
		return FAILURE;
	}

	if(fcntl(*fd, F_SETFL, 0) < 0)
	{
		HcuErrorPrint("SPS485: fcntl failed!\n");
		return FAILURE;
	}

	if(0 == isatty(STDIN_FILENO))
	{
		HcuErrorPrint("SPS485: Standard input is not a terminal device\n");
		return FAILURE;
	}

	/* Return the result */
	HcuDebugPrint("SPS485: COM %d has been openned\n", com_port_to_open);
	return SUCCESS;
}


UINT32 SerialPortSet(SerialPort_t *sp)
{

	/* Local variable */
	struct termios newtio, oldtio;
	UINT32 cflag, iflag, oflag, lflag;

	if ( tcgetattr(sp->fd, &oldtio)  !=  SUCCESS)
	{
		HcuErrorPrint("SPS485: Com port control fd is not valid\n");
		return FAILURE;
	}

	/* Set all bit to Zero */
	bzero( &newtio, sizeof( newtio ) );

	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = sp->c_lflag;

	switch( sp->nBits )
	{
	case 7:
		newtio.c_cflag |= CS7;
		break;
	case 8:
		newtio.c_cflag |= CS8;
		break;
	default:
		HcuErrorPrint("SPS485: Invalid bit lenth set for com port, nBit should be 7 or 8, error return\n");
		return FAILURE;
	}

	switch( sp->nEvent )
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E':
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':
		newtio.c_cflag &= ~PARENB;
		break;
	default:
		HcuErrorPrint("SPS485: Invalid parity for com port, nEvent should be '0', 'E', or 'N', error return\n");
		return FAILURE;
	}

	switch( sp->nSpeed )
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 38400:
		cfsetispeed(&newtio, B38400);
		cfsetospeed(&newtio, B38400);
		break;
	case 57600:
		cfsetispeed(&newtio, B57600);
		cfsetospeed(&newtio, B57600);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	case 230400:
		cfsetispeed(&newtio, B230400);
		cfsetospeed(&newtio, B230400);
		break;
	default:
		HcuErrorPrint("SPS485: Invalid speed set for com port, Error return\n");
		return FAILURE;
		break;
	}

	if( 1 == sp->nStop)
		newtio.c_cflag &=  ~CSTOPB;
	else if ( 2 == sp->nStop)
		newtio.c_cflag |=  CSTOPB;
	else
	{
		HcuErrorPrint("SPS485: Invalid stop bit for com port, Error return\n");
		return FAILURE;

	}

	/* default VTIME and VMIN */
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 1;  /* If there is one in the buffer, it will be return from block read, */

	/* Clean the buffer !!! before !!! the parameter active */
	tcflush(sp->fd, TCIFLUSH);

	/* Set parameters */
	if((tcsetattr(sp->fd,TCSANOW, &newtio))!=0)
	{
		HcuErrorPrint("SPS485: COM port set error\n");
		return FAILURE;
	}

	/* Save VTIME and VMIN */
	sp->vTime = 0;
	sp->vMin = 1;

	/* Read out the config paramter for debug */
	cflag = (UINT32)newtio.c_cflag;
	iflag = (UINT32)newtio.c_iflag;
	oflag = (UINT32)newtio.c_oflag;
	lflag = (UINT32)newtio.c_lflag;

	HcuDebugPrint("SPS485: COM port flags: c_cflag = 0x%X, c_iflag = 0x%X, c_oflag = 0x%X, c_lflag = 0x%X, VTIME = 0x%d, TMIN = 0x%d\n", cflag, iflag, oflag, lflag, newtio.c_cc[VTIME], newtio.c_cc[VMIN] = 1);

	HcuDebugPrint("SPS485: COM port set done!\n");

	return SUCCESS;
}

UINT32 SerialPortClose(UINT16 fd)
{
	/* Local variable */
	struct termios oldtio;

	if ( tcgetattr(fd, &oldtio)  !=  SUCCESS)
	{
		HcuErrorPrint("SPS485: Com port control fd is not valid\n");
		return FAILURE;
	}

	/* Close the PORT */
	close(fd);
	return SUCCESS;
}

UINT32 SerialPortSetVtimeVmin(SerialPort_t *sp, UINT8 vTime, UINT8 vMin)
{
	/* Local variable */
	struct termios oldtio;
	UINT8 fd = sp->fd;

	if ( tcgetattr(fd, &oldtio) !=  SUCCESS )
	{
		HcuErrorPrint("SPS485: Com port control fd is not valid\n");
		return FAILURE;
	}

	/* Set VTIME and VMIN */
	oldtio.c_cc[VTIME]  = vTime;
	oldtio.c_cc[VMIN] = vMin;  /* If there is one in the buffer, it will be return from block read, */

	/* Clean the buffer !!! before !!! the parameter active */
	tcflush(fd, TCIFLUSH);

	/* Set parameters */
	if((tcsetattr(fd, TCSANOW, &oldtio))!=0)
	{
		HcuErrorPrint("SPS485: COM port set error\n");
		return FAILURE;
	}

	/* Save VTIME and VMIN */
	sp->vTime = vTime;
	sp->vMin = vMin;

	return SUCCESS;
}


UINT32 hcu_sps485_serial_init(SerialPort_t *sp)
{
  UINT32 ret = SUCCESS;

  HcuDebugPrint("SPS485: InitSerialPort Start ...\n");

  ret = SerialPortOpen(sp->id, (UINT16 *)(&(sp->fd)));
  if (FAILURE == ret)
  {
    HcuErrorPrint("SPS485: InitSerialPort serial port COM%d open failure.\n", sp->id);
    return ret;
  }
  HcuDebugPrint("SPS485: InitSerialPort serial port COM%d open successfully.\n", sp->id);

  ret = SerialPortSet(sp);
  if (FAILURE == ret)
  {
    HcuErrorPrint("SPS485: Open Serial Port %d failure.\n", sp->id);
    return ret;
  }
  HcuDebugPrint("SPS485: InitSerialPort serial port COM%d properties has been set successfully.\n", sp->id);


  HcuDebugPrint("SPS485: InitSerialPort completed.\n");
  return ret;
}

UINT32 hcu_sps485_serial_port_get(SerialPort_t *sp, UINT8 *rcv_buf, UINT32 Len)
{
    UINT16  fd = sp->fd;
    UINT32 ret = SUCCESS;

    ret = read(fd, rcv_buf, Len);
    if (FAILURE == ret)
   {
        HcuErrorPrint("SPS485: Read Data failure \n");
        return FAILURE;
    }
    else if(!ret)
    {
         HcuErrorPrint("SPS485: Read return 0 byte \n");
         return FAILURE;
     }


    return ret;
}

UINT32 hcu_sps485_serial_port_send(SerialPort_t *sp, UINT8 *send_buf, UINT32 Len)
{
     UINT16  fd = sp->fd;
     UINT32 ret = SUCCESS;

     ret = write(fd, send_buf, Len);
     if (FAILURE == ret)
     {
         HcuErrorPrint("SPS485: Write device error\n");
         return FAILURE;
     }
     return ret;
}



////////////////////////////////////////////第二种串口函数方式//////////////////////////////////////////////////////
/*从头文件中的函数定义不难看出，函数的功能，使用过程如下：
（1） 打开串口设备，调用函数setTTYSpeed（）；
（2） 设置串口读写的波特率，调用函数setTTYSpeed（）；
（3） 设置串口的属性，包括停止位、校验位、数据位等，调用函数setTTYParity（）；
（4） 向串口写入数据，调用函数sendnTTY（）；
（5） 从串口读出数据，调用函数recvnTTY（）；
（6） 操作完成后，需要调用函数cleanTTY（）来释放申请的串口信息接口；
其中，lockTTY（）和unlockTTY（）是为了能够在多线程中使用。在读写操作的前后，需要锁定和释放串口资源。
具体的使用方法，在代码实现的原文件中，main（）函数中进行了演示。下面就是源代码文件： */
////////////////////////////////////////////////////////////////////////////////
TTY_INFO *readyTTY(int id)
{
    TTY_INFO *ptty;
    ptty = (TTY_INFO *)malloc(sizeof(TTY_INFO));
    if(ptty == NULL)
        return NULL;
    memset(ptty,0,sizeof(TTY_INFO));
    pthread_mutex_init(&ptty->mt,NULL);
    sprintf(ptty->name,"/dev/ttyS%d",id);
    //
    // 打开并且设置串口
    ptty->fd = open(ptty->name, O_RDWR | O_NOCTTY |O_NDELAY);
    if (ptty->fd <0)
    {
        free(ptty);
        return NULL;
    }
    //
    // 取得并且保存原来的设置
    tcgetattr(ptty->fd,&ptty->otm);
    return ptty;
}

///////////////////////////////////////////////////////////////////////////////
// 清理串口设备资源
int cleanTTY(TTY_INFO *ptty)
{
    //
    // 关闭打开的串口设备
    if(ptty->fd>0)
    {
        tcsetattr(ptty->fd,TCSANOW,&ptty->otm);
        close(ptty->fd);
        ptty->fd = -1;
        free(ptty);
        ptty = NULL;
    }
    return 0;
}


///////////////////////////////////////////////////////////////////////////////
// 设置串口通信速率
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针
// speed 参数类型(int),用来设置串口的波特率
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值
///////////////////////////////////////////////////////////////////////////////
int setTTYSpeed(TTY_INFO *ptty, int speed)
{
    //int i;
    //
    // 进行新的串口设置,数据位为8位
    bzero(&ptty->ntm, sizeof(ptty->ntm));
    tcgetattr(ptty->fd,&ptty->ntm);
    ptty->ntm.c_cflag = /*CS8 |*/ CLOCAL | CREAD;
    switch(speed)
    {
    case 300:
        ptty->ntm.c_cflag |= B300;
        break;
    case 1200:
        ptty->ntm.c_cflag |= B1200;
        break;
    case 2400:
        ptty->ntm.c_cflag |= B2400;
        break;
    case 4800:
        ptty->ntm.c_cflag |= B4800;
        break;
    case 9600:
        ptty->ntm.c_cflag |= B9600;
        break;
    case 19200:
        ptty->ntm.c_cflag |= B19200;
        break;
    case 38400:
        ptty->ntm.c_cflag |= B38400;
        break;
    case 115200:
        ptty->ntm.c_cflag |= B115200;
        break;
    }
    ptty->ntm.c_iflag = IGNPAR;
    ptty->ntm.c_oflag = 0;
    //
    //
    tcflush(ptty->fd, TCIFLUSH);
    tcsetattr(ptty->fd,TCSANOW,&ptty->ntm);
    //
    //
    return 0;
}
// 设置串口数据位，停止位和效验位
// ptty 参数类型(TTY_INFO *),已经初始化的串口设备信息结构指针
// databits 参数类型(int), 数据位,取值为7或者8
// stopbits 参数类型(int),停止位,取值为1或者2
// parity 参数类型(int),效验类型 取值为N,E,O,,S
// return 返回值类型(int),函数执行成功返回零值，否则返回大于零的值
///////////////////////////////////////////////////////////////////////////////
int setTTYParity(TTY_INFO *ptty, int databits, char parity, int stopbits)
{
    //
    // 取得串口设置

    if( tcgetattr(ptty->fd,&ptty->ntm) != 0)
    {
        HcuDebugPrint("SPS485: SetupSerial [%s]\n",ptty->name);
        return 1;
    }
    bzero(&ptty->ntm, sizeof(ptty->ntm));
    ptty->ntm.c_cflag = CS8 | CLOCAL | CREAD;
    ptty->ntm.c_iflag = IGNPAR;
    ptty->ntm.c_oflag = 0;
    //
    // 设置串口的各种参数
    ptty->ntm.c_cflag &= ~CSIZE;
    switch (databits)
    { //设置数据位数
    case 7:
        ptty->ntm.c_cflag |= CS7;
        break;
    case 8:
        ptty->ntm.c_cflag |= CS8;
        break;
    default:
        HcuDebugPrint("SPS485: Unsupported data size\n");
        return 5;
    }
    //
    //
    switch (parity)
    { // 设置奇偶校验位数
    case 'n':
    case 'N':
        ptty->ntm.c_cflag &= ~PARENB; /* Clear parity enable */
        ptty->ntm.c_iflag &= ~INPCK; /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        ptty->ntm.c_cflag |= (PARODD|PARENB); /* 设置为奇效验*/
        ptty->ntm.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        ptty->ntm.c_cflag |= PARENB; /* Enable parity */
        ptty->ntm.c_cflag &= ~PARODD; /* 转换为偶效验*/
        ptty->ntm.c_iflag |= INPCK; /* Disnable parity checking */
        break;
    case 'S':
    case 's': /*as no parity*/
        ptty->ntm.c_cflag &= ~PARENB;
        ptty->ntm.c_cflag &= ~CSTOPB;
        break;
    default:
        HcuDebugPrint("SPS485: Unsupported parity\n");
        return 2;
    }
    //
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        ptty->ntm.c_cflag &= ~CSTOPB;
        break;
    case 2:
        ptty->ntm.c_cflag |= CSTOPB;
        break;
    default:
        HcuDebugPrint("SPS485: Unsupported stop bits\n");
        return 3;
    }
    //
    //
    ptty->ntm.c_lflag = 0;
    ptty->ntm.c_cc[VTIME] = 0; // inter-character timer unused

    ptty->ntm.c_cc[VMIN] = 1; // blocking read until 1 chars received

    tcflush(ptty->fd, TCIFLUSH);
    if (tcsetattr(ptty->fd,TCSANOW,&ptty->ntm) != 0)
    {
        HcuDebugPrint("SPS485: SetupSerial \n");
        return 4;
    }

    return 0;
}

int recvnTTY(TTY_INFO *ptty,char *pbuf,int size)
{
    int ret,left,bytes;
    left = size;
    while(left>0)
    {
        ret = 0;
        bytes = 0;
        pthread_mutex_lock(&ptty->mt);
        ioctl(ptty->fd, FIONREAD, &bytes);
        if(bytes>0)
        {
            ret = read(ptty->fd,pbuf,left);
        }
        pthread_mutex_unlock(&ptty->mt);
        if(ret >0)
        {
            left -= ret;
            pbuf += ret;
        }
        usleep(100);
    }
    return size - left;
}

int sendnTTY(TTY_INFO *ptty,char *pbuf,int size)
{
    int ret,nleft;
    char *ptmp;
    ret = 0;
    nleft = size;
    ptmp = pbuf;

    while(nleft>0)
    {
        pthread_mutex_lock(&ptty->mt);
        ret = write(ptty->fd,ptmp,nleft);
        pthread_mutex_unlock(&ptty->mt);
        if(ret >0)
        {
            nleft -= ret;
            ptmp += ret;
        }
        //usleep(100);
    }
    return size - nleft;
}

int lockTTY(TTY_INFO *ptty)
{
    if(ptty->fd < 0)
    {
        return 1;
    }
    return flock(ptty->fd,LOCK_EX);
}

int unlockTTY(TTY_INFO *ptty)
{
    if(ptty->fd < 0)
    {
        return 1;
    }
    return flock(ptty->fd,LOCK_UN);
}


///////////////////////////////////////////////////////////////////////////////
// 接口测试
int sps_test_main(int argc,char **argv)
{
    TTY_INFO *ptty;
    int nbyte,idx;
    unsigned char cc[16];

    ptty = readyTTY(0);
    if(ptty == NULL)
    {
        HcuDebugPrint("SPS485: readyTTY(0) error\n");
        return 1;
    }
    //
    //

    lockTTY(ptty);
    if(setTTYSpeed(ptty,9600)>0)
    {
        HcuDebugPrint("SPS485: setTTYSpeed() error\n");
        return -1;
    }
    if(setTTYParity(ptty,8, 'N',1)>0)
    {
        HcuDebugPrint("SPS485: setTTYParity() error\n");
        return -1;
    }
    //
    idx = 0;
    while(1)
    {
        cc[0] = 0xFA;
        sendnTTY(ptty,&cc[0],1);
        nbyte = recvnTTY(ptty,cc,1);
        HcuDebugPrint("SPS485: %d:%02X\n",idx++,cc[0]);
    }
    cleanTTY(ptty);
}

////////////////////////////////////////////第三种串口函数方式//////////////////////////////////////////////////////
/// http://blog.csdn.net/lindabell/article/details/8900294
//串口配置的函数
int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if (tcgetattr(fd, &oldtio) != 0)
    {
        HcuErrorPrint("SPS485: SPS486: SetupSerial 1 \n");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    /*步骤一，设置字符大小*/
    newtio.c_cflag |= CLOCAL;   //如果设置，modem 的控制线将会被忽略。如果没有设置，则 open()函数会阻塞直到载波检测线宣告 modem 处于摘机状态为止。
    newtio.c_cflag |= CREAD;    //使端口能读取输入的数据
    /*设置每个数据的位数*/
    switch (nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /*设置奇偶校验位*/
    switch (nEvent)
    {
    case 'o':
    case 'O': //奇数
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;   //使能校验，如果不设PARODD则是偶校验
        newtio.c_cflag |= PARODD;   //奇校验
        break;
    case 'e':
    case 'E': //偶数
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'n':
    case 'N':  //无奇偶校验位
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /*设置波特率*/
    switch (nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /*
     * 设置停止位
     * 设置停止位的位数， 如果设置，则会在每帧后产生两个停止位， 如果没有设置，则产生一个
     * 停止位。一般都是使用一位停止位。需要两位停止位的设备已过时了。
     * */
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    /*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        HcuErrorPrint("SPS485: com set error\n");
        return -1;
    }
    HcuDebugPrint("SPS485: set done!\n");
    return 0;
}
//======================================================
//从串口中读取数据
int read_datas_tty(int fd, char *rcv_buf, int TimeOut, int Len)
{
    int retval;
    fd_set rfds;
    struct timeval tv;
    int ret, pos;
    tv.tv_sec = TimeOut / 1000;  //set the rcv wait time
    tv.tv_usec = TimeOut % 1000 * 1000;  //100000us = 0.1s

    pos = 0;
    while (1)
    {
        FD_ZERO(&rfds); //每次循环都要清空集合，否则不能检测描述符变化
        FD_SET(fd, &rfds);  //添加描述符
        retval = select(fd + 1, &rfds, NULL, NULL, &tv);    //fd+1:描述符最大值加1 返回负数：出错  0:timeout  >0:有文件可读写
        if (retval == -1)
        {
            HcuErrorPrint("SPS485: select()\n");
            break;
        }
        else if (retval)
        {
            ret = read(fd, rcv_buf + pos, 1);
            if (-1 == ret)
            {
                break;
            }

            pos++;
            if (Len <= pos)
            {
                break;
            }
        }
        else
        {
            break;
        }
    }

    return pos;
}

//向串口传数据
int send_data_tty(int fd, char *send_buf, int Len)
{
    ssize_t ret;

    ret = write(fd, send_buf, Len);
    if (ret == -1)
    {
        HcuDebugPrint("SPS485: write device error\n");
        return -1;
    }

    return 1;
}

//======================================
int SerFd = -1;
void ProcessInit(void)
{
    /*
     * 打开USB转串口（ttyUSB0）
     * O_RDWR 可读写设备
     * O_NOCTTY 如果欲打开的文件为终端机设备时，则不会将该终端机当成进程控制终端机
     * O_NDELAY 以不可阻断的方式打开文件，也就是无论有无数据读取或等待，都会立即返回进程之中
     */
    SerFd = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY);
    if (0 < SerFd)
    {
        set_opt(SerFd, BAUD_2400, DATA_BIT_8, PARITY_NONE, STOP_BIT_1);//设置串口参数
    }
    else
    {
        HcuDebugPrint("SPS485: open_port ERROR !\n");
    }
}

//例子程序，上层可以按照这个方式进行调用使用
int sps_test_main_linabell(int argc, char *argv[])
{
    int nTmp = 0;
    char Buf[1024];
    ProcessInit();
    send_data_tty(SerFd, "hello series\n",sizeof("hello series\n"));
    while (1)
    {
        //从串口中读取数据
        nTmp = read_datas_tty(SerFd, Buf, 100, 1024);
        if(nTmp)HcuDebugPrint("SPS485: %s\n",Buf);
        if (0 < nTmp)
        {
            //HcuDebugPrint("SPS485: rcv len=%d,data:%s\n",nTmp,Buf);
            //向串口传数据
            send_data_tty(SerFd, Buf, nTmp);
        }
    }
}

