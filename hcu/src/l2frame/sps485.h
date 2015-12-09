/*
 * serialapi.h
 *
 *  Created on: 2015年11月2日
 *      Author: test
 */

#ifndef L2FRAME_SPS485_H_
#define L2FRAME_SPS485_H_


#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_SPS485
{
	FSM_STATE_SPS485_INITED = 0x02,
	FSM_STATE_SPS485_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_SPS485_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

//Global variables
extern FsmStateItem_t FsmSps485[];

//API
extern OPSTAT fsm_sps485_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_sps485_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_sps485_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);

//Local API
OPSTAT func_sps485_int_init(void);


#define COM_PORT_0 	0x00
#define COM_PORT_1 	0x01
#define COM_PORT_2 	0x02
#define COM_PORT_3 	0x03
#define COM_PORT_4 	0x04
#define COM_PORT_5 	0x05
#define COM_PORT_6 	0x06
#define COM_PORT_7 	0x07

#define COM_PORT_INVALIAD	0xFF

/*
** 第一种串口方式
**
*/
typedef struct SerialPort
{
	UINT8 id;			/* COM1=>0, COM1=>1, COM2=>2 ....  */
	UINT32 nSpeed;		/* 1200, 2400, 4800, 4800, 9600, 19200, 38400, 57600, 115200 */
	UINT16 nBits;		/* 7 or 8 */
	UINT8 nEvent;		/* '0', 'N', 'E' */
	UINT16 nStop;		/* 0, or 1 */
	UINT32 fd;			/* file descriptor for com port */
	UINT8 vTime;        /* */
	UINT8 vMin;         /* */
	UINT32 c_lflag;     /* ICANON : enable canonical input */
} SerialPort_t;

/*
 * Function API prototype for Serial Port
 */
extern UINT32 SerialPortOpen(UINT8 com_port_to_open, UINT16 *fd);
extern UINT32 SerialPortSet(SerialPort_t *sp);
extern UINT32 SerialPortClose(UINT16 fd);
extern UINT32 SerialPortSetVtimeVmin(SerialPort_t *sp, UINT8 vTime, UINT8 vMin);

extern UINT32 hcu_sps485_serial_init(SerialPort_t *sp);
extern UINT32 hcu_sps485_serial_port_get(SerialPort_t *sp, UINT8 *send_buf, UINT32 Len);
extern UINT32 hcu_sps485_serial_port_send(SerialPort_t *sp, UINT8 *rcv_buf, UINT32 Len);

// 第二种函数方法： 串口操作函数

/*串口设备无论是在工控领域，还是在嵌入式设备领域，应用都非常广泛。而串口编程也就显得必不可少。
偶然的一次机会，需要使用串口，而且操作系统还要求是Linux，因此，趁着这次机会，综合别人的代码，
进行了一次整理和封装。具体的封装格式为C代码，这样做是为了很好的移植性，使它可以在C和C++环境下，
都可以编译和使用。代码的头文件如下： */

// 串口设备信息结构
typedef struct tty_info_t
{
    int fd; // 串口设备ID
    pthread_mutex_t mt; // 线程同步互斥对象
    char name[24]; // 串口设备名称，例："/dev/ttyS0"
    struct termios ntm; // 新的串口设备选项
    struct termios otm; // 旧的串口设备选项
} TTY_INFO;
//

extern TTY_INFO *readyTTY(int id);
extern int setTTYSpeed(TTY_INFO *ptty, int speed);
extern int setTTYParity(TTY_INFO *ptty, int databits, char parity, int stopbits);
extern int cleanTTY(TTY_INFO *ptty);
extern int sendnTTY(TTY_INFO *ptty, char *pbuf, int size);
extern int recvnTTY(TTY_INFO *ptty, char *pbuf, int size);
extern int lockTTY(TTY_INFO *ptty);
extern int unlockTTY(TTY_INFO *ptty);

// 第三种串口函数方式

//=======================
//串口
#define     TTYS0   1
#define     TTYS1   2
#define     TTYS2   3
//波特率
#define     BAUD_2400       2400
#define     BAUD_4800       4800
#define     BAUD_9600       9600
#define     BAUD_115200     115200
#define     BAUD_460800     460800
//奇偶校验位
#define     PARITY_ODD    'O' //奇数
#define     PARITY_EVEN   'E' //偶数
//#define     PARITY_NONE   'N' //无奇偶校验位
//停止位
#define     STOP_BIT_1     1
#define     STOP_BIT_2     2
//数据位
#define     DATA_BIT_7     7
#define     DATA_BIT_8     8
//========================================
//串口API
extern int open_port(int fd,int comport); /*打开串口函数*/
extern int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);  //串口配置的函数
extern int read_datas_tty(int fd,char *rcv_buf,int TimeOut,int Len); //从串口中读取数据
extern int send_data_tty(int fd, char *send_buf,int Len); //向串口传数据
extern void ProcessInit(void);

#endif /* L2FRAME_SPS485_H_ */
