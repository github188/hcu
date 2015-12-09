/*
 * camera.c
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#include "camera.h"

#include "../comvm/trace.h"
#include "../l2frame/l2comdef.h"
//#include <jpeglib.h> //jpeg
//ffmpeg库
//#include <libavformat/avformat.h>
//#include <libavcodec/avcodec.h>
//#include <libswscale/swscale.h>
//#include <SDL/SDL.h>
//#include <SDL/SDL_thread.h>


/*
** FSM of the CAMERA
*/
FsmStateItem_t FsmCamera[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_camera_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_camera_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_camera_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_CAMERA_RECEIVED,            	fsm_camera_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Global variables
HcuBufferType_t *zHcuCameraUsrBuf;
static UINT32 zHcuCameraNbuffer;

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_camera_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_CAMERA, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("CAMERA: Error Set FSM State at fsm_camera_task_entry\n");}
	return SUCCESS;
}

OPSTAT fsm_camera_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_CAMERA, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("CAMERA: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_CAMERA], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_CAMERA, FSM_STATE_CAMERA_INITED) == FAILURE){
		HcuErrorPrint("CAMERA: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化硬件接口
	if (func_camera_int_init() == FAILURE){
		HcuErrorPrint("CAMERA: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_CAMERA, FSM_STATE_CAMERA_RECEIVED) == FAILURE){
		HcuErrorPrint("CAMERA: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("CAMERA: Enter FSM_STATE_CAMERA_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)


	//进入阻塞式接收数据状态，然后继续发送

	UINT32 fd;
	//打开设备
	fd = func_camera_open_device();
	if (fd == FAILURE){
		HcuErrorPrint("CAMERA: Open device error!\n");
		return FAILURE;
	}
	//初始化设备
	if (func_camera_init_device(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Initialize device error!\n");
		return FAILURE;
	}
	//检查和设置取景框
	if (func_camera_check_scale(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Check scale error!\n");
		return FAILURE;
	}
	//检查和设置输入输出设备
	if (func_camera_check_input_output(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Check input/output error!\n");
		return FAILURE;
	}
	//检查和设置输入输出设备
	if (func_camera_check_standard(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Check standard error!\n");
		return FAILURE;
	}
	//初始化内存交换缓冲区
	if (func_camera_init_mmap(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Initialize MMAP error!\n");
		return FAILURE;
	}
	//启动捕获
	if (func_camera_start_capture(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Start capture error!\n");
		return FAILURE;
	}
	//进入主循环
	if (func_camera_mainloop(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Enter main loop of capture error!\n");
		return FAILURE;
	}
	//停止捕获
	if (func_camera_stop_capture(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Stop capture error!\n");
		return FAILURE;
	}
	//关闭设备
	if (func_camera_close_device(fd) == FAILURE){
		HcuErrorPrint("CAMERA: Close device error!\n");
		return FAILURE;
	}

	//以下为之前的旧测试程序
	/*while(1){
		//接收数据
		int dataLen=0;
		if (dataLen > 1){
			//发送数据给CLOUDCONT
			msg_struct_camera_hsmmp_data_rx_t snd;
			memset(&snd, 0, sizeof(msg_struct_camera_hsmmp_data_rx_t));
			snd.length = sizeof(msg_struct_camera_hsmmp_data_rx_t);
			ret = hcu_message_send(MSG_ID_CAMERA_HSMMP_DATA_RX, TASK_ID_HSMMP, TASK_ID_CAMERA, &snd, snd.length);
			if (ret == FAILURE){
				HcuErrorPrint("CAMERA: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_CAMERA], zHcuTaskNameList[TASK_ID_HSMMP]);
				return FAILURE;
			}
		}
		hcu_sleep(5);
	}*/

	return SUCCESS;
}

OPSTAT fsm_camera_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_camera_int_init(void)
{
	return SUCCESS;
}

/*
 * V4L2(Video For Linux Two) 是内核提供给应用程序访问音、视频驱动的统一接口。
 * 工作流程：打开设备－> 检查和设置设备属性－> 设置帧格式－> 设置一种输入输出方法（缓冲 区管理）－> 循环获取数据－> 关闭设备。
 *
 * 	在V4L2 中， 视频设备被看做一个文件。使用open 函数打开这个设备：
 * 	// 用非阻塞模式打开摄像头设备
 * 	int cameraFd ;
 * 	cameraFd = open (“/dev/video0″ , O_RDWR | O_NONBLOCK , 0);
 * 	// 如果用阻塞模式打开摄像头设备，上 述代码变为：
 * 	//cameraFd = open(”/dev/video0″, O_RDWR, 0);
 *
 */

//打开设备
int func_camera_open_device(void)
{
	int fd;
	//open video device with block
	fd = open(HCU_DEFAULT_DEVICE_USB_CAMERA0, O_RDONLY, 0);
	if(fd < 0){
		HcuErrorPrint("CAMERA: FILE_VIDEO error, %d!\n", HCU_DEFAULT_DEVICE_USB_CAMERA);
		return FAILURE;
	}
	return fd;
}

/*
 * 查询设备属性： VIDIOC_QUERYCAP
 * 查看或设置当前格式: VIDIOC_G_FMT (get), VIDIOC_S_FMT (set)
 * 查询并显示所有支持的格式：VIDIOC_ENUM_FMT
 * struct v4l2_capability
 * {
 * 	u8 driver[16]; // 驱动名字
 * 	u8 card[32]; // 设备名字
 * 	u8 bus_info[32]; // 设备在系统中的位置
 * 	u32 version; // 驱动版本号
 * 	u32 capabilities; // 设备支持的操作
 * 	u32 reserved[4]; // 保留字段
 * };
 *
 * struct v4l2_fmtdesc
 * {
 * 	u32 index; // 要查询的格式序号，应用程序设置
 * 	enum v4l2_buf_type type; // 帧类型，应用程序设置
 * 	u32 flags; // 是否为压缩格式
 * 	u8 description[32]; // 格式名称
 * 	u32 pixelformat; // 格式
 * 	u32 reserved[4]; // 保留
 * };
 *
 * struct v4l2_format
 * {
 * 	enum v4l2_buf_type type; // 帧类型，应用程序设置
 * 	union fmt
 * 	{
 * 		struct v4l2_pix_format pix; // 视频设备使用
 * 		struct v4l2_window win;
 * 		struct v4l2_vbi_format vbi;
 * 		struct v4l2_sliced_vbi_format sliced;
 * 		u8 raw_data[200];
 * 	};
 * };
 *
 * struct v4l2_pix_format
 * 	{
 * 		u32 width; // 帧宽，单位像素
 * 		u32 height; // 帧高，单位像素
 * 		u32 pixelformat; // 帧格式
 * 		enum v4l2_field field;
 * 		u32 bytesperline;
 * 		u32 sizeimage;
 * 		enum v4l2_colorspace colorspace;
 * 		u32 priv;
 * 	};
 *
 */
//初始化设备
//initial camera device
OPSTAT func_camera_init_device(UINT32 fd)
{
	//能力控制 decive fuction, such as video input
	struct v4l2_capability cap;
	//视频标准video standard,such as PAL,NTSC
	struct v4l2_standard std;
	//帧格式frame format
	struct v4l2_format tv_fmt;
	//检查控制check control
	struct v4l2_queryctrl query;
	//详细控制参数detail control value
	struct v4l2_fmtdesc fmt;
	int ret;

	//获取摄像头视频参量 get the format of video supply
	memset(&cap, 0, sizeof(cap));
	memset(&std, 0, sizeof(std));
	memset(&tv_fmt, 0, sizeof(tv_fmt));
	memset(&query, 0, sizeof(query));
	memset(&fmt, 0, sizeof(fmt));
	fmt.index = 0;
	//supply to image capture
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	//显示所有支持的视频格式 show all format of supply
	while(ioctl(fd, VIDIOC_ENUM_FMT, &fmt) != -1){
		fmt.index++;
		HcuDebugPrint("CAMER:A Support format: pixelformat = ''%c%c%c%c''\ndescription = ''%s'', flag=%d\n",fmt.pixelformat & 0xFF,
				(fmt.pixelformat >> 8) & 0xFF,(fmt.pixelformat >> 16) & 0xFF, (fmt.pixelformat >> 24) & 0xFF,fmt.description, fmt.flags);
	}
	//FLAG指示是否为压缩，一样显示出来

	//获取视频卡的完整能力，check video device driver capability
	ret = ioctl(fd, VIDIOC_QUERYCAP, &cap);
	if(ret < 0){
		HcuErrorPrint("CAMERA: Fail to ioctl VIDEO_QUERYCAP!\n");
		return FAILURE;
	}
	HcuDebugPrint("CAMERA: Driver Name=%s, Card Name=%s, Bus info=%s, Driver Version=%u.%u.%u\n", cap.driver, cap.card, cap.bus_info,
			(cap.version>>16)&0XFF, (cap.version>>8)&0XFF, cap.version&0XFF);

	//判定是否支持视频捕获 judge wherher or not to be a video-get device
	if(!(cap.capabilities & V4L2_BUF_TYPE_VIDEO_CAPTURE))
	{
		HcuErrorPrint("CAMERA: The Current device is not a video capture device!\n");
		return FAILURE;
	}

	//判定是否支持流媒体 judge whether or not to supply the form of video stream
	if(!(cap.capabilities & V4L2_CAP_STREAMING))
	{
		HcuErrorPrint("CAMERA: The Current device does not support streaming i/o!\n");
		return FAILURE;
	}

	//设置摄像头捕获参量 set the form of camera capture data
	tv_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	tv_fmt.fmt.pix.width = 680;
	tv_fmt.fmt.pix.height = 480;
	tv_fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
	tv_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	if (ioctl(fd, VIDIOC_S_FMT, &tv_fmt)< 0) {
		HcuErrorPrint("CAMERA: VIDIOC_S_FMT!\n");
		close(fd);
		return FAILURE;
	}
	return SUCCESS;
}

/*
 * 图像的缩放 VIDIOC_CROPCAP
 * 设置视频的制式和取景框
 * 相关结构体：
 * v4l2_cropcap 结构体用来设置摄像头的捕捉能力，在捕捉上视频时应先先设置
 * v4l2_cropcap 的 type 域，再通过 VIDIO_CROPCAP 操作命令获取设备捕捉能力的参数，保存于 v4l2_cropcap 结构体中，包括 bounds（最大捕捉方框的左上角坐标和宽高），
 * defrect（默认捕捉方框的左上角坐标和宽高）等。
 * v4l2_format 结构体用来设置摄像头的视频制式、帧格式等，在设置这个参数时应先填 好 v4l2_format 的各个域，如 type（传输流类型），fmt.pix.width(宽)，
 * fmt.pix.heigth(高)，fmt.pix.field(采样区域，如隔行采样)，fmt.pix.pixelformat(采样类型，如 YUV4:2:2)，然后通过 VIDIO_S_FMT 操作命令设置视频捕捉格式。
 *
 * Cropping 和 scaling 主要指的是图像的取景范围及图片的比例缩放的支持。Crop 就 是把得到的数据作一定的裁剪和伸缩，裁剪可以只取样我们可以得到的图像大小的一部分，
 * 剪裁的主要参数是位置、长度、宽度。而 scale 的设置是通过 VIDIOC_G_FMT 和 VIDIOC_S_FMT 来获得和设置当前的 image 的长度，宽度来实现的。
 *
 * 我们可以假设 bounds 是 sensor 最大能捕捉到的图像范围，而 defrect 是设备默认 的最大取样范围，这个可以通过 VIDIOC_CROPCAP 的 ioctl 来获得设备的
 * crap 相关的属 性 v4l2_cropcap，其中的 bounds 就是这个 bounds，其实就是上限。每个设备都有个默 认的取样范围，就是 defrect，就是 default rect 的
 * 意思，它比 bounds 要小一些。这 个范围也是通过 VIDIOC_CROPCAP 的 ioctl 来获得的 v4l2_cropcap 结构中的 defrect 来表示的，我们可以通过
 * VIDIOC_G_CROP 和 VIDIOC_S_CROP 来获取和设置设备当前的 crop 设置。
 *
 * 相关结构体：
 * struct v4l2_cropcap
 * {
 * 	enum v4l2_buf_type type; // 数据流的类型，应用程序设置
 * 	struct v4l2_rect bounds; // 这是 camera 的镜头能捕捉到的窗口大小的局限
 * 	struct v4l2_rect defrect; // 定义默认窗口大小，包括起点位置及长,宽的大小，大小以像素为单位
 * 	struct v4l2_fract pixelaspect; // 定义了图片的宽高比
 * };
 * struct v4l2_crop
 * {
 *	enum v4l2_buf_type type;// 应用程序设置
 *	struct v4l2_rect c;
 * }
 *
 */
//检查当前取景框的相关信息
OPSTAT func_camera_check_scale(UINT32 fd)
{
	struct v4l2_format fmt;
	memset(&fmt, 0, sizeof(fmt));
	fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (ioctl(fd, VIDIOC_G_FMT, &fmt) == -1){
		HcuErrorPrint("CAMERA: Check VIDIOC_G_FMT error!\n");
		return FAILURE;
	}
	HcuDebugPrint("Current data format information:\n\twidth:%d\n\theight:%d\n", fmt.fmt.pix.width,fmt.fmt.pix.height);

	//打印出像素格式支持情况
	struct v4l2_fmtdesc fmtdesc;
	memset(&fmtdesc, 0, sizeof(fmtdesc));
	fmtdesc.index=0;
	fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	while(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc)!=-1)
	{
		if(fmtdesc.pixelformat & fmt.fmt.pix.pixelformat)
		{
			HcuDebugPrint("CAMERA: \tformat:%s\n", fmtdesc.description);
			break;
		}
		fmtdesc.index++;
	}

	//验证RGB32的支持情况
	memset(&fmt, 0, sizeof(fmt));
	fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_RGB32;
	if(ioctl(fd,VIDIOC_TRY_FMT,&fmt)==-1){
		if(errno==EINVAL){
			HcuDebugPrint("CAMERA: Not support format RGB32!\n");
		}
	}

	//验证Cropcap性能情况，它是捕获图像的窗口和裁剪大小, 图像的缩放 VIDIOC_CROPCAP
	struct v4l2_cropcap cropcap;
	memset(&cropcap, 0, sizeof(cropcap));
	if(ioctl(fd, VIDIOC_CROPCAP, &cropcap)!=-1)
	{
		HcuDebugPrint("CAMERA: CROPCAP Type=%d, DEFRECT L=%d, T=%d, H=%d, W=%d,  BOUNDS L=%d, T=%d, H=%d, W=%d, PIXELASPECT Den=%d, Num=%d!\n",
				cropcap.type, cropcap.defrect.left, cropcap.defrect.top, cropcap.defrect.height, cropcap.defrect.width,
				cropcap.bounds.left, cropcap.bounds.top, cropcap.bounds.height, cropcap.bounds.width,
				cropcap.pixelaspect.denominator, cropcap.pixelaspect.numerator);
	}

	//设置窗口取景参数 VIDIOC_G_CROP (get) 和 VIDIOC_S_CROP (set)
	//可以获取取景参数，也可以设置取景参数，目前暂时不需要设置，只是全屏缺省的获取
	struct v4l2_crop crop;
	memset(&crop, 0, sizeof(crop));
	if(ioctl(fd, VIDIOC_G_CROP, &crop)!=-1)
		{
			HcuDebugPrint("CAMERA: CROP Type = %d, C.L=%d, C.T=%d, C.H=%d. C.W=%d!\n", crop.type, crop.c.left, crop.c.top, crop.c.height, crop.c.width);
		}

	return SUCCESS;
}

/*
 * VIDIOC_G_INPUT 和 VIDIOC_S_INPUT 用来查询和选则当前的 input，一个 video 设备 节点可能对应多个视频源，比如 saf7113 可以最多支持四路 cvbs 输入，
 * 如果上层想在四个cvbs视频输入间切换，那么就要调用 ioctl(fd, VIDIOC_S_INPUT, &input) 来切换。
 * VIDIOC_G_INPUT and VIDIOC_G_OUTPUT 返回当前的 video input和output的index.
 *
 * 相关结构体：
 * struct v4l2_input {
 * __u32 index; // Which input
 * __u8 name[32]; // Label
 * __u32 type; // Type of input
 * __u32 audioset; // Associated audios (bitfield)
 * __u32 tuner; // Associated tuner
 * v4l2_std_id std;
 * __u32 status;
 * __u32 reserved[4];
 * };
 */

//检查和设置输入输出设备，一般设备上可能会有多个输入摄像头
OPSTAT func_camera_check_input_output(UINT32 fd)
{
	struct v4l2_input input;
	struct v4l2_standard standard;

	//初始化变量
	memset (&input, 0, sizeof (input));

	//首先获得当前输入的 index,注意只是 index，要获得具体的信息，就的调用列举操作
	if (ioctl (fd, VIDIOC_G_INPUT, &input.index) == -1) {
		HcuErrorPrint("CAMERA: VIDIOC_G_INPUT error!\n");
		return FAILURE;
	}

	//调用列举操作，获得 input.index 对应的输入的具体信息
	if (-1 == ioctl (fd, VIDIOC_ENUMINPUT, &input)) {
		HcuErrorPrint("CAMERA: VIDIOC_ENUM_INPUT error!\n");
		return FAILURE;
	}
	HcuDebugPrint("CAMERA: Current input %s supports:\n", input.name);

	//初始化Standard数据变量
	memset (&standard, 0, sizeof (standard));
	standard.index = 0;

	//列举所有的所支持的 standard，如果 standard.id 与当前 input 的 input.std 有共同的bit flag，意味着当前的输入支持这个 standard,
	//这样将所有驱动所支持的 standard 列举一个遍，就可以找到该输入所支持的所有 standard 了。

	while (ioctl (fd, VIDIOC_ENUMSTD, &standard) != -1) {
		if (standard.id & input.std) HcuDebugPrint("CAMERA: %s\n", standard.name);
		standard.index++;
	}

	/* EINVAL indicates the end of the enumeration, which cannot be empty unless this device falls under the USB exception. */
	if (errno != EINVAL || standard.index == 0) {
		HcuErrorPrint("CAMERA: VIDIOC_ENUMSTD error!\n");
		return FAILURE;
	}
		return SUCCESS;
}

/*
 * Video standards
 * 当然世界上现在有多个视频标准，如NTSC和PAL，他们又细分为好多种，那么我们的设 备输入/输出究竟支持什么样的标准呢？我们的当前在使用
 * 的输入和输出正在使用的是哪个标准呢？我们怎么设置我们的某个输入输出使用的标准呢？这都是有方法的。查询我们的输入支持什么标准，首先就
 * 得找到当前的这个输入的index，然后查出它的 属性，在其属性里面可以得到该输入所支持的标准，将它所支持的各个标准与所有的标准 的信息进
 * 行比较，就可以获知所支持的各个标准的属性。一个输入所支持的标准应该是一 个集合，而这个集合是用bit与的方式用一个64位数字表示。因此我
 * 们所查到的是一个数字。
 *
 * Example： Information about the current video standard v4l2_std_id std_id; //这个就是个64bit得数
 *
 * typedef u64 v4l2_std_id;
 * struct v4l2_standard
 * {
 * 	u32 index;
 * 	v4l2_std_id id;
 * 	u8 name[24];
 * 	struct v4l2_fract frameperiod; //Frames, not fields
 * 	u32 framelines;
 * 	u32 reserved[4];
 * };
 *
 */

//检查和设置摄像头技术标准
OPSTAT func_camera_check_standard(UINT32 fd)
{
	UINT64 std_id; //这个就是个64bit得数
	struct v4l2_standard standard;

	// VIDIOC_G_STD就是获得当前输入使用的standard，不过这里只是得到了该标准的id即flag，还没有得到其具体的属性信息，具体的属性信息要通过列举操作来得到。

	if (ioctl (fd, VIDIOC_G_STD, &std_id) == -1) { //获得了当前输入使用的standard
	// Note when VIDIOC_ENUMSTD always returns EINVAL this is no video device
	// or it falls under the USB exception, and VIDIOC_G_STD returning EINVAL
	// is no error.
		HcuErrorPrint("CAMERA: VIDIOC_G_STD error!\n");
		return FAILURE;
	}

	//初始化标准变量
	memset (&standard, 0, sizeof (standard));
	standard.index = 0; //从第一个开始列举

	// VIDIOC_ENUMSTD用来列举所支持的所有的video标准的信息，不过要先给standard结构的index域制定一个数值，所列举的标 准的信息属性包含在standard里面，
	// 如果我们所列举的标准和std_id有共同的bit，那么就意味着这个标准就是当前输入所使用的标准，这样我们就得到了当前输入使用的标准的属性信息

	while (ioctl (fd, VIDIOC_ENUMSTD, &standard) != -1) {
		if (standard.id & std_id) {
			HcuDebugPrint("CAMERA: Current video standard: %s\n", standard.name);
			return SUCCESS;
		}
		standard.index++;
	}

	/* EINVAL indicates the end of the enumeration, which cannot be empty unless this device falls under the USB exception. */
	if (errno == EINVAL || standard.index == 0) {
		HcuErrorPrint("CAMERA: VIDIOC_ENUMSTD error!\n");
		return FAILURE;
	}

	return SUCCESS;
}


/*
 *  申请和管理缓冲区
 *  向设备申请缓冲区 VIDIOC_REQBUFS
 *
 *  struct v4l2_requestbuffers
 *  {
 *  	u32 count; // 缓冲区内缓冲帧的数目
 *  	enum v4l2_buf_type type; // 缓冲帧数据格式
 *  	enum v4l2_memory memory; // 区别是内存映射还是用户指针方式
 *  	u32 reserved[2];
 *  };
 *
 *  enum v4l2_memoy
 *  {
 *  	V4L2_MEMORY_MMAP,
 *  	V4L2_MEMORY_USERPTR
 *  };
 *
 *
 *	获取缓冲帧的地址，长度：VIDIOC_QUERYBUF
 *
 *  struct v4l2_buffer
 *  {
 *  	u32 index; //buffer 序号
 *  	enum v4l2_buf_type type; //buffer 类型
 *  	u32 byteused; //buffer 中已使用的字节数
 *  	u32 flags; // 区分是MMAP 还是USERPTR
 *  	enum v4l2_field field;
 *  	struct timeval timestamp; // 获取第一个字节时的系统时间
 *  	struct v4l2_timecode timecode;
 *  	u32 sequence; // 队列中的序号
 *  	enum v4l2_memory memory; //IO 方式，被应用程序设置
 *  	union m
 *  	{
 *  		u32 offset; // 缓冲帧地址，只对MMAP 有效
 *  		unsigned long userptr;
 *  	};
 *  	u32 length; // 缓冲帧长度
 *  	u32 input;
 *  	u32 reserved;
 *  };
 *
 *	内存映射MMAP 及定义一个结构体来映射每个缓冲帧。 相关结构体：
 *	struct buffer
 *	{
 *		void* start;
 *		unsigned int length;
 *	}*buffers;
 *
 *	int munmap(void *addr, size_t length);// 断开映射
 */

//初始化MMAP，内存缓存交换技术，initial video capture way(mmap)
//设置MMAP  set video capture ways(mmap)
OPSTAT func_camera_init_mmap(UINT32 fd)
{
	//to request frame cache, contain requested counts
	struct v4l2_requestbuffers reqbufs;
	//request V4L2 driver allocation video cache
	//this cache is locate in kernel and need mmap mapping
	memset(&reqbufs, 0, sizeof(reqbufs));
	reqbufs.count = 4;  //4帧还不算多
	reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	reqbufs.memory = V4L2_MEMORY_MMAP;

	if(ioctl(fd,VIDIOC_REQBUFS,&reqbufs) == -1){
		HcuErrorPrint("CAMERA: Fail to ioctl 'VIDIOC_REQBUFS'!\n");
		return FAILURE;
	}

	zHcuCameraNbuffer = reqbufs.count;
	HcuDebugPrint("CAMERA: n_buffer = %d\n", zHcuCameraNbuffer);
	//申请内存，啥时候释放，注意！！！
	//以后也采用静态变量进行全局控制，待处理！！！
	zHcuCameraUsrBuf = calloc(reqbufs.count, sizeof(zHcuCameraUsrBuf));
	if(zHcuCameraUsrBuf == NULL){
		HcuErrorPrint("CAMERA: Out of memory!\n");
		return FAILURE;
	}

	//addr 映射起始地址，一般为NULL ，让内核自动选择
	//length 被映射内存块的长度
	//prot 标志映射后能否被读写，其值为PROT_EXEC,PROT_READ,PROT_WRITE, PROT_NONE
	//flags 确定此内存映射能否被其他进程共享，MAP_SHARED,MAP_PRIVATE
	//fd,offset, 确定被映射的内存地址 返回成功映射后的地址，不成功返回MAP_FAILED ((void*)-1)
	//map kernel cache to user process
	for(zHcuCameraNbuffer = 0; zHcuCameraNbuffer < reqbufs.count; ++zHcuCameraNbuffer){
		//stand for a frame
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = zHcuCameraNbuffer;

		// 查询序号为n_buffers 的缓冲区，得到其起始物理地址和大小
		//check the information of the kernel cache requested
		if( ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
		{
			HcuErrorPrint("CAMERA: Fail to ioctl,  VIDIOC_QUERYBUF!\n");
			return FAILURE;
		}

		// 映射内存
		zHcuCameraUsrBuf[zHcuCameraNbuffer].length = buf.length;
		zHcuCameraUsrBuf[zHcuCameraNbuffer].start =
			(char *)mmap(
					NULL,
					buf.length,
					PROT_READ | PROT_WRITE,
					MAP_PRIVATE,  //MAP_SHARED
					fd,
					buf.m.offset
				);
		if(zHcuCameraUsrBuf[zHcuCameraNbuffer].start == MAP_FAILED)
		{
			HcuErrorPrint("CAMERA: Fail to mmap!\n");
			return FAILURE;
		}
	}
	return SUCCESS;
}

/*
 *  缓冲区处理好之后，就可以开始获取数据了
 *  启动 或 停止数据流 VIDIOC_STREAMON， VIDIOC_STREAMOFF
 *	在开始之前，还应当把缓冲帧放入缓冲队列：
 *		VIDIOC_QBUF// 把帧放入队列
 *		VIDIOC_DQBUF// 从队列中取出帧
 */

//启动捕获
OPSTAT func_camera_start_capture(UINT32 fd)
{
	unsigned int i;
	enum v4l2_buf_type type;
	//place the kernel cache to a queue
	for(i = 0; i < zHcuCameraNbuffer; i++){
		struct v4l2_buffer buf;
		memset(&buf, 0, sizeof(buf));
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		//把四个缓冲帧放入队列，并启动数据流
		if(-1 == ioctl(fd, VIDIOC_QBUF, &buf)){  //将内存BUF置换进去
			HcuErrorPrint("CAMERA: Fail to ioctl 'VIDIOC_QBUF'!\n");
			return FAILURE;
		}
	}

	//start capture data  开始视频采集
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == ioctl(fd, VIDIOC_STREAMON, &type)){
		close(fd);
		HcuErrorPrint("CAMERA: VIDIOC_STREAMON!\n");
		return FAILURE;
	}
	return SUCCESS;
}

//图像处理
OPSTAT func_camera_process_image(void *addr, UINT32 length)
{
	FILE *fp;
	static int num = 0;
	char image_name[20];

	sprintf(image_name, JPG, num++);
	if((fp = fopen(image_name, "w")) == NULL){
		HcuErrorPrint("CAMERA: Fail to fopen!\n");
		return FAILURE;
	}
	fwrite(addr, length, 1, fp);
	hcu_usleep(500);
	fclose(fp);
	return SUCCESS;
}

//读取缓冲帧
UINT32 func_camera_read_frame(UINT32 fd)
{
	struct v4l2_buffer buf;
	memset(&buf, 0, sizeof(buf));
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	//获取一帧并处理
	//put cache from queue
	if(-1 == ioctl(fd, VIDIOC_DQBUF,&buf)){   //将内存BUF置换出来
		HcuErrorPrint("CAMERA: Fail to ioctl 'VIDIOC_DQBUF'!\n");
		return FAILURE;
	}
	assert(buf.index < zHcuCameraNbuffer);

	//read process space's data to a file
	func_camera_process_image(zHcuCameraUsrBuf[buf.index].start, zHcuCameraUsrBuf[buf.index].length);
	if(-1 == ioctl(fd, VIDIOC_QBUF,&buf)){   //将内存BUF置换进去
		HcuErrorPrint("CAMERA: Fail to ioctl 'VIDIOC_QBUF'!\n");
		return FAILURE;
	}
	return 1;
}

//处理循环
OPSTAT func_camera_mainloop(UINT32 fd)
{
	int count = 10;

	while(count-- > 0)
	{
		for(;;)
		{
			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd,&fds);

			/*Timeout*/
			tv.tv_sec = 2;
			tv.tv_usec = 0;
			r = select(fd + 1,&fds,NULL,NULL,&tv);

			if(-1 == r)
			{
				if(EINTR == errno)
					continue;
				HcuErrorPrint("CAMERA: Fail to select!\n");
				return FAILURE;
			}

			if(0 == r)
			{
				HcuErrorPrint("CAMERA: select Timeout!\n");
				return FAILURE;
			}

			if(func_camera_read_frame(fd))
				break;
		}
	}
	return SUCCESS;
}

//停止捕获
OPSTAT func_camera_stop_capture(UINT32 fd)
{
	enum v4l2_buf_type type;
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if(-1 == ioctl(fd,VIDIOC_STREAMOFF,&type))
	{
		HcuErrorPrint("CAMERA: Fail to ioctl 'VIDIOC_STREAMOFF'!\n");
		return FAILURE;
	}
	return SUCCESS;
}

//关闭设备
OPSTAT func_camera_close_device(UINT32 fd)
{
	unsigned int i;
	for(i = 0;i < zHcuCameraNbuffer; i++)
	{
		if(-1 == munmap(zHcuCameraUsrBuf[i].start, zHcuCameraUsrBuf[i].length)){
			HcuErrorPrint("CAMERA: Close camera error!\n");
			return FAILURE;
		}
	}
	free(zHcuCameraUsrBuf);

	if(-1 == close(fd))
	{
		HcuErrorPrint("CAMERA: Fail to close fd!\n");
		return FAILURE;
	}
	return SUCCESS;
}



/*
 *
内容摘要：

      Video for Linux two(Video4Linux2)简称V4L2，是V4L的改进版。V4L2是linux操作系统下用于采集图片、视频和音频数据的API接口，配合适当的视频采集设备和相应的驱动程序，可以实现图片、视频、音频等的采集。在远程会议、可视电话、视频监控系统和嵌入式多媒体终端中都有广泛的应用。在Linux中，视频设备是设备文件，可以像访问普通文件一样对其进行读写，摄像头在/dev/video2下。
     最近想做智能机器人，想加上视频采集这个模块，于是对linux下的视频方面的编程产生了兴趣，首先从入门开始吧！
一、Video for Linux Tow
      在Linux下，所有外设都被看成一种特殊的文件，成为“设备文件”，可以象访问普通文件一样对其进行读写。一般来说，采用V4L2驱动的摄像头设备文件是/dev/v4l/video0。为了通用，可以建立一个到/dev/video0的链接。V4L2支持两种方式来采集图像：内存映射方式(mmap)和直接读取方式(read)。V4L2在include/linux/videodev.h文件中定义了一些重要的数据结构，在采集图像的过程中，就是通过对这些数据的操作来获得最终的图像数据。Linux系统V4L2的能力可在Linux内核编译阶段配置，默认情况下都有此开发接口。V4L2从Linux 2.5.x版本的内核中开始出现。
　　V4L2规范中不仅定义了通用API元素(Common API Elements)，图像的格式(Image Formats)，输入/输出方法(Input/Output)，还定义了Linux内核驱动处理视频信息的一系列接口(Interfaces)，这些接口主要有：
　　视频采集接口——Video Capture Interface;
　　视频输出接口—— Video Output Interface;
　　视频覆盖/预览接口——Video Overlay Interface;
　　视频输出覆盖接口——Video Output Overlay Interface;
　　编解码接口——Codec Interface。

二、操作流程
     关于V4L2的介绍网上很多，这里简单说下我们经常常用相关结构体：
struct v4l2_requestbuffers reqbufs;//向驱动申请帧缓冲的请求，里面包含申请的个数struct v4l2_capability cap;//这个设备的功能，比如是否是视频输入设备struct v4l2_standard std;//视频的制式，比如PAL，NTSCstruct v4l2_format fmt;//帧的格式，比如宽度，高度等
struct v4l2_buffer buf;//代表驱动中的一帧v4l2_std_id stdid;//视频制式，例如：V4L2_STD_PAL_Bstruct v4l2_queryctrl query;//查询的控制struct v4l2_control control;//具体控制的值
    下面就介绍相关的操作流程：
1.打开设备文件。 int fd=open(”/dev/video2″,O_RDWR);2.取得设备的capability，看看设备具有什么功能，比如是否具有视频输入,或者音频输入输出等。VIDIOC_QUERYCAP,struct v4l2_capability3.设置视频的制式和帧格式，制式包括PAL，NTSC，帧的格式个包括宽度和高度等。VIDIOC_S_STD,VIDIOC_S_FMT,struct v4l2_std_id,struct v4l2_format4.向驱动申请帧缓冲，一般不超过5个。struct v4l2_requestbuffers5.将申请到的帧缓冲映射到用户空间，这样就可以直接操作采集到的帧了，而不必去复制。mmap6.将申请到的帧缓冲全部入队列，以便存放采集到的数据.VIDIOC_QBUF,struct v4l2_buffer7.开始视频的采集。VIDIOC_STREAMON8.出队列以取得已采集数据的帧缓冲，取得原始采集数据。VIDIOC_DQBUF9.将缓冲重新入队列尾,这样可以循环采集。VIDIOC_QBUF10.停止视频的采集。VIDIOC_STREAMOFF11.关闭视频设备。close(fd);
以下详细介绍操作流程（相信对新手有用）：

1. 定义
V4L2(Video For Linux Two) 是内核提供给应用程序访问音、视频驱动的统一接口。
2. 工作流程：
打开设备－> 检查和设置设备属性－> 设置帧格式－> 设置一种输入输出方法（缓冲区管理）－> 循环获取数据－> 关闭设备。
3. 设备的打开和关闭：
#include <fcntl.h>
int open(const char *device_name, int flags);
#include <unistd.h>
int close(int fd);
例：
int fd=open(“/dev/video2”,O_RDWR);// 打开设备
close(fd);// 关闭设备
注意：V4L2 的相关定义包含在头文件<linux/videodev2.h> 中.
4. 查询设备属性： VIDIOC_QUERYCAP
相关函数：
int ioctl(int fd, int request, struct v4l2_capability *argp);
相关结构体：
struct v4l2_capability
{
__u8 driver[16]; // 驱动名字
__u8 card[32]; // 设备名字
__u8 bus_info[32]; // 设备在系统中的位置
__u32 version; // 驱动版本号
__u32 capabilities; // 设备支持的操作
__u32 reserved[4]; // 保留字段
};
capabilities 常用值:
V4L2_CAP_VIDEO_CAPTURE // 是否支持图像获取
例：显示设备信息
struct v4l2_capability cap;
ioctl(fd,VIDIOC_QUERYCAP,&cap);
printf(“Driver Name:%s/nCard Name:%s/nBus info:%s/nDriver Version:%u.%u.%u/n”,cap.driver,cap.card,cap.bus_info,(cap.version>>16)&0XFF, (cap.version>>8)&0XFF,cap.version&OXFF);
5. 帧格式：
VIDIOC_ENUM_FMT // 显示所有支持的格式
int ioctl(int fd, int request, struct v4l2_fmtdesc *argp);
struct v4l2_fmtdesc
{
__u32 index; // 要查询的格式序号，应用程序设置
enum v4l2_buf_type type; // 帧类型，应用程序设置
__u32 flags; // 是否为压缩格式
__u8 description[32]; // 格式名称
__u32 pixelformat; // 格式
__u32 reserved[4]; // 保留
};
例：显示所有支持的格式
struct v4l2_fmtdesc fmtdesc;
fmtdesc.index=0;
fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
printf("Support format:/n");
while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc)!=-1)
{
printf("/t%d.%s/n",fmtdesc.index+1,fmtdesc.description);
fmtdesc.index++;
}
// 查看或设置当前格式
VIDIOC_G_FMT, VIDIOC_S_FMT
// 检查是否支持某种格式
VIDIOC_TRY_FMT
int ioctl(int fd, int request, struct v4l2_format *argp);
struct v4l2_format
{
enum v4l2_buf_type type;// 帧类型，应用程序设置
union fmt
{
struct v4l2_pix_format pix;// 视频设备使用
struct v4l2_window win;
struct v4l2_vbi_format vbi;
struct v4l2_sliced_vbi_format sliced;
__u8 raw_data[200];
};
};
struct v4l2_pix_format
{
__u32 width; // 帧宽，单位像素
__u32 height; // 帧高，单位像素
__u32 pixelformat; // 帧格式
enum v4l2_field field;
__u32 bytesperline;
__u32 sizeimage;
enum v4l2_colorspace colorspace;
__u32 priv;
};
例：显示当前帧的相关信息
struct v4l2_format fmt;
fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
ioctl(fd,VIDIOC_G_FMT,&fmt);
printf(“Current data format information:/n/twidth:%d/n/theight:%d/n”,fmt.fmt.width,fmt.fmt.height);
struct v4l2_fmtdesc fmtdesc;
fmtdesc.index=0;
fmtdesc.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
while(ioctl(fd,VIDIOC_ENUM_FMT,&fmtdesc)!=-1)
{
if(fmtdesc.pixelformat & fmt.fmt.pixelformat)
{
printf(“/tformat:%s/n”,fmtdesc.description);
break;
}
fmtdesc.index++;
}
例：检查是否支持某种帧格式
struct v4l2_format fmt;
fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_RGB32;
if(ioctl(fd,VIDIOC_TRY_FMT,&fmt)==-1)
if(errno==EINVAL)
printf(“not support format RGB32!/n”);
6. 图像的缩放
VIDIOC_CROPCAP
int ioctl(int fd, int request, struct v4l2_cropcap *argp);
struct v4l2_cropcap
{
enum v4l2_buf_type type;// 应用程序设置
struct v4l2_rect bounds;// 最大边界
struct v4l2_rect defrect;// 默认值
struct v4l2_fract pixelaspect;
};
// 设置缩放
VIDIOC_G_CROP,VIDIOC_S_CROP
int ioctl(int fd, int request, struct v4l2_crop *argp);
int ioctl(int fd, int request, const struct v4l2_crop *argp);
struct v4l2_crop
{
enum v4l2_buf_type type;// 应用程序设置
struct v4l2_rect c;
}
7. 申请和管理缓冲区，应用程序和设备有三种交换数据的方法，直接 read/write ，内存映射(memory mapping) ，用户指针。这里只讨论 memory mapping.
// 向设备申请缓冲区
VIDIOC_REQBUFS
int ioctl(int fd, int request, struct v4l2_requestbuffers *argp);
struct v4l2_requestbuffers
{
__u32 count; // 缓冲区内缓冲帧的数目
enum v4l2_buf_type type; // 缓冲帧数据格式
enum v4l2_memory memory; // 区别是内存映射还是用户指针方式
__u32 reserved[2];
};
enum v4l2_memoy {V4L2_MEMORY_MMAP,V4L2_MEMORY_USERPTR};
//count,type,memory 都要应用程序设置
例：申请一个拥有四个缓冲帧的缓冲区
struct v4l2_requestbuffers req;
req.count=4;
req.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
req.memory=V4L2_MEMORY_MMAP;
ioctl(fd,VIDIOC_REQBUFS,&req);
获取缓冲帧的地址，长度：
VIDIOC_QUERYBUF
int ioctl(int fd, int request, struct v4l2_buffer *argp);
struct v4l2_buffer
{
__u32 index; //buffer 序号
enum v4l2_buf_type type; //buffer 类型
__u32 byteused; //buffer 中已使用的字节数
__u32 flags; // 区分是MMAP 还是USERPTR
enum v4l2_field field;
struct timeval timestamp;// 获取第一个字节时的系统时间
struct v4l2_timecode timecode;
__u32 sequence; // 队列中的序号
enum v4l2_memory memory;//IO 方式，被应用程序设置
union m
{
__u32 offset;// 缓冲帧地址，只对MMAP 有效
unsigned long userptr;
};
__u32 length;// 缓冲帧长度
__u32 input;
__u32 reserved;
};
MMAP ，定义一个结构体来映射每个缓冲帧。
Struct buffer
{
void* start;
unsigned int length;
}*buffers;
#include <sys/mman.h>
void *mmap(void *addr, size_t length, int prot, int flags, int fd, off_t offset);
//addr 映射起始地址，一般为NULL ，让内核自动选择
//length 被映射内存块的长度
//prot 标志映射后能否被读写，其值为PROT_EXEC,PROT_READ,PROT_WRITE, PROT_NONE
//flags 确定此内存映射能否被其他进程共享，MAP_SHARED,MAP_PRIVATE
//fd,offset, 确定被映射的内存地址
返回成功映射后的地址，不成功返回MAP_FAILED ((void*)-1);
int munmap(void *addr, size_t length);// 断开映射
//addr 为映射后的地址，length 为映射后的内存长度
例：将四个已申请到的缓冲帧映射到应用程序，用buffers 指针记录。
buffers = (buffer*)calloc (req.count, sizeof (*buffers));
if (!buffers) {
fprintf (stderr, "Out of memory/n");
exit (EXIT_FAILURE);
}
// 映射
for (unsigned int n_buffers = 0; n_buffers < req.count; ++n_buffers) {
struct v4l2_buffer buf;
memset(&buf,0,sizeof(buf));
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
buf.memory = V4L2_MEMORY_MMAP;
buf.index = n_buffers;
// 查询序号为n_buffers 的缓冲区，得到其起始物理地址和大小
if (-1 == ioctl (fd, VIDIOC_QUERYBUF, &buf))
exit(-1);
buffers[n_buffers].length = buf.length;
// 映射内存
buffers[n_buffers].start =mmap (NULL,buf.length,PROT_READ | PROT_WRITE ,MAP_SHARED,fd, buf.m.offset);
if (MAP_FAILED == buffers[n_buffers].start)
exit(-1);
}

8. 缓冲区处理好之后，就可以开始获取数据了
// 启动/ 停止数据流
VIDIOC_STREAMON,VIDIOC_STREAMOFF
int ioctl(int fd, int request, const int *argp);
//argp 为流类型指针，如V4L2_BUF_TYPE_VIDEO_CAPTURE.
在开始之前，还应当把缓冲帧放入缓冲队列：
VIDIOC_QBUF// 把帧放入队列
VIDIOC_DQBUF// 从队列中取出帧
int ioctl(int fd, int request, struct v4l2_buffer *argp);
例：把四个缓冲帧放入队列，并启动数据流
unsigned int i;
enum v4l2_buf_type type;
// 将缓冲帧放入队列
for (i = 0; i < 4; ++i)
{
struct v4l2_buffer buf;
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
buf.memory = V4L2_MEMORY_MMAP;
buf.index = i;
ioctl (fd, VIDIOC_QBUF, &buf);
}
type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
ioctl (fd, VIDIOC_STREAMON, &type);
// 这有个问题，这些buf 看起来和前面申请的buf 没什么关系，为什么呢?
例：获取一帧并处理
struct v4l2_buffer buf;
CLEAR (buf);
buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
buf.memory = V4L2_MEMORY_MMAP;
// 从缓冲区取出一个缓冲帧
ioctl (fd, VIDIOC_DQBUF, &buf);
// 图像处理
process_image (buffers[buf.index].start);
// 将取出的缓冲帧放回缓冲区
ioctl (fd, VIDIOC_QBUF, &buf);

关于视频采集方式
操作系统一般把系统使用的内存划分成用户空间和内核空间，分别由应用程序管理和操作系统管理。应用程序可以直接访问内存的地址，而内核空间存放的是供内核访问的代码和数据，用户不能直接访问。v4l2捕获的数据，最初是存放在内核空间的，这意味着用户不能直接访问该段内存，必须通过某些手段来转换地址。
一共有三种视频采集方式：使用read、write方式；内存映射方式和用户指针模式。
read、write方式:在用户空间和内核空间不断拷贝数据，占用了大量用户内存空间，效率不高。
内存映射方式：把设备里的内存映射到应用程序中的内存控件，直接处理设备内存，这是一种有效的方式。上面的mmap函数就是使用这种方式。
用户指针模式：内存片段由应用程序自己分配。这点需要在v4l2_requestbuffers里将memory字段设置成V4L2_MEMORY_USERPTR。
处理采集数据
V4L2有一个数据缓存，存放req.count数量的缓存数据。数据缓存采用FIFO的方式，当应用程序调用缓存数据时，缓存队列将最先采集到的 视频数据缓存送出，并重新采集一张视频数据。这个过程需要用到两个ioctl命令,VIDIOC_DQBUF和VIDIOC_QBUF：
structv4l2_buffer buf;
memset(&buf,0,sizeof(buf));
buf.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
buf.memory=V4L2_MEMORY_MMAP;
buf.index=0;
//读取缓存
if(ioctl(cameraFd, VIDIOC_DQBUF, &buf) == -1)
{
return-1;
}
//…………视频处理算法
//重新放入缓存队列
if(ioctl(cameraFd, VIDIOC_QBUF, &buf) == -1) {
return-1;
}
关闭视频设备
使用close函数关闭一个视频设备
close(cameraFd)
还需要使用munmap方法。
*/
