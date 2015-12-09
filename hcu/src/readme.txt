== 初始化并创建整个程序的体系架构
> 从MYC的WIP项目工程拷贝代码到本项目
> 设置Project Property, 以便编译环境可以正常运行
> project -> build configurartions -> set ative -> debug, ARM的调试环境以后再继续，等待E2E及Linux高手搞定
> add project -> properties -> Path & Symbol -> #include -> GNU-C -> add (/usr/include/libxml2)
> add project -> properties -> Path & Symbol -> #library -> 增加rt/pthread/xml2
> 编译试试，看看是否能生成完整的x86执行文件，并进行运行和Debug
> 调整系统字体以便程序不同行的程序对其: window -> preferences -> apperancet -> Colors and Fonts -> editor -> Courier Regular 10
> 将MYC之前的WIP项目程序代码全部借鉴过来，完成

==程序文件框架
> HCU工程
> SRC是包含所有源文件的目录，里面按照ComVm，L2Frame，L3App以及Webser多个子目录进行组织

》系统级主题任务
> hcu.h/c: HCU启动父进程，启动完成后即停止

》公共消息类任务
> commsg.h: 公共消息
> sysconfig.h: 系统级配置文件
> sysinfo.h/c: 定义系统参数和读取的API库
> configxml.h/c: 基于XML机制的系统配置库
> trace.h/c: 用于TRACE的API库
> vmlayer.h/c: VM虚拟机，简化上层程序调用的API库，主要包括TASK创建和消息通信机制
> mmc.h/c: 人机命令借口任务
> timer.h/c: 定时器任务

》L2底层通信接口模块任务
> L2comdef.h: 公共定义的结构体
> apigpio.h/c: GPIO接口的底层访问库
> apigprs.h/c: GPRS接口的底层访问库
> sps232.h/c: 串口的底层访问库
> sps485.h/c: 串口的底层访问库
> Ethernet.h/c: 以太网底层接口接收
> wifi.h/c: WIFI接口的底层访问库
> Usbnet.h/c: USB当后台网口使用时，底层接口接收
> con3g4g.h/c: 3G接口的底层访问库
> cloudcont.h/c: 连接后台SAE CLOUD，保持心跳及帧协议编解码
> modbus.h/c: 连接485接口并编解码MODBUS帧协议
> hwinv.h/c: 硬件状态和设备的监控及初始化处理
> hcuref.h/c: WIP项目参考代码
> Audio.h/c: 音频
> Camera.h/c: 摄像头
> Ble.h/c: 蓝牙
> Lcd.h/c: 显示屏+工控界面（界面直接使用WEB？）

》L3层3高层应用模块任务
> svrcon.h/c: 业务逻辑总控及处理
> sensoremc.h/c: EMC传感器业务逻辑处理
> sensorhumid.h/c: 湿度传感器业务逻辑处理
> sensorpm25.h/c: PM2.5传感器业务逻辑处理
> sensortemp.h/c: 温度传感器业务逻辑处理
> sensorwindspd.h/c: 风速传感器业务逻辑处理
> sensorwinddir.h/c: 风向传感器业务逻辑处理
> hsmmp.h/c: 高速多媒体处理任务

 》Web server: Http Web Server的简单服务应用

> readme.txt: 本文件，说明参考文档及开发进度说明


== 版本控制
> 软件内部控制版本，CURRENT_SW_RELEASE + CURRENT_SW_DELIVERY, 从20151102开始
> Git版本控制法，待定
> Release版本，待定
> Release ARM交叉编译版本，待定

== Update log: 2015 Nov.3, SW Version: XQ.HCU.SW.R01.002
> ZJL, 2015 Nov.3, CURRENT_SW_DELIVERY 2
= VM底层任务平台软件
> 借鉴下位机KEIL嵌入式程序样例和风格
> 创建任务和通信机制
> TRACE功能
> 简单的DEBUG控制功能
> 在Linux上创建线程，遇到不成功的问题，不明原因
> 在Linux上创建间线程和队列后，主/父进程负责最终清理，MAIN函数最后包含这部分，不然debug在第二次无法正常测试

== Update log: 2015 Nov.4, SW Version: XQ.HCU.SW.R01.003
> ZJL, 2015 Nov.4, CURRENT_SW_DELIVERY 3
= 任务的控制参数归类到VM层
= 消息传送与接受机制
= .h文件包含关系的清理
= 消息TRACE机制

== Update log: 2015 Nov.9, SW Version: XQ.HCU.SW.R01.004
> ZJL, 2015 Nov.9, CURRENT_SW_DELIVERY 4
=  优化程序和BUG FIX
= 任务创建和程序运行必须在ROOT下运行，或者给当前账户必要的权限，必然创建任务不会成功

== Update log: 2015 Nov.10, SW Version: XQ.HCU.SW.R01.005
> 消息结构体定义优化
> 消息发送接受机制测试
> MsgSend/MsgRcv函数只要出现一点点问题，就可能出现问题：输入的参数消息长度比Linux系统规定的长4个字节（整个消息体-TYPE），导致各种不可预知的问题
> 差错恢复机制，包括Task/Queue出错后，如何确保各个任务的自动恢复，目前暂时为做合理的处理，大部分都是直接退出程序体，未来需要仔细考虑恢复问题
> 测试过程中，由于各个任务都是无限工作的，所以运行测试完成后需要手动清理进程
> 子任务阻塞式工作模式工作的挺好！
> 子任务出错后，等待主任务的清理，而不是提前退出
> 硬件版本信息移到SysConfig.h文件中

== Update log: 2015 Nov.12, SW Version: XQ.HCU.SW.R01.006
> 初步FSM状态机，供大家品鉴

== Update log: 2015 Nov.13, SW Version: XQ.HCU.SW.R01.007
>  完整的FSM状态机，应用者只需要实现消息操作函数即可
>  全静态表格控制任务和状态机
>  任务创建自动化
>  问题：当创建第二个任务时，出现阻塞，可能跟任务的优先级等东东相关，TBD

== Update log: 2015 Nov.15, SW Version: XQ.HCU.SW.R01.008
>  修正FSM状态机的部分错误
> 初始化所有的任务和状态机初步内容
> 由于Linux只有一个定时器内核，为了支持多个应用任务的定时器，需要考虑单独构建一套定时器任务服务
> SVRCON主控发送消息初始化其它任务的机制完成

== Update log: 2016 Nov.15, SW Version: XQ.HCU.SW.R01.009
>  Timer机制搞定
>  第一个例子初步完成
> 基本做到以下两点：其它任务只需要按图索冀
> MSG_COM_INIT->任务做初始化，进入正常工作状态INITED
> MSG_COM_RESTART->任务做重新启动，将所有变量和状态以及QUEUE清空，进入正常工作状态INITED

== Update log: 2015 Nov.17, SW Version: XQ.HCU.SW.R01.010
> 高级信号量SIGALRM会打断MsgRcv以及SLEEP过程，使用技巧规避。据说他还会打断SOCKET的Accept函数，待确定
> 目前的妥协是，暂时使用Create_Timer函数在TIMER任务中生成定时器，并使用信号量方式，精度只有1S级别，1MS和10MS的精度暂时不考虑
> 定时器明显会打断SLEEP函数，目前使用循环规避。但1S精度的定时器信号量打断uSleep(1000)时，该函数直接返回-1,而不是剩下的微秒数，所以要注意
> 但是，如果uSleep(100)设置在100MS以下时，则没啥问题，基本上被打断的可能性降低到很低，还是可以使用的
》未来还需要仔细重新考虑实时定时器的设计问题
> 完善第一个例子PM25的状态机，给其他传感器搭建框架
> 完善MODBUS/CLOUD的程序框架
> 底层3G/SPS写STUB空函数，以便调佣和调试
> 准备MMC及HWINV任务，特别是硬件清单任务，它将是硬件底层状态维护以及PnP功能的基础

== Update log: 2015 Nov.18, SW Version: XQ.HCU.SW.R01.011
> 增加MMC以及HWINV两个任务模块，建立起程序框架
> 继续优化PM25业务逻辑，MODBUS/CLOUD两边的程序模拟数据更为接近实际情况
> 周期模式完成，瞬时模式留待未来完成
> 控制命令字还是个不确定性的问题

== Update log: 2015 Nov.19, SW Version: XQ.HCU.SW.R01.012
> 调试圆桶算法完成，基本上可用，重新将最大记录数调整为20000条
> 6个传感器全部启动了，基本上开始跑起来！

== Update log: 2015 Nov.20, SW Version: XQ.HCU.SW.R01.013
> Modbus Pack/uppack func

== Update log: 2015 Nov.21 SW Version: XQ.HCU.SW.R01.014
>  融合找到现成的串口程序以及Http Web Server程序

== Update log: 2015 Nov.22 SW Version: XQ.HCU.SW.R01.015
>  Add Linux Target Arm Project Config
>  Define TRUE/FALSE MACRO in hcu.h

== Update log: 2015 Nov.23 SW Version: XQ.HCU.SW.R01.016 //ZJL
> 增加各个底层接口的接收任务机制，采用阻塞式
> 创建ETHERNET, USBNET, WIFI, 3G4G, HSMMP, BLE, SPS232, SPS485任务模块框架
> 一种最为简单的硬盘存储传感器记录的机制搞定，以后可以逐步丰富：碎片文件/内容按照时间轴/记录数量/定时清理等等，待完成

== Update log: 2015 Nov.24 SW Version: XQ.HCU.SW.R01.017 //ZJL
> 碎片文件目录的创建和更新机制
> 碎片文件名的创建以及定时更新机制
> 碎片文件的写入，读取和清理删除（比如超过半年的周期）未来实现
> 高精度时钟10MS，采用临时启动线程的方式，基本搞定，后面通过摄像头HSMMP处理任务，看看效果，至少它没有打断SLEEP，说明其工作没大问题
> TRACE机制丰富化：MSG_TIME_OUT需要被过滤，不然TRACE消息太多，MSG_TRACE_ON = 1 (ALL) or 2 (ALL but TIME_OUT)，未来需要实现更多丰富的TRACE机制

== Update log: 2015 Nov.25 SW Version: XQ.HCU.SW.R01.018 //ZJL
> 基本的音视频程序框架， 完善整个处理流程的子函数及功能分解
> 针对TI MX6设备的设备名定义到L2COMDEF.H中，以便HWINV定期扫描以及其它应用直接使用，避免不同应用之间重复定义以及冲突
> 接收到视频图像或者视频流以后的处理，等待视频摄像头物理硬件处理过程（CAMERA模块）完善后再行处理
> 视频流的后处理：采用标准MPEG4或者H264标准格式，碎片存入硬盘，碎片传后台，或者视频流传后台？

== Update log: 2015 Nov.29 SW Version: XQ.HCU.SW.R01.020 //MYC
> Correct DebugTracePrint issue of file name length not same
> Correct Debug (x86, not for ARM) Project Configuration, remove libxml2 lib directory to remove link warning
> API to Connect USB GPRS via HW-E261 Card
  1. Files modifiled: sysinfo.h and sysinfo.c (to be cleaned?) 
  2. Data struacture defined in sysinfo.h (to be cleaned)
  3. API added forexrternal Call:
		UINT32 HcuGetSysInfo(SysInfo_t *si);
  		UINT32 UsbUmtsPppInit();
		UINT32 UsbUmtsPppdStart();
		UINT32 UsbUmtsPppdSwShutdown(); 
  		UsbUmtsCardScan();  => To be complete, as currently no USB is scanned, HW-E261 is chosen as default!!!
		UINT32 IsGprsPppdOk();
		UINT32 IsNetItfExist(char *itfname);
		void KillPppdProcess();
  4. At system startup, bring PPP for HW-E261

		//INIT the whole system
		void hcu_vm_system_init(void)
		{
			...		
			/* MYC: Start to initialized USB UMTS CARD */
			UsbUmtsPppInit();
			UsbUmtsPppdStart();
			...
		}
   5. Note1: Ctrl+C (TERM SIG) will kill hcu process, resulting in pppd killed, and all ppp interface will be down
   6. Note2: Tested in TI A8
   7. Note3: TARGET_LINUX_ARM needs to be added	
   
== Update log: 2015 Nov.29 SW Version: XQ.HCU.SW.R01.021 //LZH
> CURL OK

== Update log: 2015 Nov.29 SW Version: XQ.HCU.SW.R01.022 //MYC
> Optimize USB UMTS CARD
> CURL lib added in x86 Debug Build Config (Not working yet for ARM A8, Needs to compile ARM lib, TO DO)
  In x86 ubuntu:
		apt-get install curl libcurl3 libcurl3-dev php5-curl
		apt-get install libcurl4-gnutls-dev
  TO DO: TO ADD CURL LIB in ARM, or not possible to compile in Target!!!!   

== Update log: 2015 Dec.1 SW Version: XQ.HCU.SW.R01.023 //ZJL
> 将USB启动程序放到HWINV模块中去
> 完善GPS和LCD基础模块任务
> 基本确定连接建立的方式，完善CLOUTCONT模块相应功能
> 移动webser目录到src目录下，以便统一管理源代码

== Update log: 2015 Dec.2 SW Version: XQ.HCU.SW.R01.024 //ZJL
> 完善后台通信

== Update log: 2015 Dec.2 SW Version: XQ.HCU.SW.R01.025 //MYC
> 尝试UMTS通信和交叉编译

== Update log: 2015 Dec.7 SW Version: XQ.HCU.SW.R01.026 //ZJL
> 调同与后台的ETHERNET通信方式
> 后台通信底层与后台云SAE联调完成
> 后台数据层格式化
> EMC定时器改为6Second，各个传感器任务对应的后启动时间从随机最大60second减少为6second

== Update log: 2015 Dec.8 SW Version: XQ.HCU.SW.R01.027 //MYC
> //MYC 2015/12/08, For 335xD, RS485 is ttyO2 !!!
> "/dev/ttySAC0" changed to "/dev/ttyO0"
> #define PM25_TIMER_DURATION_PERIOD_READ 60 to #define PM25_TIMER_DURATION_PERIOD_READ 6, only for test
> fd = open(HCU_DEFAULT_DEVICE_USB_CAMERA, O_RDONLY, 0); to fd = open(HCU_DEFAULT_DEVICE_USB_CAMERA0, O_RDONLY, 0);
> Add Linux_Arm_A8 target lib for libcurl Build Configuration, target can be built
> Check log file "hcu.R01.027(PM25_OK).log for target run result (Video Camara also inserted)

== Update log: 2015 Dec.8 SW Version: XQ.HCU.SW.R01.028 //ZJL
> change PM25_TIMER_DURATION_PERIOD_READ back to 6second
> 增加中环保ZHB标准的解码功能，后台接口协议目前支持两种：XML和ZHB，需要通过SYSCONFIG.H中的预定义来选择
> 修正各种传感器离线数据都存为PM25的bug
> 定义完整各个传感器的操作命令字，并完善Modbus.c/Sensorxxx.c中对于这些操作命令足的处理框架，以备接收从后台来的操作命令并进行相应的处理








== Coming Soon...
> 485端口调通，读取一个485设备的数据
> 将485 MODBUS数据传递到后台，并正常解码
> 使用XML页面的方式，来配置系统的参数选项
> camera.c中的zHcuCameraUsrBuf为动态申请内存，待处理静态化
> 如何将UT-TEST环境包装进咱们项
> MMC机制，让人工可以通过命令行进行控制，以便启动和配置模块的运行与否
>MSG_COM_STOP->任务停留在DISABLED状态
> RESTART机制
> VM内存机制
> 从网络时钟同步机制



== 遗留问题
1. 消除WARNING，其中有一种情况是函数未在.h中进行申明
2. 自创账户没有足够的权限启动TASK，必须在ROOT权限下工作，但ROOT权限下并没有安装中文输入，导致必须两边切IDE环境，比较麻烦
3. HCU创建的文件，其权限不确定，HWINV还有bug
4. 网络在非连接转连接态，HCU并不能正常恢复到连接态，需要重启才能恢复。HWINV还有bug
5. 

==TIPS & Best Practice===
> chmod -R 777 *   => 一旦在ROOT下操作后，可能造成普通账户编译不过，文件不能访问，该命令将放开所有相关文件的权限，要在ROOT下操作
> 调试切换账户: sudo -i / root password,  then "cd /usr/eclipse“， "./eclipse", we could execute debug/run without any restrict
> 本地API都以func_开头，全局API都以hcu_开头，状态机的消息处理函数都以fsm_开头
> 全局变量都以zHcuxxx开头，typedef的结构均采用_t的形式表征结构体
> 程序中尽量少使用数值，原则上都采用.h文件中定义常量的形式来进行
> 函数传递指针给子函数，比较高效安全，直接传递结构体整体，堆栈使用太多
> 深度结构的结构型数组，取得其地址，尽量不要使用数组内的元素的地址，而应该直接用数组本身，因为元素的顺序会变化和修改，导致程序后期的维护可能出错
> 为了测试效果，PM25定时器设置为5S，随即启动延迟设置为模3，均应该调整为1分钟
> CLOUD链路检测心跳定时确定为15s，还可以更短些。测试时先保持这个标准
> 环境安装，可以参考《设计要点》
> 注意，#项目目录下的include内容是否完整，不然会出现编译问题
> HcuDebugPrint/HcuErroPrint必须按照其他例子来进行，不然会出现打印信息混乱的状态
> 升级版本变为新工程，借用原先的项目时，使用替代，而不是合并，不然不少旧文件会重现
> 函数命名规则： func_task_xxx => 本地函数 fsm_task_xxx => 状态机函数 hcu_task_xxx => 全局函数
> 全局变量命名规则： zHcuTaskxxx => 全局表单，使用extern进行修饰，以便其它模块引用





	
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, record add = %08x\n", &record);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, lastSid = %d\n", zHcuMemStorageBuf.lastSid);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, offlineNbr = %d\n", zHcuMemStorageBuf.offlineNbr);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, rdCnt = %d\n", zHcuMemStorageBuf.rdCnt);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, recordNbr = %d\n", zHcuMemStorageBuf.recordNbr);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, wrtCnt = %d\n", zHcuMemStorageBuf.wrtCnt);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, Sensortype = %d\n", record.sensortype);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, onOffLine = %d\n", record.onOffLine);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, equipid = %d\n", record.equipid);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, sid = %d\n", record.sid);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, timestamp = %d\n", record.timestamp);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, pm2d5 = %d\n", record.pm2d5);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, pm10 = %d\n", record.pm10);
	HcuDebugPrint("CLOUDCONT: Read data from MEM Storage, pm1d0 = %d\n", record.pm1d0);

	HcuDebugPrint("MODBUS: rcv.equId = %02x\n", rcv.equId);
	HcuDebugPrint("MODBUS: rcv.cmdId = %02x\n", rcv.cmdId);
	HcuDebugPrint("MODBUS: rcv.cmdIdBackType = %02x\n", rcv.cmdIdBackType);
	HcuDebugPrint("MODBUS: rcv.optId = %02x\n", rcv.optId);
	HcuDebugPrint("MODBUS: rcv.length = %02x\n", rcv.length);