/*
 * ethernet.c
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#include "ethernet.h"

#include "../comvm/trace.h"
#include "../l2frame/l2comdef.h"

/*
** FSM of the ETHERNET
*/
FsmStateItem_t FsmEthernet[] =
{
    //MessageId                 //State                   		 		//Function
	//启始点，固定定义，不要改动, 使用ENTRY/END，意味者MSGID肯定不可能在某个高位区段中；考虑到所有任务共享MsgId，即使分段，也无法实现
	//完全是为了给任务一个初始化的机会，按照状态转移机制，该函数不具备启动的机会，因为任务初始化后自动到FSM_STATE_IDLE
	//如果没有必要进行初始化，可以设置为NULL
	{MSG_ID_ENTRY,       		FSM_STATE_ENTRY,            			fsm_ethernet_task_entry}, //Starting

	//System level initialization, only controlled by HCU-MAIN
    {MSG_ID_COM_INIT,       	FSM_STATE_IDLE,            				fsm_ethernet_init},
    {MSG_ID_COM_RESTART,		FSM_STATE_IDLE,            				fsm_ethernet_restart},

    //Task level initialization
    {MSG_ID_COM_RESTART,        FSM_STATE_ETHERNET_RECEIVED,            fsm_ethernet_restart},

    //结束点，固定定义，不要改动
    {MSG_ID_END,            	FSM_STATE_END,             				NULL},  //Ending
};

//Global Variables
extern CloudControlTable_t zHcuCloudConTable;

//Main Entry
//Input parameter would be useless, but just for similar structure purpose
OPSTAT fsm_ethernet_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	//除了对全局变量进行操作之外，尽量不要做其它操作，因为该函数将被主任务/线程调用，不是本任务/线程调用
	//该API就是给本任务一个提早介入的入口，可以帮着做些测试性操作
	if (FsmSetState(TASK_ID_ETHERNET, FSM_STATE_IDLE) == FAILURE){
		HcuErrorPrint("ETHERNET: Error Set FSM State at fsm_ethernet_task_entry\n");}
	return SUCCESS;
}



OPSTAT fsm_ethernet_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	int ret=0;
	CURL *curl;
	CURLcode res;

	//Send back MSG_ID_COM_INIT_FEEDBACK to SVRCON
	msg_struct_com_init_feedback_t snd0;
	memset(&snd0, 0, sizeof(msg_struct_com_init_feedback_t));
	snd0.length = sizeof(msg_struct_com_init_feedback_t);
	ret = hcu_message_send(MSG_ID_COM_INIT_FEEDBACK, TASK_ID_SVRCON, TASK_ID_ETHERNET, &snd0, snd0.length);
	if (ret == FAILURE){
		HcuErrorPrint("ETHERNET: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_ETHERNET], zHcuTaskNameList[TASK_ID_SVRCON]);
		return FAILURE;
	}

	//收到初始化消息后，进入初始化状态
	if (FsmSetState(TASK_ID_ETHERNET, FSM_STATE_ETHERNET_INITED) == FAILURE){
		HcuErrorPrint("ETHERNET: Error Set FSM State!\n");
		return FAILURE;
	}

	//初始化参数
	if (func_ethernet_intf_init() == FAILURE){
		HcuErrorPrint("ETHERNET: Error initialize interface!\n");
		return FAILURE;
	}

	//设置状态机到目标状态
	if (FsmSetState(TASK_ID_ETHERNET, FSM_STATE_ETHERNET_RECEIVED) == FAILURE){
		HcuErrorPrint("ETHERNET: Error Set FSM State!\n");
		return FAILURE;
	}
	#if (TRACE_DEBUG_ON)
		HcuDebugPrint("ETHERNET: Enter FSM_STATE_ETHERNET_ACTIVED status, Keeping refresh here!\n");
	#endif //(TRACE_DEBUG_ON)

	//初始化MSGSEND参数
	msg_struct_ethernet_cloud_data_rx_t receiveBuffer;
	memset(&receiveBuffer, 0, sizeof(msg_struct_ethernet_cloud_data_rx_t));

	//进入阻塞式接收数据状态，收到数据后发送给CLOUDCONT进行处理
	//这里可能存在一个问题，就是接受到数据的长度超过Message Buffer的长度，一旦超过，直接丢弃
	while(1){
		//zHcuEthernetCurlPtr属于正常状态，进入无限循环

		//为了实现各个接受任务模块的互斥，必须得知当前是哪一个物理底层在工作，不然HTTP-CURL会导致冲突而出错
		//假设链路的初始化都已经完成
		curl = curl_easy_init();

		if (curl == NULL){
			HcuErrorPrint("ETHERNET: Init CURL error!\n");
		}else if ((curl != NULL ) && (zHcuCloudConTable.curCon == CLOUD_CONTROL_PHY_CON_ETHERNET)){
			//for test
			//HcuDebugPrint("ETHERNET: Enter while(1) loop once again for blocking receive data!\n");

			//如果你想CURL报告每一件意外的事情，设置这个选项为一个非零值
			//curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);

			//设置头，以便返回完整的信息
			//curl_easy_setopt(curl, CURLOPT_HEADER, 0);

			//也就是说，默认情况下libcurl完成一个任务以后，出于重用连接的考虑不会马上关闭如果没有新的TCP请求来重用这个连接，那么只能等到CLOSE_WAIT超时
			//这个时间默认在7200秒甚至更高，太多的CLOSE_WAIT连接会导致性能问题。这里设置为0就是为了重用
			//curl_easy_setopt(curl, CURLOPT_FORBID_REUSE, 0);

			//设置目标地址
			curl_easy_setopt(curl, CURLOPT_URL, CLOUD_HTTP_ADDRESS_SAE);
			//curl_easy_setopt(curl, CURLOPT_URL, CLOUD_HTTP_ADDRESS_LOCAL);
			//curl_easy_setopt(curl, CURLOPT_URL, CLOUD_HTTP_ADDRESS_TEST_NULL);

			//设置超时时长，这里是接收阻塞模式，希望永远阻塞，故而也可以不设置，但考虑到这本身就是一个循环，放在这里也无伤大雅
			//curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);
			//curl_easy_setopt(curl, CURLOPT_TIMEOUT, 0);
			//curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 0);
			//curl_easy_setopt(curl, CURLOPT_CONNECT_ONLY, 1);

			//就是当多个线程都使用超时处理的时候，同时主线程中有sleep或是wait等操作。如果不设置这个选项，libcurl将会发信号打断这个wait从而导致程序退出。
			curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

			//设置回调函数
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, hcu_cloud_write_callback);

			//设置接收buffer
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&receiveBuffer);

			//实施调用
			res = curl_easy_perform(curl);

			//Clean curl pointer
			curl_easy_cleanup(curl);

			if(res != CURLE_OK){
				HcuErrorPrint("ETHERNET: curl_easy_perform() failed: %s\n", curl_easy_strerror(res));  //need a VM API for fprintf
				//return FAILURE;  不能返回，而是接着继续调用
			}
			else
			{
				//for test!
				//HcuDebugPrint("ETHERNET: curl_easy_perform result=%d, Len=%d, Buffer=%s\n", res, receiveBuffer.length,  receiveBuffer.buf);
			}

			//将数据发送给CLOUD
			if (receiveBuffer.length > 1){
				//发送数据给CLOUDCONT
				ret = hcu_message_send(MSG_ID_ETHERNET_CLOUD_DATA_RX, TASK_ID_CLOUD, TASK_ID_ETHERNET, receiveBuffer.buf, receiveBuffer.length);
				if (ret == FAILURE){
					HcuErrorPrint("ETHERNET: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_ETHERNET], zHcuTaskNameList[TASK_ID_CLOUD]);
					return FAILURE;
				}
			}//end of send data

		}//end of working condition

		//Wait for a while to continue judge the condition of receiving meet or not.
		hcu_sleep(1);
	}//while(1) end

	return SUCCESS;
}

OPSTAT fsm_ethernet_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len)
{
	return SUCCESS;
}

OPSTAT func_ethernet_intf_init(void)
{
	return SUCCESS;
}

OPSTAT hcu_ethernet_date_send(CloudDataSendBuf_t *buf)
{
	CURLcode curlRes;
	int ret = 0;
	CURL *curl;

	//初始化MSGSEND参数
	msg_struct_ethernet_cloud_data_rx_t receiveBuffer;
	memset(&receiveBuffer, 0, sizeof(msg_struct_ethernet_cloud_data_rx_t));

	//For test
	/*HcuDebugPrint("ETHERNET: %02X %02X %02X %02X %02X %02X %02X %02X \n", buf->curBuf[0], buf->curBuf[1], buf->curBuf[2], buf->curBuf[3], buf->curBuf[4], buf->curBuf[5], buf->curBuf[6], buf->curBuf[7]);

	//测试值，未来需要改为buf的数值，包括对信息做编码，而且应该在CLOUDCINT中完成
	char postBuffer[2*MAX_HCU_MSG_BUF_LENGTH] = "TEST!";
	char tplFormat[] = "<xml><ToUserName><![CDATA[%s]]></ToUserName><FromUserName><![CDATA[%s]]></FromUserName>\
			<CreateTime>%s</CreateTime><MsgType><![CDATA[%s]]></MsgType><Content><![CDATA[%s]]></Content>\
			<FuncFlag>0</FuncFlag></xml>"; //send data for debug

	UINT8 test[] = {0x11, 0x22};
	memcpy(buf, test, sizeof(test));
	char s[MAX_HCU_MSG_BUF_LENGTH];
	int i = 0;
	for (i=0; i<sizeof(test); i++){

	}
	sprintf(s, "%02X", test[0]);
	sprintf(s+2, "%02X", test[1]);
	s[4] = 0;
	sprintf(postBuffer, tplFormat, "LZH", "ZJL", "Time0", "MsgType0", s);
	HcuDebugPrint("ETHERNET: send data = [%s]\n", postBuffer);
	*/

	curl = curl_easy_init();

	if (curl == NULL){
		HcuErrorPrint("ETHERNET: Init CURL error!\n");
	}else{
		//打印测试
		//HcuDebugPrint("ETHERNET: Enter data send status, to send data for back-cloud!\n");

		//如果你想CURL报告每一件意外的事情，设置这个选项为一个非零值
		//curl_easy_setopt(curl, CURLOPT_VERBOSE, 0);

		//设置头，以便返回完整的信息
		//curl_easy_setopt(curl, CURLOPT_HEADER, 0);

		//也就是说，默认情况下libcurl完成一个任务以后，出于重用连接的考虑不会马上关闭如果没有新的TCP请求来重用这个连接，那么只能等到CLOSE_WAIT超时
		//这个时间默认在7200秒甚至更高，太多的CLOSE_WAIT连接会导致性能问题。这里设置为0就是为了重用
		//curl_easy_setopt(curl, CURLOPT_FORBID_REUSE, 0);

		//设置目标地址
		curl_easy_setopt(curl, CURLOPT_URL, CLOUD_HTTP_ADDRESS_SAE);
		//curl_easy_setopt(curl, CURLOPT_URL, CLOUD_HTTP_ADDRESS_LOCAL);

		//设置超时时长，做为发送API，这个设置绝对必要，不然会阻塞在这儿
		//curl_easy_setopt(curl, CURLOPT_TIMEOUT, 1L);

		//就是当多个线程都使用超时处理的时候，同时主线程中有sleep或是wait等操作。如果不设置这个选项，libcurl将会发信号打断这个wait从而导致程序退出。
		curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);

		//设置发送数据的信息
		curl_easy_setopt(curl, CURLOPT_POST, 1);
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, buf->curBuf);

		//设置回调函数
		curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, hcu_cloud_write_callback);

		//设置接收buffer
		curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)&receiveBuffer);

		//实施调用
		curlRes = curl_easy_perform(curl);

		//Clean curl pointer
		curl_easy_cleanup(curl);

		if(curlRes != CURLE_OK){
			HcuErrorPrint("ETHERNET: curl_easy_perform() failed: %s\n", curl_easy_strerror(curlRes));

			return FAILURE;
		}else
		{
			//for test!
			//HcuDebugPrint("ETHERNET: Snd/Rcv pair operation curl_easy_perform data Len=%d, Buffer=%s\n", receiveBuffer.length,  receiveBuffer.buf);
		}

		//将数据发送给CLOUD
		if (receiveBuffer.length > 1){
			//发送数据给CLOUDCONT
			ret = hcu_message_send(MSG_ID_ETHERNET_CLOUD_DATA_RX, TASK_ID_CLOUD, TASK_ID_ETHERNET, receiveBuffer.buf, receiveBuffer.length);
			if (ret == FAILURE){
				HcuErrorPrint("ETHERNET: Send message error, TASK [%s] to TASK[%s]!\n", zHcuTaskNameList[TASK_ID_ETHERNET], zHcuTaskNameList[TASK_ID_CLOUD]);
				return FAILURE;
			}
		}//end of send data

	}//End of working condition

	return SUCCESS;
}

OPSTAT hcu_ethernet_phy_link_setup(void)
{
	return SUCCESS;
}

OPSTAT hcu_ethernet_phy_link_disconnect(void)
{
	return SUCCESS;
}

/*
static int base64_encode(char *str, int str_len, char *encode, int encode_len){
    BIO *bmem,*b64;
    BUF_MEM *bptr;
    b64=BIO_new(BIO_f_base64());
    bmem=BIO_new(BIO_s_mem());
    b64=BIO_push(b64,bmem);
    BIO_write(b64,str,str_len); //encode
    BIO_flush(b64);
    BIO_get_mem_ptr(b64,&bptr);
    if(bptr->length>encode_len){
        DPRINTF("encode_len too small\n");
        return -1;
    }
    encode_len=bptr->length;
    memcpy(encode,bptr->data,bptr->length);
//  write(1,encode,bptr->length);
    BIO_free_all(b64);
    return encode_len;
}

static int base64_decode(char *str,int str_len,char *decode,int decode_buffer_len){
    int len=0;
    BIO *b64,*bmem;
    b64=BIO_new(BIO_f_base64());
    bmem=BIO_new_mem_buf(str,str_len);
    bmem=BIO_push(b64,bmem);
    len=BIO_read(bmem,decode,str_len);
    decode[len]=0;
    BIO_free_all(bmem);
    return 0;
}
*/
