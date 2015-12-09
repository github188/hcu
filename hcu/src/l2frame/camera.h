/*
 * camera.h
 *
 *  Created on: 2015年11月22日
 *      Author: test
 */

#ifndef L2FRAME_CAMERA_H_
#define L2FRAME_CAMERA_H_

#include "../comvm/commsg.h"
#include "../comvm/vmlayer.h"

//State definition
//#define FSM_STATE_ENTRY  0x00
//#define FSM_STATE_IDLE  0x01
enum FSM_STATE_CAMERA
{
	FSM_STATE_CAMERA_INITED = 0x02,
	FSM_STATE_CAMERA_RECEIVED,  //所有人机命令在此状态下直接送达各个线程任务模块
	FSM_STATE_CAMERA_MAX,
};
//#define FSM_STATE_END   0xFE
//#define FSM_STATE_INVALID 0xFF

#define JPG "/lxm/picture/image%d.jpg"

typedef struct HcuBufferType
{
	void *start;
	UINT32 length;
}HcuBufferType_t;


//Global variables
extern FsmStateItem_t FsmCamera[];

//API
extern OPSTAT fsm_camera_task_entry(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_camera_init(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);
extern OPSTAT fsm_camera_restart(UINT32 dest_id, UINT32 src_id, void * param_ptr, UINT32 param_len);

//Local API
int func_camera_open_device(void);
OPSTAT func_camera_int_init(void);
OPSTAT func_camera_check_scale(UINT32 fd);
OPSTAT func_camera_check_input_output(UINT32 fd);
OPSTAT func_camera_check_standard(UINT32 fd);
OPSTAT func_camera_init_mmap(UINT32 fd);
OPSTAT func_camera_init_device(UINT32 fd);
OPSTAT func_camera_start_capture(UINT32 fd);
OPSTAT func_camera_process_image(void *addr, UINT32 length);
UINT32 func_camera_read_frame(UINT32 fd);
OPSTAT func_camera_mainloop(UINT32 fd);
OPSTAT func_camera_stop_capture(UINT32 fd);
OPSTAT func_camera_close_device(UINT32 fd);


#endif /* L2FRAME_CAMERA_H_ */
