//
// Created by xhuanc on 2021/10/10.
//

#ifndef _CHASSIS_H_
#define _CHASSIS_H_

/*include*/
#include "struct_typedef.h"
#include "bsp_can.h"
#include "PID.h"
#include "remote.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "ramp.h"
#include "bsp_buzzer.h"
#include "math.h"
#include "Detection.h"
#include "protocol_shaob.h"
#include "packet.h"


/*define*/
//底盘在motor_3508_measure中的标号
typedef enum {
    RF=0,
    LF,
    LB,
    RB
}chassis_motor_index_e;

//任务开始空闲一段时间

#define CHASSIS_TASK_INIT_TIME 157

#define CHASSIS_Y_CHANNEL 0

#define CHASSIS_X_CHANNEL 1

#define CHASSIS_Z_CHANNEL 4

#define CHASSIS_MODE_CHANNEL 0

#define CHASSIS_CONTROL_TIME_MS 2

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.3f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 4.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.8f

#define chassis_start_buzzer buzzer_on  (31, 19999)
#define chassis_buzzer_off   buzzer_off()            //buzzer off，关闭蜂鸣器

#define MAX_CHASSIS_VX_SPEED 1.0f //根据3508最高转速计算底盘最快移动速度应为3.3m/s左右
#define MAX_CHASSIS_VY_SPEED 1.0f
#define MAX_CHASSIS_VW_SPEED 1.0f
#define CHASSIS_SWING_SPEED 1.0f

#define MAX_CHASSIS_AUTO_VX_SPEED 3.0f //根据3508最高转速计算底盘最快移动速度应为3300左右
#define MAX_CHASSIS_AUTO_VY_SPEED 3.0f
#define MAX_CHASSIS_AUTO_VW_SPEED 3.0f//150

#define RC_TO_VX  (MAX_CHASSIS_VX_SPEED/660)
#define RC_TO_VY  (MAX_CHASSIS_VY_SPEED/660)
#define RC_TO_VW  (MAX_CHASSIS_VW_SPEED/660)    //MAX_CHASSIS_VR_SPEED / RC_MAX_VALUE

#define CHASSIS_CURRENT_LIMIT_TOTAL 20000
#define CHASSIS_CURRENT_LIMIT_40W 7300
#define CHASSIS_POWER_BUFF 60


//底盘机械信息 /m
#define Wheel_axlespacing 0.2f //H
#define Wheel_spacing 0.25f //W
#define GIMBAL_OFFSET 0
#define PERIMETER 0.251328f //zhou chang /m
#define M3508_DECELE_RATIO 1.0f/36.0f
#define M3508_MAX_RPM 18000
#define TREAD 480 //lun ju
#define WHEEL_MOTO_RATE 0.000116355f

//枚举 结构体
typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_AUTO,
    CHASSIS_DEBUG
} chassis_mode_e;



typedef struct {
    fp32 power_buff;
    fp32 limit_k;
    fp32 total_current;
    fp32 total_current_limit;

}chassis_power_limit_t;

typedef struct
{
    chassis_mode_e mode;
    chassis_mode_e last_mode;
    chassis_motor_t motor_chassis[4];

    pid_t chassis_vw_pid;
    fp32 vx;
    fp32 vy;
    fp32 vw;

    chassis_power_limit_t chassis_power_limit;
} chassis_t;

//函数声明
_Noreturn extern void chassis_task(void const *pvParameters);

extern void chassis_feedback_update();
//
#endif

