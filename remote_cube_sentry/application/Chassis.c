//
// Created by xhuanc on 2021/10/10.
//

/*include*/
#include "Chassis.h"
#include "bsp_can.h"
#include "can_hardwares.h"
/*define*/
/*轮子控制映射：                                 解算坐标：      x(前)
            ****      前       ****                                 |
           * 2 LF *          * 1 RF *                              |
            ****              ****                                 |
                                                                   |
           左                   右                 y  --------------z-----------
                                                                   |
            ****              ****                                 |
          * 3 LB *          * 4 RB *                               |
            ****      后      ****                                 |

*/
/*变量*/
extern RC_ctrl_t rc_ctrl;
extern QueueHandle_t CHASSIS_motor_queue;
ramp_function_source_t chassis_auto_vx_ramp;
ramp_function_source_t chassis_auto_vy_ramp;
ramp_function_source_t chassis_auto_vw_ramp;
ramp_function_source_t chassis_3508_ramp[4];
chassis_t chassis;
//上位机下发数据
extern robot_ctrl_info_t robot_ctrl;
//底盘解算发送数据
extern chassis_odom_info_t chassis_odom;
extern uint32_t nav_time;
uint32_t reset_time;
//发送机器人id
vision_t vision_data;
static fp32 rotate_ratio_f = ((Wheel_axlespacing + Wheel_spacing) / 2.0f - GIMBAL_OFFSET); //rad
static fp32 rotate_ratio_b = ((Wheel_axlespacing + Wheel_spacing) / 2.0f + GIMBAL_OFFSET);
static fp32 wheel_rpm_ratio = 60.0f / (PERIMETER * M3508_DECELE_RATIO); //che lun zhuan su bi

/*      函数及声明   */
static void chassis_init();

static void chassis_set_mode();

static void chassis_ctrl_info_get();

static void chassis_relax_handle();

static void chassis_auto_handle();

static void chassis_wheel_cal();

static void chassis_wheel_loop_cal();

void send_robot_id();

void chassis_can_send_back_mapping();


/*程序主体*/
_Noreturn void chassis_task(void const *pvParameters) {

    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    chassis_init();//底盘初始化
    //主任务循环
    int key_time=0;
    while (1) {
        chassis_feedback_update();
        vTaskSuspendAll(); //锁住RTOS内核防止控制过程中断，造成错误
        chassis_set_mode();
//        if((HAL_GetTick() - nav_time)<1000)
//            chassis.mode = CHASSIS_AUTO;
//        else chassis.mode = CHASSIS_RELAX;

        if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_RESET)
        {
            key_time++;
            if(key_time>=100)
            {
                if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==GPIO_PIN_RESET)
                {
                    vision_data.shoot_sta = 1;
                    reset_time = HAL_GetTick();
                    vision_data.shoot_speed = 1;//reset robot
                }
            }
        }
        if(HAL_GetTick()-reset_time>=500)
            vision_data.shoot_speed= 0;
        if(vision_data.shoot_sta == 1)
        {
            detect_handle(DETECT_AUTO_AIM);
            detect_handle(DETECT_REMOTE);

        }
        //判断底盘模式选择 决定是否覆盖底盘转速vw;
        switch (chassis.mode) {
            case CHASSIS_DEBUG:
                chassis_ctrl_info_get(); //遥控器获取底盘方向矢量
                break;

            case CHASSIS_AUTO://自动巡逻
                chassis_auto_handle();
                break;

            case CHASSIS_RELAX:
                chassis_relax_handle();
                break;
        }

        if (chassis.mode != CHASSIS_RELAX) {
            //底盘解算
            chassis_wheel_cal();
            //驱电机闭环
            chassis_wheel_loop_cal();
            //电机映射
            chassis_can_send_back_mapping();
        }
        else{
            chassis_can_send_back_mapping();
        }
        xTaskResumeAll();

        vTaskDelay(1);
    }

}
static void chassis_init(void) {

    pid_init(&chassis.chassis_vw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, (uint32_t) CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,
             CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD);
    //底盘驱动电机速度环初始化和电机数据结构体获取
    motor_init(chassis.motor_chassis);
    //初始时底盘模式为失能
    chassis.mode = chassis.last_mode = CHASSIS_RELAX;
    ramp_init(&chassis_3508_ramp[LF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RF],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[RB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_3508_ramp[LB],0.0001f,M3508_MAX_RPM,-M3508_MAX_RPM);
    ramp_init(&chassis_auto_vx_ramp, 0.3f, MAX_CHASSIS_AUTO_VX_SPEED, -MAX_CHASSIS_AUTO_VX_SPEED);
    ramp_init(&chassis_auto_vy_ramp, 0.3f, MAX_CHASSIS_AUTO_VY_SPEED, -MAX_CHASSIS_AUTO_VY_SPEED);
    ramp_init(&chassis_auto_vw_ramp, 0.3f, MAX_CHASSIS_AUTO_VW_SPEED, -MAX_CHASSIS_AUTO_VW_SPEED);

}

static void chassis_set_mode() {

    if (switch_is_down(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode = chassis.mode;
        chassis.mode = CHASSIS_RELAX;
    }
    else if (switch_is_mid(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode = chassis.mode;
        chassis.mode = CHASSIS_DEBUG;
    }
    else if (switch_is_up(rc_ctrl.rc.s[RC_s_R])) {
        chassis.last_mode = chassis.mode;
        chassis.mode = CHASSIS_AUTO;
    }
    //防止导航发疯后切换为手动模式再切回自动模式时没有对最后发布的控制指令清零导致继续发疯的问题
    if(chassis.mode == CHASSIS_DEBUG){
        robot_ctrl.vx = 0;
        robot_ctrl.vy = 0;
        robot_ctrl.vw = 0;
    }
}

static void chassis_ctrl_info_get() {
    chassis.vx = -(float)(rc_ctrl.rc.ch[CHASSIS_X_CHANNEL]) * RC_TO_VX;
    chassis.vy = -(float)(rc_ctrl.rc.ch[CHASSIS_Y_CHANNEL]) * RC_TO_VY;
    chassis.vw = (float)(rc_ctrl.rc.ch[CHASSIS_Z_CHANNEL]) * 0.01f;
}

static void chassis_auto_handle() {
    ramp_calc(&chassis_auto_vx_ramp, robot_ctrl.vx);
    robot_ctrl.vx = chassis_auto_vx_ramp.out;
    ramp_calc(&chassis_auto_vy_ramp, robot_ctrl.vy);
    robot_ctrl.vy = chassis_auto_vy_ramp.out;
    ramp_calc(&chassis_auto_vw_ramp, robot_ctrl.vw);
    robot_ctrl.vw = chassis_auto_vw_ramp.out;

    chassis.vx = -robot_ctrl.vx;
    chassis.vy = robot_ctrl.vy;
    chassis.vw = robot_ctrl.vw;
}


//将期望速度转为转子期望转速
static void chassis_wheel_cal(){
    fp32 max=0;
    fp32 wheel_rpm[4];
    fp32 vx, vy, vw;

    vx=chassis.vx;
    vy=chassis.vy;
    vw=chassis.vw;
    //麦克纳姆轮运动学解算
    wheel_rpm[0] = (-vy - vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = (-vy + vx - vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (vy + vx - vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (vy - vx - vw * rotate_ratio_b) * wheel_rpm_ratio;

    // find max item
    for (uint8_t i = 0; i < 4; i++) {
        if (abs(wheel_rpm[i]) > max) max = abs(wheel_rpm[i]);
    }
    // equal proportion
    if (max > M3508_MAX_RPM) {
        float rate = M3508_MAX_RPM / max;
        for (uint8_t i = 0; i < 4; i++) wheel_rpm[i] *= rate;
    }

    chassis.motor_chassis[RF].rpm_set=wheel_rpm[0];
    chassis.motor_chassis[LF].rpm_set=wheel_rpm[1];
    chassis.motor_chassis[LB].rpm_set=wheel_rpm[2];
    chassis.motor_chassis[RB].rpm_set=wheel_rpm[3];

}
void chassis_feedback_update()
{

    //update odom vel data
    chassis_odom.vx = -((float)(chassis.motor_chassis[1].motor_info.speed_rpm
                            + chassis.motor_chassis[2].motor_info.speed_rpm
                            - chassis.motor_chassis[0].motor_info.speed_rpm
                            - chassis.motor_chassis[3].motor_info.speed_rpm
                           )*(WHEEL_MOTO_RATE))/4;

    chassis_odom.vy = ((float)(chassis.motor_chassis[2].motor_info.speed_rpm
                            - chassis.motor_chassis[1].motor_info.speed_rpm
                            - chassis.motor_chassis[0].motor_info.speed_rpm
                            + chassis.motor_chassis[3].motor_info.speed_rpm
                           )*(WHEEL_MOTO_RATE))/4;

    chassis_odom.vw = ((float )(-chassis.motor_chassis[2].motor_info.speed_rpm
                            - chassis.motor_chassis[1].motor_info.speed_rpm
                            - chassis.motor_chassis[0].motor_info.speed_rpm
                            - chassis.motor_chassis[3].motor_info.speed_rpm
                           )*(WHEEL_MOTO_RATE))/4;
    rm_queue_data( CHASSIS_ODOM_CMD_ID,&chassis_odom,sizeof (chassis_odom_info_t));
    vTaskDelay(5);
    rm_queue_data(VISION_ID,&vision_data,sizeof(vision_t));
}

static void chassis_wheel_loop_cal() {

    ramp_calc(&chassis_3508_ramp[RF],chassis.motor_chassis[RF].rpm_set);
    chassis.motor_chassis[RF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RF].speed_p,
                                                     chassis.motor_chassis[RF].motor_info.speed_rpm,
                                                     chassis.motor_chassis[RF].rpm_set);

    ramp_calc(&chassis_3508_ramp[LF],chassis.motor_chassis[LF].rpm_set);

    chassis.motor_chassis[LF].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LF].speed_p,
                                                     chassis.motor_chassis[LF].motor_info.speed_rpm,
                                                     chassis.motor_chassis[LF].rpm_set);

    ramp_calc(&chassis_3508_ramp[RB],chassis.motor_chassis[RB].rpm_set);

    chassis.motor_chassis[RB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[RB].speed_p,
                                                     chassis.motor_chassis[RB].motor_info.speed_rpm,
                                                     chassis.motor_chassis[RB].rpm_set);

    ramp_calc(&chassis_3508_ramp[LB],chassis.motor_chassis[LB].rpm_set);
    chassis.motor_chassis[LB].give_current= (int16_t)pid_calc(&chassis.motor_chassis[LB].speed_p,
                                                     chassis.motor_chassis[LB].motor_info.speed_rpm,
                                                     chassis.motor_chassis[LB].rpm_set);

}

//把can接收时对真实电机的映射，在发送控制时映射回去为真实的电机，因为控制函数要按电机ID 1～4发送
void chassis_can_send_back_mapping(){

    int16_t *real_motor_give_current[4];

    real_motor_give_current[0] = &chassis.motor_chassis[LF].give_current;
    real_motor_give_current[1] = &chassis.motor_chassis[RF].give_current;
    real_motor_give_current[2] = &chassis.motor_chassis[RB].give_current;
    real_motor_give_current[3] = &chassis.motor_chassis[LB].give_current;

    can_send_dji_motor(CAN_1,
                  CAN_DJI_MOTOR_0x200_ID,
                  *real_motor_give_current[0],
                  *real_motor_give_current[1],
                  *real_motor_give_current[2],
                  *real_motor_give_current[3]
                  );

}
void chassis_device_offline_handle() {
    if(detect_list[DETECT_REMOTE].status==OFFLINE)
        chassis.mode=CHASSIS_RELAX;//防止出现底盘疯转
}
static void chassis_relax_handle() {
    for (int i = 0; i < 4; ++i) {
        chassis.motor_chassis[i].give_current = 0;
    }
}
