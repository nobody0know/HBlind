#include "bsp_can.h"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "Detection.h"
#include "Chassis.h"
#include "can_hardwares.h"

extern QueueHandle_t CAN1_receive_queue;
extern QueueHandle_t CAN2_receive_queue;
extern QueueHandle_t CHASSIS_motor_queue;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern chassis_t  chassis;

/**
  * @brief          通过can1/can2，按id以0x201～0x204（CMD_ID=0x200）或0x205～0x208（CMD_ID=0x1FF）的 顺 序 发送电机控制数据
  * @param[in]      选择发送总线为CAN1/CAN2
  * @param[in]      选择所发送的标识符
  * @param[in]      给id为 0x201/0x205 的电机发送控制数据
  * @param[in]      给id为 0x202/0x206 的电机发送控制数据
  * @param[in]      给id为 0x203/0x207 的电机发送控制数据
  * @param[in]      给id为 0x204/0x208 的电机发送控制数据
  * @retval         返回空
  */
void can_send_dji_motor(CAN_TYPE can_type, DJI_MOTOR_ID CMD_ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    CAN_TxHeaderTypeDef tx_message;
    uint8_t can_send_data[8];
    tx_message.StdId = CMD_ID;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    can_send_data[0] = motor1 >> 8;
    can_send_data[1] = motor1;
    can_send_data[2] = motor2 >> 8;
    can_send_data[3] = motor2;
    can_send_data[4] = motor3 >> 8;
    can_send_data[5] = motor3;
    can_send_data[6] = motor4 >> 8;
    can_send_data[7] = motor4;

    if (can_type == CAN_1) {
        HAL_CAN_AddTxMessage(&hcan1, &tx_message, can_send_data, &send_mail_box);
    } else if (can_type == CAN_2) {
        HAL_CAN_AddTxMessage(&hcan2, &tx_message, can_send_data, &send_mail_box);
    }
}

/*!
 *
 * @brief 大疆C610/C620/GM6020信息解码
 * @param motor 电机数据结构体
 * @param data  can消息报文
 * @retval none
 */
void dji_motor_decode(motor_measure_t *motor,const uint8_t *data)
{
    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)(data[0]<<8 | data[1]);
    motor->speed_rpm = (int16_t)(data[2]<<8 | data[3]);
    motor->given_current = (int16_t)(data[4]<<8 | data[5]);
    motor->temperate = data[6];
}

void dji_motor_round_count(motor_measure_t *motor)
{
    if(motor->ecd - motor->last_ecd>4192){
        motor->round_count--;
    }
    else if(motor->ecd-motor->last_ecd< -4192)
    {
        motor->round_count++;
    }
    motor->total_ecd = motor->round_count*8192 + (motor->ecd - motor->offset_ecd);
}

void motor_init(chassis_motor_t *chassis_motor) {
    for (int i = 0; i < 4; i++) {
        pid_init(&chassis_motor[i].speed_p,
                 CHASSIS_3508_PID_MAX_OUT,
                 CHASSIS_3508_PID_MAX_IOUT,
                 CHASSIS_3508_PID_KP,
                 CHASSIS_3508_PID_KI,
                 CHASSIS_3508_PID_KD);
    }
}
/*!
 *
 * @brief 底盘信息解码
 * @param motor 电机数据结构体
 * @param can_id can_id
 * @param can_msg  can消息报文
 * @retval none
 */
void chassis_motor_decode(chassis_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_CHASSIS_3508_MOTOR_RF: {
                dji_motor_decode(&motor[RF].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LF: {
                dji_motor_decode(&motor[LF].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LF);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_LB: {
                dji_motor_decode(&motor[LB].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_LB);
            }
                break;

            case CAN_CHASSIS_3508_MOTOR_RB: {
                dji_motor_decode(&motor[RB].motor_info, can_msg);
                detect_handle(DETECT_CHASSIS_3508_RB);
            }
                break;

            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {

    }
}
/*!
 *
 * @brief 云台信息解码
 * @param motor 电机数据结构体
 * @param can_id can_id
 * @param can_msg  can消息报文
 * @retval none
 */
void gimbal_motor_decode(gimbal_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_GIMBAL_6020_YAW: {
                detect_handle(DETECT_GIMBAL_6020_YAW);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_GIMBAL_6020_PITCH: {
                detect_handle(DETECT_GIMBAL_6020_PITCH);
            }
                break;
        }
    }
}

void launcher_motor_decode(launcher_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg)
{
    if(can_type == CAN_1)
    {
        switch (can_id) {
            case CAN_LAUNCHER_2006_TRIGGER: {
                detect_handle(DETECT_LAUNCHER_2006_TRIGGER);
            }
                break;
            default: {
                break;
            }
        }
    }
    else if(can_type == CAN_2)
    {
        switch (can_id) {
            case CAN_LAUNCHER_3508_FIRE_L: {
                detect_handle(DETECT_LAUNCHER_3508_FIRE_L);
            }
                break;
            case CAN_LAUNCHER_3508_FIRE_R: {
                detect_handle(DETECT_LAUNCHER_3508_FIRE_R);
            }
                break;
            case CAN_LAUNCHER_2006_BARREL: {
            }
                break;
        }
    }
}