#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "struct_typedef.h"
#include "PID.h"

#define CHASSIS_3508_PID_KP     8.0f
#define CHASSIS_3508_PID_KI     0.0f//1.0f
#define CHASSIS_3508_PID_KD     0.0f
#define CHASSIS_3508_PID_MAX_OUT 8000.0f
#define CHASSIS_3508_PID_MAX_IOUT 1000.0f

typedef enum {
    CAN_1,
    CAN_2,
}CAN_TYPE;

typedef enum{
    CAN_DJI_MOTOR_0x200_ID = 0x200,//C620/C610 id=1~4 (0x201~0x204)
    CAN_DJI_MOTOR_0x1FF_ID = 0x1FF,//C620/C610 id=5~8 (0x205~0x208);GM6020 id=1~4 (0x205~0x208)
    CAN_DJI_MOTOR_0x2FF_ID = 0x2FF,//GM6020 id=5~7 (0x209~0x20B)
}DJI_MOTOR_ID;

typedef enum{
    //0X200对应的电机ID(CAN1)
    CAN_CHASSIS_3508_MOTOR_RF=0x202,
    CAN_CHASSIS_3508_MOTOR_LF=0x201,
    CAN_CHASSIS_3508_MOTOR_LB=0x204,
    CAN_CHASSIS_3508_MOTOR_RB=0x203,

    //0X1FF对应的电机ID(CAN1)
    CAN_GIMBAL_6020_YAW=0x205,
    CAN_LAUNCHER_2006_TRIGGER=0x206,

    //0X200对应的电机ID(CAN2)

    //0X1FF对应的电机ID(CAN2)
    CAN_LAUNCHER_3508_FIRE_R=0X205,
    CAN_GIMBAL_6020_PITCH=0x206,
    CAN_LAUNCHER_3508_FIRE_L=0X207,
    CAN_LAUNCHER_2006_BARREL=0x208,
}CAN_ID;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;

    int32_t total_ecd;   //电机旋转的总编码器数值
    int32_t round_count;
    uint16_t offset_ecd;//电机的校准编码值
} motor_measure_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t speed_p;

    fp32 rpm_set;
    int16_t give_current;
}chassis_motor_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t angle_p;
    pid_t relative_angle_p;
    pid_t speed_p;
    pid_t relative_speed_p;

    fp32 relative_angle_get;
    fp32 relative_angle_set; //°
    fp32 relative_gyro_get;

    fp32 gyro_set;  //转速设置
    int16_t give_current; //最终电流值
}gimbal_motor_t;

typedef struct
{
    motor_measure_t motor_info;
    pid_t angle_p;
    pid_t speed_p;

    fp32 rpm_set;
    int16_t give_current;

}launcher_motor_t;
extern void get_can_msg(CAN_TYPE can_type,uint32_t *can_id, uint8_t * can_ms);
extern void chassis_motor_decode(chassis_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void gimbal_motor_decode(gimbal_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void launcher_motor_decode(launcher_motor_t *motor,uint8_t can_type,uint32_t can_id,uint8_t * can_msg);
extern void can_send_dji_motor(CAN_TYPE can_type, DJI_MOTOR_ID CMD_ID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void motor_init(chassis_motor_t *chassis_motor);
#endif
