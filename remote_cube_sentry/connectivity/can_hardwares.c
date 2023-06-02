//
// Created by nobody_knows on 23-3-13.
//

#include "can_hardwares.h"
#include "main.h"
#include "FreeRTOS.h" //FreeRTOS.h must code before queue.h!!!!!!!!
#include "queue.h"
#include "bsp_can.h"
#include "string.h"
#include "Chassis.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern chassis_t chassis;
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_MSG can_msg;
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&can_msg.rx_header,can_msg.rx_date);
    if(hcan == &hcan1)
    {
        chassis_motor_decode(chassis.motor_chassis,CAN_1,can_msg.rx_header.StdId,can_msg.rx_date);
    }
    else if(hcan == &hcan2)
    {
        chassis_motor_decode(chassis.motor_chassis,CAN_2,can_msg.rx_header.StdId,can_msg.rx_date);

    }
}
