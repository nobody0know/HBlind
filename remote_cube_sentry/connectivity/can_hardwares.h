//
// Created by nobody_knows on 23-3-13.
//

#ifndef REMOTE_CUBE_SENTRY_CAN_HARDWARES_H
#define REMOTE_CUBE_SENTRY_CAN_HARDWARES_H

#include "struct_typedef.h"

typedef struct
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_date[8];
}CAN_MSG;
extern void can_filter_init(void);
#endif //REMOTE_CUBE_SENTRY_CAN_HARDWARES_H
