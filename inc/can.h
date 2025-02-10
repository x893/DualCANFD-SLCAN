#ifndef __CAN_H__
#define __CAN_H__

#include <stdint.h>

void can_config(void);
void can_process(uint32_t time);

int slcan_process(uint8_t *cmd, uint16_t length);

#endif
