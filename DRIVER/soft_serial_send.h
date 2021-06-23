#ifndef _SOFT_SERIAL_SEND_H
#define _SOFT_SERIAL_SEND_H

#include <stdint.h>

void soft_erial_send_init(void);
void soft_erial_send(uint8_t serial_num, uint8_t data);

#endif
