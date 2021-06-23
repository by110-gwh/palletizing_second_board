#ifndef _SECOND_SERIAL_H
#define _SECOND_SERIAL_H

#include <stdint.h>
#include "circ_buf.h"

extern circ_buf_t second_serial_cicr_buf;

void second_serial_init(void);
void second_serial_send(uint8_t data);

#endif
