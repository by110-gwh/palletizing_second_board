#ifndef _MAIN_SERIAL_H
#define _MAIN_SERIAL_H

#include <stdint.h>
#include "circ_buf.h"

extern circ_buf_t main_serial_cicr_buf;

void main_serial_init(void);
void main_serial_send(void);

#endif
