#ifndef _STEERING_TASK_H
#define _STEERING_TASK_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

typedef struct {
	uint8_t cmd;
	uint8_t chanel;
	uint16_t data;
} action_group_t;

typedef struct {
	uint8_t header;
	uint8_t cmd;
	uint8_t chanel;
	uint8_t data_l;
	uint8_t data_h;
} rec_cmd_t;

extern action_group_t action_group_save_main[8192];
extern action_group_t action_group_save_second[8192];
extern QueueHandle_t rec_data_queue;
extern volatile uint8_t steering_speed[16];
extern volatile uint16_t steering_position[16];
extern volatile uint8_t steering_task_exit;

void steering_task_create(void);

#endif
