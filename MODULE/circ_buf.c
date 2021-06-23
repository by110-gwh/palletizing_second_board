/**
 * @file    circular_buffer.c
 * @brief   Implementation of a circular buffer
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2016-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "circ_buf.h"

void util_assert(uint8_t BOOL) {
    while (!BOOL);
}
#define MIN(a, b) (a < b) ? a : b

void circ_buf_init(circ_buf_t *circ_buf, uint8_t *buffer, uint32_t size)
{
    __disable_irq();

    circ_buf->buf = buffer;
    circ_buf->size = size;
    circ_buf->head = 0;
    circ_buf->tail = 0;
    
    __enable_irq();
}

void circ_buf_push_two(circ_buf_t *circ_buf, uint8_t data1, uint8_t data2)
{
    __disable_irq();

    circ_buf->buf[circ_buf->tail] = data1;
    circ_buf->buf[circ_buf->tail + 1] = data2;
    circ_buf->tail += 2;
    if (circ_buf->tail >= circ_buf->size) {
        util_assert(circ_buf->tail == circ_buf->size);
        circ_buf->tail = 0;
    }

    // Assert no overflow
    util_assert(circ_buf->head != circ_buf->tail);
    
    __enable_irq();
}

void circ_buf_push(circ_buf_t *circ_buf, uint8_t data)
{
    __disable_irq();

    circ_buf->buf[circ_buf->tail] = data;
    circ_buf->tail += 1;
    if (circ_buf->tail >= circ_buf->size) {
        util_assert(circ_buf->tail == circ_buf->size);
        circ_buf->tail = 0;
    }

    // Assert no overflow
    util_assert(circ_buf->head != circ_buf->tail);
    
    __enable_irq();
}

uint8_t circ_buf_pop(circ_buf_t *circ_buf)
{
    uint8_t data;
    __disable_irq();

    // Assert buffer isn't empty
    util_assert(circ_buf->head != circ_buf->tail);

    data = circ_buf->buf[circ_buf->head];
    circ_buf->head += 1;
    if (circ_buf->head >= circ_buf->size) {
        util_assert(circ_buf->head == circ_buf->size);
        circ_buf->head = 0;
    }

    __enable_irq();

    return data;
}

uint32_t circ_buf_count_used(circ_buf_t *circ_buf)
{
    uint32_t cnt;
    __disable_irq();

    if (circ_buf->tail >= circ_buf->head) {
        cnt = circ_buf->tail - circ_buf->head;
    } else {
        cnt = circ_buf->tail + circ_buf->size - circ_buf->head;
    }

    __enable_irq();
    return cnt;
}

uint32_t circ_buf_count_free(circ_buf_t *circ_buf)
{
    uint32_t cnt;
    __disable_irq();

    cnt = circ_buf->size - circ_buf_count_used(circ_buf) - 1;

    __enable_irq();
    return cnt;
}

uint32_t circ_buf_read(circ_buf_t *circ_buf, uint8_t *data, uint32_t size)
{
    uint32_t cnt;
    uint32_t i;

    cnt = circ_buf_count_used(circ_buf);
    cnt = MIN(size, cnt);
    for (i = 0; i < cnt; i++) {
        data[i] = circ_buf_pop(circ_buf);
    }

    return cnt;
}

uint32_t circ_buf_write(circ_buf_t *circ_buf, const uint8_t *data, uint32_t size)
{
    uint32_t cnt;
    uint32_t i;

    cnt = circ_buf_count_free(circ_buf);
    cnt = MIN(size, cnt);
    for (i = 0; i < cnt; i++) {
        circ_buf_push(circ_buf, data[i]);
    }

    return cnt;
}
