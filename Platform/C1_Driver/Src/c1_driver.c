/**
 * Copyright (c) May 18, 2022 Lautaro Vera <lautarovera93@gmail.com>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file    : c1_driver.c
 * @date    : May 18, 2022
 * @author  : Lautaro Vera <lautarovera93@gmail.com>
 * @version : v1.0.0
 */

/********************** inclusions *******************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/********************** macros and definitions *******************************/
#define UART_RX_IT_COUNT 1

/********************** internal data declaration ****************************/
static void (*rx_cb_func)(uint8_t*);
static uint8_t rx_buffer[UART_RX_IT_COUNT];

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

/********************** external data definition *****************************/
extern osMessageQueueId_t QueueOutputHandle;
extern UART_HandleTypeDef huart3;
extern void c2_parser_rx_cb(uint8_t *data_ptr);

/********************** internal functions definition ************************/
static void c1_driver_init(void (*rx_cb)(uint8_t*))
{
  rx_cb_func = rx_cb;

  HAL_UART_Receive_IT(&huart3, rx_buffer, UART_RX_IT_COUNT);
}

static void c1_driver_tx(uint8_t *data, uint16_t size)
{
  HAL_UART_Transmit(&huart3, data, size,100);
}

/********************** external functions definition ************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (*rx_cb_func)(rx_buffer);

  HAL_UART_Receive_IT(huart, rx_buffer, UART_RX_IT_COUNT);
}

void c1_driver_task(void *args)
{
  uint8_t *msg_out = NULL;

  c1_driver_init(c2_parser_rx_cb);

  for (;;)
  {
    osMessageQueueGet(QueueOutputHandle, (uint8_t *)&msg_out, 0, osWaitForever);
    size_t msg_out_len = strlen((const char *)msg_out);

    c1_driver_tx(msg_out, msg_out_len);
    vPortFree(msg_out);
    msg_out = NULL;
  }
}
/********************** end of file ******************************************/

