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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/********************** macros and definitions *******************************/

#define UART_RX_IT_COUNT 1

/********************** internal data declaration ****************************/

static void (*tx_cb_func)(uint8_t*);
static void (*rx_cb_func)(uint8_t*);

static uint8_t rx_buffer[UART_RX_IT_COUNT];

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

/********************** external data definition *****************************/

extern UART_HandleTypeDef huart3;

/********************** internal functions definition ************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (*rx_cb_func)(rx_buffer);

  HAL_UART_Receive_IT(huart, rx_buffer, UART_RX_IT_COUNT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  (*tx_cb_func)((uint8_t*)"TX");
}
/********************** external functions definition ************************/

void c1_driver_init(void (*tx_cb)(uint8_t*), void (*rx_cb)(uint8_t*))
{
  tx_cb_func = tx_cb;
  rx_cb_func = rx_cb;

  HAL_UART_Receive_IT(&huart3, rx_buffer, UART_RX_IT_COUNT);
}

void c1_driver_tx(uint8_t *data){
  //CUIDADO ACA, QUIZAS HAY QUE HACER UN MEMCPY DE DATA? SE ESTA PASANDO EL PUNTERO
  HAL_UART_Transmit_IT(&huart3, data, 1u);
}

/********************** end of file ******************************************/

