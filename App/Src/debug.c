/**
 * Copyright (c) May 6, 2022 Lautaro Vera <lautarovera93@gmail>.
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
 * @file    : debug.c
 * @date    : May 6, 2022
 * @author  : Lautaro Vera <lautarovera93@gmail.com>
 * @version : v1.0.0
 */

/********************** inclusions *******************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f4xx_hal.h"

/********************** macros and definitions *******************************/

#define PRINT_BUFFER_SIZE       50u

/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

UART_HandleTypeDef uart_handler;

static uint8_t buffer[PRINT_BUFFER_SIZE];

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

/********************** external functions definition ************************/

bool debug_init(void)
{
  bool result = true;

  uart_handler.Instance = USART3;
  uart_handler.Init.BaudRate = 115200;
  uart_handler.Init.WordLength = UART_WORDLENGTH_8B;
  uart_handler.Init.StopBits = UART_STOPBITS_1;
  uart_handler.Init.Parity = UART_PARITY_NONE;
  uart_handler.Init.Mode = UART_MODE_TX_RX;
  uart_handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart_handler.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&uart_handler) != HAL_OK)
  {
    result = false;
  }

  return result;
}

void debug_printf(const char *format, ...)
{
  memset(buffer, 0, PRINT_BUFFER_SIZE);

  va_list va;
  va_start(va, format);
  vsnprintf((char *)buffer, PRINT_BUFFER_SIZE, format, va);
  va_end(va);

  HAL_UART_Transmit(&uart_handler, buffer, PRINT_BUFFER_SIZE, HAL_MAX_DELAY);
}

/********************** end of file ******************************************/
