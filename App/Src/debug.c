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

#define RX_BUFFER_SIZE 5
#define UART_RX_IT_COUNT 1

#define USARTx_IRQn     USART3_IRQn

/* Exported functions ------------------------------------------------------- */

void USART3_IRQHandler(void);
/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

UART_HandleTypeDef uart_handler;

//static uint8_t buffer[PRINT_BUFFER_SIZE];

static uint8_t aRXBufferUser[RX_BUFFER_SIZE];
static uint8_t aRXBufferA[RX_BUFFER_SIZE];

static bool rxFlag;
static bool txFlag;
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

  /*
   NVIC configuration if you need to use interrupt process (HAL_UART_Transmit_IT()
   and HAL_UART_Receive_IT() APIs):
   (+++) Configure the USARTx interrupt priority.
   (+++) Enable the NVIC USART IRQ handle.

   The specific UART interrupts (Transmission complete interrupt,
   RXNE interrupt and Error Interrupts) will be managed using the macros
   __HAL_UART_ENABLE_IT() and __HAL_UART_DISABLE_IT() inside the transmit
   and receive process.
   */

  if (HAL_UART_Init(&uart_handler) != HAL_OK)
  {
    result = false;
  }

  HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(USARTx_IRQn);
  //__HAL_UART_ENABLE_IT(&uart_handler, UART_IT_RXNE);

  if (HAL_OK != HAL_UART_Receive_IT(&uart_handler, aRXBufferUser, UART_RX_IT_COUNT))
  {
    result = false;
  }

  HAL_UART_Transmit(&uart_handler, (uint8_t*)"OKAY", RX_BUFFER_SIZE, HAL_MAX_DELAY);

  return result;
}

void debug_printf(const char *format, ...)
{
  /*
   memset(buffer, 0, PRINT_BUFFER_SIZE);

   va_list va;
   va_start(va, format);
   vsnprintf((char*)buffer, PRINT_BUFFER_SIZE, format, va);
   va_end(van);

  HAL_UART_Transmit(&uart_handler, buffer, PRINT_BUFFER_SIZE, HAL_MAX_DELAY);
  */
  if (rxFlag)
  {
    HAL_UART_Transmit_IT(&uart_handler, aRXBufferA, UART_RX_IT_COUNT);
    rxFlag = false;
  }
  if (txFlag)
    {
      HAL_UART_Transmit(&uart_handler, (uint8_t*)"TXMSG", RX_BUFFER_SIZE, HAL_MAX_DELAY);
      txFlag = false;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  rxFlag = true;
  for (uint8_t i = 0; i < UART_RX_IT_COUNT; i++)
  {
    aRXBufferA[i] = aRXBufferUser[i];
  }

  UART_Start_Receive_IT(UartHandle, aRXBufferUser, UART_RX_IT_COUNT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  txFlag = true;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
}

/**
 * @brief This function handles USART3 global interrupt.
 */
void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&uart_handler);
}
/********************** end of file ******************************************/
