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
 * @file    : app_main.c
 * @date    : May 6, 2022
 * @author  : Lautaro Vera <lautarovera93@gmail.com>
 * @version : v1.0.0
 */

/********************** inclusions *******************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/********************** macros and definitions *******************************/

#define PRINT_RATE_MS        5000u
#define TIMEOUT_MS           500u

#define DEBUG_PRINT(...)\
  taskENTER_CRITICAL();\
  debug_printf(__VA_ARGS__);\
  taskEXIT_CRITICAL()\

/********************** internal data declaration ****************************/

TimerHandle_t timer;

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

/********************** external data definition *****************************/

extern bool debug_init(void);
extern void debug_printf(const char *format, ...);

/********************** internal functions definition ************************/

static bool sys_clk_cfg(void)
{
  bool result = true;

  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /* Initializes the RCC Oscillators according to the specified parameters in the RCC_OscInitTypeDef structure. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    result = false;
  }
  /* Initializes the CPU, AHB and APB buses clocks. */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    result = false;
  }

  return result;
}

static void timer_callback(TimerHandle_t timer_handle)
{
  DEBUG_PRINT("Timeout!\r\n");
}

static void task_rx_print(void *pvParameters)
{
  debug_init();

  xTimerStart(timer, 0u);

  for (;;)
  {
    DEBUG_PRINT("Hello World\r\n");
    vTaskDelay(PRINT_RATE_MS / portTICK_RATE_MS);
    xTimerReset(timer, 0u);
  }
}

/********************** external functions definition ************************/

void mcu_init(void)
{
  BaseType_t res;

  HAL_Init();

  res = sys_clk_cfg();
  configASSERT(res == pdPASS);
}

void app(void)
{
  BaseType_t res;

  timer = xTimerCreate("", TIMEOUT_MS / portTICK_RATE_MS, pdFALSE, NULL, timer_callback);

  res = xTaskCreate(task_rx_print, (const char*)"TaskPrint", configMINIMAL_STACK_SIZE, (void*) NULL,
                    (tskIDLE_PRIORITY + 1ul), (xTaskHandle*) NULL);
  configASSERT(res == pdPASS);
}

/********************** end of file ******************************************/
