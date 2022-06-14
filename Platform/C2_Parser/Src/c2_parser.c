/**
 * Copyright (c) 31 may. 2022 Lautaro Vera <lautarovera93@gmail.com>.
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
 * @file    : c2_parser.c
 * @date    : 31 may. 2022
 * @author  : Lautaro Vera <lautarovera93@gmail.com>
 * @version : v1.0.0
 */

/********************** inclusions *******************************************/
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "cmsis_os2.h"
#include "c1_driver.h"
#include "c2_parser.h"
#include "crc8.h"
#include "FreeRTOS.h"
/********************** macros and definitions *******************************/
#define SOM                     '('
#define EOM                     ')'
#define MAX_MSG_SIZE            200u
#define ID_SIZE                 4u
#define OPCPDE_SIZE             1u
#define CRC_SIZE                2u
#define TIMEOUT_MS              4u

/********************** internal data declaration ****************************/
typedef enum
{
  IDLE, ID_RECEIVE, DATA_RECEIVE
} state_t;

/********************** internal functions declaration ***********************/
static void message_error(uint8_t *msg);
static bool check_crc(uint8_t *msg, uint16_t len);

/********************** internal data definition *****************************/

osMemoryPoolId_t pdu_pool_id;
extern osTimerId_t timeoutHandle;
#define timeout_id timeoutHandle
/********************** external data definition *****************************/
static bool new_message = false;
static bool timeout = false;
static uint8_t *msg_in = NULL;

extern osMessageQueueId_t QueueDownstreamHandle;
extern osMessageQueueId_t QueueUpstreamHandle;
extern osMessageQueueId_t QueueOutputHandle;
/********************** internal functions definition ************************/
void timeout_cb(void *argument)
{
  timeout = true;
//  c2_parser_rx_cb(0x00);        //Se actualiza la ME con un código de error y timeot=true
}

static void message_error(uint8_t *msg)
{
  if (msg != NULL)
  {
    osMemoryPoolFree(pdu_pool_id, msg);
    msg = NULL;
  }
}

static bool check_crc(uint8_t *msg, uint16_t len)
{
  uint8_t target_crc = (uint8_t)strtol((const char*)&msg[len - CRC_SIZE], NULL, 16);
  uint8_t value_crc = crc8_calc(0u, (uint8_t*)msg, len - CRC_SIZE);

  return value_crc == target_crc ? true : false;
}

static uint8_t *c2_create_sdu(uint8_t *msg)
{
  uint8_t *sdu = NULL;
  uint16_t len = strlen((const char *)msg);

  sdu = (uint8_t *)pvPortMalloc(len - ID_SIZE - CRC_SIZE + 1);

  memcpy(sdu, &msg[ID_SIZE], len - ID_SIZE - CRC_SIZE);

  return sdu;
}

static bool c2_is_new_message(void)
{
  return new_message;
}

static void c2_parser_init(void)
{
  pdu_pool_id = osMemoryPoolNew(10, MAX_MSG_SIZE, NULL);
}

/********************** external functions definition ************************/
void c2_parser_rx_cb(uint8_t *data_ptr)
{
  static uint16_t msg_len = 0;
  static uint8_t *msg_ptr = NULL;
  static state_t state_reg = IDLE;
  uint8_t data = *data_ptr;

  switch (state_reg)
  {
    case IDLE:
      if (SOM == data)
      {
        msg_in = osMemoryPoolAlloc(pdu_pool_id, 0);
        if (NULL == msg_in)
        {
          message_error(msg_in);
          state_reg = IDLE;
        }
        msg_ptr = msg_in;
        msg_len = 0;

        osTimerStart(timeout_id, TIMEOUT_MS);
        state_reg = ID_RECEIVE;
      }
      break;

    case ID_RECEIVE:
      if (SOM == data || timeout)
      {
        timeout = false;
        message_error(msg_in);
        state_reg = IDLE;
      }
      else //if (osTimerStop(timeout_id) == osOK)
      {
        if (isxdigit(data) != 0u && islower(data) == 0u)
        {
          *msg_ptr++ = data;
          osTimerStart(timeout_id, TIMEOUT_MS);

          state_reg = (ID_SIZE == ++msg_len) ? DATA_RECEIVE : ID_RECEIVE;
        }
        else
        {
          message_error(msg_in);
          state_reg = IDLE;
        }
      }
     /*else
      {
        message_error(msg_in);
        state_reg = IDLE;
      }*/
      break;

    case DATA_RECEIVE:
      if (SOM == data || MAX_MSG_SIZE < msg_len || timeout)
      {
        timeout = false;
        message_error(msg_in);
        state_reg = IDLE;
      }
      else //if (osTimerStop(timeout_id) == osOK)
      {
        if (EOM == data)
        {
          new_message = check_crc(msg_in, msg_len);
          if (!new_message)
          {
            message_error(msg_in);
          }
          //TODO: Incluir acá la acción a ejecutarse cuando el paquete es válido
          state_reg = IDLE;
        }
        else
        {
          osTimerStart(timeout_id, TIMEOUT_MS);
          *msg_ptr++ = data;
          msg_len++;
        }
      }
    /*  else
      {
        message_error(msg_in);
        state_reg = IDLE;
      }*/
      break;
    default:
      break;
  }
}

void c2_parser_task(void *args)
{
  uint8_t *sdu = NULL;
  uint8_t *pdu = NULL;
  uint8_t *msg_out = NULL;
  uint8_t id[ID_SIZE];

  c2_parser_init();

  for (;;)
  {
    if (c2_is_new_message())
    {
      sdu = c2_create_sdu(msg_in);
      memcpy(id, msg_in, ID_SIZE);
      osMemoryPoolFree(pdu_pool_id, msg_in);

      osMessageQueuePut(QueueUpstreamHandle, (uint8_t *)&sdu, 0, osWaitForever);

      osMessageQueueGet(QueueDownstreamHandle, (uint8_t *)&pdu, 0, osWaitForever);

      size_t msg_out_len = ID_SIZE + strlen((const char *)pdu) + CRC_SIZE;
      msg_out = pvPortMalloc(msg_out_len);

      memcpy(msg_out, id, ID_SIZE);
      memcpy(&msg_out[ID_SIZE], pdu, strlen((const char *)pdu));
      vPortFree(pdu);
      pdu = NULL;

      sprintf((char *)&msg_out[msg_out_len - CRC_SIZE], "%X", crc8_calc(0u, (uint8_t*)msg_out, msg_out_len - CRC_SIZE));

      osMessageQueuePut(QueueOutputHandle, (uint8_t *)&msg_out, 0, osWaitForever);
    }
  }
  osDelay(10);
}

//void c2_read_message(void)
//{
//  new_message = false;
//}

/********************** end of file ******************************************/
