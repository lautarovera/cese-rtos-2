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

/********************** macros and definitions *******************************/
#define SOM                     '('
#define EOM                     ')'
#define MAX_MSG_SIZE            200u
#define ID_SIZE                 4u
#define CRC_SIZE                2u
#define TIMEOUT_MS              4u

/********************** internal data declaration ****************************/
typedef enum
{
  IDLE, ID_RECEIVE, DATA_RECEIVE
} state_t;

/********************** internal functions declaration ***********************/
//static void timeout_cb(void const *arg);
static void message_error(uint8_t *msg);
static bool check_crc(uint8_t *msg, uint16_t len);

/********************** internal data definition *****************************/
/*//TODO: Cambiar ACA
static osTimerDef(timeout, timeout_cb);
static osTimerId timeout_id;
static osPoolDef(pdu_pool, 1u, MAX_MSG_SIZE);
static osPoolId pdu_pool_id;
*/
osMemoryPoolId_t pdu_pool_id;
extern osTimerId_t timeoutHandle;
#define timeout_id timeoutHandle
/********************** external data definition *****************************/
static bool new_message = false;
static bool timeout = false;

/********************** internal functions definition ************************/
void timeout_cb(void const *arg)
{
  timeout = true;
  c2_parser_rx_cb(0x00);        //Se actualiza la ME con un código de error y timeot=true
}

static void message_error(uint8_t *msg)
{
  if (msg != NULL)
  {
    //TODO: Cambio CMSIS_V2 ACA(void)osPoolFree(pdu_pool_id, msg);
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

/********************** external functions definition ************************/
void c2_parser_rx_cb(uint8_t data)
{
  static uint16_t msg_len = 0;
  static uint8_t *msg = NULL;
  static uint8_t *msg_ptr = NULL;
  static state_t state_reg = IDLE;

  switch (state_reg)
  {
    case IDLE:
      if (SOM == data)
      {
        //TODO: Cambio CMSIS_V2 msg = (uint8_t*)osPoolCAlloc(pdu_pool_id);
        msg = osMemoryPoolAlloc(pdu_pool_id, 0);
        if (NULL == msg)
        {
          message_error(msg);
          state_reg = IDLE;
        }
        msg_ptr = msg;
        msg_len = 0;

        osTimerStart(timeout_id, TIMEOUT_MS);
        state_reg = ID_RECEIVE;
      }
      break;

    case ID_RECEIVE:
      if (SOM == data || timeout)
      {
        timeout = false;
        message_error(msg);
        state_reg = IDLE;
      }
      else if (osTimerStop(timeout_id) == osOK)
      {
        if (isxdigit(data) != 0u && islower(data) == 0u)
        {
          *msg_ptr++ = data;
          osTimerStart(timeout_id, TIMEOUT_MS);

          state_reg = (ID_SIZE == ++msg_len) ? DATA_RECEIVE : ID_RECEIVE;
        }
        else
        {
          message_error(msg);
          state_reg = IDLE;
        }
      }
      else
      {
        message_error(msg);
        state_reg = IDLE;
      }
      break;

    case DATA_RECEIVE:
      if (SOM == data || MAX_MSG_SIZE < msg_len || timeout)
      {
        timeout = false;
        message_error(msg);
        state_reg = IDLE;
      }
      else if (osTimerStop(timeout_id) == osOK)
      {
        if (EOM == data)
        {
          new_message = check_crc(msg, msg_len);
          if(!new_message)
          {
            message_error(msg);
          }else{
            c2_create_sdu(msg);
          }
          state_reg = IDLE;
        }else{
          osTimerStart(timeout_id, TIMEOUT_MS);
          *msg_ptr++ = data;
          msg_len++;
        }
      }
      else
      {
        message_error(msg);
        state_reg = IDLE;
      }
      break;
    default:
      break;
  }
}

void c2_parser_init(void)
{
  //TODO: Setear los callback de la capa C1
 //c1_driver_init(&c2_parser_tx_cb , &c2_parser_rx_cb);
//  crc8_init();

 /* TODO: Cambio, el time se crea en el main, podria hacerse acá tambien
  timeout_id = osTimerCreate(osTimer(timeout), osTimerOnce, NULL);
  if(timeout_id == NULL)

  //TODO: Cambio CMSIS_V2, pdu_pool_id = osPoolCreate(osPool(pdu_pool));*/

  pdu_pool_id = osMemoryPoolNew(10, MAX_MSG_SIZE, NULL);
}

uint8_t *c2_create_sdu(uint8_t *msg)
{
  uint8_t *sdu = NULL;
  uint16_t len = strlen((const char *)msg);

  sdu = (uint8_t *)malloc(len - ID_SIZE - CRC_SIZE);

  memcpy(sdu, &msg[ID_SIZE], strlen((const char *)sdu));

  osMemoryPoolFree(pdu_pool_id, msg);
  return sdu;
}

bool c2_is_new_message(void)
{
  return new_message;
}

void c2_read_message(void)
{
  new_message = false;
}
void c2_parser_tx_cb(void)
{

}
/********************** end of file ******************************************/
