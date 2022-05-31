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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os.h"
#include "c1_driver.h"

/********************** macros and definitions *******************************/

#define SOM                     '('
#define EOM                     ')'
#define MAX_MSG_SIZE            200u
#define METADA_SIZE             9u
#define MAX_PAYLOAD_SIZE        (MAX_MSG_SIZE - METADA_SIZE)

/********************** internal data declaration ****************************/

typedef struct
{
    uint8_t som;
    uint32_t id;
    uint8_t c;
    uint8_t data[MAX_PAYLOAD_SIZE];
    uint16_t crc;
    uint8_t eom;
} msg_t;

typedef enum {IDLE , ID_RECEIVE , DATA_RECEIVE} state_t;

/********************** internal functions declaration ***********************/
static void message_error(msg_t *msg);
static void check_crc(msg_t *msg);
static msg_t *init_message(void);
/********************** internal data definition *****************************/

static osPoolDef (msg_pool, 10u, msg_t);
static osPoolId  (msg_pool_id);
/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static msg_t *init_message(void)
{
  msg_t *msg = NULL;

  msg = (msg_t*)osPoolCAlloc(msg_pool_id);

  return msg;
}

static void check_crc(msg_t *msg){
  //TODO: chequear CRC(últimos dos bytes recibidos)
}

static void message_error(msg_t *msg){
  //TODO: liberar memoria dinámica
  (void)osPoolFree(msg_pool_id, msg);
}

void c2_parser_rx_cb(uint8_t data)
{
  static uint8_t *msg_ptr;
  static msg_t *msg;
  static int count_id = 0, count_data = 0;
  static state_t state_reg = IDLE;

  switch (state_reg)
  {
    case IDLE:
      if (SOM == data)
      {
        msg = init_message();
        if (NULL == msg){
          // error memoria dinámica
        }
        msg_ptr = (uint8_t*)msg;
        *msg_ptr++ = data;
        count_id = 0;
        state_reg = ID_RECEIVE;
      }
      break;

    case ID_RECEIVE:
      if (SOM == data)
      {
        message_error(msg);
        state_reg = IDLE;
      }
      if (5 == count_id)
      {
        if ('C' == data || 'P' == data)
        {           //Valid charter c
          count_data = 0;
          state_reg = DATA_RECEIVE;
        }
        else
        {
          message_error(msg);
          state_reg = IDLE;
        }
      }
      else
      {
        *msg_ptr++ = data;
        count_id++;
      }
      break;

    case DATA_RECEIVE:
      if (EOM == data)
      {
        check_crc(msg);
      }
      else
      {
        if ((data >= '0' && data <= '9') || (data >= 'A' && data <= 'Z'))
        {
          *msg_ptr++ = data;
          count_data++;
        }else{
          message_error(msg);
          state_reg = IDLE;
        }
      }
      break;
    default:
      break;
  }
}
/********************** external functions definition ************************/
void c2_parser_init(void)
{
  //TODO: Setear los callback de la capa C1
  //c1_driver_init(tx_cb, rx_cb)
  msg_pool_id = osPoolCreate(osPool(msg_pool));
}
/********************** end of file ******************************************/
