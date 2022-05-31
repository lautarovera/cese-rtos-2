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

typedef enum {IDLE , ID_receive , DATA_receive} state_t;

/********************** internal functions declaration ***********************/
static void message_error(void);
static void check_crc(void);
static msg_t *init_message(void);
/********************** internal data definition *****************************/

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static msg_t *init_message(void)
{
  static msg_t msg_str;
  //TODO: implementar alocación de memoria dinámica
  msg_t *msg = &msg_str;
  return msg;
}

static void check_crc(void){}

static void message_error(void){}

void c2_parser_rx_cb(uint8_t data)
{
  static uint8_t *msg_ptr;

  static int count_id = 0, count_data = 0;
  static state_t state_reg = IDLE;

  switch (state_reg)
  {
    case IDLE:
      if (SOM == data)
      {
        msg_t *msg = init_message();
        msg_ptr = (uint8_t*)msg;
        *msg_ptr++ = data;
        count_id = 0;
        state_reg = ID_receive;
      }
      break;
    case ID_receive:
      if (SOM == data){
        message_error();
        state_reg = IDLE;
      }

      *msg_ptr++ = data;
      count_id++;
      if(5 == count_id){
        if('C'||'P' == data){           //Valid charter c
          count_data = 0;
          state_reg = DATA_receive;
        }else{
          message_error();
          state_reg = IDLE;
        }
      }
      break;
    case DATA_receive:
      if (EOM == data)
      {
        check_crc();
      }
      else
      {
        if ((data >= '0' && data <= '9') || (data >= 'A' && data <= 'Z'))
        {
          *msg_ptr++ = data;
          count_data++;
        }
      }
      break;
    default:
      break;
  }
}
/********************** external functions definition ************************/

/********************** end of file ******************************************/
