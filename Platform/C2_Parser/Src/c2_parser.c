/**
 * Copyright (c) May 21, 2022 Lautaro Vera <lautarovera93@gmail.com>.
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
 * @date    : May 21, 2022
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
    uint8_t *data; //[MAX_PAYLOAD_SIZE];
    uint16_t crc;
    uint8_t eom;
} msg_t;

/********************** internal functions declaration ***********************/
static void c2_parser_rx_cb(void);
static void c2_parser_tx_cb(void);
static void process_som(msg_t *msg);
static void process_eom(msg_t *msg);

/********************** internal data definition *****************************/

static osPoolDef (msg_pool, 10u, msg_t);
static osPoolId  (msg_pool_id);
//static msg_t *msg = NULL;

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static void process_som(msg_t *msg)
{
  if (NULL != msg)
  {
    (void)osPoolFree(msg_pool_id, msg);
  }

  msg = (msg_t*)osPoolCAlloc(msg_pool_id);
}

static void process_eom(msg_t *msg)
{
  if (NULL != msg)
  {
   // msg_ongoing = false;
  }
}

static void process_msg(uint8_t data, msg_t *msg)
{
 /* static uint8_t *msg_ptr = NULL != msg ? &msg->id : NULL;

  if (NULL != msg_ptr && msg_ptr < &msg->eom)
  {
    if (msg_ptr >= &msg->c && msg_ptr < &msg->crc)
    {
      *msg_ptr++ = data;
    }
    else
    {
      if (data >= '0' && data <= '9' || data >= 'A' && data <= 'Z')
      {
        *msg_ptr++ = data;
      }
      else
      {
        (void)osPoolFree(msg_pool_id, msg);
      }
    }
  }*/
}

static void c2_parser_tx_cb(void)
{

}

static void c2_parser_rx_cb(void)
{
  uint8_t data = c1_read_data();
  static msg_t *msg = NULL;

  switch(data)
  {
    case SOM:
      process_som(msg);
      //char_count = 0u;
      break;
    case EOM:
      //process_eom();
      break;
    default:
      process_msg(data, msg);
  }
}
/********************** external functions definition ************************/
//if (data >= '0' && data <= '9' || data >= 'A' && data <= 'Z')
//{
void c2_parser_init(void)
{
  c1_driver_init(&c2_parser_tx_cb , &c2_parser_rx_cb);    //TODO: revisar el tipo en las llamadas a las callback
  msg_pool_id = osPoolCreate(osPool(msg_pool));
}

void c2_parser_task(const void *argument)
{
  //parser_init();

  for(;;)
  {

  }
}


void c2_parser_error_handler(void)
{

}

/********************** end of file ******************************************/
