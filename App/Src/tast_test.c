/*
 * Copyright (c) YEAR NOMBRE <MAIL>.
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
 * @file   : tast_test.c
 * @date   : Jun 11, 2022
 * @author : NOMBRE <MAIL>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "supportingFunctions.h"
#include "c2_parser.h"
#include "cmsis_os2.h"
/********************** macros and definitions *******************************/
#define DELAY_MS 2
#define TEST_X ( 0 )

#if( TEST_X == 0 )
// Paquete correcto
#define SIZE_PACKET  13
static const char *packet_test = "(2546PHOLA5F)";
// Paquete con CRC erróneo
static const char *packet_test2 = "(2546PHOLA48)";
/* Funciona bien
 new_message = true */
#endif

#if( TEST_X == 1 )
// Paquete con CRC erróneo
#define SIZE_PACKET  13
static const char *packet_test = "(2546PHOLA48)";
/* Detecta CRC erróneo
 new_message = false */
#endif

#if( TEST_X == 2 )
// Paquete incompleto sin EOM
#define SIZE_PACKET  3
static const char *packet_test = "(52";
/* Termina por Timeout
 * La maquina de estados se actualiza solo cuando recibe un dato desde la uart,
 * al producirse el timeout debe llevarse la FSM al estado IDLE, descartando lo
 * recibido *
 * */

#endif
/********************** internal data declaration ****************************/

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/

/********************** external data definition *****************************/
extern osTimerId_t timeoutHandle;
extern osMemoryPoolId_t pdu_pool_id;
/********************** internal functions definition ************************/

/********************** external functions definition ************************/
void task_test(void const *argument)
{
  /* Print out the name of this task. */
  vPrintString("Inicio tarea de test \n\n");
  uint8_t *tmp = NULL ,i=0 , k=5;


  c2_parser_init();

  /*
   * Envío de paquete correcto 10 paquetes
   * en el quinto se intercala una demora
   * octavo CRC inválido
   */
  for (k = 0; k < SIZE_PACKET; k++)
  {
    tmp = (uint8_t*)packet_test;
    if (k == 8)
    {
      tmp = (uint8_t*)packet_test2;
    }

    for (i = 0; i < SIZE_PACKET; i++)
    {
      c2_parser_rx_cb(*tmp++);
      osDelay(DELAY_MS);
      // Se intercala demora
      if (k == 5)
      {
        osDelay(3 * DELAY_MS);
      }
    }

    osDelay(10 * DELAY_MS);
    if (c2_is_new_message())
    {
      vPrintString("Mensaje correcto \n");
      c2_read_message();
    }
    else
    {
      vPrintString("Error detectado \n");
    }

    osDelay(DELAY_MS);
  }
  //************************

  vPrintString("\nFin de test \n");

  while(1){
    osDelay(10*DELAY_MS);
  }
}
/********************** end of file ******************************************/
