/* Copyright 2020, Juan Manuel Cruz.
 * All rights reserved.
 *
 * This file is part of Project => freertos_book_Example1_6
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*--------------------------------------------------------------------*-

 taskDemo.c (Released 2022-05)

 --------------------------------------------------------------------

 task file for FreeRTOS - Event Driven System (EDS) - Project for
 STM32F429ZI_NUCLEO_144.

 See readme.txt for project information.

 -*--------------------------------------------------------------------*/

// ------ Includes -------------------------------------------------
/* Project includes. */
#include <c2_parser.h>
#include "main.h"
#include "cmsis_os.h"

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Demo includes. */
#include "supportingFunctions.h"

/* Application includes. */
#include "taskDemo.h"

#define DELAY_MS 2

#if( TASKS_SCOPE == TASKS_OUTSIDE_MAIN)
// ------ Private constants ----------------------------------------

#define TEST_X ( 0 )

#if( TEST_X == 0 )
// Paquete correcto
#define SIZE_PACKET  13
static const char *packet_test = "(2546PHOLA5F)";
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
/* Termina por Timeout */

#endif


// ------ Private variables ----------------------------------------

// ------ Public functions prototypes ------------------------------

/* Task Function thread */
void vTaskDemo(void const *argument);
void task_test(void const *argument);
// ------ Public functions -----------------------------------------

/*------------------------------------------------------------------*/
/* Task Function thread */
void vTaskDemo(void const *argument)
{
  /* The string to print out is passed in via the parameter.  Cast this to a
   character pointer. */
  char *pcTaskName;
  pcTaskName = (char*)argument;

  /* Print out the name of this task. */
  vPrintString(pcTaskName);
  uint8_t *tmp = NULL ,i=0 ;

  tmp = (uint8_t*)packet_test;
 // uint32_t xLastExecutionTime = osKernelSysTick();
 // const uint32_t xBlockPeriod = 100;

  c2_parser_init();

  /* As per most tasks, this task is implemented in an infinite loop. */
  for (i=0 ; i < SIZE_PACKET ; i++)
  {

    c2_parser_rx_cb(*tmp++);

 /*   if(i==6){
      osDelay(DELAY_MS);
      osDelay(DELAY_MS);
    }
    */
    osDelay(DELAY_MS);
  }
  osDelay(4*DELAY_MS);

  if(c2_is_new_message())
  {

    vPrintString("Mensaje correcto \n");
  }else{
    vPrintString("Error detectado \n");
  }

  while(1){} // Fin del programa

}

#endif

/*------------------------------------------------------------------*-
 ---- END OF FILE -------------------------------------------------
 -*------------------------------------------------------------------*/
