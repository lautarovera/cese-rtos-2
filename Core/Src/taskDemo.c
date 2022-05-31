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


#if( TASKS_SCOPE == TASKS_OUTSIDE_MAIN)
// ------ Private constants ----------------------------------------
#define SIZE_TEST 10 //TODO: Solo debug
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



  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;)
  {

  }
}

/* Task Function thread */
void task_test(void const *argument)
{
  /* The string to print out is passed in via the parameter.  Cast this to a
   character pointer. */
  char *pcTaskName;
  pcTaskName = (char*)argument;

 uint8_t myStr[SIZE_TEST] = {'(','1','2','3','4','5','6','7','8','9'};
 uint16_t counter = 0;

  uint32_t xLastExecutionTime = osKernelSysTick();
  const uint32_t xBlockPeriod = 100;
  /* Print out the name of this task. */
  vPrintString(pcTaskName);


  c2_parser_init();
  /* As per most tasks, this task is implemented in an infinite loop. */
  for (;;)
  {
    //sprintf(myStr,"%u", counter);
    //c1_driver_tx((uint8_t*)myStr);

    c2_parser_rx_cb(myStr[counter]);
    counter++;

    if(SIZE_TEST == counter){counter=1;}
    osDelayUntil( &xLastExecutionTime, xBlockPeriod );
  }
}
#endif

/*------------------------------------------------------------------*-
 ---- END OF FILE -------------------------------------------------
 -*------------------------------------------------------------------*/
