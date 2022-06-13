/*
 * Copyright (c) 2022 Alejandro Virgillo <avirgillo.95@gmail.com>.
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
 * @file   : c3_app.c
 * @date   : Jun 11, 2022
 * @author : Alejandro Virgillo <avirgillo.95@gmail.com>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/

#include "c3_app.h"
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"

/********************** macros and definitions *******************************/

#define WORD_MAX 15
#define WORD_MIN 1
#define CHARS_PER_WORD_MAX 10
#define CHARS_PER_WORD_MIN 1
#define OPCODE_INDEX 0
#define MAX_STRING_LENGTH 200

#define CHAR_LOWERCASE_CHECK(c) (c >= 'a' && c <= 'z')
#define CHAR_UPPERCASE_CHECK(c) (c >= 'A' && c <= 'Z')
#define CHAR_SPACE_CHECK(c) (c == ' ')
#define CHAR_UNDERSCORE_CHECK(c) (c == '_')

/********************** internal data declaration ****************************/

typedef enum
{
  CAMEL_CASE = 'C', PASCAL_CASE = 'P', SNAKE_CASE = 'S'
} action_codes_t;

typedef enum
{
  ERROR_INVALID_DATA = 0, ERROR_INVALID_OPCODE, ERROR_SYSTEM, NO_ERROR
} error_codes_t;

typedef enum
{
  CHAR_TYPE_LOWERCASE = 0, CHAR_TYPE_UPPERCASE, CHAR_TYPE_SPACE, CHAR_TYPE_UNDERSCORE, CHAR_TYPE_INVALID
} char_types_t;

typedef enum
{
  CHAR_NEXT_LOWERCASE = 0, CHAR_NEXT_UPPERCASE, CHAR_NEXT_SEPARATOR, CHAR_NEXT_INVALID
} char_next_t;

typedef struct
{
  action_codes_t action;
  error_codes_t error;
  char_types_t last_char;
  char_next_t next_char;
  char in_string[MAX_STRING_LENGTH];
  char out_string[MAX_STRING_LENGTH];
  uint8_t separators_indexes[WORD_MAX];
  uint8_t uppercase_indexes[WORD_MAX];
} state_t;

/********************** internal functions declaration ***********************/

static void state_reset(state_t *fsm);

static error_codes_t process_string(state_t *in_state);

static void process_response(error_codes_t error, char *response_string);

static void send_response(char *response_string);

static error_codes_t modify_string(
    char *string_in,
    uint8_t length,
    char *string_out,
    uint8_t *uppercase_indexes,
    uint8_t *separator_indexes,
    action_codes_t opcode);

static error_codes_t verify_invalid_chars(
    char *string,
    uint8_t length,
    uint8_t *uppercase_indexes,
    uint8_t *separator_indexes);

static char_types_t get_char_type(char char_to_check);

static char_next_t compare_char_types(char char_to_check, char_types_t previous_char);

static error_codes_t process_opcode(char *string_in, action_codes_t *opcode_out);

/********************** internal data definition *****************************/

/*
static osPoolDef(pdu_pool, 1u, MAX_STRING_LENGTH);
static osPoolId pdu_pool_id;

static osPoolDef(msg_response_pool, 1u, MAX_STRING_LENGTH);
static osPoolId msg_response_pool_id;
*/

/********************** external data definition *****************************/

/********************** internal functions definition ************************/

static void state_reset(state_t *fsm)
{
  fsm->action = CAMEL_CASE;
  fsm->error = NO_ERROR;
  fsm->last_char = CHAR_TYPE_UNDERSCORE;
  fsm->next_char = CHAR_NEXT_INVALID;
  for (uint8_t index = 0; index < MAX_STRING_LENGTH; index++)
  {
    fsm->in_string[index] = '\0';
    fsm->out_string[index] = '\0';
  }
  for (uint8_t index = 0; index < WORD_MAX; index++)
  {
    fsm->separators_indexes[index] = '\0';
    fsm->uppercase_indexes[index] = '\0';
  }
}

static error_codes_t process_string(state_t *in_state)
{
  error_codes_t error_code = NO_ERROR;
  if ((error_code = process_opcode(in_state->in_string, &in_state->action)) == NO_ERROR)
  {
    if ((error_code = verify_invalid_chars(in_state->in_string + 1, strlen(in_state->in_string) - 1,
                                           in_state->uppercase_indexes, in_state->separators_indexes)) == NO_ERROR)
    {
      error_code = modify_string(in_state->in_string + 1, strlen(in_state->in_string) - 1, in_state->out_string,
                                 in_state->uppercase_indexes, in_state->separators_indexes, in_state->action);
    }
  }
  return error_code;
}

static void process_response(error_codes_t error, char *response_string)
{
  switch (error)
  {
    case ERROR_INVALID_DATA:
    {
      strcpy(response_string, "E00");
      break;
    }
    case ERROR_INVALID_OPCODE:
    {
      strcpy(response_string, "E01");
      break;
    }
    case ERROR_SYSTEM:
    {
      strcpy(response_string, "E02");
      break;
    }
    default:
    {
      break;
    }
  }
}

static void send_response(char *response_string)
{
  //TODO send response to c2
  //HABRIA QUE RESERVAR LA MEMORIA Y PASAR POR COLA
}

static error_codes_t modify_string(
    char *string_in,
    uint8_t length,
    char *string_out,
    uint8_t *uppercase_indexes,
    uint8_t *separator_indexes,
    action_codes_t opcode)
{
  uint8_t separator_index = 0;
  uint8_t uppercase_index = 0;
  //Se modifica el primer caracter segun el opcode
  switch (opcode)
  {
    case CAMEL_CASE:
    {
      *string_out++ = tolower(*string_in);
      break;
    }
    case SNAKE_CASE:
    {
      *string_out++ = tolower(*string_in);
      break;
    }
    case PASCAL_CASE:
    {
      *string_out++ = toupper(*string_in);
      break;
    }
  }
  //Se hace el chequeo si el primer caracter es un separador(invalido) o una mayuscula y se ajusta el contador acorde
  if (separator_indexes[separator_index] == 0)
  {
    separator_index++;
  }
  if (uppercase_indexes[uppercase_index] == 0)
  {
    uppercase_index++;
  }
  //Se modifica todo el string (Que debiera ya estar validado) segun el opcode
  for (uint8_t index = 1; index < length; index++)
  {
    if (index == separator_indexes[separator_index] || index == uppercase_indexes[uppercase_index])
    {
      switch (opcode)
      {
        case CAMEL_CASE:
        {
          if (index == separator_indexes[separator_index])
          {
            index++;
            *string_out++ = toupper(string_in[index]);
            separator_index++;
          }
          else if (index == uppercase_indexes[uppercase_index])
          {
            *string_out++ = string_in[index];
            uppercase_index++;
          }
          break;
        }
        case SNAKE_CASE:
        {
          if (index == separator_indexes[separator_index])
          {
            *string_out++ = '_';
            index++;
            *string_out++ = tolower(string_in[index]);
            separator_index++;
          }
          else if (index == uppercase_indexes[uppercase_index])
          {
            *string_out++ = '_';
            *string_out++ = tolower(string_in[index]);
            uppercase_index++;
          }
          break;
        }
        case PASCAL_CASE:
        {
          if (index == separator_indexes[separator_index])
          {
            index++;
            *string_out++ = toupper(string_in[index]);
            separator_index++;
          }
          else if (index == uppercase_indexes[uppercase_index])
          {
            *string_out++ = string_in[index];
            uppercase_index++;
          }
          break;
        }
      }
    }
    else
    {
      *string_out++ = string_in[index];
    }
  }
  return NO_ERROR;
}

static error_codes_t verify_invalid_chars(
    char *string,
    uint8_t length,
    uint8_t *uppercase_indexes,
    uint8_t *separator_indexes)
{
  char_types_t char_type = CHAR_TYPE_UNDERSCORE;
  char_next_t char_next;
  uint8_t uppercase_index = 0;
  uint8_t separator_index = 0;
  for (uint8_t index = 0; index < length; index++)
  {
    //Se hace la comparacion de caracteres contiguos para saber si hay doble '_' o doble ' '
    char_next = compare_char_types(string[index], char_type);
    if (char_next == CHAR_NEXT_INVALID)
    {
      return ERROR_INVALID_DATA;
    }
    //Se mira el caracter actual y se valida
    char_type = get_char_type(string[index]);
    if (CHAR_TYPE_INVALID == char_type)
    {
      return ERROR_INVALID_DATA;
    }
    //Se cuenta la cantidad de palabras y se discrimina por palabras separadas por separados y por mayuscula
    else if (CHAR_TYPE_UPPERCASE == char_type)
    {
      if (uppercase_index >= WORD_MAX)
      {
        return ERROR_INVALID_DATA;
      }
      else
      {
        uppercase_indexes[uppercase_index++] = index;
      }
    }
    else if (CHAR_TYPE_UNDERSCORE == char_type || CHAR_TYPE_SPACE == char_type)
    {
      if (separator_index >= WORD_MAX)
      {
        return ERROR_INVALID_DATA;
      }
      else
      {
        separator_indexes[separator_index++] = index;
      }
    }
  }
  //Se revisa la cantidad de palabras
  if (separator_index + uppercase_index >= WORD_MAX)
  {
    return ERROR_INVALID_DATA;
  }
  //Se revisa que el ultimo caracter no sea '_' o ' '
  if (char_type == CHAR_TYPE_UNDERSCORE && char_type == CHAR_TYPE_SPACE)
  {
    return ERROR_INVALID_DATA;
  }
  return NO_ERROR;
}

static char_types_t get_char_type(char char_to_check)
{
  uint8_t char_type = CHAR_TYPE_INVALID;
  if (CHAR_LOWERCASE_CHECK(char_to_check))
  {
    char_type = CHAR_TYPE_LOWERCASE;
  }
  else if (CHAR_UPPERCASE_CHECK(char_to_check))
  {
    char_type = CHAR_TYPE_UPPERCASE;
  }
  else if (CHAR_SPACE_CHECK(char_to_check))
  {
    char_type = CHAR_TYPE_SPACE;
  }
  else if (CHAR_UNDERSCORE_CHECK(char_to_check))
  {
    char_type = CHAR_TYPE_UNDERSCORE;
  }
  return char_type;
}

static char_next_t compare_char_types(char char_to_check, char_types_t previous_char)
{
  char_next_t next_char = CHAR_NEXT_INVALID;
  char_types_t char_to_check_type = CHAR_TYPE_INVALID;
  char_to_check_type = get_char_type(char_to_check);
  if (CHAR_TYPE_INVALID != char_to_check_type && CHAR_TYPE_INVALID != previous_char)
  {
    if ((previous_char == CHAR_TYPE_SPACE || previous_char == CHAR_TYPE_UNDERSCORE)
        && (char_to_check_type == CHAR_TYPE_SPACE || char_to_check_type == CHAR_TYPE_UNDERSCORE))
    {
      next_char = CHAR_NEXT_INVALID;
    }
    else if (CHAR_TYPE_SPACE == char_to_check_type || CHAR_TYPE_UNDERSCORE == char_to_check_type)
    {
      next_char = CHAR_NEXT_SEPARATOR;
    }
    else if (CHAR_TYPE_UPPERCASE == char_to_check_type)
    {
      next_char = CHAR_NEXT_UPPERCASE;
    }
    else if (CHAR_TYPE_LOWERCASE == char_to_check_type)
    {
      next_char = CHAR_NEXT_LOWERCASE;
    }
  }
  else
  {
    next_char = CHAR_NEXT_INVALID;
  }
  return next_char;
}

static error_codes_t process_opcode(char *string_in, action_codes_t *opcode_out)
{
  error_codes_t error_code = ERROR_INVALID_OPCODE;
  switch (string_in[OPCODE_INDEX])
  {
    case CAMEL_CASE:
    {
      error_code = NO_ERROR;
      *opcode_out = CAMEL_CASE;
      break;
    }
    case PASCAL_CASE:
    {
      error_code = NO_ERROR;
      *opcode_out = PASCAL_CASE;
      break;
    }
    case SNAKE_CASE:
    {
      error_code = NO_ERROR;
      *opcode_out = SNAKE_CASE;
      break;
    }
  }
  return error_code;
}
/********************** external functions definition ************************/

void init_c3(void)
{
  //TODO init C3 and initialize C2
  //pdu_pool_id = osPoolCreate(osPool(pdu_pool));
  //msg_response_pool_id = osPoolCreate(osPool(msg_response_pool));
}

void c3_task(void)
{

  state_t fsm;
  error_codes_t error = NO_ERROR;
  char error_response[10];

  //TODO determine how should c3 work
  while (1)
  {
    //queue_receive ACA HABRIA QUE RECIBIR DE LA COLA DE RTOS
    //LIBERAR MEMORIA
    state_reset(&fsm);
    error = process_string(&fsm);
    if (NO_ERROR != error)
    {
      process_response(error, error_response);
      send_response(error_response);
    }
    else{
      send_response(fsm.out_string);
    }
  }

}
/********************** end of file ******************************************/
