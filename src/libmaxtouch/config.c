//------------------------------------------------------------------------------
/// \file   config.c
/// \brief  Configuration file handling
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2012 Atmel Corporation. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
//    2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY ATMEL ''AS IS'' AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
// EVENT SHALL ATMEL OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
// OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "log.h"
#include "libmaxtouch.h"
#include "info_block.h"

//******************************************************************************
/// \brief  Save configuration to file
/// \return 0 = success, negative = fail
int mxt_save_config_file(const char *cfg_file)
{
  int obj_idx, i, instance, num_bytes;
  uint8_t *temp;
  object_t *object;
  FILE *fp;
  int retval;

  LOG(LOG_INFO, "Opening config file %s...", cfg_file);

  fp = fopen(cfg_file, "w");

  fprintf(fp, "OBP_RAW V1\n");

  fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X\n",
          info_block.id->family_id, info_block.id->variant_id,
          info_block.id->version, info_block.id->build,
          info_block.id->matrix_x_size, info_block.id->matrix_y_size,
          info_block.id->num_declared_objects);

  fprintf(fp, "%06X\n", info_block_crc(info_block.crc));

  /* can't read object table CRC at present */
  fprintf(fp, "000000\n");

  for (obj_idx = 0; obj_idx < info_block.id->num_declared_objects; obj_idx++)
  {
    object = &(info_block.objects[obj_idx]);
    num_bytes = object->size + 1;

    temp = (uint8_t *)malloc(sizeof(char)*(num_bytes));
    if (temp == NULL)
    {
      LOG(LOG_ERROR, "Failed to allocate memory");
      retval = -1;
      goto close;
    }

    for (instance = 0; instance < object->instances + 1; instance++)
    {
      fprintf(fp, "%04X %04X %04X", object->object_type, instance, num_bytes);

      mxt_read_register(temp,
                       get_start_position(*object, instance),
                       num_bytes);

      for (i=0; i< num_bytes; i++)
      {
        fprintf(fp, " %02X", *(temp + i));
      }

      fprintf(fp, "\n");
    }

    free(temp);
  }

  retval = 0;

close:
  fclose(fp);
  return retval;
}

//******************************************************************************
/// \brief  Load configuration file
/// \return 0 = success, negative = fail
int mxt_load_config_file(const char *cfg_file)
{
  FILE *fp;
  uint8_t *mem;
  int c;

  char object[255];
  char tmp[255];
  char *substr;
  int object_id;
  int instance;
  int object_address;
  uint16_t expected_address;
  int object_size;
  uint8_t expected_size;
  int data;
  int file_read = 0;
  bool ignore_line = false;

  int i, j;
  int bytes_read;

  mem = malloc(255);
  if (mem == NULL) {
    LOG(LOG_ERROR, "Error allocating memory");
    return -1;
  }

  LOG(LOG_INFO, "Opening config file %s...", cfg_file);

  fp = fopen(cfg_file, "r");

  if (fp == NULL)
  {
    LOG(LOG_ERROR, "Error opening %s!", cfg_file);
    return -1;
  }

  while(!file_read)
  {
    /* First character is expected to be '[' - skip empty lines and spaces  */
    c = getc(fp);
    while((c == '\n') || (c == '\r') || (c == 0x20))
    {
      c = getc(fp);
    }

    if (c != '[')
    {
      if (c == EOF)
        break;

      /* If we are ignoring the current section then look for the next section */
      if (ignore_line)
      {
        continue;
      }

      LOG(LOG_ERROR, "Parse error: expected '[', read ascii char %c!", c);
      return -1;
    }

    if (fscanf(fp, "%s", object) != 1)
    {
      printf("Parse error\n");
      return -1;
    }

    /* Ignore the comments and file header sections */
    if (strncmp(object, "COMMENTS", 8) == 0
        || strncmp(object, "VERSION_INFO_HEADER", 19) == 0
        || strncmp(object, "APPLICATION_INFO_HEADER", 23) == 0)
    {
      ignore_line = true;
      LOG(LOG_INFO, "Skipping %s", object);
      continue;
    }
    ignore_line = false;
    if (fscanf(fp, "%s", tmp) != 1)
    {
      printf("Parse error\n");
      return -1;
    }

    if (strcmp(tmp, "INSTANCE"))
    {
      LOG(LOG_ERROR, "Parse error, expected INSTANCE");
      return(-1);
    }
    if (fscanf(fp, "%d", &instance) != 1)
    {
      printf("Parse error\n");
      return -1;
    }

    /* Read rest of header section */
    while(c != ']')
    {
      c = getc(fp);
      if (c == '\n')
      {
        LOG(LOG_ERROR, "Parse error, expected ] before end of line");
        return(-1);
      }
    }

    while(c != '\n')
    {
      c = getc(fp);
    }

    while ((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    if (fscanf(fp, "%d", &object_address) != 1)
    {
      printf("Parse error\n");
      return -1;
    }
    c = getc(fp);
    while((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    if (fscanf(fp, "%d", &object_size) != 1)
    {
      printf("Parse error\n");
      return -1;
    }

    c = getc(fp);


    /* Find object type ID number at end of object string */
    substr = strrchr(object, '_');
    if (substr == NULL || (*(substr + 1) != 'T'))
    {
      LOG(LOG_ERROR, "Parse error, could not find T number in %s", object);
      return -1;
    }

    if (sscanf(substr + 2, "%d", &object_id) != 1)
    {
      LOG(LOG_ERROR, "Unable to get object type ID for %s", object);
      return -1;
    }

    LOG(LOG_DEBUG, "%s T%u OBJECT_ADDRESS=%d OBJECT_SIZE=%d",
        object, object_id, object_address, object_size);

    /* Check the address of the object */
    expected_address = get_object_address((uint8_t)object_id, (uint8_t)instance);
    if (object_address != (int)expected_address)
    {
      LOG
      (
        LOG_WARN,
        "Address of %s in config file (0x%04X) does not match chip (0x%04X)",
        object, object_address, expected_address
      );

      object_address = expected_address;
    }

    /* Check the size of the object */
    expected_size = get_object_size((uint8_t)object_id);
    if (object_size != (int)expected_size)
    {
      LOG
      (
        LOG_WARN,
        "Size of %s in config file (%d bytes) does not match chip (%d bytes)",
        object, object_size, expected_size
      );
    }

    LOG(LOG_VERBOSE, "Writing object of size %d at address %d...", object_size,
         object_address);

    for (j = 0; j < object_size; j++) {
      *(mem + j) = 0;
    }

    bytes_read = 0;
    while (bytes_read < object_size)
    {
      /* Find next line, check first character valid and rewind */
      c = getc(fp);
      while((c == '\n') || (c == '\r') || (c == 0x20)) {
        c = getc(fp);
      }
      fseek(fp, -1, SEEK_CUR);
      if (c == '[') {
        LOG(LOG_WARN, "Skipping %d bytes at end of T%u", object_size - bytes_read, object_id);
        break;
      }

      /* Read address (discarded as we don't really need it) */
      if (fscanf(fp, "%d", &i) != 1)
      {
        printf("Parse error\n");
        return -1;
      }

      /* Read byte count of this register (max 2) */
      if (fscanf(fp, "%d", &i) != 1)
      {
        printf("Parse error\n");
        return -1;
      }

      while((c != '=') && (c != EOF))
      {
        c = getc(fp);
      }
      if (fscanf(fp, "%d", &data) != 1)
      {
        printf("Parse error\n");
        return -1;
      }
      c = getc(fp);

      if (i == 1) {
        *(mem + bytes_read) = (char) data;
        bytes_read++;
      } else if (i == 2) {
        *(mem + bytes_read) = (char) data & 0xFF;
        bytes_read++;
        *(mem + bytes_read) = (char) ((data >> 8) & 0xFF);
        bytes_read++;
      } else {
        LOG(LOG_ERROR, "Only 16-bit / 8-bit config values supported!");
        return -1;
      }
    }

    if (mxt_write_register(mem, object_address, object_size) < 0)
    {
      LOG(LOG_ERROR, "Error writing to mxt!");
      return -1;
    }
  }
  return 0;
}
