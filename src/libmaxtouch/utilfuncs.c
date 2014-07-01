//------------------------------------------------------------------------------
/// \file   utilfuncs.c
/// \brief  Utility functions for Linux maXTtouch app.
/// \author Iiro Valkonen.
//------------------------------------------------------------------------------
// Copyright 2011 Atmel Corporation. All rights reserved.
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
#include <string.h>
#include <ctype.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>

#include "libmaxtouch.h"
#include "utilfuncs.h"

#define BYTETOBINARYPATTERN "%d%d%d%d %d%d%d%d"
#define BYTETOBINARY(byte)  \
  (byte & 0x80 ? 1 : 0), \
  (byte & 0x40 ? 1 : 0), \
  (byte & 0x20 ? 1 : 0), \
  (byte & 0x10 ? 1 : 0), \
  (byte & 0x08 ? 1 : 0), \
  (byte & 0x04 ? 1 : 0), \
  (byte & 0x02 ? 1 : 0), \
  (byte & 0x01 ? 1 : 0)

//******************************************************************************
/// \brief Print out info block
void mxt_print_info_block(struct mxt_device *mxt)
{
  int i;
  int report_id = 1;
  int report_id_start, report_id_end;
  struct mxt_object obj;
  struct mxt_id_info *id = mxt->info.id;

  /* Show the Version Info */
  printf("\nFamily: %u Variant: %u Firmware V%u.%u.%02X Objects: %u\n",
         id->family, id->variant,
         id->version >> 4,
         id->version & 0x0F,
         id->build, id->num_objects);

  printf("Matrix size: X%uY%u\n",
         id->matrix_x_size, id->matrix_y_size);
  /* Show the CRC */
  printf("Information Block CRC: 0x%06X\n\n", mxt->info.crc);

  /* Show the object table */
  printf("Type Start Size Instances ReportIds Name\n");
  printf("-----------------------------------------------------------------\n");
  for (i = 0; i < id->num_objects; i++) {
    obj = mxt->info.objects[i];

    if (obj.num_report_ids > 0) {
      report_id_start = report_id;
      report_id_end = report_id_start + obj.num_report_ids * MXT_INSTANCES(obj) - 1;
      report_id = report_id_end + 1;
    } else {
      report_id_start = 0;
      report_id_end = 0;
    }

    printf("T%-3u %4u  %4u    %2u       %2u-%-2u   ",
           obj.type,
           mxt_get_start_position(obj, 0),
           MXT_SIZE(obj),
           MXT_INSTANCES(obj),
           report_id_start, report_id_end);

    const char *obj_name = mxt_get_object_name(obj.type);
    if (obj_name)
      printf("%s\n", obj_name);
    else
      printf("UNKNOWN_T%d\n", obj.type);
  }

  printf("\n");
}

//******************************************************************************
/// \brief Convert object type to object name
/// \return null terminated string, or NULL for object not found
const char *mxt_get_object_name(uint8_t objtype)
{
  switch(objtype) {
    OBJECT_LIST(F_SWITCH)

  default:
    return NULL;
  }
}
//******************************************************************************
/// \brief Menu function to read values from object
/// \return #mxt_rc
int mxt_read_object(struct mxt_device *mxt, uint16_t object_type,
                    uint8_t instance, uint16_t address,
                    size_t count, bool format)
{
  uint8_t *databuf;
  uint16_t object_address = 0;
  uint16_t i;
  int ret;

  if (object_type > 0) {
    object_address = mxt_get_object_address(mxt, object_type, instance);
    if (object_address == OBJECT_NOT_FOUND) {
      printf("No such object\n");
      return MXT_ERROR_OBJECT_NOT_FOUND;
    }

    mxt_dbg(mxt->ctx, "T%u address:%u offset:%u", object_type,
            object_address, address);
    address = object_address + address;

    if (count == 0) {
      count = mxt_get_object_size(mxt, object_type);
    }
  } else if (count == 0) {
    mxt_err(mxt->ctx, "No length information");
    return MXT_ERROR_BAD_INPUT;
  }

  databuf = (uint8_t *)calloc(count, sizeof(uint8_t));
  if (databuf == NULL) {
    mxt_err(mxt->ctx, "Memory allocation failure");
    return MXT_ERROR_NO_MEM;
  }

  ret = mxt_read_register(mxt, databuf, address, count);
  if (ret) {
    printf("Read error\n");
    goto free;
  }

  if (format) {
    if (object_type > 0) {
      const char *obj_name = mxt_get_object_name(object_type);
      if (obj_name)
        printf("%s\n\n", obj_name);
      else
        printf("UNKNOWN_T%d\n\n", object_type);
    }

    for (i = 0; i < count; i++) {
      printf("%02d:\t0x%02X\t%3d\t" BYTETOBINARYPATTERN "\n",
             address - object_address + i,
             databuf[i],
             databuf[i],
             BYTETOBINARY(databuf[i]));
    }
  } else {
    for (i = 0; i < count; i++) {
      printf("%02X ", databuf[i]);
    }

    printf("\n");
  }

  ret = MXT_SUCCESS;

free:
  free(databuf);
  return ret;
}

//******************************************************************************
/// \brief Convert hex nibble to digit
static char to_digit(char hex)
{
  char decimal;

  if (hex >= '0' && hex <= '9')
    decimal = hex - '0';
  else if (hex >= 'A' && hex <= 'F')
    decimal = hex - 'A' + 10;
  else if (hex >= 'a' && hex <= 'f')
    decimal = hex - 'a' + 10;
  else
    decimal = 0;

  return decimal;
}

//******************************************************************************
/// \brief Convert ASCII buffer containing hex digits to binary
/// \return #mxt_rc
int mxt_convert_hex(char *hex, unsigned char *databuf,
                    uint16_t *count, unsigned int buf_size)
{
  unsigned int pos = 0;
  uint16_t datapos = 0;
  char highnibble;
  char lownibble;

  while (1) {
    highnibble = *(hex + pos);
    lownibble = *(hex + pos + 1);

    /* end of string */
    if (highnibble == '\0' || highnibble == '\n')
      break;

    /* uneven number of hex digits */
    if (lownibble == '\0' || lownibble == '\n')
      return MXT_ERROR_BAD_INPUT;

    *(databuf + datapos) = (to_digit(highnibble) << 4)
                           | to_digit(lownibble);
    datapos++;

    pos += 2;
    if (pos > buf_size)
      return MXT_ERROR_NO_MEM;
  }

  *count = datapos;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Output timestamp to stream with millisecond accuracy
/// \param stream pointer to FILE object
/// \param date   output date, true or false
/// \return #mxt_rc
int mxt_print_timestamp(FILE *stream, bool date)
{
  struct timeval tv;
  time_t nowtime;
  struct tm *nowtm;
  char tmbuf[64];
  int ret;

  gettimeofday(&tv, NULL);
  nowtime = tv.tv_sec;
  nowtm = localtime(&nowtime);

  if (date) {
    strftime(tmbuf, sizeof(tmbuf), "%c", nowtm);
    ret = fprintf(stream, "%s", tmbuf);
  } else {
    strftime(tmbuf, sizeof(tmbuf), "%H:%M:%S", nowtm);
    ret = fprintf(stream, "%s.%06ld", tmbuf, tv.tv_usec);
  }

  return (ret < 0) ? MXT_ERROR_IO : MXT_SUCCESS;
}
