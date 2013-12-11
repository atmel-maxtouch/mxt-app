//------------------------------------------------------------------------------
/// \file   msg.c
/// \brief  Message handling functions
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

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "libmaxtouch.h"
#include "msg.h"

char msg_string[255];
unsigned char databuf[20];

//******************************************************************************
/// \brief  Get number of messages
/// \return Number of messages or negative error
int t44_get_msg_count(struct mxt_device *mxt)
{
  int ret;
  uint8_t count;
  uint16_t addr;

  addr = mxt_get_object_address(mxt, SPT_MESSAGECOUNT_T44, 0);
  if (addr == OBJECT_NOT_FOUND)
    return -1;

  /* Get T44 count */
  ret = mxt_read_register(mxt, &count, addr, 1);
  if (ret < 0)
    return ret;

  return count;
}

//******************************************************************************
/// \brief  Get next MSG as a string
/// \return String or NULL for error
char *t44_get_msg_string(struct mxt_device *mxt)
{
  int ret, i;
  uint16_t size;
  size_t length;

  size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5);

  ret = t44_get_msg_bytes(mxt, &databuf[0], sizeof(databuf));
  if (ret < 0)
    return NULL;

  length = snprintf(msg_string, sizeof(msg_string), MSG_PREFIX);
  for (i = 0; i < size; i++)
  {
    length += snprintf(msg_string + length, sizeof(msg_string) - length,
                       "%02X ", databuf[i]);
  }

  return &msg_string[0];
}

//******************************************************************************
/// \brief  Get next MSG into byte buffer
/// \return number of bytes read or negative error
int t44_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf, size_t buflen)
{
  int ret;
  uint16_t addr;
  uint16_t size;

  addr = mxt_get_object_address(mxt, GEN_MESSAGEPROCESSOR_T5, 0);
  if (addr == OBJECT_NOT_FOUND)
    return -1;

  /* Do not read CRC byte */
  size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;
  if (size > buflen)
  {
    mxt_err(mxt->ctx, "buffer too small!");
    return -1;
  }

  ret = mxt_read_register(mxt, buf, addr, size);
  if (ret < 0)
    return ret;

  /* Check for invalid message */
  if (buf[0] == 255u)
    return -1;

  return size;
}

//******************************************************************************
/// \brief  Discard all messages
/// \return zero for success or negative error
int t44_msg_reset(struct mxt_device *mxt)
{
  int count, i, ret;

  count = t44_get_msg_count(mxt);
  if (count < 0)
    return count;

  for (i = 0; i < count; i++)
  {
    ret = t44_get_msg_bytes(mxt, &databuf[0], sizeof(databuf));
    if (ret < 0)
      return ret;
  }

  return 0;
}
