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
#include <time.h>

#include "libmaxtouch.h"
#include "msg.h"

//******************************************************************************
/// \brief  Get number of messages
/// \return #mxt_rc
int t44_get_msg_count(struct mxt_device *mxt, int *count_out)
{
  uint16_t addr;
  int ret;
  uint8_t count;

  addr = mxt_get_object_address(mxt, SPT_MESSAGECOUNT_T44, 0);
  if (addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Get T44 count */
  ret = mxt_read_register(mxt, &count, addr, 1);
  if (ret)
    return ret;

  *count_out = count;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Get next MSG as a string
/// \return String or NULL for error
char *t44_get_msg_string(struct mxt_device *mxt)
{
  int ret, i;
  int size;
  size_t length;
  unsigned char databuf[20];

  ret = t44_get_msg_bytes(mxt, databuf, sizeof(databuf), &size);
  if (ret)
    return NULL;

  length = snprintf(mxt->msg_string, sizeof(mxt->msg_string), MSG_PREFIX);
  for (i = 0; i < size; i++) {
    length += snprintf(mxt->msg_string + length, sizeof(mxt->msg_string) - length,
                       "%02X ", databuf[i]);
  }

  return &mxt->msg_string[0];
}

//******************************************************************************
/// \brief  Get next MSG into byte buffer
/// \return #mxt_rc
int t44_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf,
                      size_t buflen, int *count)
{
  int ret;
  uint16_t addr;
  uint16_t size;

  addr = mxt_get_object_address(mxt, GEN_MESSAGEPROCESSOR_T5, 0);
  if (addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Do not read CRC byte */
  size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;
  if (size > buflen) {
    mxt_err(mxt->ctx, "Buffer too small!");
    return MXT_ERROR_NO_MEM;
  }

  ret = mxt_read_register(mxt, buf, addr, size);
  if (ret)
    return ret;

  /* Check for invalid message */
  if (buf[0] == 255u) {
    mxt_verb(mxt->ctx, "Invalid message");
    return MXT_ERROR_NO_MESSAGE;
  }

  *count = size;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Discard all messages
/// \return #mxt_rc
int t44_msg_reset(struct mxt_device *mxt)
{
  int count, i, ret, size;
  unsigned char databuf[20];

  ret = t44_get_msg_count(mxt, &count);
  if (ret) {
    mxt_verb(mxt->ctx, "rc = %d", ret);
    return ret;
  }

  for (i = 0; i < count; i++) {
    ret = t44_get_msg_bytes(mxt, &databuf[0], sizeof(databuf), &size);
    if (ret) {
      mxt_verb(mxt->ctx, "rc = %d", ret);
      return ret;
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Get messages from device and display to user
/// \param timeout_seconds Represent the time in seconds to continuously
///   display messages to the user. By setting timeout_seconds to 0, or
///   MSG_NO_WAIT the T5 object is read only once. By setting timeout_seconds to
///   -1, or  MSG_CONTINUOUS the T5 object is repeatedly read until the user
///   presses Ctrl-C.
/// \param  mxt  Maxtouch Device
/// \param  context Additional context required by msg_func
/// \param  msg_func Pointer to function to read object status
/// \param  flag Pointer to control flag
/// \return #mxt_rc
int mxt_read_messages(struct mxt_device *mxt, int timeout_seconds, void *context,
                      int (*msg_func)(struct mxt_device *mxt, uint8_t *msg,
                                      void *context, uint8_t size), int *flag)
{
  int count, len;
  time_t now;
  time_t start_time = time(NULL);
  uint8_t buf[10];
  int ret;

  while (!*flag) {
    mxt_msg_wait(mxt, MXT_MSG_POLL_DELAY_MS);

    ret = mxt_get_msg_count(mxt, &count);
    if (ret)
      return ret;

    while (count--) {
      ret = mxt_get_msg_bytes(mxt, buf, sizeof(buf), &len);
      if (ret && ret != MXT_ERROR_NO_MESSAGE)
        return ret;

      if (len > 0) {
        ret = ((*msg_func)(mxt, buf, context, len));
        if (ret != MXT_MSG_CONTINUE)
          return ret;
      }
    }

    if (timeout_seconds == 0) {
      return MXT_SUCCESS;
    } else if (timeout_seconds > 0) {
      now = time(NULL);
      if ((now - start_time) > timeout_seconds) {
        mxt_err(mxt->ctx, "Timeout");
        return MXT_ERROR_TIMEOUT;
      }
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Flush messages in buffer
/// \return #mxt_rc
int mxt_flush_msgs(struct mxt_device *mxt)
{
  int dummy;

  mxt_dbg(mxt->ctx, "Flushing messages");

  return mxt_get_msg_count(mxt, &dummy);
}

//******************************************************************************
/// \brief Gets checksum from T6 message
/// \return #mxt_rc
static int get_checksum_message(struct mxt_device *mxt, uint8_t *msg,
                                void *context, uint8_t size)
{
  if (mxt_report_id_to_type(mxt, msg[0]) == GEN_COMMANDPROCESSOR_T6) {
    uint32_t *checksum = context;
    *checksum = msg[2] | (msg[3] << 8) | (msg[4] << 16);

    return MXT_SUCCESS;
  }
  return MXT_MSG_CONTINUE;
}

//******************************************************************************
/// \brief  Reads checksum from T6 messages
/// \return #mxt_rc
uint32_t mxt_get_config_crc(struct mxt_device *mxt)
{
  int ret;
  int flag = false;
  uint32_t checksum;

  ret = mxt_report_all(mxt);
  if (ret)
    return 0;

  ret = mxt_read_messages(mxt, 2, &checksum, get_checksum_message, &flag);
  if (ret)
    return 0;

  return checksum;
}
