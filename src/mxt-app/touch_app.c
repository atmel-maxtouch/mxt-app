//------------------------------------------------------------------------------
/// \file   touch_app.c
/// \brief  Utility functions for mxt-app
/// \author Iiro Valkonen
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
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <linux/input.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <time.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

#define MXT_MSG_POLL_DELAY_MS 10

//******************************************************************************
/// \brief Get messages from device and display to user
/// \param timeout_seconds Represent the time in seconds to continuously
///   display messages to the user. By setting timeout_seconds to 0, or
///   MSG_NO_WAIT the T5 object is read only once. By setting timeout_seconds to
///   -1, or  MSG_CONTINUOUS the T5 object is continually read until the user
///   presses Ctrl-C.
/// \param  mxt  Maxtouch Device
/// \param  context Additional context required by msg_func
/// \param  msg_func Pointer to function to read object status
/// \return #mxt_rc
int mxt_read_messages(struct mxt_device *mxt, int timeout_seconds, void *context,
                      int (*msg_func)(struct mxt_device *mxt, uint8_t *msg,
                      void *context, uint8_t size))
{
  int count, len;
  time_t now;
  time_t start_time = time(NULL);
  uint8_t buf[10];
  int ret;

  struct sigaction sa;
  mxt_init_sigint_handler(mxt, sa);

  while (!mxt_get_sigint_flag())
  {
    mxt_msg_wait(mxt, MXT_MSG_POLL_DELAY_MS);

    ret = mxt_get_msg_count(mxt, &count);
    if (ret)
      return ret;

    while (count--) {
      ret = mxt_get_msg_bytes(mxt, buf, sizeof(buf), &len);
      if (ret && ret != MXT_ERROR_NO_MESSAGE)
        return ret;

      if (len > 0)
      {
        ret = ((*msg_func)(mxt, buf, context, len));
        if (ret != MXT_MSG_CONTINUE)
          return ret;
      }
    }

    if (timeout_seconds == 0) {
      return MXT_SUCCESS;
    } else if (timeout_seconds > 0) {
      now = time(NULL);
      if ((now - start_time) > timeout_seconds)
      {
        mxt_err(mxt->ctx, "Timeout");
        return MXT_ERROR_TIMEOUT;
      }
    }
  }

  mxt_release_sigint_handler(mxt, sa);
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Print message as hex
/// \return #mxt_rc
static int print_message_hex(struct mxt_device *mxt, uint8_t *msg,
                             void *context, uint8_t size)
{
  const uint16_t object_type = *((uint16_t*)context);
  int j;
  int len;

  if (object_type == 0 || object_type == mxt_report_id_to_type(mxt, msg[0]))
  {
    len = snprintf(mxt->msg_string, sizeof(mxt->msg_string), MSG_PREFIX);
    for (j = 0; j < size; j++)
    {
      len += snprintf(mxt->msg_string + len, sizeof(mxt->msg_string) - len,
          "%02X ", msg[j]);
    }

    printf("%s\n", mxt->msg_string);
    fflush(stdout);
  }

  return MXT_MSG_CONTINUE;
}

//******************************************************************************
/// \brief Print messages
/// \return #mxt_rc
int print_raw_messages(struct mxt_device *mxt, int timeout, uint16_t object_type)
{
  mxt_read_messages(mxt, timeout, &object_type, print_message_hex);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Handle status messages from the T6 command processor object
void print_t6_status(uint8_t status)
{
  printf("T6 status: %s%s%s%s%s%s%s\n",
         (status == 0) ? "OK":"",
         (status & 0x04) ? "COMSERR ":"",
         (status & 0x08) ? "CFGERR ":"",
         (status & 0x10) ? "CAL ":"",
         (status & 0x20) ? "SIGERR ":"",
         (status & 0x40) ? "OFL ":"",
         (status & 0x80) ? "RESET ":"");
}
