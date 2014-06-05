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
/// \brief Get messages
/// \return #mxt_rc
int mxt_read_messages(struct mxt_device *mxt, int timeout_seconds, void *context,
                      int (*msg_func)(struct mxt_device *mxt, uint8_t *msg,
                      void *context))
{
  int count,len;
  time_t now;
  time_t start_time = time(NULL);
  uint8_t buf[10];
  int ret;

  while (true)
  {
    mxt_msg_wait(mxt, MXT_MSG_POLL_DELAY_MS);

    ret = mxt_get_msg_count(mxt, &count);
    if (ret)
      return ret;

    do {
      ret = mxt_get_msg_bytes(mxt, buf, sizeof(buf), &len);
      if (ret)
        return ret;

      if (len > 0)
      {
        ret = ((*msg_func)(mxt, buf, context));
        if (ret != MXT_MSG_CONTINUE)
          return ret;
      }

      now = time(NULL);
      if ((now - start_time) > timeout_seconds)
      {
        mxt_err(mxt->ctx, "Timeout");
        return MXT_ERROR_TIMEOUT;
      }

    } while(count--);
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Print messages
/// \return #mxt_rc
int print_raw_messages(struct mxt_device *mxt, int timeout, uint16_t object_type)
{
  int count, i, j, ret;
  time_t now;
  time_t start_time = time(NULL);

  do {
    /* Get the number of new messages */
    ret = mxt_get_msg_count(mxt, &count);
    if (ret) {
      return ret;
    } else if (count > 0) {
      /* Print any new messages */
      for (i = 0; i < count; i++)
      {
        int len, size;
        unsigned char msg[20];

        ret = mxt_get_msg_bytes(mxt, &msg[0], sizeof(msg), &size);
        if (ret)
          return ret;

        if (object_type == 0 || object_type == mxt_report_id_to_type(mxt, msg[0])) {
          len = snprintf(mxt->msg_string, sizeof(mxt->msg_string), MSG_PREFIX);
          for (j = 0; j < size; j++)
          {
            len += snprintf(mxt->msg_string + len, sizeof(mxt->msg_string) - len,
                            "%02X ", msg[j]);
          }
          printf("%s\n", mxt->msg_string);
          fflush(stdout);
        }
      }
    }

    now = time(NULL);
  } while((now - start_time) < timeout || timeout == 0);

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
