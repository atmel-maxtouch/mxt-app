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

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

//******************************************************************************
/// \brief Print messages
/// \return #mxt_rc
int print_raw_messages(struct mxt_device *mxt)
{
  int count, i, ret;

  /* Get the number of new messages */
  ret = mxt_get_msg_count(mxt, &count);
  if (ret)
    return ret;

  /* Print any new messages */
  if (count == 0)
  {
    printf("(no messages)\n");
  }
  else if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      printf("%s\n", mxt_get_msg_string(mxt));
      fflush(stdout);
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Handle status messages from the T6 command processor object
void print_t6_state(uint8_t state)
{
  printf("T6 status: %s%s%s%s%s%s%s\n",
         (state == 0) ? "OK":"",
         (state & 0x04) ? "COMSERR ":"",
         (state & 0x08) ? "CFGERR ":"",
         (state & 0x10) ? "CAL ":"",
         (state & 0x20) ? "SIGERR ":"",
         (state & 0x40) ? "OFL ":"",
         (state & 0x80) ? "RESET ":"");
}
