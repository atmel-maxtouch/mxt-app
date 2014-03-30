//------------------------------------------------------------------------------
/// \file   self_cap.c
/// \brief  Self Capacitance functions
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2014 Atmel Corporation. All rights reserved.
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
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

#define T109_TIMEOUT                    30

#define T109_CMD_OFFSET                 3
#define T109_CMD_TUNE                   1
#define T109_CMD_STORE_TO_CONFIG_RAM    4

//******************************************************************************
/// \brief Send command then check status
/// \return #mxt_rc
static int mxt_self_cap_command(struct mxt_device *mxt, uint16_t addr, uint8_t cmd)
{
  int count, i, len;
  time_t now;
  time_t start_time = time(NULL);
  uint8_t buf[10];
  unsigned int object_type;
  int ret;

  mxt_info(mxt->ctx, "Writing %u to T109 CMD register", cmd);
  ret = mxt_write_register(mxt, &cmd, addr + T109_CMD_OFFSET, 1);
  if (ret)
    return ret;

  while (true)
  {
    mxt_msg_wait(mxt, 100);

    now = time(NULL);
    if ((now - start_time) > T109_TIMEOUT)
    {
      mxt_err(mxt->ctx, "Timeout");
      return MXT_ERROR_TIMEOUT;
    }

    ret = mxt_get_msg_count(mxt, &count);
    if (ret)
      return ret;

    if (count > 0)
    {
      for (i = 0; i < count; i++)
      {
        ret = mxt_get_msg_bytes(mxt, buf, sizeof(buf), &len);
        if (ret)
          return ret;

        if (len > 0)
        {
          object_type = mxt_report_id_to_type(mxt, buf[0]);

          mxt_verb(mxt->ctx, "Received message from T%u", object_type);

          if (object_type == SPT_SELFCAPGLOBALCONFIG_T109)
          {
            if (buf[1] == cmd)
            {
              switch (buf[2])
              {
                case 0: return MXT_SUCCESS;
                case 1: return MXT_ERROR_SELFCAP_TUNE;
              }
            }
          }
          else if (object_type == GEN_COMMANDPROCESSOR_T6)
          {
            print_t6_state(buf[1]);
          }
        }
      }
    }
  }
}

//******************************************************************************
/// \brief Run self cap tuning procedure
/// \return #mxt_rc
int mxt_self_cap_tune(struct mxt_device *mxt)
{
   int ret;
   uint16_t t6_addr;
   uint16_t t109_addr;
   uint8_t backupnv_value;

   mxt_msg_reset(mxt);

   // Enable self test object & reporting
   t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
   if (t6_addr == OBJECT_NOT_FOUND)
     return MXT_ERROR_OBJECT_NOT_FOUND;

   t109_addr = mxt_get_object_address(mxt, SPT_SELFCAPGLOBALCONFIG_T109, 0);
   if (t109_addr == OBJECT_NOT_FOUND)
     return MXT_ERROR_OBJECT_NOT_FOUND;

   mxt_info(mxt->ctx, "Stopping T70");
   backupnv_value = 0x33;
   ret = mxt_write_register(mxt, &backupnv_value, t6_addr + MXT_T6_BACKUPNV_OFFSET, 1);
   if (ret)
     return ret;

   mxt_info(mxt->ctx, "Tuning");
   ret = mxt_self_cap_command(mxt, t109_addr, T109_CMD_TUNE);
   if (ret)
     return ret;

   mxt_info(mxt->ctx, "Store to config RAM");
   ret = mxt_self_cap_command(mxt, t109_addr, T109_CMD_STORE_TO_CONFIG_RAM);
   if (ret)
     return ret;

   mxt_info(mxt->ctx, "Saving configuration");
   ret = mxt_backup_config(mxt);
   if (ret)
     return ret;

   ret = mxt_reset_chip(mxt, false);
   if (ret)
     return ret;

   return 0;
}
