//------------------------------------------------------------------------------
/// \file   gr.c
/// \brief  T66 golden reference calibration
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
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

#define GR_CTRL              0

#define GR_ENABLE            (1 << 0)
#define GR_RPTEN             (1 << 1)
#define GR_FCALCMD_PRIME     (1 << 2)
#define GR_FCALCMD_GENERATE  (1 << 3)
#define GR_FCALCMD_STORE     (GR_FCALCMD_PRIME | GR_FCALCMD_GENERATE)
#define GR_FCALCMD_MASK      GR_FCALCMD_STORE
#define GR_TESTONINIT        (1 << 4)
#define GR_TESTONCAL         (1 << 5)

#define GR_STATE_BADSTOREDATA     (1 << 0)
#define GR_STATE_IDLE             0
#define GR_STATE_PRIMED           (1 << 1)
#define GR_STATE_GENERATED        (1 << 2)
#define GR_STATE_FCALSTATE_MASK   (GR_STATE_PRIMED | GR_STATE_GENERATED)
#define GR_STATE_FCALSEQERR       (1 << 3)
#define GR_STATE_FCALSEQTO        (1 << 4)
#define GR_STATE_FCALSEQDONE      (1 << 5)
#define GR_STATE_FCALPASS         (1 << 6)
#define GR_STATE_FCALFAIL         (1 << 7)

#define GR_TIMEOUT            30

//******************************************************************************
/// \brief Handle status messages from the T66 golden references object
static void mxt_gr_print_status(struct mxt_device *mxt, uint8_t status)
{
  mxt_info(mxt->ctx,
           "T66 state: %02X %s%s%s%s%s%s%s%s%s", status,
           (status & GR_STATE_FCALFAIL) ? "FCALFAIL " : "",
           (status & GR_STATE_FCALPASS) ? "FCALPASS " : "",
           (status & GR_STATE_FCALSEQDONE) ? "FCALSEQDONE " : "",
           (status & GR_STATE_FCALSEQTO) ? "FCALSEQTO " : "",
           (status & GR_STATE_FCALSEQERR) ? "FCALSEQERR " : "",
           ((status & GR_STATE_FCALSTATE_MASK) == GR_STATE_IDLE)     ? "Idle " : "",
           ((status & GR_STATE_FCALSTATE_MASK) == GR_STATE_GENERATED)?"Generated ":"",
           ((status & GR_STATE_FCALSTATE_MASK) == GR_STATE_PRIMED)   ? "Primed " : "",
           (status & GR_STATE_BADSTOREDATA) ? "BADSTOREDATA " : "");
}

//******************************************************************************
/// \brief Handle status messages from the T66 golden references object
static int mxt_gr_get_status(struct mxt_device *mxt, uint8_t *msg,
                             void *context, uint8_t size, uint8_t msg_count)
{
  unsigned int object_type;
  uint8_t *status = context;

  object_type = mxt_report_id_to_type(mxt, msg[0]);

  mxt_verb(mxt->ctx, "Received message from T%u", object_type);

  if (object_type == SPT_GOLDENREFERENCES_T66) {
    *status = msg[1];
    mxt_gr_print_status(mxt, *status);
    return MXT_SUCCESS;
  } else if (object_type == GEN_COMMANDPROCESSOR_T6) {
    print_t6_status(msg[1]);
    return MXT_MSG_CONTINUE;
  }

  return MXT_MSG_CONTINUE;
}

//******************************************************************************
/// \brief Send command then check status
/// \return #mxt_rc
static int mxt_gr_run_command(struct mxt_device *mxt, uint16_t addr, uint8_t cmd,
                              uint8_t wanted_fcal_state, uint8_t wanted_statebit)
{
  int ret;
  uint8_t actual_state;

  cmd |= GR_ENABLE | GR_RPTEN;

  mxt_info(mxt->ctx, "Writing %u to ctrl register", cmd);
  ret = mxt_write_register(mxt, &cmd, addr + GR_CTRL, 1);
  if (ret)
    return ret;

  ret = mxt_read_messages_sigint(mxt, GR_TIMEOUT, &actual_state, mxt_gr_get_status);
  if (ret)
    return ret;

  if (((actual_state & GR_STATE_FCALSTATE_MASK) == wanted_fcal_state)
      && (actual_state & wanted_statebit)) {
    return MXT_SUCCESS;
  } else {
    mxt_err(mxt->ctx, "Failed to enter correct state");
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }
}

//******************************************************************************
/// \brief Store golden reference calibration
int mxt_store_golden_refs(struct mxt_device *mxt)
{
  uint16_t addr;
  int ret;

  ret = mxt_msg_reset(mxt);
  if (ret)
    return ret;

  addr = mxt_get_object_address(mxt, SPT_GOLDENREFERENCES_T66, 0);

  if (addr == OBJECT_NOT_FOUND) {
    mxt_err(mxt->ctx, "T66 GOLDEN REFERENCES OBJECT NOT FOUND\n");
    return MXT_ERROR_OBJECT_NOT_FOUND;
  }

  mxt_info(mxt->ctx, "Priming");
  ret = mxt_gr_run_command(mxt, addr, GR_FCALCMD_PRIME,
                           GR_STATE_PRIMED, GR_STATE_PRIMED);
  if (ret)
    return ret;

  mxt_info(mxt->ctx, "Generating");
  ret = mxt_gr_run_command(mxt, addr, GR_FCALCMD_GENERATE,
                           GR_STATE_GENERATED, GR_STATE_FCALPASS);
  if (ret)
    return ret;

  mxt_info(mxt->ctx, "Storing");
  ret = mxt_gr_run_command(mxt, addr, GR_FCALCMD_STORE,
                           GR_STATE_IDLE, GR_STATE_FCALSEQDONE);
  if (ret)
    return ret;

  mxt_info(mxt->ctx, "Done");
  return MXT_SUCCESS;
}
