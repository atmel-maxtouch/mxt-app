#pragma once
//------------------------------------------------------------------------------
/// \file   serial_data.h
/// \brief  T68 Serial Data Header
/// \author Michael Gong
//------------------------------------------------------------------------------
// Copyright 2016 Atmel Corporation. All rights reserved.
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
#include <limits.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"
#include "buffer.h"

#define T68_CTRL                   0
#define T68_CTRL_ENABLE            (1 << 0)
#define T68_CTRL_RPTEN             (1 << 1)
#define T68_DATATYPE               3

#define T68_CMD_NONE               0
#define T68_CMD_START              1
#define T68_CMD_CONTINUE           2
#define T68_CMD_END                3

#define T68_LENGTH                 5
#define T68_DATA                   6

#define T68_TIMEOUT                30

//******************************************************************************
/// \brief T68 Serial Data Command Context object
struct t68_ctx {
  struct mxt_device *mxt;
  struct libmaxtouch_ctx *lc;
  const char *filename;
  struct mxt_buffer buf;
  uint32_t t68_object_id;
  uint16_t t68_instance;
  uint16_t t68_addr;
  uint8_t t68_size;
  uint16_t t68_cmd_addr;
  uint16_t t68_data_size;
  uint16_t t68_datatype;
  uint16_t t68_length;
  bool t68_last_frame;
  uint32_t t68_checksum;
};

int mxt_load_t68_payload(struct mxt_device *mxt, struct t68_ctx *ctx);
int mxt_serial_data_upload(struct mxt_device *mxt, const char *filename, uint16_t datatype);