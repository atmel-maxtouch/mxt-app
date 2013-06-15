//------------------------------------------------------------------------------
/// \file   serial_data.c
/// \brief  T68 Serial Data Command support
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
#include <time.h>
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
struct t68_ctx
{
  const char *filename;
  struct mxt_buffer buf;
  uint16_t t68_addr;
  uint8_t t68_size;
  uint16_t t68_cmd_addr;
  uint16_t t68_data_size;
};

//******************************************************************************
/// \brief Print T68 status messages
static void mxt_t68_print_status(uint8_t status)
{
  printf("T68 status: %02X %s%s%s%s%s%s\n",
      status,
      (status == 0x00) ? "Success/No error" : "",
      (status == 0x01) ? "Command supplied in CMD.COMMAND is out of sequence" : "",
      (status == 0x02) ? "Supplied DATATYPE value is not supported" : "",
      (status == 0x03) ? "Supplied LENGTH value exceeds length of DATA[] array" : "",
      (status == 0x04) ? "More bytes supplied than can be accommodated by this data type" : "",
      (status == 0x0F) ? "The action could not be completed due to an error outside of this object" : "");
}

//******************************************************************************
/// \brief  Handle status messages from the T68 Serial Data Command object
/// \return 0 on success, negative error
static int mxt_t68_get_status(void)
{
  uint16_t count, i;
  time_t now;
  time_t start_time = time(NULL);
  uint8_t buf[10];
  size_t len;
  unsigned int object_type;
  uint8_t status;

  while (true)
  {
    now = time(NULL);
    if ((now - start_time) > T68_TIMEOUT)
    {
      printf("Timeout\n");
      return -1;
    }

    count = mxt_get_msg_count();

    if (count > 0)
    {
      for (i = 0; i < count; i++)
      {
        len = mxt_get_msg_bytes(buf, sizeof(buf));

        if (len > 0)
        {
          object_type = report_id_to_type(buf[0]);

          LOG(LOG_VERBOSE, "Received message from T%u", object_type);

          if (object_type == SERIAL_DATA_COMMAND_T68)
          {
            /* mask off reserved bits */
            status = buf[1] & 0x0F;

            mxt_t68_print_status(status);

            return (status == 0) ? 0 : -1;
          }
          else if (object_type == GEN_COMMANDPROCESSOR_T6)
          {
            print_t6_state(buf[1]);
          }
        }
      }
    }

    sleep(1);
  }
}

//******************************************************************************
/// \brief  Send command then check status
/// \return 0 on success, negative error
static int mxt_t68_command(struct t68_ctx *ctx, uint8_t cmd)
{
  int ret;

  LOG(LOG_INFO, "Writing %u to CMD register", cmd);
  ret = mxt_write_register(&cmd, ctx->t68_cmd_addr, 1);
  if (ret < 0)
    return ret;

  return mxt_t68_get_status();
}

//******************************************************************************
/// \brief  Enable T68
/// \return 0 on success, negative error
static int mxt_t68_enable(uint16_t addr)
{
  int ret;
  uint8_t cmd = T68_CTRL_RPTEN | T68_CTRL_ENABLE;

  LOG(LOG_DEBUG, "Enabling T68 object");
  LOG(LOG_VERBOSE, "Writing %u to ctrl register", cmd);

  ret = mxt_write_register(&cmd, addr + T68_CTRL, 1);
  if (ret < 0)
    return ret;

  return 0;
}

//******************************************************************************
/// \brief  Read hex encoded data from file
/// \return 0 on success, negative error
static int mxt_t68_load_file(struct t68_ctx *ctx)
{
  int ret;
  uint8_t value = 0;
  FILE *fp;
  bool file_read = false;
  char hexbuf[3];
  uint16_t hexcount;
  int c;

  /* open file */
  fp = fopen(ctx->filename, "r");
  if (fp == NULL)
  {
    LOG(LOG_ERROR, "Error opening %s", ctx->filename);
    return -1;
  }

  ret = mxt_buf_init(&ctx->buf);
  if (ret < 0)
    return -1;

  while (!file_read)
  {
    /* Read next value from file */
    c = getc(fp);
    if (c == EOF)
    {
      break;
    }
    /* skip spaces, newlines, commas*/
    else if (c == 0x20 || c == '\r' || c == '\n' || c == ',')
    {
      continue;
    }
    /* Ignore comment lines */
    else if (c == '[')
    {
      while (c != '\n')
      {
        c = getc(fp);
      }
      continue;
    }
    /* A value looks like "0xABu," */
    else if (c == '0')
    {
      if (fscanf(fp, "x%2su", (char *)&hexbuf) != 1)
      {
        LOG(LOG_ERROR, "Parse error");
        ret = -1;
        goto fail;
      }

      ret = mxt_convert_hex(hexbuf, &value, &hexcount, 3);
      if (ret < 0)
        goto fail;

      ret = mxt_buf_add(&ctx->buf, value);
      if (ret < 0)
        goto fail;
    }
    else
    {
      LOG(LOG_ERROR, "Unexpected character \"%c\"", c);
      ret = -1;
      goto fail;
    }
  }

  return 0;

fail:
  mxt_buf_free(&ctx->buf);
  return ret;
}

//******************************************************************************
/// \brief Write LENGTH
static int mxt_t68_write_length(struct t68_ctx *ctx, uint8_t length)
{
  LOG(LOG_DEBUG, "Writing LENGTH=%u", length);

  return mxt_write_register(&length, ctx->t68_addr + T68_LENGTH, 1);
}

//******************************************************************************
/// \brief Zero entire T68 object
static int mxt_t68_zero_data(struct t68_ctx *ctx)
{
  int ret;
  uint8_t zeros[ctx->t68_data_size];

  LOG(LOG_DEBUG, "Zeroing DATA");

  ret = mxt_write_register(zeros,
                           ctx->t68_addr + T68_DATA,
                           sizeof(zeros));

  return ret;
}

//******************************************************************************
/// \brief Send frames of T68 data to chip
static int mxt_t68_send_frames(struct t68_ctx *ctx)
{
  int ret;
  size_t offset = 0;
  uint16_t frame_size;
  uint16_t frame_number = 1;

  while (offset < ctx->buf.size)
  {
    frame_size = MIN(ctx->buf.size - offset, ctx->t68_data_size);

    printf("Writing frame %u, %u bytes\n", frame_number, frame_size);

    if (frame_size > UCHAR_MAX)
    {
      LOG(LOG_ERROR, "Serial data frame size miscalculation");
      return -1;
    }

    ret = mxt_write_register(ctx->buf.data + offset,
                             ctx->t68_addr + T68_DATA,
                             frame_size);
    if (ret < 0)
      return ret;

    ret = mxt_t68_write_length(ctx, frame_size);
    if (ret < 0)
      return ret;

    /* for each frame */
    ret = mxt_t68_command(ctx, T68_CMD_CONTINUE);
    if (ret < 0)
      return ret;

    offset += frame_size;
    frame_number++;
  }

  return 0;
}

//******************************************************************************
/// \brief Write DATATYPE
static int mxt_t68_write_datatype(struct t68_ctx *ctx, uint16_t datatype)
{
  uint8_t buf[2];

  buf[0] = (datatype & 0xFF);
  buf[1] = (datatype & 0xFF00) >> 8;

  LOG(LOG_INFO, "Writing %u to DATATYPE register", datatype);
  return mxt_write_register(&buf[0], ctx->t68_addr + T68_DATATYPE, sizeof(buf));
}

//******************************************************************************
/// \brief Check chip is not in deep sleep
static int mxt_t68_check_power_cfg(void)
{
  uint16_t t7_addr;
  uint8_t buf[2];
  int ret;

  t7_addr = get_object_address(GEN_POWERCONFIG_T7, 0);
  if (t7_addr == OBJECT_NOT_FOUND)
    return 0;

  ret = mxt_read_register(&buf[0], t7_addr, sizeof(buf));
  if (ret < 0)
    return ret;

  LOG(LOG_VERBOSE, "T7 IDLEACQINT=%u ACTVACQINT=%u", buf[0], buf[1]);

  if ((buf[0] == 0) || (buf[1] == 0))
  {
    printf("Warning: The T7 power configuration object shows that the chip\n"
           "is in deep sleep, and so will not process T68 serial data\n"
           "commands. Please set the T7 power configuration idle acquisition\n"
           "interval to a non-zero value and try again.\n");

    return -1;
  }
  else
  {
    return 0;
  }
}

//******************************************************************************
/// \brief Upload file to T68 Serial Data Object
int mxt_serial_data_upload(const char *filename, uint16_t datatype)
{
  int ret;
  struct t68_ctx ctx;

  ret = mxt_msg_reset();
  if (ret < 0)
    return ret;

  printf("Checking T7 Power Config\n");
  ret = mxt_t68_check_power_cfg();
  if (ret < 0)
    return ret;

  /* Check for existence of T68 object */
  ctx.t68_addr = get_object_address(SERIAL_DATA_COMMAND_T68, 0);
  if (ctx.t68_addr == OBJECT_NOT_FOUND)
    return -1;

  /* Calculate position of CMD register */
  ctx.t68_size = get_object_size(SERIAL_DATA_COMMAND_T68);
  ctx.t68_cmd_addr = ctx.t68_addr + ctx.t68_size - 3;

  /* Calculate frame size */
  ctx.t68_data_size = ctx.t68_size - 9;

  /* Read input file */
  ctx.filename = filename;
  ret = mxt_t68_load_file(&ctx);
  if (ret < 0)
    return ret;

  ret = mxt_t68_enable(ctx.t68_addr);
  if (ret < 0)
    goto release;

  ret = mxt_t68_zero_data(&ctx);
  if (ret < 0)
    return ret;

  ret = mxt_t68_write_length(&ctx, 0);
  if (ret < 0)
    return ret;

  printf("Configuring T68\n");
  ret = mxt_t68_write_datatype(&ctx, datatype);
  if (ret < 0)
    goto release;

  printf("Sending start command\n");
  ret = mxt_t68_command(&ctx, T68_CMD_START);
  if (ret < 0)
    goto release;

  printf("Sending data\n");
  ret = mxt_t68_send_frames(&ctx);
  if (ret < 0)
    goto release;

  printf("Sending end command\n");
  ret = mxt_t68_command(&ctx, T68_CMD_END);
  if (ret < 0)
    goto release;

  printf("Done\n");
  ret = 0;

release:
  mxt_buf_free(&ctx.buf);
  return ret;
}
