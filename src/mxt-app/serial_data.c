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

#include "serial_data.h"

//******************************************************************************
/// \brief Print T68 status messages
static void mxt_t68_print_status(struct t68_ctx *ctx, uint8_t status)
{
  mxt_info(ctx->lc, "T68 status: %02X %s%s%s%s%s%s%s",
           status,
           (status == 0x00) ? "Success/No error" : "",
           (status == 0x01) ? "Command supplied in CMD.COMMAND is out of sequence" : "",
           (status == 0x02) ? "Supplied DATATYPE value is not supported" : "",
           (status == 0x03) ? "Supplied LENGTH value exceeds length of DATA[] array" : "",
           (status == 0x04) ? "More bytes supplied than can be accommodated by this data type" : "",
           (status == 0x05) ? "Data content is invalid" : "",
           (status == 0x0F) ? "The action could not be completed due to an error outside of this object" : "");
}

//******************************************************************************
/// \brief  Handle status messages from the T68 Serial Data Command object
/// \return #mxt_rc
static int mxt_t68_get_status(struct mxt_device *mxt, uint8_t *msg,
                              void *context, uint8_t size, uint8_t msg_count)
{
  struct t68_ctx *ctx = context;
  unsigned int object_type = mxt_report_id_to_type(mxt, msg[0]);
  uint8_t status;

  mxt_verb(mxt->ctx, "Received message from T%u", object_type);

  if (object_type == SERIAL_DATA_COMMAND_T68) {
    /* mask off reserved bits */
    status = msg[1] & 0x0F;

    mxt_t68_print_status(ctx, status);

    return (status == 0) ? MXT_SUCCESS : MXT_ERROR_SERIAL_DATA_FAILURE;
  } else if (object_type == GEN_COMMANDPROCESSOR_T6) {
    print_t6_status(msg[1]);
  }
  return MXT_MSG_CONTINUE;
}

//******************************************************************************
/// \brief  Send command then check status
/// \return #mxt_rc
static int mxt_t68_command(struct t68_ctx *ctx, uint8_t cmd)
{
  int ret = 0;

  ctx->mxt->mxt_enc.enc_cfg_write = false;

  mxt_verb(ctx->lc, "Writing %u to CMD register", cmd);
  ret = mxt_write_register(ctx->mxt, &cmd, ctx->t68_cmd_addr, 1);
  if (ret)
    goto t68_command_end;

  ret = mxt_read_messages_sigint(ctx->mxt, T68_TIMEOUT, ctx, mxt_t68_get_status);
  if (ret)
    goto t68_command_end;

t68_command_end:
  ctx->mxt->mxt_enc.enc_cfg_write = true;
  return ret;
}

//******************************************************************************
/// \brief  Enable T68
/// \return #mxt_rc
static int mxt_t68_enable(struct t68_ctx *ctx)
{
  int ret = 0;
  uint8_t cmd = T68_CTRL_RPTEN | T68_CTRL_ENABLE;

  ctx->mxt->mxt_enc.enc_cfg_write = false;

  mxt_dbg(ctx->lc, "Enabling T68 object");
  mxt_dbg(ctx->lc, "Writing %u to ctrl register", cmd);

  ret = mxt_write_register(ctx->mxt, &cmd, ctx->t68_addr + T68_CTRL, 1);

  ctx->mxt->mxt_enc.enc_cfg_write = true;

  return ret;
}

//******************************************************************************
/// \brief  Read hex encoded data from file
/// \return #mxt_rc
static int mxt_t68_load_file(struct t68_ctx *ctx)
{
  int ret;
  uint8_t value = 0;
  FILE *fp;
  bool file_read = false;
  char buf[256];
  uint16_t hexcount;
  int c;

  /* open file */
  fp = fopen(ctx->filename, "r");
  if (fp == NULL) {
    mxt_err(ctx->lc, "Error opening %s", ctx->filename);
    return mxt_errno_to_rc(errno);
  }

  ret = mxt_buf_init(&ctx->buf);
  if (ret) {
    mxt_err(ctx->lc, "Error initialising buffer");
    goto close;
  }

  while (!file_read) {
    /* Read next value from file */
    c = getc(fp);
    if (c == EOF) {
      break;
    }
    /* skip spaces, newlines, commas*/
    else if (c == 0x20 || c == '\r' || c == '\n' || c == ',') {
      continue;
    }
    /* Ignore comment lines */
    else if (c == '[') {
      // Grab comment key
      if (fscanf(fp, "%255[^]]", buf) != 1) {
        ret = MXT_ERROR_FILE_FORMAT;
        goto fail;
      }

      mxt_verb(ctx->lc, "[%s]", buf);

      if (!strncasecmp(buf, "datatype=", 9)) {
        if (sscanf(buf + 9, "%d", &c) != 1) {
          mxt_warn(ctx->lc, "Unable to parse datatype");
        } else {
          ctx->t68_datatype = c;
          mxt_dbg(ctx->lc, "DATATYPE set to %u by file", ctx->t68_datatype);
        }
      }

      // Read until end of line
      while (c != '\n') {
        c = getc(fp);
      }
      continue;
    }
    /* A value looks like "0xABu," */
    else if (c == '0') {
      if (fscanf(fp, "x%2su", (char *)&buf) != 1) {
        mxt_err(ctx->lc, "Parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto fail;
      }

      ret = mxt_convert_hex(buf, &value, &hexcount, 3);
      if (ret)
        goto fail;

      ret = mxt_buf_add(&ctx->buf, value);
      if (ret)
        goto fail;
    } else {
      mxt_err(ctx->lc, "Unexpected character \"%c\"", c);
      ret = MXT_ERROR_FILE_FORMAT;
      goto fail;
    }
  }

  mxt_info(ctx->lc, "Loaded file %s, %zu bytes", ctx->filename, ctx->buf.size);

  return MXT_SUCCESS;

fail:
  mxt_buf_free(&ctx->buf);
close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief Write LENGTH
/// \return #mxt_rc
static int mxt_t68_write_length(struct t68_ctx *ctx, uint8_t length)
{
  int ret;


  ctx->mxt->mxt_enc.enc_cfg_write = false;

  mxt_dbg(ctx->lc, "Writing LENGTH=%u", length);

  ret = mxt_write_register(ctx->mxt, &length, ctx->t68_addr + T68_LENGTH, 1);

  ctx->mxt->mxt_enc.enc_cfg_write = true;

  return  ret;
}

//******************************************************************************
/// \brief Zero entire T68 object
/// \return #mxt_rc
static int mxt_t68_zero_data(struct t68_ctx *ctx)
{
  int ret;

  ctx->mxt->mxt_enc.enc_cfg_write = false;

  uint8_t zeros[ctx->t68_data_size];

  mxt_dbg(ctx->lc, "Zeroing DATA");

  memset(&zeros, 0, sizeof(zeros));

  ret =  mxt_write_register(ctx->mxt, zeros,
                            ctx->t68_addr + T68_DATA,
                            sizeof(zeros));
  ctx->mxt->mxt_enc.enc_cfg_write = true;

  return ret;  
}

//******************************************************************************
/// \brief Send frames of T68 data to chip
/// \return #mxt_rc
static int mxt_t68_send_frames(struct t68_ctx *ctx)
{
  int ret = 0;
  size_t offset = 0;
  uint8_t frame_size = 0;
  int frame = 1;
  uint8_t cmd;

  ctx->mxt->mxt_enc.enc_cfg_write = false;

    while (cmd != T68_CMD_END) {
    frame_size = MIN(ctx->t68_length - offset, ctx->t68_data_size);

    mxt_dbg(ctx->lc, "Writing frame %u, %u bytes", frame, frame_size);

    if (frame_size > UCHAR_MAX) {
      mxt_err(ctx->lc, "Serial data frame size miscalculation");
      ctx->mxt->mxt_enc.enc_cfg_write = true;
      return MXT_INTERNAL_ERROR;
    }

  ret = mxt_write_register(ctx->mxt, ctx->buf.data + offset,
                           ctx->t68_addr + T68_DATA,
                           ctx->t68_data_size);

    if (ret)
      goto t68_send_frames_end;

    ret = mxt_t68_write_length(ctx, frame_size);
    if (ret)
      goto t68_send_frames_end;

    if (frame == 1)
      cmd = T68_CMD_START;
    else if (offset >= ctx->t68_length) {
      cmd = T68_CMD_END;
    } else
      cmd = T68_CMD_CONTINUE;

    offset += frame_size;

    ret = mxt_t68_command(ctx, cmd);
    if (ret)
      goto t68_send_frames_end;

    frame++;
  }

t68_send_frames_end:
  ctx->mxt->mxt_enc.enc_cfg_write = true;
  return ret;
}

//******************************************************************************
/// \brief Write DATATYPE
/// \return #mxt_rc
static int mxt_t68_write_datatype(struct t68_ctx *ctx)
{
  uint8_t buf[2];
  int ret = 0;

  ctx->mxt->mxt_enc.enc_cfg_write = false;

  buf[0] = (ctx->t68_datatype & 0xFF);
  buf[1] = (ctx->t68_datatype & 0xFF00) >> 8;

  mxt_dbg(ctx->lc, "Writing %u to DATATYPE register", ctx->t68_datatype);
  ret = mxt_write_register(ctx->mxt, &buf[0], ctx->t68_addr + T68_DATATYPE, sizeof(buf));

  ctx->mxt->mxt_enc.enc_cfg_write = true;

  return ret;
}

//******************************************************************************
/// \brief Check chip is not in deep sleep
/// \return #mxt_rc
static int mxt_t68_check_power_cfg(struct t68_ctx *ctx)
{
  uint16_t t7_addr;
  uint8_t buf[2];
  int ret;

  /* Skip if object not present */
  t7_addr = mxt_get_object_address(ctx->mxt, GEN_POWERCONFIG_T7, 0);
  if (t7_addr == OBJECT_NOT_FOUND)
    return MXT_SUCCESS;

  ret = mxt_read_register(ctx->mxt, &buf[0], t7_addr, sizeof(buf));
  if (ret)
    return ret;

  mxt_verb(ctx->lc, "T7 IDLEACQINT=%u ACTVACQINT=%u", buf[0], buf[1]);

  if ((buf[0] == 0) || (buf[1] == 0)) {
    mxt_err(ctx->lc, "\nWarning: The T7 power object is in deep sleep,\n" 
        "and will not process T68 serial data commands.\n"
            "Please set the T7 power configuration idle acquisition \n"
            "interval to a non-zero value and try again.\n");

    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  } else {
    return MXT_SUCCESS;
  }
}

//******************************************************************************
/// \brief Upload file to T68 Serial Data Object
/// \return #mxt_rc
int mxt_serial_data_upload(struct mxt_device *mxt, const char *filename, uint16_t datatype)
{
  int ret;
  struct t68_ctx ctx;

  ctx.mxt = mxt;
  ctx.lc = mxt->ctx;

  ret = mxt_msg_reset(mxt);
  if (ret)
    return ret;

  mxt_info(ctx.lc, "Checking T7 Power Config");
  ret = mxt_t68_check_power_cfg(&ctx);
  if (ret)
    return ret;

  /* Check for existence of T68 object */
  ctx.t68_addr = mxt_get_object_address(mxt, SERIAL_DATA_COMMAND_T68, 0);
  if (ctx.t68_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Calculate position of CMD register */
  ctx.t68_size = mxt_get_object_size(mxt, SERIAL_DATA_COMMAND_T68);
  ctx.t68_cmd_addr = ctx.t68_addr + ctx.t68_size - 3;

  /* Calculate frame size */
  ctx.t68_data_size = ctx.t68_size - 9;

  /* Set datatype from command line */
  ctx.t68_datatype = datatype;
  mxt_dbg (ctx.lc, "Loading T68 file");

  /* Read input file */
  ctx.filename = filename;

  if (ctx.filename != NULL) {
    ret = mxt_t68_load_file(&ctx);
    if (ret)
      return ret;
  } else {
    mxt_info (ctx.lc, "File was null");
  }

  ret = mxt_t68_enable(&ctx);
  if (ret)
    goto release;

  ret = mxt_t68_zero_data(&ctx);
  if (ret)
    goto release;

  ret = mxt_t68_write_length(&ctx, 0);
  if (ret)
    goto release;

  mxt_dbg(ctx.lc, "Configuring T68");
  ret = mxt_t68_write_datatype(&ctx);
  if (ret)
    goto release;

  mxt_dbg(ctx.lc, "Sending data");
  ret = mxt_t68_send_frames(&ctx);
  if (ret) {
    mxt_err(ctx.lc, "Error sending data");
    goto release;
  }

  mxt_info(ctx.lc, "Load T68 Done");
  ret = MXT_SUCCESS;

release:
  mxt_buf_free(&ctx.buf);
  return ret;
}

//******************************************************************************
/// \brief Upload file to T68 Serial Data Object
/// \return #mxt_rc
int mxt_load_t68_payload(struct mxt_device *mxt, struct t68_ctx *ctx)
{
  int ret;

  ctx->mxt = mxt;
  ctx->lc = mxt->ctx;

  mxt_info(ctx->lc, "Uploading T68 payload");

  ret = mxt_msg_reset(ctx->mxt);
  if (ret)
    return ret;

  if (!(CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED))) {
    mxt_info(ctx->lc, "Checking T7 Power Config");
    ret = mxt_t68_check_power_cfg(ctx);
    if (ret)
      return ret;
  }

  /* Check for existence of T68 object */
  ctx->t68_addr = mxt_get_object_address(ctx->mxt, SERIAL_DATA_COMMAND_T68, 0);
  if (ctx->t68_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;


  /* Calculate position of CMD register */
  ctx->t68_size = mxt_get_object_size(ctx->mxt, SERIAL_DATA_COMMAND_T68);
  ctx->t68_cmd_addr = ctx->t68_addr + ctx->t68_size - 3;

 /* Calculate frame size */
  ctx->t68_data_size = ctx->t68_size - 9;

  /* Set datatype from command line */
  //ctx->t68_datatype = datatype;

  ret = mxt_t68_enable(ctx);
  if (ret)
    return ret;

  ret = mxt_t68_zero_data(ctx);
  if (ret)
   return ret;

  ret = mxt_t68_write_length(ctx, 0);
  if (ret)
    return ret;

  mxt_dbg(ctx->lc, "Configuring T68");
  ret = mxt_t68_write_datatype(ctx);
  if (ret)
    return ret;

  mxt_dbg(ctx->lc, "Sending data");
  ret = mxt_t68_send_frames(ctx);
  if (ret) {
    mxt_err(ctx->lc, "Error sending data");
    return ret;
  }

  mxt_info(ctx->lc, "T68 Done");
  ret = MXT_SUCCESS;

release_buf:
  mxt_buf_free(&ctx->buf);

  return ret;
}