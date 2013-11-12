//------------------------------------------------------------------------------
/// \file   bootloader.c
/// \brief  Bootloader functions
/// \author Nick Dyer
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
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/sysfs/sysfs_device.h"

#ifdef HAVE_LIBUSB
#include "libmaxtouch/usb/usb_device.h"
#endif

#include "mxt_app.h"

#define MXT_UNLOCK_CMD_MSB      0xaa
#define MXT_UNLOCK_CMD_LSB      0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD 0xc0 /* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA   0x80 /* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK      0x02
#define MXT_FRAME_CRC_FAIL       0x03
#define MXT_FRAME_CRC_PASS       0x04
#define MXT_APP_CRC_FAIL         0x40 /* valid 7 6 bit only */
#define MXT_BOOT_STATUS_MASK     0x3f

#define FIRMWARE_BUFFER_SIZE     1024

#define MXT_RESET_TIME           2
#define MXT_BOOTLOADER_DELAY     50000

//******************************************************************************
/// \brief Bootloader context object
struct bootloader_ctx
{
  struct mxt_device *mxt;
  struct mxt_conn_info conn;
  struct libmaxtouch_ctx *libmaxtouch_ctx;
  bool have_bootloader_version;
  bool extended_id_mode;
  FILE *fp;
  char curr_version[MXT_FW_VER_LEN];
  int appmode_address;
  bool check_version;
  const char *new_version;
  bool usb_bootloader;
};

static int wait_for_chg(struct bootloader_ctx *ctx)
{
#ifdef HAVE_LIBUSB
  int try = 0;
  if (ctx->mxt->conn.type == E_USB)
  {
    while (usb_read_chg(ctx->mxt))
    {
      if (++try > 100)
      {
        LOG(LOG_WARN, "Timed out awaiting CHG");
        return -1;
      }

      usleep(1000);
    }

    LOG(LOG_VERBOSE, "CHG line cycles %d", try);
  }
  else
#endif
  {
    usleep(MXT_BOOTLOADER_DELAY);
  }

  return 0;
}

static int unlock_bootloader(struct bootloader_ctx *ctx)
{
  unsigned char buf[2];

  buf[0] = MXT_UNLOCK_CMD_LSB;
  buf[1] = MXT_UNLOCK_CMD_MSB;

  return mxt_bootloader_write(ctx->mxt, buf, sizeof(buf));
}

static int mxt_check_bootloader(struct bootloader_ctx *ctx, unsigned int state)
{
    unsigned char buf[3];
    unsigned char val;
    unsigned char bootloader_id;
    unsigned char bootloader_version;

recheck:
    if (state != MXT_WAITING_BOOTLOAD_CMD)
      wait_for_chg(ctx);

    if ((!ctx->have_bootloader_version) && ctx->extended_id_mode
        && (state == MXT_WAITING_FRAME_DATA))
    {
       LOG(LOG_INFO, "Attempting to retrieve bootloader version");
       if (mxt_bootloader_read(ctx->mxt, &buf[0], 3) != 0) {
           return -1;
       }

       val = buf[0];
       bootloader_id = buf[1];
       bootloader_version = buf[2];

       LOG(LOG_INFO, "Bootloader ID:%d Version:%d",
         bootloader_id, bootloader_version);

       ctx->have_bootloader_version = true;
    }
    else
    {
      if (mxt_bootloader_read(ctx->mxt, &val, 1) != 0) {
        return -1;
      }
    }

    LOG(LOG_VERBOSE, "Bootloader status %02X", val);

    switch (state) {
    case MXT_WAITING_BOOTLOAD_CMD:
        bootloader_id = val & MXT_BOOT_STATUS_MASK;
        val &= ~MXT_BOOT_STATUS_MASK;

        if (val == MXT_APP_CRC_FAIL) {
            LOG(LOG_INFO, "Bootloader reports APP CRC failure");
            goto recheck;
        } else if (val == MXT_WAITING_FRAME_DATA) {
            LOG(LOG_INFO, "Bootloader already unlocked");
            return -3;
        }

        break;
    case MXT_WAITING_FRAME_DATA:
        if (val == MXT_FRAME_CRC_PASS) {
          LOG(LOG_INFO, "Bootloader still giving CRC PASS");
          goto recheck;
        }
        val &= ~MXT_BOOT_STATUS_MASK;
        break;
    case MXT_FRAME_CRC_PASS:
        if (val == MXT_FRAME_CRC_CHECK) {
            goto recheck;
        } else if (val == MXT_FRAME_CRC_FAIL) {
            LOG(LOG_ERROR, "Bootloader reports FRAME_CRC_FAIL");
            return -4;
        }
        break;
    default:
        return -2;
    }

    if (val != state) {
        LOG(LOG_ERROR, "Invalid bootloader mode state %X", val);
        return -2;
    }

    if (!ctx->have_bootloader_version
        && state == MXT_WAITING_BOOTLOAD_CMD)
    {
      if (bootloader_id | 0x20)
      {
        LOG(LOG_INFO, "Bootloader using extended ID mode");
        ctx->extended_id_mode = true;
      }
      else
      {
        bootloader_id &= 0x1f;
        LOG(LOG_INFO, "Bootloader ID:%d", bootloader_id);
        ctx->have_bootloader_version = true;
      }
    }

    return 0;
}

static int get_hex_value(struct bootloader_ctx *ctx, unsigned char *ptr)
{
  char str[] = "00\0";
  int val;
  int ret;

  str[0] = fgetc(ctx->fp);
  str[1] = fgetc(ctx->fp);

  if (feof(ctx->fp)) return EOF;

  ret = sscanf(str, "%x", &val);

  *ptr =  val;

  return ret;
}

static int send_frames(struct bootloader_ctx *ctx)
{
  unsigned char buffer[FIRMWARE_BUFFER_SIZE];
  int ret;
  int i;
  int frame_size = 0;
  int frame;
  int frame_retry = 0;
  int bytes_sent = 0;

  ctx->have_bootloader_version = false;
  ctx->extended_id_mode = false;

  ret = mxt_check_bootloader(ctx, MXT_WAITING_BOOTLOAD_CMD);
  if (ret == 0)
  {
    LOG(LOG_INFO, "Unlocking bootloader");

    if (unlock_bootloader(ctx) < 0) {
      LOG(LOG_ERROR, "Failure to unlock bootloader");
      return -1;
    }

    LOG(LOG_INFO, "Bootloader unlocked");
  }
  else if (ret == -3)
  {
    LOG(LOG_INFO, "Bootloader found");
  }
  else
  {
    LOG(LOG_ERROR, "Bootloader not found");
    return -1;
  }

  LOG(LOG_INFO, "Sending frames...");

  frame = 1;

  while (!feof(ctx->fp)) {
    if (frame_retry == 0) {
        if (get_hex_value(ctx, &buffer[0]) == EOF) {
            LOG(LOG_INFO, "End of file");
            break;
        }

        if (get_hex_value(ctx, &buffer[1]) == EOF) {
            LOG(LOG_ERROR, "Unexpected end of firmware file");
            return -1;
        }

        frame_size = (buffer[0] << 8) | buffer[1];

        LOG(LOG_DEBUG, "Frame %d: size %d", frame, frame_size);

        /* Allow for CRC bytes at end of frame */
        frame_size += 2;

        if (frame_size > FIRMWARE_BUFFER_SIZE) {
                LOG(LOG_ERROR, "Frame too big");
                return -1;
            }

        for (i = 2; i < frame_size; i++)
        {
            ret = get_hex_value(ctx, &buffer[i]);

            if (ret == EOF)
            {
                LOG(LOG_ERROR, "Unexpected end of firmware file");
                return -1;
            }
        }
    }

    if (mxt_check_bootloader(ctx, MXT_WAITING_FRAME_DATA) < 0) {
        LOG(LOG_ERROR, "Unexpected bootloader state");
        return -1;
    }

    /* Write one frame to device */
    mxt_bootloader_write(ctx->mxt, buffer, frame_size);

    // Check CRC
    LOG(LOG_VERBOSE, "Checking CRC");
    ret = mxt_check_bootloader(ctx, MXT_FRAME_CRC_PASS);
    if (ret) {
        if (frame_retry > 0) {
          LOG(LOG_ERROR, "Failure sending frame %d - aborting", frame);
          return -1;
        } else {
          frame_retry++;
          LOG(LOG_ERROR, "Frame %d: CRC fail, retry %d", frame, frame_retry);
        }
    } else {
        LOG(LOG_DEBUG, "CRC pass");
        frame++;
        bytes_sent += frame_size;
        if (frame % 20 == 0) {
          LOG(LOG_INFO, "Frame %d: Sent %d bytes", frame, bytes_sent);
        } else {
          LOG(LOG_VERBOSE, "Frame %d: Sent %d bytes", frame, bytes_sent);
        }
    }
  }

  fclose(ctx->fp);

  LOG(LOG_INFO, "Done");

  sleep(MXT_RESET_TIME);

  return 0;
}

static int lookup_bootloader_addr(struct bootloader_ctx *ctx, int addr)
{
  switch (addr)
  {
    case 0x4a:
    case 0x4b:
      if (ctx->mxt->info_block.id->family_id >= 0xa2)
      {
        return (addr - 0x24);
      }
      /* Fall through for normal case */
    case 0x4c:
    case 0x4d:
    case 0x5a:
    case 0x5b:
      return (addr - 0x26);
      break;
    default:
      return -1;
  }
}

static int mxt_bootloader_init_chip(struct bootloader_ctx *ctx)
{
  int ret;

  if (ctx->conn.type == E_I2C_DEV)
  {
    if (lookup_bootloader_addr(ctx, ctx->conn.i2c_dev.address) == -1)
    {
      LOG(LOG_INFO, "Trying bootloader");
      ctx->appmode_address = -1;
      return 0;
    }
  }
  else if (ctx->conn.type == E_NONE)
  {
    ret = mxt_scan(ctx->libmaxtouch_ctx, false, &ctx->conn);
    if (ret < 1)
    {
      LOG(LOG_INFO, "Could not find a device");
      return -1;
    }

    switch (ctx->conn.type)
    {
    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      LOG(LOG_INFO, "Switching to i2c-dev mode");
      ctx->conn.type = E_I2C_DEV;
      ctx->conn.i2c_dev.adapter = sysfs_get_i2c_adapter(ctx->mxt);
      ctx->conn.i2c_dev.address = sysfs_get_i2c_address(ctx->mxt);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      break;
#endif

    default:
      LOG(LOG_ERROR, "Unsupported device type");
      return -1;
    }
  }

  ret = mxt_new_device(ctx->libmaxtouch_ctx, ctx->conn, &ctx->mxt);
  if (ret)
  {
    LOG(LOG_ERROR, "Could not open device");
    return ret;
  }

  ret = mxt_get_info(ctx->mxt);
  if (ret)
  {
    LOG(LOG_ERROR, "Could not get info block");
    return ret;
  }

  LOG(LOG_INFO, "Chip detected");

#ifdef HAVE_LIBUSB
  if (ctx->conn.type == E_USB && usb_is_bootloader(ctx->mxt))
  {
    LOG(LOG_INFO, "USB device in bootloader mode");
    ctx->usb_bootloader = true;
    mxt_free_device(ctx->mxt);
    return 0;
  }
  else
  {
    ctx->usb_bootloader = false;
  }
#endif

  mxt_get_firmware_version(ctx->mxt, (char *)&ctx->curr_version);
  LOG(LOG_INFO, "Current firmware version: %s", ctx->curr_version);

  if (ctx->check_version && !strcmp((char *)&ctx->curr_version, ctx->new_version))
  {
    LOG(LOG_INFO, "Version already %s, exiting",
        ctx->curr_version);
    return -1;
  }
  else
  {
    LOG(LOG_INFO, "Skipping version check");
  }

  /* Change to the bootloader mode */
  ret = mxt_reset_chip(ctx->mxt, true);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Reset failure - aborting");
    return -1;
  }
  else
  {
    sleep(MXT_RESET_TIME);
  }

  if (ctx->mxt->conn.type == E_I2C_DEV)
  {
    ctx->appmode_address = ctx->conn.i2c_dev.address;
    ctx->conn.i2c_dev.address = lookup_bootloader_addr(ctx, ctx->appmode_address);
  }

  mxt_free_device(ctx->mxt);

  return 0;
}

//******************************************************************************
/// \brief  Flash firmware to chip
int mxt_flash_firmware(struct libmaxtouch_ctx *libmaxtouch_ctx,
                       struct mxt_device *maxtouch,
                       const char *filename, const char *new_version,
                       struct mxt_conn_info conn)
{
  struct bootloader_ctx ctx;
  int ret;

  ctx.libmaxtouch_ctx = libmaxtouch_ctx;
  ctx.mxt = maxtouch;
  ctx.conn = conn;

  LOG(LOG_INFO, "Opening firmware file %s", filename);

  ctx.fp = fopen(filename, "r");
  if (ctx.fp == NULL)
  {
    LOG(LOG_ERROR, "Cannot open firmware file %s!", filename);
    return -1;
  }

  if (strlen(new_version) > 0)
  {
    ctx.check_version = true;
    ctx.new_version = new_version;
    LOG(LOG_DEBUG, "New firmware version is:%s", ctx.new_version);
  }
  else
  {
    ctx.check_version = false;
    LOG(LOG_DEBUG, "check_version:%d", ctx.check_version);
  }

  ret = mxt_bootloader_init_chip(&ctx);
  if (ret)
    return ret;

  if (ctx.conn.type == E_I2C_DEV)
  {
    if (ctx.conn.i2c_dev.address == -1)
    {
      LOG(LOG_ERROR, "No bootloader address!");
      return -1;
    }

    LOG(LOG_DEBUG, "i2c_adapter:%d", ctx.conn.i2c_dev.adapter);
    LOG(LOG_DEBUG, "bootloader_address:%02X", ctx.conn.i2c_dev.address);
    LOG(LOG_DEBUG, "appmode_address:%02X", ctx.appmode_address);
  }

  ret = mxt_new_device(ctx.libmaxtouch_ctx, ctx.conn, &ctx.mxt);
  if (ret < 0)
  {
    LOG(LOG_INFO, "Could not initialise chip");
    return ret;
  }

  ret = send_frames(&ctx);
  if (ret != 0)
    return ret;

  mxt_free_device(ctx.mxt);

  if (ctx.conn.type == E_I2C_DEV && ctx.conn.i2c_dev.address < 0)
  {
    LOG(LOG_INFO, "Sent all firmware frames");
    return 0;
  }

  if (ctx.mxt->conn.type == E_I2C_DEV)
  {
    ctx.conn.i2c_dev.address = ctx.appmode_address;
  }

  ret = mxt_new_device(ctx.libmaxtouch_ctx, ctx.conn, &ctx.mxt);
  if (ret != 0)
  {
    LOG(LOG_ERROR, "FAILURE - chip did not reset");
    return -1;
  }

#ifdef HAVE_LIBUSB
  if (ctx.mxt->conn.type == E_USB && usb_is_bootloader(ctx.mxt))
  {
    LOG(LOG_ERROR, "USB device still in bootloader mode");
    ret = -1;
    goto release;
  }
#endif

  ret = mxt_get_info(ctx.mxt);
  if (ret != 0)
  {
    LOG(LOG_ERROR, "Failed to get info block");
    ret = -1;
    goto release;
  }

  mxt_get_firmware_version(ctx.mxt, (char *)&ctx.curr_version);

  if (!ctx.check_version)
  {
    LOG(LOG_INFO, "SUCCESS - version is %s", ctx.curr_version);
    ret = 0;
    goto release;
  }

  if (!strcmp(ctx.curr_version, ctx.new_version))
  {
    LOG(LOG_INFO, "SUCCESS - version %s verified", ctx.curr_version);
    ret = 0;
  }
  else
  {
    LOG(LOG_ERROR, "FAILURE - detected version is %s", ctx.curr_version);
    ret = -1;
  }

release:
  mxt_free_device(ctx.mxt);
  return ret;
}
