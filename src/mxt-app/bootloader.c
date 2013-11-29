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
#include <libgen.h>
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
struct flash_context
{
  struct mxt_device *mxt;
  struct mxt_conn_info *conn;
  struct libmaxtouch_ctx *ctx;
  bool have_bootloader_version;
  bool extended_id_mode;
  FILE *fp;
  char curr_version[MXT_FW_VER_LEN];
  int i2c_adapter;
  int appmode_address;
  int bootloader_address;
  bool check_version;
  const char *new_version;
  bool usb_bootloader;
};

static int wait_for_chg(struct mxt_device *mxt)
{
#ifdef HAVE_LIBUSB
  int try = 0;
  if (mxt->conn->type == E_USB)
  {
    while (usb_read_chg(mxt))
    {
      if (++try > 100)
      {
        mxt_warn(mxt->ctx, "Timed out awaiting CHG");
        return -1;
      }

      usleep(1000);
    }

    mxt_verb(mxt->ctx, "CHG line cycles %d", try);
  }
  else
#endif
  {
    usleep(MXT_BOOTLOADER_DELAY);
  }

  return 0;
}

static int unlock_bootloader(struct flash_context *fw)
{
  unsigned char buf[2];

  buf[0] = MXT_UNLOCK_CMD_LSB;
  buf[1] = MXT_UNLOCK_CMD_MSB;

  return mxt_bootloader_write(fw->mxt, buf, sizeof(buf));
}

static int mxt_check_bootloader(struct flash_context *fw, unsigned int state)
{
    unsigned char buf[3];
    unsigned char val;
    unsigned char bootloader_id;
    unsigned char bootloader_version;

recheck:
    if (state != MXT_WAITING_BOOTLOAD_CMD)
      wait_for_chg(fw->mxt);

    if ((!fw->have_bootloader_version) && fw->extended_id_mode
        && (state == MXT_WAITING_FRAME_DATA))
    {
       mxt_info(fw->ctx, "Attempting to retrieve bootloader version");
       if (mxt_bootloader_read(fw->mxt, &buf[0], 3) != 0) {
           return -1;
       }

       val = buf[0];
       bootloader_id = buf[1];
       bootloader_version = buf[2];

       mxt_info(fw->ctx, "Bootloader ID:%d Version:%d",
         bootloader_id, bootloader_version);

       fw->have_bootloader_version = true;
    }
    else
    {
      if (mxt_bootloader_read(fw->mxt, &val, 1) != 0) {
        return -1;
      }
    }

    mxt_verb(fw->ctx, "Bootloader status %02X", val);

    switch (state) {
    case MXT_WAITING_BOOTLOAD_CMD:
        bootloader_id = val & MXT_BOOT_STATUS_MASK;
        val &= ~MXT_BOOT_STATUS_MASK;

        if (val == MXT_APP_CRC_FAIL) {
            mxt_info(fw->ctx, "Bootloader reports APP CRC failure");
            goto recheck;
        } else if (val == MXT_WAITING_FRAME_DATA) {
            mxt_info(fw->ctx, "Bootloader already unlocked");
            return -3;
        }

        break;
    case MXT_WAITING_FRAME_DATA:
        if (val == MXT_FRAME_CRC_PASS) {
          mxt_info(fw->ctx, "Bootloader still giving CRC PASS");
          goto recheck;
        }
        val &= ~MXT_BOOT_STATUS_MASK;
        break;
    case MXT_FRAME_CRC_PASS:
        if (val == MXT_FRAME_CRC_CHECK) {
            goto recheck;
        } else if (val == MXT_FRAME_CRC_FAIL) {
            mxt_info(fw->ctx, "Bootloader reports FRAME_CRC_FAIL");
            return -4;
        }
        break;
    default:
        return -2;
    }

    if (val != state) {
        mxt_info(fw->ctx, "Invalid bootloader mode state %X", val);
        return -2;
    }

    if (!fw->have_bootloader_version
        && state == MXT_WAITING_BOOTLOAD_CMD)
    {
      if (bootloader_id | 0x20)
      {
        mxt_info(fw->ctx, "Bootloader using extended ID mode");
        fw->extended_id_mode = true;
      }
      else
      {
        bootloader_id &= 0x1f;
        mxt_info(fw->ctx, "Bootloader ID:%d", bootloader_id);
        fw->have_bootloader_version = true;
      }
    }

    return 0;
}

static int get_hex_value(struct flash_context *fw, unsigned char *ptr)
{
  char str[] = "00\0";
  int val;
  int ret;

  str[0] = fgetc(fw->fp);
  str[1] = fgetc(fw->fp);

  if (feof(fw->fp)) return EOF;

  ret = sscanf(str, "%x", &val);

  *ptr =  val;

  return ret;
}

static int send_frames(struct flash_context *fw)
{
  unsigned char buffer[FIRMWARE_BUFFER_SIZE];
  int ret;
  int i;
  int frame_size = 0;
  int frame;
  int frame_retry = 0;
  int bytes_sent = 0;

  fw->have_bootloader_version = false;
  fw->extended_id_mode = false;

  ret = mxt_check_bootloader(fw, MXT_WAITING_BOOTLOAD_CMD);
  if (ret == 0)
  {
    mxt_info(fw->ctx, "Unlocking bootloader");

    if (unlock_bootloader(fw) < 0) {
      mxt_err(fw->ctx, "Failure to unlock bootloader");
      return -1;
    }

    mxt_info(fw->ctx, "Bootloader unlocked");
  }
  else if (ret == -3)
  {
    mxt_info(fw->ctx, "Bootloader found");
  }
  else
  {
    mxt_err(fw->ctx, "Bootloader not found");
    return -1;
  }

  mxt_info(fw->ctx, "Sending frames...");

  frame = 1;

  while (!feof(fw->fp)) {
    if (frame_retry == 0) {
        if (get_hex_value(fw, &buffer[0]) == EOF) {
            mxt_info(fw->ctx, "End of file");
            break;
        }

        if (get_hex_value(fw, &buffer[1]) == EOF) {
            mxt_err(fw->ctx, "Unexpected end of firmware file");
            return -1;
        }

        frame_size = (buffer[0] << 8) | buffer[1];

        mxt_dbg(fw->ctx, "Frame %d: size %d", frame, frame_size);

        /* Allow for CRC bytes at end of frame */
        frame_size += 2;

        if (frame_size > FIRMWARE_BUFFER_SIZE) {
                mxt_err(fw->ctx, "Frame too big");
                return -1;
            }

        for (i = 2; i < frame_size; i++)
        {
            ret = get_hex_value(fw, &buffer[i]);

            if (ret == EOF)
            {
                mxt_err(fw->ctx, "Unexpected end of firmware file");
                return -1;
            }
        }
    }

    if (mxt_check_bootloader(fw, MXT_WAITING_FRAME_DATA) < 0) {
        mxt_err(fw->ctx, "Unexpected bootloader state");
        return -1;
    }

    /* Write one frame to device */
    mxt_bootloader_write(fw->mxt, buffer, frame_size);

    // Check CRC
    mxt_verb(fw->ctx, "Checking CRC");
    ret = mxt_check_bootloader(fw, MXT_FRAME_CRC_PASS);
    if (ret) {
        if (frame_retry > 0) {
          mxt_err(fw->ctx, "Failure sending frame %d - aborting", frame);
          return -1;
        } else {
          frame_retry++;
          mxt_err(fw->ctx, "Frame %d: CRC fail, retry %d", frame, frame_retry);
        }
    } else {
        mxt_dbg(fw->ctx, "CRC pass");
        frame++;
        bytes_sent += frame_size;
        if (frame % 20 == 0) {
          mxt_info(fw->ctx, "Frame %d: Sent %d bytes", frame, bytes_sent);
        } else {
          mxt_verb(fw->ctx, "Frame %d: Sent %d bytes", frame, bytes_sent);
        }
    }
  }

  fclose(fw->fp);

  return 0;
}

static int lookup_bootloader_addr(struct flash_context *fw, int addr)
{
  switch (addr)
  {
    case 0x4a:
    case 0x4b:
      if (fw->mxt->info_block.id->family_id >= 0xa2)
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

static int mxt_bootloader_init_chip(struct flash_context *fw)
{
  int ret;

  if (!fw->conn)
  {
    ret = mxt_scan(fw->ctx, &fw->conn, false);
    if (ret < 1)
    {
      mxt_info(fw->ctx, "Could not find a device");
      return -1;
    }
  }

  switch (fw->conn->type)
  {
    case E_SYSFS:
      mxt_info(fw->ctx, "Switching to i2c-dev mode");

      struct mxt_conn_info *new_conn;
      ret = mxt_new_conn(&new_conn, E_I2C_DEV);
      if (ret < 0)
        return ret;

      ret = sscanf(basename(fw->conn->sysfs.path), "%d-%x",
                   &fw->i2c_adapter, &fw->appmode_address);
      if (ret != 2)
      {
        mxt_err(fw->ctx, "Couldn't parse sysfs path for adapter/address");
        return -1;
      }

      new_conn->i2c_dev.adapter = fw->i2c_adapter;
      new_conn->i2c_dev.address = fw->appmode_address;

      mxt_unref_conn(fw->conn);
      fw->conn = new_conn;
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      break;
#endif

    case E_I2C_DEV:
      if (lookup_bootloader_addr(fw, fw->conn->i2c_dev.address) == -1)
      {
        mxt_info(fw->ctx, "Trying bootloader");
        fw->appmode_address = -1;
        return 0;
      }
      break;
  }

  ret = mxt_new_device(fw->ctx, fw->conn, &fw->mxt);
  if (ret)
  {
    mxt_err(fw->ctx, "Could not open device");
    return ret;
  }

#ifdef HAVE_LIBUSB
  if (fw->conn->type == E_USB && usb_is_bootloader(fw->mxt))
  {
    mxt_info(fw->ctx, "USB device in bootloader mode");
    fw->usb_bootloader = true;
    mxt_free_device(fw->mxt);
    return 0;
  }
  else
  {
    fw->usb_bootloader = false;
  }
#endif

  ret = mxt_get_info(fw->mxt);
  if (ret)
  {
    mxt_err(fw->ctx, "Could not get info block");
    return ret;
  }

  mxt_info(fw->ctx, "Chip detected");

  mxt_get_firmware_version(fw->mxt, (char *)&fw->curr_version);
  mxt_info(fw->ctx, "Current firmware version: %s", fw->curr_version);

  if (fw->check_version && !strcmp((char *)&fw->curr_version, fw->new_version))
  {
    mxt_info(fw->ctx, "Version already %s, exiting",
        fw->curr_version);
    return -1;
  }
  else
  {
    mxt_info(fw->ctx, "Skipping version check");
  }

  /* Change to the bootloader mode */
  ret = mxt_reset_chip(fw->mxt, true);
  if (ret < 0)
  {
    mxt_err(fw->ctx, "Reset failure - aborting");
    return -1;
  }
  else
  {
    sleep(MXT_RESET_TIME);
  }

  if (fw->conn->type == E_I2C_DEV)
  {
    fw->appmode_address = fw->conn->i2c_dev.address;

    fw->conn->i2c_dev.address = lookup_bootloader_addr(fw, fw->appmode_address);
    if (fw->conn->i2c_dev.address == -1)
    {
      mxt_err(fw->ctx, "No bootloader address!");
      return -1;
    }

    mxt_dbg(fw->ctx, "I2C Adapter:%d", fw->conn->i2c_dev.adapter);
    mxt_dbg(fw->ctx, "Bootloader addr:0x%02x", fw->conn->i2c_dev.address);
    mxt_dbg(fw->ctx, "App mode addr:0x%02x", fw->appmode_address);
  }

  mxt_free_device(fw->mxt);

  return 0;
}

//******************************************************************************
/// \brief  Flash firmware to chip
int mxt_flash_firmware(struct libmaxtouch_ctx *ctx,
                       struct mxt_device *maxtouch,
                       const char *filename, const char *new_version,
                       struct mxt_conn_info *conn)
{
  struct flash_context fw;
  int ret;

  fw.ctx = ctx;
  fw.mxt = maxtouch;
  fw.conn = conn;

  mxt_info(fw.ctx, "Opening firmware file %s", filename);

  fw.fp = fopen(filename, "r");
  if (fw.fp == NULL)
  {
    mxt_err(fw.ctx, "Cannot open firmware file %s!", filename);
    return -1;
  }

  if (strlen(new_version) > 0)
  {
    fw.check_version = true;
    fw.new_version = new_version;
    mxt_dbg(fw.ctx, "New firmware version is:%s", fw.new_version);
  }
  else
  {
    fw.check_version = false;
    mxt_dbg(fw.ctx, "check_version:%d", fw.check_version);
  }

  ret = mxt_bootloader_init_chip(&fw);
  if (ret)
    return ret;

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret < 0)
  {
    mxt_info(fw.ctx, "Could not initialise chip");
    return ret;
  }

  ret = send_frames(&fw);
  if (ret != 0)
    return ret;

  /* Handle transition back to appmode address */
  if (fw.mxt->conn->type == E_I2C_DEV)
  {
    sleep(MXT_RESET_TIME);

    if (fw.appmode_address < 0)
    {
      mxt_info(fw.ctx, "Sent all firmware frames");
      ret = 0;
      goto release;
    }
    else
    {
      mxt_info(fw.ctx, "Switching back to app mode");
      struct mxt_conn_info *new_conn;
      ret = mxt_new_conn(&new_conn, E_I2C_DEV);
      if (ret < 0)
        return ret;

      new_conn->i2c_dev.adapter = fw.i2c_adapter;
      new_conn->i2c_dev.address = fw.appmode_address;

      mxt_unref_conn(fw.conn);
      fw.conn = new_conn;
    }
  }
#ifdef HAVE_LIBUSB
  else if (fw.mxt->conn->type == E_USB)
  {
    int usb_device_number;
    int tries = 10;

    ret = usb_find_max_address(fw.mxt, &usb_device_number);
    if (ret < 0)
      return ret;

    while (tries--)
    {
      sleep(MXT_RESET_TIME);

      ret = usb_rediscover_device(fw.mxt, usb_device_number);
      if (ret == 0)
        break;
    }

    if (ret < 0)
    {
      mxt_err(fw.ctx, "Did not find device after reset");
      return ret;
    }
  }
#endif

  mxt_free_device(fw.mxt);

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret != 0)
  {
    mxt_err(fw.ctx, "FAILURE - chip did not reset");
    return -1;
  }

#ifdef HAVE_LIBUSB
  if (fw.mxt->conn->type == E_USB && usb_is_bootloader(fw.mxt))
  {
    mxt_err(fw.ctx, "USB device still in bootloader mode");
    ret = -1;
    goto release;
  }
#endif

  ret = mxt_get_info(fw.mxt);
  if (ret != 0)
  {
    mxt_err(fw.ctx, "Failed to get info block");
    ret = -1;
    goto release;
  }

  mxt_get_firmware_version(fw.mxt, (char *)&fw.curr_version);

  if (!fw.check_version)
  {
    mxt_info(fw.ctx, "SUCCESS - version is %s", fw.curr_version);
    ret = 0;
    goto release;
  }

  if (!strcmp(fw.curr_version, fw.new_version))
  {
    mxt_info(fw.ctx, "SUCCESS - version %s verified", fw.curr_version);
    ret = 0;
  }
  else
  {
    mxt_err(fw.ctx, "FAILURE - detected version is %s", fw.curr_version);
    ret = -1;
  }

release:
  mxt_free_device(fw.mxt);
  mxt_unref_conn(fw.conn);
  return ret;
}
