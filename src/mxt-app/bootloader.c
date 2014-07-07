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
#include <errno.h>
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
struct flash_context {
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

//******************************************************************************
/// \brief Wait for CHG line to indicate bootloader state change
/// \return #mxt_rc
static int wait_for_chg(struct mxt_device *mxt)
{
#ifdef HAVE_LIBUSB
  int try = 0;
  int ret;
  bool chg;

  if (mxt->conn->type == E_USB) {
    while (true) {
      ret = usb_read_chg(mxt, &chg);
      if (ret)
        return ret;

      if (!chg)
        break;

      if (++try > 100) {
          mxt_warn(mxt->ctx, "Timed out awaiting CHG");
          return MXT_ERROR_TIMEOUT;
        }

      usleep(1000);
    }

    mxt_verb(mxt->ctx, "CHG line cycles %d", try);
  } else
#endif
  {
    usleep(MXT_BOOTLOADER_DELAY);
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Send a frame with length field set to 0x0000. This should force a
//         bootloader reset
/// \return #mxt_rc
static int send_zero_frame(struct flash_context *fw)
{
  unsigned char buf[2];

  buf[0] = 0;
  buf[1] = 0;

  mxt_info(fw->ctx, "Attempting bootloader reset");

  return mxt_bootloader_write(fw->mxt, buf, sizeof(buf));
}

//******************************************************************************
/// \brief Send command to unlock bootloader
/// \return #mxt_rc
static int unlock_bootloader(struct flash_context *fw)
{
  unsigned char buf[2];

  buf[0] = MXT_UNLOCK_CMD_LSB;
  buf[1] = MXT_UNLOCK_CMD_MSB;

  return mxt_bootloader_write(fw->mxt, buf, sizeof(buf));
}

//******************************************************************************
/// \brief Read bootloader state
/// \return #mxt_rc
static int mxt_check_bootloader(struct flash_context *fw, unsigned int state)
{
  unsigned char buf[3];
  unsigned char val;
  unsigned char bootloader_id;
  unsigned char bootloader_version;
  int ret;

recheck:
  if (state != MXT_WAITING_BOOTLOAD_CMD)
    wait_for_chg(fw->mxt);

  if ((!fw->have_bootloader_version) && fw->extended_id_mode
      && (state == MXT_WAITING_FRAME_DATA)) {
    mxt_dbg(fw->ctx, "Attempting to retrieve bootloader version");
    ret = mxt_bootloader_read(fw->mxt, buf, sizeof(buf));
    if (ret)
      return ret;

    val = buf[0];
    bootloader_id = buf[1];
    bootloader_version = buf[2];

    mxt_info(fw->ctx, "Bootloader ID:%d Version:%d",
             bootloader_id, bootloader_version);

    fw->have_bootloader_version = true;
  } else {
    ret = mxt_bootloader_read(fw->mxt, &val, 1);
    if (ret) {
      mxt_err(fw->ctx, "Bootloader read failure");
      return ret;
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
      return MXT_ERROR_BOOTLOADER_UNLOCKED;
    }

    break;
  case MXT_WAITING_FRAME_DATA:
    val &= ~MXT_BOOT_STATUS_MASK;
    break;
  case MXT_FRAME_CRC_PASS:
    if (val == MXT_FRAME_CRC_CHECK) {
      goto recheck;
    } else if (val == MXT_FRAME_CRC_FAIL) {
      mxt_info(fw->ctx, "Bootloader reports FRAME_CRC_FAIL");
      return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
    }
    break;
  default:
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }

  if (val != state) {
    mxt_info(fw->ctx, "Invalid bootloader mode state %02X", val);

    if (state == MXT_WAITING_BOOTLOAD_CMD)
      send_zero_frame(fw);

    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }

  if (!fw->have_bootloader_version
      && state == MXT_WAITING_BOOTLOAD_CMD) {
    if (bootloader_id | 0x20) {
      mxt_dbg(fw->ctx, "Bootloader using extended ID mode");
      fw->extended_id_mode = true;
    } else {
      bootloader_id &= 0x1f;
      mxt_info(fw->ctx, "Bootloader ID:%d", bootloader_id);
      fw->have_bootloader_version = true;
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read hexadecimal value from file
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

//******************************************************************************
/// \brief Send firmware frames to bootloader
/// \return #mxt_rc
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
  if (ret == MXT_SUCCESS) {
    mxt_info(fw->ctx, "Unlocking bootloader");

    ret = unlock_bootloader(fw);
    if (ret) {
      mxt_err(fw->ctx, "Failure to unlock bootloader");
      return ret;
    }

    mxt_info(fw->ctx, "Bootloader unlocked");
  } else if (ret == MXT_ERROR_BOOTLOADER_UNLOCKED) {
    mxt_info(fw->ctx, "Bootloader found");
  } else {
    mxt_err(fw->ctx, "Bootloader not found");
    return MXT_ERROR_NO_DEVICE;
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
        return MXT_ERROR_FILE_FORMAT;
      }

      frame_size = (buffer[0] << 8) | buffer[1];

      mxt_dbg(fw->ctx, "Frame %d: size %d", frame, frame_size);

      /* Allow for CRC bytes at end of frame */
      frame_size += 2;

      if (frame_size > FIRMWARE_BUFFER_SIZE) {
        mxt_err(fw->ctx, "Frame too big");
        return MXT_ERROR_NO_MEM;
      }

      for (i = 2; i < frame_size; i++) {
        ret = get_hex_value(fw, &buffer[i]);

        if (ret == EOF) {
          mxt_err(fw->ctx, "Unexpected end of firmware file");
          return MXT_ERROR_FILE_FORMAT;
        }
      }
    }

    if (mxt_check_bootloader(fw, MXT_WAITING_FRAME_DATA) < 0) {
      mxt_err(fw->ctx, "Unexpected bootloader state");
      return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
    }

    /* Write one frame to device */
    ret = mxt_bootloader_write(fw->mxt, buffer, frame_size);
    if (ret)
      return ret;

    // Check CRC
    mxt_verb(fw->ctx, "Checking CRC");
    ret = mxt_check_bootloader(fw, MXT_FRAME_CRC_PASS);
    if (ret == MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL) {
      if (frame_retry > 0) {
        mxt_err(fw->ctx, "Failure sending frame %d - aborting", frame);
        return MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL;
      } else {
        frame_retry++;
        mxt_err(fw->ctx, "Frame %d: CRC fail, retry %d", frame, frame_retry);
      }
    } else if (ret) {
      mxt_err(fw->ctx, "Unexpected bootloader state");
      return ret;
    } else {
      mxt_verb(fw->ctx, "CRC pass");
      frame++;
      bytes_sent += frame_size;
      if (frame % 20 == 0) {
        mxt_info(fw->ctx, "Sent %d frames, %d bytes", frame, bytes_sent);
      } else {
        mxt_verb(fw->ctx, "Sent %d frames, %d bytes", frame, bytes_sent);
      }
    }
  }

  fclose(fw->fp);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Lookup bootloader I2C address
static int lookup_bootloader_addr(struct flash_context *fw, int addr)
{
  switch (addr) {
  case 0x4a:
  case 0x4b:
    if (fw->mxt->info.id->family >= 0xa2) {
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

//******************************************************************************
/// \brief Initialise chip in bootloader mode
/// \return #mxt_rc
static int mxt_bootloader_init_chip(struct flash_context *fw)
{
  int ret;

  if (!fw->conn) {
    ret = mxt_scan(fw->ctx, &fw->conn, false);
    if (ret) {
      mxt_info(fw->ctx, "Could not find a device");
      return ret;
    }
  }

  switch (fw->conn->type) {
  case E_SYSFS:
    mxt_info(fw->ctx, "Switching to i2c-dev mode");

    struct mxt_conn_info *new_conn;
    ret = mxt_new_conn(&new_conn, E_I2C_DEV);
    if (ret)
      return ret;

    ret = sscanf(basename(fw->conn->sysfs.path), "%d-%x",
                 &fw->i2c_adapter, &fw->appmode_address);
    if (ret != 2) {
      mxt_err(fw->ctx, "Couldn't parse sysfs path for adapter/address");
      return MXT_INTERNAL_ERROR;
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
    if (fw->conn->i2c_dev.address < 0x4a) {
      mxt_info(fw->ctx, "Using bootloader address");
      fw->appmode_address = -1;
      return MXT_SUCCESS;
    }
    break;
  }

  ret = mxt_new_device(fw->ctx, fw->conn, &fw->mxt);
  if (ret) {
    mxt_err(fw->ctx, "Could not open device");
    return ret;
  }

#ifdef HAVE_LIBUSB
  if (fw->conn->type == E_USB && usb_is_bootloader(fw->mxt)) {
    mxt_info(fw->ctx, "USB device in bootloader mode");
    fw->usb_bootloader = true;
    mxt_free_device(fw->mxt);
    return MXT_SUCCESS;
  } else {
    fw->usb_bootloader = false;
  }
#endif

  ret = mxt_get_info(fw->mxt);
  if (ret) {
    mxt_err(fw->ctx, "Could not get info block");
    return ret;
  }

  mxt_info(fw->ctx, "Chip detected");

  mxt_get_firmware_version(fw->mxt, (char *)&fw->curr_version);
  mxt_info(fw->ctx, "Current firmware version: %s", fw->curr_version);

  if (!fw->check_version) {
    mxt_info(fw->ctx, "Skipping version check");
  } else if (!strcmp((char *)&fw->curr_version, fw->new_version)) {
    mxt_info(fw->ctx, "Version already %s, exiting",
             fw->curr_version);
    return MXT_FIRMWARE_UPDATE_NOT_REQUIRED;
  }

  /* Change to the bootloader mode */
  ret = mxt_reset_chip(fw->mxt, true);
  if (ret) {
    mxt_err(fw->ctx, "Reset failure - aborting");
    return ret;
  } else {
    sleep(MXT_RESET_TIME);
  }

  if (fw->conn->type == E_I2C_DEV) {
    fw->appmode_address = fw->conn->i2c_dev.address;

    fw->conn->i2c_dev.address = lookup_bootloader_addr(fw, fw->appmode_address);
    if (fw->conn->i2c_dev.address == -1) {
      mxt_err(fw->ctx, "No bootloader address!");
      return MXT_ERROR_BOOTLOADER_NO_ADDRESS;
    }

    mxt_dbg(fw->ctx, "I2C Adapter:%d", fw->conn->i2c_dev.adapter);
    mxt_dbg(fw->ctx, "Bootloader addr:0x%02x", fw->conn->i2c_dev.address);
    mxt_dbg(fw->ctx, "App mode addr:0x%02x", fw->appmode_address);
  }

  mxt_free_device(fw->mxt);

  return MXT_SUCCESS;
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
  if (!fw.fp) {
    mxt_err(fw.ctx, "Cannot open firmware file %s!", filename);
    return mxt_errno_to_rc(errno);
  }

  if (strlen(new_version) > 0) {
    fw.check_version = true;
    fw.new_version = new_version;
    mxt_dbg(fw.ctx, "New firmware version is:%s", fw.new_version);
  } else {
    fw.check_version = false;
    mxt_dbg(fw.ctx, "check_version:%d", fw.check_version);
  }

  ret = mxt_bootloader_init_chip(&fw);
  if (ret)
    return ret;

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret) {
    mxt_info(fw.ctx, "Could not initialise chip");
    return ret;
  }

  ret = send_frames(&fw);
  if (ret)
    return ret;

  /* Handle transition back to appmode address */
  if (fw.mxt->conn->type == E_I2C_DEV) {
    sleep(MXT_RESET_TIME);

    if (fw.appmode_address < 0) {
      mxt_info(fw.ctx, "Sent all firmware frames");
      ret = 0;
      goto release;
    } else {
      mxt_info(fw.ctx, "Switching back to app mode");
      struct mxt_conn_info *new_conn;
      ret = mxt_new_conn(&new_conn, E_I2C_DEV);
      if (ret)
        return ret;

      new_conn->i2c_dev.adapter = fw.i2c_adapter;
      new_conn->i2c_dev.address = fw.appmode_address;

      mxt_unref_conn(fw.conn);
      fw.conn = new_conn;
    }
  }
#ifdef HAVE_LIBUSB
  else if (fw.mxt->conn->type == E_USB) {
    bool bus_devices[USB_MAX_BUS_DEVICES] = { 0 };
    int tries = 10;

    ret = usb_find_bus_devices(fw.mxt, bus_devices);
    if (ret)
      return ret;

    while (tries--) {
      sleep(MXT_RESET_TIME);

      ret = usb_rediscover_device(fw.mxt, bus_devices);
      if (ret == MXT_SUCCESS)
        break;
    }

    if (ret) {
      mxt_err(fw.ctx, "Did not find device after reset");
      return ret;
    }
  }
#endif

  mxt_free_device(fw.mxt);

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret) {
    mxt_err(fw.ctx, "FAILURE - chip did not reset");
    return MXT_ERROR_RESET_FAILURE;
  }

#ifdef HAVE_LIBUSB
  if (fw.mxt->conn->type == E_USB && usb_is_bootloader(fw.mxt)) {
    mxt_err(fw.ctx, "USB device still in bootloader mode");
    ret = MXT_ERROR_RESET_FAILURE;
    goto release;
  }
#endif

  ret = mxt_get_info(fw.mxt);
  if (ret) {
    mxt_err(fw.ctx, "Failed to get info block");
    goto release;
  }

  mxt_get_firmware_version(fw.mxt, (char *)&fw.curr_version);

  if (!fw.check_version) {
    mxt_info(fw.ctx, "SUCCESS - version is %s", fw.curr_version);
    ret = MXT_SUCCESS;
    goto release;
  }

  if (!strcmp(fw.curr_version, fw.new_version)) {
    mxt_info(fw.ctx, "SUCCESS - version %s verified", fw.curr_version);
    ret = MXT_SUCCESS;
  } else {
    mxt_err(fw.ctx, "FAILURE - detected version is %s", fw.curr_version);
    ret = MXT_ERROR_FIRMWARE_UPDATE_FAILED;
  }

release:
  mxt_free_device(fw.mxt);
  mxt_unref_conn(fw.conn);
  return ret;
}
