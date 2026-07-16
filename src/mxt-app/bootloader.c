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
#include <errno.h>
#include <ctype.h>
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
#define MXT_BOOT_ID_MASK         0x1f

/* Bootloader mode status for FW Authentication */
#define MXT_SIGNATURE_MODE        0x0C
#define MXT_INFO_VALID            0x0D
#define MXT_INFO_INVALID          0x0E
#define MXT_SIGN_IN_PROGRESS      0x0F
#define MXT_SIGN_MODE_FINISHED    0x10

#define FIRMWARE_BUFFER_SIZE     1024

#define MXT_RESET_TIME           1
#define MXT_BOOTLOADER_DELAY     50000

#define HEX_PER_LINE             128
#define BYTES_PER_LINE           64
#define SIG_HEADER_SIZE          5
#define HASH_SIG_SIZE            64

//******************************************************************************
/// \brief Bootloader context object
struct flash_context {
  struct mxt_device *mxt;
  struct mxt_conn_info *conn;
  struct libmaxtouch_ctx *ctx;
  bool have_bootloader_version;
  bool extended_id_mode;
  FILE *fp;
  long file_size;
  char curr_version[MXT_FW_VER_LEN];
  int i2c_adapter;
  int appmode_address;
  int bootloader_address;
  bool check_version;
  const char *new_version;
  bool usb_bootloader;
  bool is_mtch_driver;
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
    bootloader_id = buf[1] & MXT_BOOT_ID_MASK;
    bootloader_version = buf[2];

    mxt_info(fw->ctx, "Bootloader ID: %d Version: %d",
             bootloader_id, bootloader_version);

    fw->have_bootloader_version = true;
  } else {
    ret = mxt_bootloader_read(fw->mxt, &val, 1);
    if (ret) {
      mxt_err(fw->ctx, "Bootloader read failure");
      return ret;
    }
  }

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
    if (bootloader_id & 0x20) {
      mxt_dbg(fw->ctx, "Bootloader using extended ID mode");
      fw->extended_id_mode = true;
    } else {
      bootloader_id &= MXT_BOOT_ID_MASK;
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
  uint8_t last_percent = 100;
  uint8_t cur_percent = 0;
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
      frame_retry = 0;
      frame++;
      bytes_sent += frame_size;
      cur_percent = (unsigned char)(0.5f + (100.0 * ftell(fw->fp)) / fw->file_size);

      /* Display at 10% or difference is greater than 10% */
      if (cur_percent % 10 == 0 || (cur_percent - last_percent) > 10) {
        /* No need to repeat for the same percentage */
        if (last_percent != cur_percent) {
          mxt_info(fw->ctx, "Sent %d frames, %d bytes. % 3d%%", frame, bytes_sent, cur_percent);
          last_percent = cur_percent;
        }
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
  /* MTCH devices: 0x3a -> 0x16, 0x3b -> 0x17 */
  if (fw->is_mtch_driver) {
    switch (addr) {
    case 0x3a:
    case 0x3b:
      return (addr - 0x24);
    default:
      return -1;
    }
  }

  /* MXT devices */
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
  struct mxt_conn_info *new_conn;
  int ret;

  /* Always scan if no connection or if sysfs path not yet populated */
  if (!fw->conn ||
      ((fw->conn->type == E_SYSFS_I2C || fw->conn->type == E_SYSFS_SPI) &&
       !fw->conn->sysfs.path)) {
    ret = mxt_scan(fw->ctx, &fw->conn, false);
    if (ret) {
      mxt_info(fw->ctx, "Could not find a device");
      return ret;
    }
  }

  switch (fw->conn->type) {
  case E_SYSFS_SPI:
     //Do nothing, no init needed
    break;
     
  case E_SYSFS_I2C:
    mxt_info(fw->ctx, "Switching to i2c-dev mode");

    /* Save driver type before switching connection */
    fw->is_mtch_driver = (fw->conn->sysfs.driver == SYSFS_DRIVER_MTCH);

    struct mxt_conn_info *new_conn;
    ret = mxt_new_conn(&new_conn, E_I2C_DEV);
    if (ret)
      return ret;

    ret = sysfs_get_i2c_address(fw->ctx, fw->conn,
                                &fw->i2c_adapter, &fw->appmode_address);
    if (ret)
      return ret;

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
    fw->i2c_adapter = fw->conn->i2c_dev.adapter;
    /* Check if already in bootloader mode based on address */
    /* MXT bootloader: 0x24-0x27, MTCH bootloader: 0x16-0x17 */
    if (fw->conn->i2c_dev.address >= 0x16 && fw->conn->i2c_dev.address <= 0x17) {
      mxt_info(fw->ctx, "Using MTCH bootloader address");
      fw->appmode_address = -1;
      fw->is_mtch_driver = true;
      return MXT_DEVICE_IN_BOOTLOADER;
    } else if (fw->conn->i2c_dev.address >= 0x24 && fw->conn->i2c_dev.address <= 0x27) {
      mxt_info(fw->ctx, "Using MXT bootloader address");
      fw->appmode_address = -1;
      return MXT_DEVICE_IN_BOOTLOADER;
    }
    break;

  case E_HIDRAW:
    mxt_err(fw->ctx, "Device type not supported");

    return MXT_ERROR_NOT_SUPPORTED;
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
    return MXT_DEVICE_IN_BOOTLOADER;
  } else {
    fw->usb_bootloader = false;
  }
#endif

  ret = mxt_get_info(fw->mxt);
  if (ret) {
    mxt_err(fw->ctx, "Could not get info block");
    return ret;
  }

  mxt_dbg(fw->ctx, "Chip detected\n");

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Test firmware to flash against current firmware
/// \return #mxt_rc
static int mxt_check_firmware_version(struct flash_context *fw)
{
  mxt_get_firmware_version(fw->mxt, fw->curr_version);
  mxt_info(fw->ctx, "Current firmware version: %s", fw->curr_version);

  if (!strcmp(fw->curr_version, fw->new_version)) {

    mxt_info(fw->ctx, "Version already %s, exiting",
             fw->curr_version);
    return MXT_FIRMWARE_UPDATE_NOT_REQUIRED;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Check if device can be flashed
/// \return #mxt_rc
static int mxt_check_device_flashable(struct flash_context *fw)
{

  if (fw->conn->type == E_SYSFS_SPI){
    if (fw->mxt->info.id->family == 0xa6 && fw->mxt->info.id->variant == 0x15) {
      mxt_info(fw->ctx, "Flashing is not with this device on secondary interface\n");

      return MXT_ERROR_FIRMWARE_UPDATE_FAILED;
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Reset into bootloader mode
/// \return #mxt_rc
static int mxt_enter_bootloader_mode(struct flash_context *fw)
{
  int ret;

  if (fw->conn->type == E_SYSFS_SPI)
    ret = sysfs_set_debug_irq(fw->mxt, false);

  /* Change to the bootloader mode */
  ret = mxt_reset_chip(fw->mxt, true, 0);
  if (ret) {
    mxt_err(fw->ctx, "Reset failure - aborting");
    return ret;
  } else {
    sleep(MXT_RESET_TIME);
  }

  if (fw->conn->type == E_I2C_DEV) {
    fw->i2c_adapter = fw->conn->i2c_dev.adapter;
    fw->appmode_address = fw->conn->i2c_dev.address;

    mxt_info(fw->ctx, "is_mtch_driver: %d, appmode_address: 0x%02x",
             fw->is_mtch_driver, fw->appmode_address);

    fw->conn->i2c_dev.address = lookup_bootloader_addr(fw, fw->appmode_address);
    if (fw->conn->i2c_dev.address == -1) {
      mxt_err(fw->ctx, "No bootloader address!");
      return MXT_ERROR_BOOTLOADER_NO_ADDRESS;
    }

    mxt_info(fw->ctx, "I2C Adapter:%d", fw->conn->i2c_dev.adapter);
    mxt_info(fw->ctx, "Bootloader addr:0x%02x", fw->conn->i2c_dev.address);
    mxt_info(fw->ctx, "App mode addr:0x%02x", fw->appmode_address);

  } else if (fw->conn->type == E_SYSFS_SPI){
      ret = sysfs_set_bootloader(fw->mxt, true);
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Flash firmware to chip
int mxt_flash_firmware(struct libmaxtouch_ctx *ctx,
                       struct mxt_device *maxtouch,
                       const char *filename, const char *new_version,
                       struct mxt_conn_info *conn)
{
  struct flash_context fw = { 0 };
  int ret;
  int mode;

  fw.ctx = ctx;
  fw.mxt = maxtouch;
  fw.conn = conn;

  mxt_info(fw.ctx, "Opening firmware file %s", filename);

  fw.fp = fopen(filename, "r");
  if (!fw.fp) {
    mxt_err(fw.ctx, "Cannot open firmware file %s!", filename);
    return mxt_errno_to_rc(errno);
  }
  fseek(fw.fp, 0L, SEEK_END);
  fw.file_size = ftell(fw.fp);
  rewind(fw.fp);

  mode = mxt_bootloader_init_chip(&fw);
  if (mode && (mode != MXT_DEVICE_IN_BOOTLOADER))
    return mode;

  if (mode != MXT_DEVICE_IN_BOOTLOADER) {
    if (strlen(new_version) > 0) {
      fw.check_version = true;
      fw.new_version = new_version;
      mxt_dbg(fw.ctx, "New firmware version is:%s", fw.new_version);
      ret = mxt_check_firmware_version(&fw);
      if (ret)
        goto release;
    } else {
      fw.check_version = false;
      mxt_dbg(fw.ctx, "check_version:%d", fw.check_version);
    }

  ret = mxt_check_device_flashable(&fw);

    if (ret) {
      mxt_info(fw.ctx, "Flash upgrade not supported on this interface\n");
      return MXT_ERROR_FIRMWARE_UPDATE_FAILED;
  }

   ret = mxt_enter_bootloader_mode(&fw);
    if (ret) {
      mxt_err(fw.ctx, "Could not enter bootloader mode");
      goto release;
    }
  }

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret) {
    mxt_info(fw.ctx, "Could not initialise chip");
    return ret;
  }

  ret = send_frames(&fw);
  if (ret)
    return ret;

  if (fw.mxt->conn->type == E_SYSFS_SPI) {
    ret = sysfs_set_bootloader(fw.mxt, false);
    ret = sysfs_set_debug_irq(fw.mxt, true);
  } else if (fw.mxt->conn->type == E_SYSFS_I2C) {
      if (fw.mxt->mxt_crc.crc_enabled == true)
        ret = sysfs_set_debug_irq(fw.mxt, true);
  }

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
  } else if (fw.mxt->conn->type == E_SYSFS_SPI) {

    mxt_info(fw.ctx, "WAIT: Reset Time\n");
    sleep(MXT_RESET_TIME);

    mxt_info(fw.ctx, "Sent all firmware frames");
    ret = 0;
    goto release;

    /* Finish flashing now recover */

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

//******************************************************************************
/// \brief  Bootloader version query
int mxt_bootloader_version(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, struct mxt_conn_info *conn)
{
  struct flash_context fw = {0};
  int ret;
  unsigned char buf[3];

  fw.ctx = ctx;
  fw.mxt = mxt;
  fw.conn = conn;

  ret = mxt_bootloader_init_chip(&fw);
  if (ret && ret != MXT_DEVICE_IN_BOOTLOADER) {
    mxt_err(fw.ctx, "Could not init device");
    goto release;
  }

  if (ret != MXT_DEVICE_IN_BOOTLOADER) {
    ret = mxt_enter_bootloader_mode(&fw);
    if (ret) {
      mxt_err(fw.ctx, "Could not enter bootloader mode");

      goto release;
    }
  }

  ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
  if (ret) {
    mxt_err(fw.ctx, "Could not open device");
    goto release;
  }

  ret = mxt_check_bootloader(&fw, MXT_WAITING_BOOTLOAD_CMD);
  if (ret)
    goto release;

  ret = mxt_bootloader_read(fw.mxt, buf, sizeof(buf));
  if (ret)
    goto release;

  if (fw.extended_id_mode)
    printf("Bootloader ID:%d Version:%d\n",
           (buf[1] & MXT_BOOT_ID_MASK), buf[2]);
  else
    printf("Bootloader ID:%d\n", (buf[1] & MXT_BOOT_ID_MASK));

release:
  mxt_info(fw.ctx, "Reset into app mode");

  if (fw.mxt->conn->type == E_SYSFS_SPI) {
    ret = sysfs_set_bootloader(fw.mxt, false);
    ret = sysfs_set_debug_irq(fw.mxt, true);
  } else if (fw.mxt->conn->type == E_SYSFS_I2C) {
      if (fw.mxt->mxt_crc.crc_enabled == true)
        ret = sysfs_set_debug_irq(fw.mxt, true);
  }

  send_zero_frame(&fw);
  mxt_free_device(fw.mxt);
  mxt_unref_conn(fw.conn);

  return ret;
}

//******************************************************************************
/// \brief Read authenication state
/// \return #mxt_rc
static int mxt_check_authen_status(struct flash_context *fw, unsigned int state,
    struct fw_authen_options *fw_opts)
{
  unsigned char sbuf[3];
  unsigned char bootloader_id;
  unsigned char bootloader_version;
  uint8_t count = 0;
  unsigned char val;
  bool done = false;
  int ret = 0;

  while (!done) {

    ret = mxt_bootloader_read(fw->mxt, &val, 1);
    if(ret)
      return ret;

    switch(state) {
      case MXT_WAITING_BOOTLOAD_CMD:
        if ((val & ~MXT_BOOT_STATUS_MASK) == MXT_WAITING_BOOTLOAD_CMD) {
          bootloader_id = val & MXT_BOOT_STATUS_MASK;
          val &= ~MXT_BOOT_STATUS_MASK;
          mxt_dbg(fw->ctx, "\nMXT_WAITING_BOOTLOAD_CMD state\n");

          if (bootloader_id & 0x20) {
              mxt_info(fw->ctx, "Bootloader using extended ID mode");
              fw->extended_id_mode = true;

              ret = mxt_bootloader_read(fw->mxt, sbuf, sizeof(sbuf));
              if (ret)
                return ret;

              mxt_info(fw->ctx, "Bootloader ID: %d Version: %d\n",
                     sbuf[1] & MXT_BOOT_ID_MASK, sbuf[2]);
          }
        }
        break;
      case MXT_INFO_VALID:
        if (val == MXT_INFO_INVALID) {
          msleep(10);
          continue; /* Check again for INFO_VALID */
        }
        msleep(1000);
        mxt_info(fw->ctx, "INFO_VALID mode found = %02X", val);
        break;
      case MXT_SIGNATURE_MODE:
        mxt_info(fw->ctx, "Signature mode found = %02X", val);
        break;
      case MXT_SIGN_IN_PROGRESS:
        mxt_info(fw->ctx, "Signature in progress = %02X\n", val);
        break;
      case MXT_SIGN_MODE_FINISHED:
        mxt_info(fw->ctx, "Signature mode done = %02X\n", val);
        break;
      default:
        /* TBD -- Determine action if failure, currently return success */
        mxt_info(fw->ctx, "Device is in state %02X\n. Expected state %02X\n", val, state);
        break;
    }

    if (val != state) {
      mxt_err(fw->ctx, "Invalid bootloader state %02X != %02X\n",
        val, state);
      ret = MXT_BTLR_WRONG_STATE;
      break;
    }

    done = true;
  }

  return ret;
}

static int mxt_enter_authen_mode(struct flash_context *fw, struct fw_authen_options *fw_opts)
{
  unsigned char tbuf[6];
  int ret = 0;

  if (fw_opts->authen_type == 0x01) {         /* Signature mode */
    mxt_info(fw->ctx, "Unlocking to signature mode\n");
    tbuf[0] = MXT_UNLOCK_CMD_LSB;
    tbuf[1] = MXT_UNLOCK_CMD_MSB;
    tbuf[2] = 0x0F;
    tbuf[3] = 0xAC;
    tbuf[4] = 0xEC;
    tbuf[5] = 0x05;
  } else if (fw_opts->authen_type == 0x02) {  /* ACFA mode, not yet supported */
    tbuf[0] = MXT_UNLOCK_CMD_LSB;
    tbuf[1] = MXT_UNLOCK_CMD_MSB;
    tbuf[2] = 0x0F;
    tbuf[3] = 0xAC;
    tbuf[4] = 0x87;
    tbuf[5] = 0xCA;
  }

  ret = mxt_bootloader_write(fw->mxt, tbuf, sizeof(tbuf));

  if (ret) {
    mxt_err(fw->ctx, "Could not write to bootloader");
    return ret;
  }

  return MXT_SUCCESS;
}

static int mxt_send_sig_authen_req(struct flash_context *fw, struct fw_authen_options *fw_opts)
{
  unsigned char buf[9];
  uint32_t calc_crc;
  int ret = 0;

  buf[0] = 0x0C;    /* FW Authentication mode with SHA */

  if (fw_opts->req_mode == 0x00) {        /* Block Mode Request */
    buf[1] = 0x00;
    buf[2] = (fw_opts->blk_idx & 0xFF00) >> 8;
    buf[3] = (fw_opts->blk_idx & 0x00FF);
    buf[4] = (fw_opts->num_of_blks & 0xFF00) >> 8;
    buf[5] = (fw_opts->num_of_blks & 0x00FF);
  } else if (fw_opts->req_mode == 0xFF) { /* Segment Mode Request */
    buf[1] = 0xFF;
    buf[2] = 00;
    buf[3] = fw_opts->seg_id;     /* 0x00 - Btld, 0x01 - FW, 0x02 - NVM */
    buf[4] = 0xFF;
    buf[5] = 0xFF;
  }

  /* Calculate checksum */
  ret = mxt_calculate_crc(fw->ctx, &calc_crc, buf, 6);
  if (ret)
    return ret;

  buf[6] = (calc_crc & 0x00FF0000) >> 16;
  buf[7] = (calc_crc & 0x0000FF00) >> 8;
  buf[8] = (calc_crc & 0x000000FF);

  return mxt_bootloader_write(fw->mxt, buf, sizeof(buf));
}

static int mxt_send_acfa_req(struct flash_context *fw, struct fw_authen_options *fw_opts)
{
  /* Pending for acfa mode */
}

/* Convert two hex characters to a byte */
static unsigned char hex_pair_to_byte(struct flash_context *fw, char high, char low)
{
  unsigned char value;

    int hi = (high >= '0' && high <= '9') ? high - '0' :
             (high >= 'A' && high <= 'F') ? high - 'A' + 10 :
             (high >= 'a' && high <= 'f') ? high - 'a' + 10 : -1;
    int lo = (low >= '0' && low <= '9') ? low - '0' :
             (low >= 'A' && low <= 'F') ? low - 'A' + 10 :
             (low >= 'a' && low <= 'f') ? low - 'a' + 10 : -1;

    if (hi == -1 || lo == -1) 
     value = 0;
    else
      value = (hi << 4) | lo;
    
    return value;
}

int mxt_read_input_file(struct flash_context *fw, struct fw_authen_options *fw_opts,
                        const char *strbuf, unsigned char **out)
{

  char line[HEX_PER_LINE + 2];  /* +2 for newline and null terminator */
  int line_count = 0;
  int line_num;
  size_t len;
  int i, j;

  fw_opts->in_file = fopen(strbuf, "r");
  if (fw_opts->in_file == NULL) {
    fprintf(fw_opts->a_log, "Error opening input file %s\n", strbuf);
    return MXT_ERROR_IO;
  } else {
    fprintf(fw_opts->a_log, "\nSuccessfully opened %s input file.\n\n", strbuf);
  }

  /* Read the data into 2D array by line count/index */

  while(fgets(line, sizeof(line), fw_opts->in_file)) {
    /* remove new line, if present */
    len = strlen(line);
    if (len > 0 && (line[len - 1] == '\n' || line[len - 1] == 'r')) {
      line[len - 1] = '\0';
      len--;
    }

    if (len < HEX_PER_LINE) {
      mxt_err(fw->ctx, "line %d is too short (%zu chars).\n", line_num + 1, len);
      continue;
    }

    /* Convert line from hex string to hex buffer */
    for (i = 0; i < BYTES_PER_LINE; ++i) {
      out[line_count][i] = hex_pair_to_byte(fw, line[2 * i], line [2 * i + 1]);
    }

    line_count++;
  }

  return MXT_SUCCESS;
}

static int mxt_test_my_loop(struct flash_context *fw, struct fw_authen_options *fw_opts)
{
  int myloop = 50;

  do {

    myloop--;
    printf("myloop %d\n", myloop);

  } while (myloop > 0);

  mxt_info(fw->ctx, "Finished the loop\n");

  return MXT_SUCCESS;
}

static int mxt_compare_data(struct flash_context *fw, struct fw_authen_options *fw_opts,
                            unsigned char **in_buf,int index)
{
  unsigned char *chip_data;
  uint8_t read_pkt_size;
  uint16_t sblk_idx = 0;
  uint16_t nblk_idx = 0;
  uint16_t rd_offset = 0;
  uint16_t curr_index;
  uint16_t curr_chip_index;
  bool is_segment;
  bool error_flag = false;
  uint16_t error_count = 0;
  uint16_t block_loop = 1;
  uint8_t retry = 0;
  int ret;
  int i;
  int j;

  /* Make a copy of the current index, prevent corruption to index */
  curr_index = index;

  /* Setup number of loops based on num of blocks */
  if (fw_opts->req_mode == 0x00) {
    if (fw_opts->num_of_blks != 0x00) {
      block_loop = fw_opts->num_of_blks;
    } 
  }

  read_pkt_size = SIG_HEADER_SIZE + HASH_SIG_SIZE;

  chip_data = (char *)calloc(read_pkt_size, sizeof(char));   /* 5 byte header, 64 bytes of data */
  if (chip_data == NULL)
    return MXT_ERROR_NO_MEM;

  /* Loop until finished for block mode.  Segment mode recommend to do in script */
  /* This is a block mode request, modification of index valid only in function */

  do {
    ret = mxt_bootloader_read(fw->mxt, chip_data + rd_offset, read_pkt_size);
      if (ret) {
        msleep(5);
        mxt_warn(fw->ctx, "Could not read the signature data");

        retry++;

        if (retry == 3) {
          mxt_err(fw->ctx, "Failed to read bootloader data");
          ret = MXT_ERROR_IO;
          break;
        }

        continue;
      }

    sblk_idx = (chip_data[1] << 8) | chip_data[2];
    curr_chip_index = sblk_idx;    /* Keep track of current chip index */

    /* Start at non-zero offset with block */
    rd_offset = (chip_data[3] << 8) | chip_data[4];

    for (j = 0; j < BYTES_PER_LINE; j++) {
      if (in_buf[curr_index][j] != chip_data[5 + j]) {
        fprintf(fw_opts->a_log, "Error: Diff at byte[%d]: input=0x%02X, chip=0x%02X at index %d\n",
                j, in_buf[curr_index][j], chip_data[5 + j], curr_index);

        printf("Error: Diff at byte[%d]: input=0x%02X, chip=0x%02X at index %d\n", j,
             in_buf[curr_index][j], chip_data[5 + j], curr_index);
        error_flag = true;
        error_count++;
      }
    }

    if (error_flag == false) {
      fprintf(fw_opts->a_log, "Index %d comparison passed\n", curr_index);
      printf("Index %d comparison passed\n", curr_index);
    }

    curr_index++;
    block_loop--;
    error_flag = false;

  } while (block_loop > 0);

  if (fw_opts->req_mode == 0xFF) {
    is_segment = true;
  } else {
    is_segment = false;
  }

  if ((error_count == 0) && (ret == 0)) {
    fprintf(fw_opts->a_log, "\n%ssignature comparison completed successfully with no errors\n\n",
            is_segment ? "Segment " : "Block index");
    printf("\n%ssignature comparison completed successfully with no errors\n",
            is_segment ? "Segment " : "Block index");
  } else {
    fprintf(fw_opts->a_log, "\n%ssignature comparison failed\n\n",
            is_segment ? "Segment " : "Block_index");
    printf("\n%ssignature comparison failed\n\n",
           is_segment ? "Segment " : "Block index");
  }

  return MXT_SUCCESS;
}

static int mxt_create_outfile(struct flash_context *fw, struct fw_authen_options *fw_opts,
                                  const char *strbuf, const char *strbuf2)
{
  int ret = 0;
  char *output_file = NULL;   /* Empty file */
  unsigned char *buffer = NULL;
  char default_log[12] = "results.log";
  char line[HEX_PER_LINE + 2];  /* +2 for newline and null terminator */
  int i;

  /* Create log file, Use default name, if strbuf2 argv is NULL */
 if (strbuf2[0] != '\0') {
    output_file = (char *)calloc(strlen(strbuf2) + 1, sizeof(char));
    strncpy(output_file, strbuf2, strlen(strbuf2));
    output_file[strlen(strbuf2) - 1] = '\0';
  } else {
    output_file = (char *)calloc(strlen(default_log) + 1, sizeof(char));
    strcpy(output_file, default_log);
  }

  fw_opts->a_log = fopen(output_file, "w");
  if (fw_opts->a_log == NULL) {
    mxt_err(fw->ctx, "Failed to open output file %s", output_file);
    return MXT_ERROR_IO;
  }

  mxt_info(fw->ctx, "Output file: %s created\n", output_file);

  /* Generate a header file for the output file */

  for (i = 0; i < 40; i++) {
    fprintf(fw_opts->a_log, "-");
  }

  fprintf(fw_opts->a_log, "\nFW Authentication Output File\n");

  for (i = 0; i < 40; i++)
    fprintf(fw_opts->a_log, "-");

  fprintf(fw_opts->a_log, "\n\n");

  for (i = 0; i < 40; i++)
    fprintf(fw_opts->a_log, "-");

  fprintf(fw_opts->a_log, "\nSelected Input parameters\n");

  for (i = 0; i < 40; i++)
    fprintf(fw_opts->a_log, "-");

  fprintf(fw_opts->a_log, "\nAuthentication type = %02X\n", fw_opts->authen_type);
  fprintf(fw_opts->a_log, "Input file = %s\n", strbuf);
  fprintf(fw_opts->a_log, "Output file = %s\n", output_file);
  fprintf(fw_opts->a_log, "Request mode = %02X\n", fw_opts->req_mode);

  if (fw_opts->req_mode == 0xFF)
    fprintf(fw_opts->a_log, "Segment ID = %d\n", fw_opts->seg_id);
  else
    fprintf(fw_opts->a_log, "Starting block index = %d\n", fw_opts->blk_idx);
  
  fprintf(fw_opts->a_log, "Number of blocks = %d\n", fw_opts->num_of_blks);

  for (i = 0; i < 40; i++)
    fprintf(fw_opts->a_log, "-");

  fprintf(fw_opts->a_log, "\n");
  return MXT_SUCCESS;
}

int mxt_authentication_handler(struct mxt_device *maxtouch, struct mxt_conn_info *conn, 
                               struct fw_authen_options *fw_opts, const char *strbuf,
                               const char *strbuf2)
{
  struct flash_context fw = { 0 };
  unsigned char **dataout;
  uint8_t segment_loop = 0;
  int index;
  int mode;
  int i, j;
  int ret = 0;

  fw.ctx = maxtouch->ctx;
  fw.mxt = maxtouch;
  fw.conn = conn;

  mxt_info(fw.ctx, "Starting authentication\n");

  /* Create output log add header */
  ret = mxt_create_outfile(&fw, fw_opts, strbuf, strbuf2);
  if (ret) {
    mxt_err(fw.ctx, "Error: Could not create output log file\n");
    return MXT_ERROR_IO;
  }

  /* Allocate memory for the array of pointers, index # */
  /* Changed to dynamic to reduce memory use later, fixed size for now */

  dataout = (unsigned char **)malloc(4096 * sizeof(unsigned char*));
  if (dataout == NULL) {
    mxt_err(fw.ctx, "Could not initalize input index buffer\n");
    return MXT_ERROR_NO_MEM;
  }

  /* Loop to allocate memory for each 64 byte hash data */
  for (i = 0; i < 4096; i++) {
    dataout[i] = (unsigned char *)malloc(64 * sizeof(unsigned char));

    if (dataout[i] == NULL) {
      mxt_err(fw.ctx, "Failed to allocate memory out data buffer\n");
      for (j = 0; j < i; j++) {
        free(dataout[j]);
      }

      free(dataout);

      return MXT_ERROR_NO_MEM;
    }

    for (j = 0; j < 64; j++) {
      dataout[i][j] = 0;
    }
  }

  /* Create input log file and read signatures into buffer */
  ret = mxt_read_input_file(&fw, fw_opts, strbuf, dataout);
  if (ret) {
    mxt_info(fw.ctx, "Failed to open input file.\n");
    return MXT_ERROR_IO;
  }

  if ((fw_opts->req_mode == 0xFF) && (fw_opts->seg_id == 0x03)) {
    segment_loop = 3;
  } else {
    segment_loop = 1;
  }

  mxt_info(fw.ctx, "Initializing the bootloader");

  while (segment_loop > 0) {

    mode = mxt_bootloader_init_chip(&fw);

    /* Send the flash command to enter bootloader mode */
    if (mode != MXT_DEVICE_IN_BOOTLOADER) {
      mxt_info(fw.ctx,"Switching to bootloader mode");
      ret = mxt_enter_bootloader_mode(&fw);
      if (ret) {
        mxt_err(fw.ctx, "Could not enter bootloader mode");
        goto authen_release;
      }
    }

    /* Find new device after entering bootloader mode */
    ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
    if (ret) {
      mxt_err(fw.ctx, "Could not open device for FW authentication");
      return ret;
    }

    ret = mxt_check_authen_status(&fw, MXT_WAITING_BOOTLOAD_CMD, fw_opts);
    if (ret) {
      mxt_err(fw.ctx, "Bootloader incorrect state, Waiting bootloader CMD. Exiting\n");
      return ret;
    }

    mxt_info(fw.ctx, "Found new device in bootloader mode");

    if (ret == MXT_SUCCESS) {
      mxt_info(fw.ctx, "Unlocking bootloader mode");
      ret = mxt_enter_authen_mode(&fw, fw_opts);
      if (ret) {
        mxt_err(fw.ctx, "Could not enter authentication mode");
        return ret;
      }
    }

    ret = mxt_check_authen_status(&fw, MXT_SIGNATURE_MODE, fw_opts);

    mxt_info(fw.ctx, "Initializing into Signature mode");

    if (ret == MXT_SUCCESS) {
      mxt_info(fw.ctx, "Device is in Signature mode\n");
  
      if (fw_opts->authen_type == 0x01) {
        ret = mxt_send_sig_authen_req(&fw, fw_opts);
        if (ret) {
          mxt_err(fw.ctx, "Failed to send signature authentication request");
          goto authen_release;
        }
      } else if (fw_opts->authen_type == 0x02) {
        mxt_info(fw.ctx, "ACFA mode - Not yet available\n");
        //ret = mxt_send_acfa_req(&fw, fw_opts);
        ret = 0x01; //pending for next release
        if (ret) {
          mxt_err(fw.ctx, "Failed to send ACFA authentication request");
        } 
      }
    } else {
        mxt_err(fw.ctx, "Bootloader incorrect state.  Expecting Signature Mode. Exiting\n");
        return ret;
    }

    ret = mxt_check_authen_status(&fw, MXT_INFO_VALID, fw_opts);

    if (ret == MXT_SUCCESS) {
      /* Index is either segment ID or start of block index */
      if (fw_opts->req_mode == 0xFF) {
        index = fw_opts->seg_id; 
      } else if (fw_opts->req_mode == 0x00) {
        index = fw_opts->blk_idx;
      }

      ret = mxt_check_authen_status(&fw, MXT_SIGN_IN_PROGRESS, fw_opts);

      ret = mxt_compare_data(&fw, fw_opts, dataout, index);
      if (ret) {
        mxt_err(fw.ctx, "Signature data comparison error");
        return ret;
      }

    } else {
      mxt_err(fw.ctx, "Bootloader incorrect state. Expecting INFO VALID state\n");
    }

    ret = mxt_check_authen_status(&fw, MXT_SIGN_MODE_FINISHED, fw_opts);
    if (ret) {
      mxt_err(fw.ctx, "Signature mode finished state not found");
      return ret;
    }

    mxt_info(fw.ctx, "Signature mode finished. Returning to APP mode");

    segment_loop--;
  }

    /* Recover from bootloader mode */
    ret = mxt_new_device(fw.ctx, fw.conn, &fw.mxt);
    if (ret) {
      mxt_err(fw.ctx, "Could not open device for FW authentication");
    }

authen_release:
  mxt_free_device(fw.mxt);
  mxt_unref_conn(fw.conn);
  fclose(fw_opts->a_log);
  fclose(fw_opts->in_file);

  for (i = 0; i < 4096; i++)
    free(dataout[i]);

  free(dataout);

  return ret;

}
