//------------------------------------------------------------------------------
/// \file   libmaxtouch.c
/// \brief  MXT device low level access
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
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>

#include "libmaxtouch.h"
#include "libmaxtouch/sysfs/dmesg.h"
#include "msg.h"

void msleep(int tms)
{ 
  struct timeval tv;
  tv.tv_sec = tms/1000;
  tv.tv_usec = (tms % 1000) * 1000;
  select (0, NULL, NULL, NULL, &tv);

}

//******************************************************************************
/// \brief  Initialise libmaxtouch library
/// \return #mxt_rc
int mxt_new(struct libmaxtouch_ctx **ctx)
{
  struct libmaxtouch_ctx *new_ctx;

  new_ctx = calloc(1, sizeof(struct libmaxtouch_ctx));
  if (!new_ctx)
    return MXT_ERROR_NO_MEM;

  new_ctx->log_level = LOG_ERROR;
  new_ctx->query = false;
  new_ctx->log_fn = mxt_log_stderr;
  new_ctx->i2c_block_size = I2C_DEV_MAX_BLOCK;

  *ctx = new_ctx;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Close libmaxtouch library
/// \return #mxt_rc
int mxt_free(struct libmaxtouch_ctx *ctx)
{
#ifdef HAVE_LIBUSB
  usb_close(ctx);
#endif
  free(ctx);
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Set function for logging output
/// \return #mxt_rc
void mxt_set_log_fn(struct libmaxtouch_ctx *ctx,
                    void (*log_fn)(struct libmaxtouch_ctx *ctx,
                                   enum mxt_log_level level, const char *format, va_list args))
{
  ctx->log_fn = log_fn;
}

//******************************************************************************
/// \brief  Scan for devices on the I2C bus and USB
/// \note   Checks for I2C devices first
/// \return #mxt_rc
int mxt_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn,
             bool query)
{
  int ret = 0;
  struct mxt_conn_info *cn;

  ctx->query = query;
  ctx->scan_count = 0;

  if (*conn == NULL) {
    mxt_new_conn(&cn, E_SYSFS_I2C);
    *conn = cn;
  } else {
    cn = *conn;
  }

  /* Return if connector type I2C_DEV */

  if (cn->type == E_I2C_DEV)
    return MXT_SUCCESS;

  /* Check if i2c bus and address exist */

  ret = sysfs_scan(ctx, conn);

#ifdef HAVE_LIBUSB
  if (query || ret) {
    /* If no I2C devices are found then scan the USB */
    ret = usb_scan(ctx, conn);
  }
#endif /* HAVE_LIBUSB */

  if (query) {
    /* Clear query flag in case of context re-use */
    ctx->query = false;

    if (ctx->scan_count) {
      return MXT_SUCCESS;
    }
  }

  return ret;
}

//******************************************************************************
/// \brief Create connection object
/// \return #mxt_rc
int mxt_new_conn(struct mxt_conn_info **conn, enum mxt_device_type type)
{
  struct mxt_conn_info *c = calloc(1, sizeof(struct mxt_conn_info));
  if (!c)
    return MXT_ERROR_NO_MEM;

  c->type = type;
  c->refcount = 1;

  if (c->type == E_SYSFS_SPI)
    c->sysfs.spi_found = true; 

  *conn = c;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Take a reference to the connection object
struct mxt_conn_info *mxt_ref_conn(struct mxt_conn_info *conn)
{
  if (conn == NULL)
    return NULL;

  conn->refcount++;
  return conn;
}

//******************************************************************************
/// \brief Free connection object
struct mxt_conn_info *mxt_unref_conn(struct mxt_conn_info *conn)
{
  if (conn == NULL)
    return NULL;

  conn->refcount--;

  if (conn->refcount > 0)
    return conn;

  switch (conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    free(conn->sysfs.path);
    break;

  default:
    break;
  }

  free(conn);
  return NULL;
}

//******************************************************************************
/// \brief Open device
/// \return #mxt_rc
int mxt_new_device(struct libmaxtouch_ctx *ctx, struct mxt_conn_info *conn,
                   struct mxt_device **mxt)
{
  int ret;
  struct mxt_device *new_dev;

  new_dev = calloc(1, sizeof(struct mxt_device));
  if (!new_dev)
    return MXT_ERROR_NO_MEM;

  new_dev->ctx = ctx;
  new_dev->conn = mxt_ref_conn(conn);

  if (conn == NULL) {
    mxt_err(ctx, "New device connection parameters not valid");
    return MXT_ERROR_NO_DEVICE;
  }

  switch (conn->type) {
  case E_SYSFS_I2C:
    ret = sysfs_open_i2c(new_dev);
    break;

  case E_SYSFS_SPI:
    ret = sysfs_open_spi(new_dev);
    break;

  case E_I2C_DEV:
    ret = i2c_dev_open(new_dev);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_open(new_dev);
    break;
#endif /* HAVE_LIBUSB */

  case E_HIDRAW:
    ret = hidraw_register(new_dev);
    break;

  default:
    mxt_err(ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    goto failure;
  }

  if (ret != 0)
    goto failure;

  *mxt = new_dev;
  return MXT_SUCCESS;

failure:
  mxt_unref_conn(conn);
  free(new_dev);
  return ret;
}

//******************************************************************************
/// \brief Read information block
/// \return #mxt_rc
int mxt_get_info(struct mxt_device *mxt)
{
  int ret;

  ret = mxt_read_info_block(mxt);
  if (ret)
    return ret;

  ret = mxt_calc_report_ids(mxt);
  if (ret) {
    mxt_err(mxt->ctx, "Failed to generate report ID look-up table");
    return ret;
  }

  mxt_display_chip_info(mxt);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Close device
void mxt_free_device(struct mxt_device *mxt)
{
  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    sysfs_release(mxt);
    break;

  case E_I2C_DEV:
    i2c_dev_release(mxt);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    usb_release(mxt);
    break;
#endif /* HAVE_LIBUSB */

  case E_HIDRAW:
    hidraw_release(mxt);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
  }


  mxt->conn = mxt_unref_conn(mxt->conn);

  free(mxt->info.raw_info);
  free(mxt->report_id_map);
  free(mxt);
}

//******************************************************************************
/// \brief Calculate the 8bit CRC
uint8_t mxt_calc_crc8(unsigned char crc, unsigned char data)
{
  static const uint8_t crcpoly = 0x8C;
  uint8_t index;
  uint8_t fb;
  index = 8;

  do {
    fb = (crc ^ data) & 0x01;
    data >>=1;
    crc >>=1;
    if (fb)
      crc ^= crcpoly;
  } while (--index);

  return crc;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
static int mxt_read_register_block(struct mxt_device *mxt, uint8_t *buf,
                                   int start_register, int count,
                                   size_t *bytes)
{
  int ret, err;
  bool irq_val = false;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:  
  case E_SYSFS_I2C:

//Stop the handling of interrupts in driver
  if (mxt->mxt_crc.crc_enabled == true) {
    err = sysfs_set_debug_irq(mxt, false);
    
    if (err)
      mxt_dbg(mxt->ctx, "Failed to disable debug_irq");
  }

    ret = sysfs_read_register(mxt, buf, start_register, count, bytes);

    if (ret)
      mxt_err(mxt->ctx, "Error reading register");

    if (mxt->mxt_crc.crc_enabled == true) {
      if ((mxt->mxt_crc.reset_triggered == false) && (mxt->mxt_crc.config_triggered == false)) {
        err = sysfs_set_debug_irq(mxt, true);
    
        if (err)
          mxt_dbg(mxt->ctx, "Failed to enable debug_irq");
      }
    }

    break;

  case E_I2C_DEV:

    ret = i2c_dev_read_register(mxt, buf, start_register, count, bytes);

    if ((mxt->mxt_crc.config_triggered == false) && (mxt->mxt_crc.processing_msg == false) &&
      (mxt->debug_fs.enabled == true)) {

      err = debugfs_set_irq(mxt, true);

      if (err)
        mxt_dbg(mxt->ctx, "Could not disable IRQ");
    }

    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_read_register(mxt, buf, start_register, count, bytes);
    break;
#endif /* HAVE_LIBUSB */

  case E_HIDRAW:
    ret = hidraw_read_register(mxt, buf, start_register, count, bytes);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
  }

  return ret;
}

//******************************************************************************
/// \brief  Read registers from MXT chip, in blocks
/// \return #mxt_rc
int mxt_read_register(struct mxt_device *mxt, uint8_t *buf,
                      int start_register, size_t count)
{
  int ret;
  size_t received;
  size_t off = 0;

  mxt_verb(mxt->ctx, "%s start_register:%d count:%zu", __func__,
           start_register, count);

  while (off < count) {
    ret = mxt_read_register_block(mxt, buf + off, start_register + off,
                                  count - off, &received);
    if (ret)
      return ret;

    off += received;
  }

  mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "RX:", buf, count);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int mxt_write_register(struct mxt_device *mxt, uint8_t const *buf,
                       int start_register, size_t count)
{
  int ret, err;

  mxt_verb(mxt->ctx, "%s start_register:%d count:%zu", __func__,
           start_register, count);

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    if (mxt->mxt_crc.crc_enabled == true) {
      err = sysfs_set_debug_irq(mxt, false);

      if (err)
        mxt_dbg(mxt->ctx, "Failed to disable debug_irq");
    }

    ret = sysfs_write_register(mxt, buf, start_register, count);

    if (mxt->mxt_crc.crc_enabled == true) {
      if (mxt->mxt_crc.reset_triggered == false) {
        err = sysfs_set_debug_irq(mxt, true);
      
      if (err)
        mxt_dbg(mxt->ctx, "Failed to enable debug_irq");
      }
    }

    break;

  case E_I2C_DEV:
    if (mxt->mxt_crc.crc_enabled)
      ret = i2c_dev_write_crc(mxt, buf, start_register, count);
    else
      ret = i2c_dev_write_register(mxt, buf, start_register, count);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_write_register(mxt, buf, start_register, count);
    break;
#endif /* HAVE_LIBUSB */

  case E_HIDRAW:
    ret = hidraw_write_register(mxt, buf, start_register, count);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
  }

  if (ret == MXT_SUCCESS)
    mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "TX:", buf, count);

  return ret;
}

//******************************************************************************
/// \brief  Write bytes to MXT chip
/// \return #mxt_rc
int mxt_write_bytes(struct mxt_device *mxt, uint8_t const *buf,
                       int start_register, size_t count)
{
  int ret;

  mxt_verb(mxt->ctx, "%s start_register:%d count:%zu", __func__,
           start_register, count);

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    ret = sysfs_write_register(mxt, buf, start_register, count);
    break;

  case E_I2C_DEV:
    if (mxt->mxt_crc.crc_enabled)
      ret = i2c_dev_write_crc(mxt, buf, start_register, count);
    else
      ret = i2c_dev_write_register(mxt, buf, start_register, count);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
  }

  if (ret == MXT_SUCCESS)
    mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "TX:", buf, count);

  return ret;
}

//******************************************************************************
/// \brief  Enable/disable MSG retrieval
/// \return #mxt_rc
int mxt_set_debug(struct mxt_device *mxt, bool debug_state)
{
  int ret;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    ret = sysfs_set_debug(mxt, debug_state);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
#endif
  case E_I2C_DEV:
  case E_HIDRAW:
   
    /* Check for debugfs files */

    ret = MXT_SUCCESS;
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
  }

  return ret;
}


//******************************************************************************
/// \brief  Get debug state
/// \param  mxt device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int mxt_get_debug(struct mxt_device *mxt, bool *value)
{
  int ret;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    ret = sysfs_get_debug(mxt, value);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    mxt_warn(mxt->ctx, "Kernel debug not supported for USB devices");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
#endif /* HAVE_LIBUSB */

  case E_I2C_DEV:
  case E_HIDRAW:
  default:
    ret = MXT_ERROR_NOT_SUPPORTED;
    mxt_err(mxt->ctx, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Perform fallback reset
/// \return #mxt_rc
static int mxt_send_reset_command(struct mxt_device *mxt, bool bootloader_mode, uint16_t reset_time_ms)
{
  int ret = 0, err = 0;
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;
  uint16_t curr_tx_seqnum;
  uint16_t tx_seq_num;
  bool crc_state;
  bool irq_val;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  mxt->mxt_crc.reset_triggered = true;

  if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {
    err = debugfs_set_irq(mxt, false);
    if (err)
      mxt_dbg(mxt->ctx, "Could not disable IRQ");
  }

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode) {

    mxt_info(mxt->ctx, "Resetting in bootloader mode");
   
    write_value = BOOTLOADER_COMMAND;

    ret = mxt_write_register(mxt, &write_value, t6_addr + MXT_T6_RESET_OFFSET, 1);

  } else {

      if (mxt->conn->type != E_I2C_DEV)
        err = sysfs_reset_chip(mxt);

      /* Direct write to command processor, if mxt_reset file not found */
      if (err || mxt->conn->type == E_I2C_DEV) {
        mxt_info(mxt->ctx, "Sending reset command");
        ret = mxt_write_register(mxt, &write_value, t6_addr + MXT_T6_RESET_OFFSET, 1);
      }
 
      /*Re-enable irq processing in sysfs mode */
      if (mxt->mxt_crc.crc_enabled == true && mxt->conn->type != E_I2C_DEV) {
        err = sysfs_set_debug_irq(mxt, true);

        if (err)
          mxt_dbg(mxt->ctx, "debug_irq not available\n");
      }

    if (reset_time_ms == 0)
        msleep(MXT_SOFT_RESET_TIME);
    else
        msleep(reset_time_ms);

    if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {

      if (mxt->mxt_crc.crc_enabled == true) {

        err = debugfs_set_tx_seq_num(mxt, 0x00);
        if (err)
          mxt_dbg(mxt->ctx, "Failed to set tx seq numer\n");
      }

      err = debugfs_set_irq(mxt, true);
        if (err)
          mxt_dbg(mxt->ctx, "Could not enable IRQ");
    }

    mxt->mxt_crc.reset_triggered = false;
  }

  return ret;
}

//******************************************************************************
/// \brief  Perform fallback reset
/// \return #mxt_rc
static int mxt_send_flash_command(struct mxt_device *mxt, bool bootloader_mode, bool crc_state)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value[5];
  unsigned char flash_command = BOOTLOADER_COMMAND;
  int i;
  uint8_t crc_data;
  uint8_t header = 0x03;
  uint8_t count = 0;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode) {
    mxt_info(mxt->ctx, "Resetting into bootloader mode");
  } else {
    mxt_info(mxt->ctx, "Sending reset command");
  }

  if (mxt->mxt_crc.crc_enabled){
    write_value[0] = BOOTLOADER_COMMAND;
    write_value[1] = 0x00;    //Default to 0x00 for first sequence number as test
    write_value[2] = t6_addr & 0xff;    //Get lower byte
    write_value[3] = (t6_addr >> 8) & 0xff; //Get uppper byte

  for (i=0; i < (header); i++){

    crc_data = mxt_calc_crc8(crc_data, write_value[i]);

    mxt_dbg(mxt->ctx, "Buffer[%d] = %x, count = %d, crc = 0x%x\n", i, write_value[i], count, crc_data);
  }
}

  /* Write to command processor register to perform command */
  ret = mxt_write_register
        (
          mxt, write_value, t6_addr + MXT_T6_RESET_OFFSET, 2
        );

  return ret;
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return 0 = success, negative = fail
int mxt_reset_chip(struct mxt_device *mxt, bool bootloader_mode, uint16_t reset_time_ms)
{
  int ret = 0;
  struct mxt_id_info *id = mxt->info.id;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
    if ((id->family == 0xa6) && (id->variant == 0x15)) {
      mxt_info(mxt->ctx, "Reset not supported on secondary interface\n");
      break;
  }
  case E_SYSFS_I2C:
  case E_I2C_DEV:
  case E_HIDRAW:
    ret = mxt_send_reset_command(mxt, bootloader_mode, reset_time_ms);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_reset_chip(mxt, bootloader_mode, reset_time_ms);
    break;
#endif /* HAVE_LIBUSB */

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
  }

  return ret;
}

//******************************************************************************
/// \brief Handle calibration messages
/// \return #mxt_rc
static int handle_calibrate_msg(struct mxt_device *mxt, uint8_t *msg,
                                void *context, uint8_t size, uint8_t msg_count)
{
  int *last_status = context;
  int status = msg[1];

  if (mxt_report_id_to_type(mxt, msg[0]) == GEN_COMMANDPROCESSOR_T6) {
    if (status & 0x10) {
      mxt_dbg(mxt->ctx, "Device calibrating");
    } else if (!(status & 0x10) && (*last_status & 0x10)) {
      mxt_info(mxt->ctx, "Device calibrated");
      return MXT_SUCCESS;
    }

    *last_status = status;
  }

  return MXT_MSG_CONTINUE;
}

//******************************************************************************
/// \brief  Calibrate maxtouch chip
/// \return 0 = success, negative = fail
int mxt_calibrate_chip(struct mxt_device *mxt)
{
  int ret, err;
  uint16_t t6_addr;
  unsigned char write_value = CALIBRATE_COMMAND;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  mxt_flush_msgs(mxt);

  if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {
    err = debugfs_set_irq(mxt, false);

    if (err)
      mxt_dbg(mxt->ctx, "Could not disable IRQ");

    mxt->mxt_crc.processing_msg = true;
  }

  /* Write to command processor register to perform command */
  ret = mxt_write_register(mxt, &write_value, t6_addr + MXT_T6_CALIBRATE_OFFSET, 1);
  if (ret == 0) {
    mxt_info(mxt->ctx, "Sent calibration command");
  } else {
    mxt_err(mxt->ctx, "Failed to send calibration command");
  }

  int state = 0;
  int flag = false;

  ret = mxt_read_messages(mxt, MXT_CALIBRATE_TIMEOUT, &state,
                          handle_calibrate_msg, &flag);

  if (ret == MXT_ERROR_TIMEOUT) {
    mxt_warn(mxt->ctx, "WARN: timed out waiting for calibrate status");
    ret = MXT_SUCCESS;
  } else if (ret) {
    mxt_err(mxt->ctx, "FAIL: device calibration failed");
  }

end:
  
  if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {
    mxt->mxt_crc.processing_msg = false;

    err = debugfs_set_irq(mxt, true);

    if (err)
     mxt_dbg(mxt->ctx, "Could not enable IRQ");
    }

  return ret;
}

//******************************************************************************
/// \brief  Backup configuration settings to non-volatile memory
/// \return #mxt_rc
int mxt_backup_config(struct mxt_device *mxt, uint8_t backup_command)
{
  int ret;
  uint16_t t6_addr;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Write to command processor register to perform command */
  ret = mxt_write_register
        (
          mxt, &backup_command, t6_addr + MXT_T6_BACKUPNV_OFFSET, 1
        );

  if (ret == MXT_SUCCESS)
    mxt_info(mxt->ctx, "Backed up settings to the non-volatile memory");
  else
    mxt_err(mxt->ctx, "Failed to back up settings");

  return ret;
}

//******************************************************************************
/// \brief  Issue REPORTALL command to device
/// \return #mxt_rc
int mxt_report_all(struct mxt_device *mxt)
{
  int ret;
  uint16_t t6_addr;
  const uint8_t report_all_cmd = 0xff;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Write to command processor register to perform command */
  ret = mxt_write_register
        (
          mxt, &report_all_cmd, t6_addr + MXT_T6_REPORTALL_OFFSET, 1
        );

  if (ret == MXT_SUCCESS)
    mxt_info(mxt->ctx, "REPORTALL command issued");
  else
    mxt_err(mxt->ctx, "Failed to issue REPORTALL command");

  return ret;
}

//******************************************************************************
/// \brief  Get number of debug messages available
/// \note   This may retrieve and buffer messages
/// \return #mxt_rc
int mxt_get_msg_count(struct mxt_device *mxt, int *count)
{
  int ret;

  switch (mxt->conn->type) {
  case E_SYSFS_I2C:
  case E_SYSFS_SPI:
    if (sysfs_has_debug_v2(mxt))
      ret = sysfs_get_msgs_v2(mxt, count);
    else
      ret = dmesg_get_msgs(mxt, count, false);

    break;

#ifdef HAVE_LIBUSB
  case E_USB:
#endif /* HAVE_LIBUSB */
  case E_I2C_DEV:
  case E_HIDRAW:
    ret = t44_t144_get_msg_count(mxt, count);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
  }

  return ret;
}

//******************************************************************************
/// \brief  Get T5 message as string
/// \return Message string (null for no message)
char *mxt_get_msg_string(struct mxt_device *mxt)
{
  char *msg_string = NULL;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    if (sysfs_has_debug_v2(mxt))
      msg_string = sysfs_get_msg_string_v2(mxt);
    else
      msg_string = dmesg_get_msg_string(mxt);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
#endif /* HAVE_LIBUSB */
  case E_I2C_DEV:
  case E_HIDRAW:
    msg_string = t44_get_msg_string(mxt);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    break;
  }

  if (msg_string)
    mxt_dbg(mxt->ctx, "%s", msg_string);

  return msg_string;
}

//******************************************************************************
/// \brief  Get T5 message as byte array
/// \param  mxt  Maxtouch Device
/// \param  buf  Pointer to buffer
/// \param  buflen  Length of buffer
/// \param  count length of message in bytes
/// \return #mxt_rc
int mxt_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf,
                      size_t buflen, int *count)
{
  int ret;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    if (sysfs_has_debug_v2(mxt))
      ret = sysfs_get_msg_bytes_v2(mxt, buf, buflen, count);
    else
      ret = dmesg_get_msg_bytes(mxt, buf, buflen, count);
    break;

#ifdef HAVE_LIBUSB
  case E_USB:
#endif /* HAVE_LIBUSB */
  case E_I2C_DEV:
  case E_HIDRAW:
    ret = t44_get_msg_bytes(mxt, buf, buflen, count);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
  }

 if (ret == MXT_SUCCESS)
   mxt_log_buffer(mxt->ctx, LOG_DEBUG, MSG_PREFIX, buf, *count);

  return ret;
}

//******************************************************************************
/// \brief  Discard all previous messages
/// \return #mxt_rc
int mxt_msg_reset(struct mxt_device *mxt)
{
  int ret;

  switch (mxt->conn->type) {
  case E_SYSFS_SPI:
  case E_SYSFS_I2C:
    if (sysfs_has_debug_v2(mxt))
      ret = sysfs_msg_reset_v2(mxt);
    else
      ret = dmesg_reset(mxt);

    break;

#ifdef HAVE_LIBUSB
  case E_USB:
#endif /* HAVE_LIBUSB */
  case E_I2C_DEV:
  case E_HIDRAW:
    ret = t44_t144_msg_reset(mxt);
    break;

  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
  }

  return ret;
}

//******************************************************************************
/// \brief  Get fd for message polling
int mxt_get_msg_poll_fd(struct mxt_device *mxt)
{
  if (sysfs_has_debug_v2(mxt))
    return sysfs_get_debug_v2_fd(mxt);
  else
    return 0;
}

//******************************************************************************
/// \brief  Wait for messages
int mxt_msg_wait(struct mxt_device *mxt, int timeout_ms)
{
  int ret;
  int fd = 0;
  int numfds = 0;
  struct pollfd fds[1];

  fd = mxt_get_msg_poll_fd(mxt);
  if (fd) {
    fds[0].fd = fd;
    fds[0].events = POLLPRI;
    numfds = 1;
  }

  ret = poll(fds, numfds, timeout_ms);
  if (ret == -1 && errno == EINTR) {
    mxt_dbg(mxt->ctx, "Interrupted");
    return MXT_ERROR_INTERRUPTED;
  } else if (ret < 0) {
    mxt_err(mxt->ctx, "poll returned %d (%s)", errno, strerror(errno));
    return MXT_ERROR_IO;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read from bootloader
/// \return #mxt_rc
int mxt_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count)
{
  int ret;

  switch (mxt->conn->type) {
#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_bootloader_read(mxt, buf, count);
    break;
#endif /* HAVE_LIBUSB */

  case E_SYSFS_SPI:
    ret = sysfs_bootloader_read(mxt, buf, count);
    break;

  case E_I2C_DEV:
    ret = i2c_dev_bootloader_read(mxt, buf, count);
    break;

  case E_HIDRAW:
  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
  }

  return ret;
}

//******************************************************************************
/// \brief Write to bootloader
/// \return #mxt_rc
static int i2c_dev_bootloader_write_blks(struct mxt_device *mxt, unsigned char const *buf, int count)
{
  int ret;
  size_t received;
  int off = 0;

  while (off < count) {
    ret = i2c_dev_bootloader_write(mxt, buf + off, count - off, &received);
    if (ret)
      return ret;

    off += received;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Write to bootloader
/// \return #mxt_rc
int mxt_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count)
{
  int ret;

  switch (mxt->conn->type) {
#ifdef HAVE_LIBUSB
  case E_USB:
    ret = usb_bootloader_write(mxt, buf, count);
    break;
#endif /* HAVE_LIBUSB */

  case E_SYSFS_SPI:
    ret = sysfs_bootloader_write(mxt, buf, count);
    break;

  case E_I2C_DEV:
    ret = i2c_dev_bootloader_write_blks(mxt, buf, count);
    break;

  case E_HIDRAW:
  default:
    mxt_err(mxt->ctx, "Device type not supported");
    ret = MXT_ERROR_NOT_SUPPORTED;
    break;
  }

  return ret;
}

//******************************************************************************
/// \brief Convert errno codes from the Linux API to #mxt_rc values
/// \return #mxt_rc
int mxt_errno_to_rc(int errno_in)
{
  switch (errno_in) {
  case EACCES:
    return MXT_ERROR_ACCESS;

  case ENOMEM:
    return MXT_ERROR_NO_MEM;

  case ETIMEDOUT:
    return MXT_ERROR_TIMEOUT;

  case ENOENT:
    return MXT_ERROR_NOENT;

  default:
    return MXT_ERROR_IO;
  }
}
