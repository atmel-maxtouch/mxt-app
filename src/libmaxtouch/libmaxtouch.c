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

#include "libmaxtouch.h"
#include "msg.h"

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
  int ret;

  ctx->query = query;
  ctx->query_found_device = false;

  /* Scan the I2C bus first because it will return quicker */
  ret = sysfs_scan(ctx, conn);
#ifdef HAVE_LIBUSB
  if (query || ret)
  {
    /* If no I2C devices are found then scan the USB */
    ret = usb_scan(ctx, conn);
  }
#endif /* HAVE_LIBUSB */

  if (query)
  {
    /* Clear query flag in case of context re-use */
    ctx->query = false;

    if (ctx->query_found_device)
    {
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

  switch (conn->type)
  {
    case E_SYSFS:
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

  if (conn == NULL)
  {
    mxt_err(ctx, "New device connection parameters not valid");
    return MXT_ERROR_NO_DEVICE;
  }

  switch (conn->type)
  {
    case E_SYSFS:
      ret = sysfs_open(new_dev);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_open(new_dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_open(new_dev);
      break;
#endif /* HAVE_LIBUSB */

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
  {
    mxt_err(mxt->ctx, "Failed to read information block from mXT device");
    return ret;
  }

  ret = mxt_calc_report_ids(mxt);
  if (ret)
  {
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
  switch (mxt->conn->type)
  {
    case E_SYSFS:
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

    default:
      mxt_err(mxt->ctx, "Device type not supported");
  }


  mxt->conn = mxt_unref_conn(mxt->conn);

  free(mxt->info.raw_info);
  free(mxt->report_id_map);
  free(mxt);
}

//******************************************************************************
/// \brief  Get the filename of the input event file for the connected device
/// \return Filename as a string, or NULL if unsuccessful
/// \todo   Get the filename for sysfs devices from sysfs so the code does not
///         need to be edited to run on different sysfs devices
const char* mxt_get_input_event_file(struct mxt_device *mxt)
{
  switch (mxt->conn->type)
  {
    case E_SYSFS:
      /* This must be adjusted depending on which sysfs device the code is running on */
      return "/dev/input/event2";

#ifdef HAVE_LIBUSB
    case E_USB:
      return "/dev/input/by-id/usb-Atmel_Atmel_maXTouch_Digitizer-event-mouse";
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
    default:
      mxt_err(mxt->ctx, "Device type not supported");
  }

  return NULL;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int mxt_read_register(struct mxt_device *mxt, uint8_t *buf,
                      int start_register, int count)
{
  int ret;

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      ret = sysfs_read_register(mxt, buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_read_register(mxt, buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_read_register(mxt, buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

    default:
      mxt_err(mxt->ctx, "Device type not supported");
      ret = MXT_ERROR_NOT_SUPPORTED;
  }

  if (ret == MXT_SUCCESS)
    mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "RX:", buf, count);

  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int mxt_write_register(struct mxt_device *mxt, uint8_t const *buf,
                       int start_register, int count)
{
  int ret;

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      ret = sysfs_write_register(mxt, buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_write_register(mxt, buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_write_register(mxt, buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

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

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      ret = sysfs_set_debug(mxt, debug_state);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif
    case E_I2C_DEV:
      /* No need to enable MSG output */
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

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      ret = sysfs_get_debug(mxt, value);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      mxt_warn(mxt->ctx, "Kernel debug not supported for USB devices");
      ret = MXT_ERROR_NOT_SUPPORTED;
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
    default:
      ret = MXT_ERROR_NOT_SUPPORTED;
      mxt_err(mxt->ctx, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Perform fallback reset
/// \return #mxt_rc
static int mxt_send_reset_command(struct mxt_device *mxt, bool bootloader_mode)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode)
  {
    mxt_info(mxt->ctx, "Resetting in bootloader mode");
    write_value = BOOTLOADER_COMMAND;
  }
  else
  {
    mxt_info(mxt->ctx, "Sending reset command");
  }

  /* Write to command processor register to perform command */
  ret = mxt_write_register
  (
    mxt, &write_value, t6_addr + MXT_T6_RESET_OFFSET, 1
  );

  return ret;
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return 0 = success, negative = fail
int mxt_reset_chip(struct mxt_device *mxt, bool bootloader_mode)
{
  int ret;

  switch (mxt->conn->type)
  {
    case E_SYSFS:
    case E_I2C_DEV:
      ret = mxt_send_reset_command(mxt, bootloader_mode);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_reset_chip(mxt, bootloader_mode);
      break;
#endif /* HAVE_LIBUSB */

    default:
      mxt_err(mxt->ctx, "Device type not supported");
      ret = MXT_ERROR_NOT_SUPPORTED;
  }

  return ret;
}

//******************************************************************************
/// \brief  Calibrate maxtouch chip
/// \return 0 = success, negative = fail
int mxt_calibrate_chip(struct mxt_device *mxt)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value = CALIBRATE_COMMAND;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Write to command processor register to perform command */
  ret = mxt_write_register
  (
    mxt, &write_value, t6_addr + MXT_T6_CALIBRATE_OFFSET, 1
  );

  if (ret == 0)
  {
    mxt_info(mxt->ctx, "Send calibration command");
  }
  else
  {
    mxt_err(mxt->ctx, "Failed to send calibration command");
  }

  return ret;
}

//******************************************************************************
/// \brief  Backup configuration settings to non-volatile memory
/// \return #mxt_rc
int mxt_backup_config(struct mxt_device *mxt)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value = BACKUPNV_COMMAND;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Write to command processor register to perform command */
  ret = mxt_write_register
  (
    mxt, &write_value, t6_addr + MXT_T6_BACKUPNV_OFFSET, 1
  );

  if (ret == MXT_SUCCESS)
    mxt_info(mxt->ctx, "Backed up settings to the non-volatile memory");
  else
    mxt_err(mxt->ctx, "Failed to back up settings");

  return ret;
}

//******************************************************************************
/// \brief  Get number of debug messages available
/// \note   This may retrieve and buffer messages
/// \return #mxt_rc
int mxt_get_msg_count(struct mxt_device *mxt, int *count)
{
  int ret;

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      if (sysfs_has_debug_v2(mxt))
        ret = sysfs_get_msg_count_v2(mxt, count);
      else
        ret = sysfs_get_msg_count(mxt, count);

      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      ret = t44_get_msg_count(mxt, count);
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

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      if (sysfs_has_debug_v2(mxt))
        msg_string = sysfs_get_msg_string_v2(mxt);
      else
        msg_string = sysfs_get_msg_string(mxt);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
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

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      if (sysfs_has_debug_v2(mxt))
        ret = sysfs_get_msg_bytes_v2(mxt, buf, buflen, count);
      else
        ret = sysfs_get_msg_bytes(mxt, buf, buflen, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
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

  switch (mxt->conn->type)
  {
    case E_SYSFS:
      if (sysfs_has_debug_v2(mxt))
        ret = sysfs_msg_reset_v2(mxt);
      else
        ret = sysfs_msg_reset(mxt);

      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      ret = t44_msg_reset(mxt);
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
  if (fd)
  {
    fds[0].fd = fd;
    fds[0].events = POLLPRI;
    numfds = 1;
  }

  ret = poll(fds, numfds, timeout_ms);
  if (ret == -1 && errno == EINTR)
  {
    mxt_dbg(mxt->ctx, "Interrupted");
    return MXT_ERROR_INTERRUPTED;
  }
  else if (ret < 0)
  {
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

  switch (mxt->conn->type)
  {
#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_read(mxt, buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_read(mxt, buf, count);
      break;

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
int mxt_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count)
{
  int ret;

  switch (mxt->conn->type)
  {
#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_write(mxt, buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_write(mxt, buf, count);
      break;

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
  switch (errno_in)
  {
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
