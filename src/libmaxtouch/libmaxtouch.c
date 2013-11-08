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

#include "log.h"
#include "info_block.h"
#include "libmaxtouch.h"
#include "msg.h"

//******************************************************************************
/// \brief  Initialise libmaxtouch library
/// \return 0 = success, negative = error
int mxt_init(struct libmaxtouch_ctx **ctx)
{
  struct libmaxtouch_ctx *new_ctx;

  new_ctx = calloc(1, sizeof(struct libmaxtouch_ctx));
  if (!new_ctx)
    return -ENOMEM;

  *ctx = new_ctx;

  return 0;
}

//******************************************************************************
/// \brief  Close libmaxtouch library
/// \return 0 = success, negative = error
int mxt_close(struct libmaxtouch_ctx *ctx)
{
#ifdef HAVE_LIBUSB
  usb_close(&ctx->usb);
#endif
  free(ctx);
  return 0;
}

//******************************************************************************
/// \brief  Scan for devices on the I2C bus and USB
/// \note   Checks for I2C devices first
/// \return 0 = no device found, 1 = device found, negative = error
int mxt_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info *conn,
             bool query)
{
  int ret = 0;

  ctx->query = query;
  conn->type = E_NONE;

  /* Scan the I2C bus first because it will return quicker */
  ret = sysfs_scan(ctx, conn);
#ifdef HAVE_LIBUSB
  if (ret != 1)
  {
    /* If no I2C devices are found then scan the USB */
    ret = usb_scan(ctx, conn);
  }
#endif /* HAVE_LIBUSB */

  /* Clear query flag in case of context re-use */
  ctx->query = false;

  return ret;
}

//******************************************************************************
/// \brief Open device
int mxt_open(struct libmaxtouch_ctx *ctx, struct mxt_conn_info conn, struct mxt_device **mxt)
{
  int ret;
  struct mxt_device *new_dev;

  new_dev = calloc(1, sizeof(struct mxt_device));
  if (!new_dev)
    return -ENOMEM;

  new_dev->ctx = ctx;
  new_dev->conn = conn;

  switch (conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      ret = -1;
      goto failure;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
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
      LOG(LOG_ERROR, "Device type not supported");
      ret = -1;
      goto failure;
  }

  if (ret != 0)
    goto failure;

  *mxt = new_dev;
  return 0;

failure:
  free(new_dev);
  return ret;
}


//******************************************************************************
/// \brief Read information block
int mxt_get_info(struct mxt_device *mxt)
{
  int ret;

  ret = read_information_block(mxt);
  if (ret != 0)
  {
    LOG(LOG_ERROR, "Failed to read information block from mXT device");
    return ret;
  }

  ret = calc_report_ids(mxt);
  if (ret != 0)
  {
    LOG(LOG_ERROR, "Failed to generate report ID look-up table");
    return ret;
  }

  display_chip_info(mxt);

  return 0;
}

//******************************************************************************
/// \brief  Close device
void mxt_release(struct mxt_device *dev)
{
  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      sysfs_release(dev);
      break;

    case E_I2C_DEV:
      i2c_dev_release(dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      usb_release(dev);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  // Free memory
  free(dev->raw_info);
  free(dev);
}

//******************************************************************************
/// \brief  Get the filename of the input event file for the connected device
/// \return Filename as a string, or NULL if unsuccessful
/// \todo   Get the filename for sysfs devices from sysfs so the code does not
///         need to be edited to run on different sysfs devices
const char* mxt_get_input_event_file(struct mxt_device *dev)
{
  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      /* This must be adjusted depending on which sysfs device the code is running on */
      return "/dev/input/event2";

#ifdef HAVE_LIBUSB
    case E_USB:
      return "/dev/input/by-id/usb-Atmel_Atmel_maXTouch_Digitizer-event-mouse";
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return NULL;
}

//*****************************************************************************
/// \brief Debug buffer
static void debug_buffer(const unsigned char *data, size_t count, bool tx)
{
  unsigned int i;
  char *hexbuf;
  size_t strsize = count*3 + 1;

  if (log_level > LOG_VERBOSE)
    return;

  hexbuf = (char *)calloc(strsize, sizeof(char));
  if (hexbuf == NULL)
  {
    LOG(LOG_ERROR, "%s: calloc failure", __func__);
    return;
  }

  for (i = 0; i < count; i++)
    sprintf(&hexbuf[3 * i], "%02X ", data[i]);

  LOG(LOG_VERBOSE, "%s: %s", tx ? "TX": "RX", hexbuf);

  free(hexbuf);
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return 0 = success, negative = fail
int mxt_read_register(struct mxt_device *dev, unsigned char *buf,
                      int start_register, int count)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_read_register(dev, buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_read_register(dev, buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_read_register(dev, buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  if (ret == 0)
    debug_buffer(buf, count, false);

  return (ret);
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return 0 = success, negative = fail
int mxt_write_register(struct mxt_device *dev, unsigned char const *buf,
                       int start_register, int count)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_write_register(dev, buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_write_register(dev, buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_write_register(dev, buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  if (ret == 0)
    debug_buffer(buf, count, true);

  return ret;
}

//******************************************************************************
/// \brief  Enable/disable MSG retrieval
/// \return 0 = success, negative = fail
int mxt_set_debug(struct mxt_device *dev, bool debug_state)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_set_debug(dev, debug_state);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif
    case E_I2C_DEV:
      /* No need to enable MSG output */
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return ret;
}


//******************************************************************************
/// \brief  Get debug state
/// \return true (debug enabled) or false (debug disabled)
bool mxt_get_debug(struct mxt_device *dev)
{
  bool ret = false;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_get_debug(dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      LOG(LOG_WARN, "Kernel debug not supported for USB devices");
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Perform fallback reset
/// \return 0 = success, negative = fail
static int mxt_send_reset_command(struct mxt_device *dev, bool bootloader_mode)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(dev, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode)
  {
    LOG(LOG_INFO, "Resetting in bootloader mode");
    write_value = BOOTLOADER_COMMAND;
  }
  else
  {
    LOG(LOG_INFO, "Sending reset command");
  }

  /* Write to command processor register to perform command */
  ret = mxt_write_register
  (
    dev, &write_value, t6_addr + RESET_OFFSET, 1
  );

  return ret;
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return 0 = success, negative = fail
int mxt_reset_chip(struct mxt_device *dev, bool bootloader_mode)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_I2C_DEV:
    case E_SYSFS_DEBUG_NG:
      ret = mxt_send_reset_command(dev, bootloader_mode);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_reset_chip(dev, bootloader_mode);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Calibrate maxtouch chip
/// \return 0 = success, negative = fail
int mxt_calibrate_chip(struct mxt_device *dev)
{
  int ret = -1;
  uint16_t t6_addr;
  unsigned char write_value = CALIBRATE_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(dev, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  if (dev->conn.type == E_NONE)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      dev, &write_value, t6_addr + CALIBRATE_OFFSET, 1
    );

    if (ret == 0)
    {
      LOG(LOG_INFO, "Send calibration command");
    }
    else
    {
      LOG(LOG_ERROR, "Failed to send calibration command");
    }
  }

  return ret;
}

//******************************************************************************
/// \brief  Backup configuration settings to non-volatile memory
/// \return 0 = success, negative = fail
int mxt_backup_config(struct mxt_device *dev)
{
  int ret = -1;
  uint16_t t6_addr;
  unsigned char write_value = BACKUPNV_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(dev, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  if (dev->conn.type == E_NONE)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      dev, &write_value, t6_addr + BACKUPNV_OFFSET, 1
    );

    if (ret == 0)
    {
      LOG(LOG_INFO, "Backed up settings to the non-volatile memory");
    }
    else
    {
      LOG(LOG_ERROR, "Failed to back up settings");
    }
  }

  return ret;
}

//******************************************************************************
/// \brief  Get debug messages
/// \return Number of messages, negative error
int mxt_get_msg_count(struct mxt_device *dev)
{
  int count = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      count = sysfs_get_msg_count(dev);
      break;

    case E_SYSFS_DEBUG_NG:
      count = sysfs_get_msg_count_ng(dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      count = t44_get_msg_count(dev);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return count;
}

//******************************************************************************
/// \brief  Get T5 message as string
/// \return Message string (null for no message)
char *mxt_get_msg_string(struct mxt_device *dev)
{
  char *msg_string = NULL;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      msg_string = sysfs_get_msg_string(dev);
      break;

    case E_SYSFS_DEBUG_NG:
      msg_string = sysfs_get_msg_string_ng(dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      msg_string = t44_get_msg_string(dev);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  if (msg_string)
    LOG(LOG_DEBUG, "%s", msg_string);

  return msg_string;
}

//******************************************************************************
/// \brief  Get T5 message as byte array
/// \param  buf  Pointer to buffer
/// \param  buflen  Length of buffer
/// \return number of bytes read, negative for error
int mxt_get_msg_bytes(struct mxt_device *dev, unsigned char *buf, size_t buflen)
{
  int count = -1;
  int byte, length;
  char msg_string[50];

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      count = sysfs_get_msg_bytes(dev, buf, buflen);
      break;

    case E_SYSFS_DEBUG_NG:
      count = sysfs_get_msg_bytes_ng(dev, buf, buflen);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      count = t44_get_msg_bytes(dev, buf, buflen);
      break;
    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  if (count > 0)
  {
    length = snprintf(msg_string, sizeof(msg_string), MSG_PREFIX);

    /* Dump message to debug */
    for (byte = 0; byte < count; byte++)
    {
      length += snprintf(msg_string + length, sizeof(msg_string) - length,
                         "%02X ", buf[byte]);
    }

    LOG(LOG_DEBUG, "%s", msg_string);
  }
  return count;
}

//******************************************************************************
/// \brief  Discard all previous messages
/// \return Number of messages, negative error
int mxt_msg_reset(struct mxt_device *dev)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      ret = sysfs_msg_reset(dev);
      break;

    case E_SYSFS_DEBUG_NG:
      ret = sysfs_msg_reset_ng(dev);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      ret = t44_msg_reset(dev);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}

//******************************************************************************
/// \brief  Get fd for message polling
int mxt_get_msg_poll_fd(struct mxt_device *dev)
{
  if (dev->conn.type == E_SYSFS_DEBUG_NG)
    return sysfs_get_debug_ng_fd(dev);
  else
    return 0;
}

//******************************************************************************
/// \brief  Wait for messages
void mxt_msg_wait(struct mxt_device *dev, int timeout_ms)
{
  int ret;
  int fd = 0;
  int numfds = 0;
  struct pollfd fds[1];

  fd = mxt_get_msg_poll_fd(dev);
  if (fd)
  {
    fds[0].fd = fd;
    fds[0].events = POLLPRI;
    numfds = 1;
  }

  ret = poll(fds, numfds, timeout_ms);
  if (ret == -1 && errno == EINTR)
  {
    LOG(LOG_DEBUG, "Interrupted");
  }
  else if (ret == -1)
  {
    ret = -errno;
    LOG(LOG_ERROR, "poll returned %d (%s)", errno, strerror(errno));
  }
}

//******************************************************************************
/// \brief Read from bootloader
int mxt_bootloader_read(struct mxt_device *dev, unsigned char *buf, int count)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_read(dev, buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_read(dev, buf, count);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}

//******************************************************************************
/// \brief Write to bootloader
int mxt_bootloader_write(struct mxt_device *dev, unsigned char const *buf, int count)
{
  int ret = -1;

  switch (dev->conn.type)
  {
    case E_NONE:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_write(dev, buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_write(dev, buf, count);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}
