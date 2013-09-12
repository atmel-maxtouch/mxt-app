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

#include "log.h"
#include "libmaxtouch.h"
#include "info_block.h"
#include "msg.h"
#include "sysfs/sysfs_device.h"
#include "i2c_dev/i2c_dev_device.h"
#include "usb/usb_device.h"

#define MAX_DEVICES 1

/* Interface of detected devices */
mxt_device_type gDeviceType = E_UNCONNECTED;

//******************************************************************************
/// \brief  Scan for devices on the I2C bus and USB
/// \note   Checks for I2C devices first
/// \return 1 = device found, 0 = not found, negative for error
int mxt_scan()
{
  int ret = 0;

  /* Scan the I2C bus first because it will return quicker */
  ret = sysfs_scan();
  if (ret == 1)
  {
    gDeviceType = sysfs_has_debug_ng() ? E_SYSFS_DEBUG_NG : E_SYSFS;
  }
#ifdef HAVE_LIBUSB
  else
  {
    /* If no I2C devices are found then scan the USB */
    ret = usb_scan();
    if (ret == 1)
    {
      gDeviceType = E_USB;
    }
  }
#endif /* HAVE_LIBUSB */

  return ret;
}

//******************************************************************************
/// \brief Set device type
void mxt_set_device_type(mxt_device_type type)
{
  gDeviceType = type;
}

//******************************************************************************
/// \brief Read information block
int mxt_get_info()
{
  int ret;

  ret = read_information_block();
  if (ret != 0)
  {
    LOG(LOG_ERROR, "Failed to read information block from mXT device");
    return -1;
  }

  ret = calc_report_ids();
  if (ret != 0)
  {
    LOG(LOG_ERROR, "Failed to generate report ID look-up table");
    return -1;
  }

  display_chip_info();

  return 0;
}

//******************************************************************************
/// \brief  Release device
void mxt_release()
{
  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      sysfs_release();
      break;

    case E_I2C_DEV:
      i2c_dev_release();
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      usb_release();
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }
}

//******************************************************************************
/// \brief  Get the interface type of the device currently connected
/// \return Device type, or E_UNCONNECTED for no device
mxt_device_type mxt_get_device_type()
{
  return gDeviceType;
}

//******************************************************************************
/// \brief  Get the filename of the input event file for the connected device
/// \return Filename as a string, or NULL if unsuccessful
/// \todo   Get the filename for sysfs devices from sysfs so the code does not
///         need to be edited to run on different sysfs devices
const char* mxt_get_input_event_file()
{
  switch (gDeviceType)
  {
    case E_UNCONNECTED:
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

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return 0 = success, negative = fail
int mxt_read_register(unsigned char *buf, int start_register, int count)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_read_register(buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_read_register(buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_read_register(buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return (ret);
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return 0 = success, negative = fail
int mxt_write_register(unsigned char const *buf, int start_register, int count)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_write_register(buf, start_register, count);
      break;

    case E_I2C_DEV:
      ret = i2c_dev_write_register(buf, start_register, count);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_write_register(buf, start_register, count);
      break;
#endif /* HAVE_LIBUSB */

    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Set debug state
/// \return 0 = success, negative = fail
int mxt_set_debug(bool debug_state)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_set_debug(debug_state);
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
bool mxt_get_debug()
{
  bool ret = false;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_SYSFS_DEBUG_NG:
      ret = sysfs_get_debug();
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
static int mxt_send_reset_command(bool bootloader_mode)
{
  int ret;
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
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
    &write_value, t6_addr + RESET_OFFSET, 1
  );

  return ret;
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return 0 = success, negative = fail
int mxt_reset_chip(bool bootloader_mode)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
    case E_I2C_DEV:
    case E_SYSFS_DEBUG_NG:
      ret = mxt_send_reset_command(bootloader_mode);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_reset_chip(bootloader_mode);
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
int mxt_calibrate_chip()
{
  int ret = -1;
  uint16_t t6_addr;
  unsigned char write_value = CALIBRATE_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  if (gDeviceType == E_UNCONNECTED)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      &write_value, t6_addr + CALIBRATE_OFFSET, 1
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
int mxt_backup_config()
{
  int ret = -1;
  uint16_t t6_addr;
  unsigned char write_value = BACKUPNV_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  if (gDeviceType == E_UNCONNECTED)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      &write_value, t6_addr + BACKUPNV_OFFSET, 1
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
int mxt_get_msg_count(void)
{
  int count = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      count = sysfs_get_msg_count();
      break;

    case E_SYSFS_DEBUG_NG:
      count = sysfs_get_msg_count_ng();
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      count = t44_get_msg_count();
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
char *mxt_get_msg_string(void)
{
  char *msg_string = NULL;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      msg_string = sysfs_get_msg_string();
      break;

    case E_SYSFS_DEBUG_NG:
      msg_string = sysfs_get_msg_string_ng();
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      msg_string = t44_get_msg_string();
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
int mxt_get_msg_bytes(unsigned char *buf, size_t buflen)
{
  int count = -1;
  int byte, length;
  char msg_string[50];

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      count = sysfs_get_msg_bytes(buf, buflen);
      break;

    case E_SYSFS_DEBUG_NG:
      count = sysfs_get_msg_bytes_ng(buf, buflen);
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      count = t44_get_msg_bytes(buf, buflen);
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
int mxt_msg_reset(void)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      ret = sysfs_msg_reset();
      break;

    case E_SYSFS_DEBUG_NG:
      ret = sysfs_msg_reset_ng();
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
#endif /* HAVE_LIBUSB */
    case E_I2C_DEV:
      ret = t44_msg_reset();
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}

//******************************************************************************
/// \brief  Get fd for message polling
int mxt_get_msg_poll_fd(void)
{
  if (gDeviceType == E_SYSFS_DEBUG_NG)
    return sysfs_get_debug_ng_fd();
  else
    return 0;
}

//******************************************************************************
/// \brief Read from bootloader
int mxt_bootloader_read(unsigned char *buf, int count)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_read(buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_read(buf, count);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}

//******************************************************************************
/// \brief Write to bootloader
int mxt_bootloader_write(unsigned char const *buf, int count)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

#ifdef HAVE_LIBUSB
    case E_USB:
      ret = usb_bootloader_write(buf, count);
      break;
#endif /* HAVE_LIBUSB */

    case E_I2C_DEV:
      ret = i2c_dev_bootloader_write(buf, count);
      break;

    default:
      LOG(LOG_ERROR, "Device type not supported");
      break;
  }

  return ret;
}
