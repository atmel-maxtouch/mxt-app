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
#include "dmesg.h"
#include "libmaxtouch.h"
#include "info_block.h"
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
    gDeviceType = E_SYSFS;
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

  if (ret != 1)
  {
    LOG(LOG_ERROR, "Unable to find any mXT devices");
  }

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
char * mxt_get_input_event_file()
{
  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      /* This must be adjusted depending on which sysfs device the code is running on */
      return "/dev/input/event2";

    case E_USB:
      return "/dev/input/by-id/usb-Atmel_Atmel_maXTouch_Digitizer-event-mouse";

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
int mxt_write_register(unsigned char *buf, int start_register, int count)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
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
      ret = sysfs_set_debug(debug_state);
      break;

    case E_USB:
      LOG(LOG_WARN, "Kernel debug not supported for USB devices");
      break;

    case E_I2C_DEV:
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
/// \brief  Set pause state
/// \return 0 = success, negative = fail
int mxt_set_pause(bool pause_state)
{
  int ret = -1;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      ret = sysfs_set_pause(pause_state);
      break;

    case E_USB:
      LOG(LOG_WARN, "Pause state not supported for USB devices");
      break;

    case E_I2C_DEV:
    default:
      LOG(LOG_ERROR, "Device type not supported");
  }

  return ret;
}

//******************************************************************************
/// \brief  Get pause state
/// \return true (driver paused) or false (normal operation)
bool mxt_get_pause()
{
  bool ret = false;

  switch (gDeviceType)
  {
    case E_UNCONNECTED:
      LOG(LOG_ERROR, "Device uninitialised");
      break;

    case E_SYSFS:
      ret = sysfs_get_pause();
      break;

    case E_USB:
      LOG(LOG_WARN, "Pause state not supported for USB devices");
      break;

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
  unsigned char write_value = RESET_COMMAND;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode)
  {
    LOG(LOG_INFO, "Resetting in bootloader mode");
    write_value = BOOTLOADER_COMMAND;
  }

  /* Write to command processor register to perform command */
  ret = mxt_write_register
  (
    &write_value, command_processor_address + RESET_OFFSET, 1
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
  unsigned char write_value = CALIBRATE_COMMAND;

  if (gDeviceType == E_UNCONNECTED)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      &write_value, command_processor_address + CALIBRATE_OFFSET, 1
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
  unsigned char write_value = BACKUPNV_COMMAND;

  if (gDeviceType == E_UNCONNECTED)
  {
    LOG(LOG_ERROR, "Device uninitialised");
  }
#ifdef HAVE_LIBUSB
  else if (gDeviceType == E_USB)
  {
    LOG(LOG_ERROR, "Device type not supported");
  }
#endif /* HAVE_LIBUSB */
  else
  {
    /* Write to command processor register to perform command */
    ret = mxt_write_register
    (
      &write_value, command_processor_address + BACKUPNV_OFFSET, 1
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
/// \brief  Save configuration to file
/// \return 0 = success, negative = fail
int mxt_save_config_file(const char *cfg_file)
{
  int obj_idx, i, instance, num_bytes;
  uint8_t *temp;
  object_t *object;
  FILE *fp;
  int retval;

  LOG(LOG_INFO, "Opening config file %s...", cfg_file);

  fp = fopen(cfg_file, "w");

  fprintf(fp, "OBP_RAW V1\n");

  fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X\n",
          info_block.id->family_id, info_block.id->variant_id,
          info_block.id->version, info_block.id->build,
          info_block.id->matrix_x_size, info_block.id->matrix_y_size,
          info_block.id->num_declared_objects);

  fprintf(fp, "%06X\n", info_block_crc(info_block.crc));

  /* can't read object table CRC at present */
  fprintf(fp, "000000\n");

  for (obj_idx = 0; obj_idx < info_block.id->num_declared_objects; obj_idx++)
  {
    object = &(info_block.objects[obj_idx]);
    num_bytes = object->size + 1;

    temp = (uint8_t *)malloc(sizeof(char)*(num_bytes));
    if (temp == NULL)
    {
      LOG(LOG_ERROR, "Failed to allocate memory");
      retval = -1;
      goto close;
    }

    for (instance = 0; instance < object->instances + 1; instance++)
    {
      fprintf(fp, "%04X %04X %04X", object->object_type, instance, num_bytes);

      mxt_read_register(temp,
                       get_start_position(*object) + num_bytes * instance,
                       num_bytes);

      for (i=0; i< num_bytes; i++)
      {
        fprintf(fp, " %02X", *(temp + i));
      }

      fprintf(fp, "\n");
    }

    free(temp);
  }

  retval = 0;

close:
  fclose(fp);
  return retval;
}

//******************************************************************************
/// \brief  Load configuration file
/// \note   Ignores the COMMENTS and VERSION_INFO_HEADER sections
/// \return 0 = success, negative = fail
int mxt_load_config_file(const char *cfg_file, bool override_checking)
{
  FILE *fp;
  uint8_t *mem;
  int c;

  char object[255];
  char tmp[255];
  int object_id;
  int instance;
  int object_address;
  uint16_t expected_address;
  int object_size;
  uint8_t expected_size;
  int data;
  int file_read = 0;
  bool ignore_line = false;

  int i, j;
  int bytes_read;

  mem = malloc(255);
  if (mem == NULL) {
    LOG(LOG_ERROR, "Error allocating memory");
    return -1;
  }

  LOG(LOG_INFO, "Opening config file %s...", cfg_file);

  fp = fopen(cfg_file, "r");

  if (fp == NULL)
  {
    LOG(LOG_ERROR, "Error opening %s!", cfg_file);
    return -1;
  }

  while(!file_read)
  {
    /* First character is expected to be '[' - skip empty lines and spaces  */
    c = getc(fp);
    while((c == '\n') || (c == '\r') || (c == 0x20))
    {
      c = getc(fp);
    }

    if (c != '[')
    {
      if (c == EOF)
        break;

      /* If we are ignoring the current section then look for the next section */
      if (ignore_line)
      {
        continue;
      }

      LOG(LOG_ERROR, "Parse error: expected '[', read ascii char %c!", c);
      return -1;
    }

    fscanf(fp, "%s", object);
    /* Ignore the comments and file header sections */
    if (strncmp(object, "COMMENTS", 8) == 0 || strncmp(object, "VERSION_INFO_HEADER", 19) == 0)
    {
      ignore_line = true;
      continue;
    }
    ignore_line = false;
    fscanf(fp, "%s", tmp);

    if (strcmp(tmp, "INSTANCE"))
    {
      LOG(LOG_ERROR, "Parse error, expected INSTANCE");
      return(-1);
    }
    fscanf(fp, "%d", &instance);

    /* Last character is expected to be ']' */
    c = getc(fp);
    if (c != ']')
    {
      LOG(LOG_ERROR, "Parse error, expected ]");
      return(-1);
    }

    while(c != '\n')
    {
      c = getc(fp);
    }

    while ((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    fscanf(fp, "%d", &object_address);
    c = getc(fp);
    while((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    fscanf(fp, "%d", &object_size);
    c = getc(fp);

    /* Check object parameters match those in the chip's Object Table */
    if (!override_checking)
    {
      /* Get object type ID number at end of object string (1 or 2 digits) */
      if (sscanf(object + strlen(object) - 2, "%d", &object_id) != 1)
      {
        if (sscanf(object + strlen(object) - 1, "%d", &object_id) != 1)
        {
          LOG(LOG_ERROR, "Unable to get object type ID for %s", object);
          return -1;
        }
      }

      LOG(LOG_INFO, "Reading object T%d", object_id);

      /* Check the address of the object */
      expected_address = get_object_address((uint8_t)object_id, (uint8_t)instance);
      if (object_address != (int)expected_address)
      {
        LOG
        (
          LOG_ERROR,
          "Address of %s in config file (0x%04X) does not match chip (0x%04X)",
          object, object_address, expected_address
        );
        return -1;
      }

      /* Check the size of the object */
      expected_size = get_object_size((uint8_t)object_id);
      if (object_size != (int)expected_size)
      {
        LOG
        (
          LOG_ERROR,
          "Size of %s in config file (%d bytes) does not match chip (%d bytes)",
          object, object_size, expected_size
        );
        return -1;
      }
    }

    LOG(LOG_VERBOSE, "Writing object of size %d at address %d...", object_size,
         object_address);

    for (j = 0; j < object_size; j++) {
      *(mem + j) = 0;
    }

    bytes_read = 0;
    while (bytes_read < object_size)
    {
      /* Find next line, check first character valid and rewind */
      c = getc(fp);
      while((c == '\n') || (c == '\r') || (c == 0x20)) {
        c = getc(fp);
      }
      fseek(fp, -1, SEEK_CUR);
      if (c == '[') {
        LOG(LOG_WARN, "Skipping %d bytes at end of T%u", object_size - bytes_read, object_id);
        break;
      }

      /* Read address (discarded as we don't really need it) */
      fscanf(fp, "%d", &i);
      /* Read byte count of this register (max 2) */
      fscanf(fp, "%d", &i);

      while((c != '=') && (c != EOF))
      {
        c = getc(fp);
      }
      fscanf(fp, "%d", &data);
      c = getc(fp);

      if (i == 1) {
        *(mem + bytes_read) = (char) data;
        bytes_read++;
      } else if (i == 2) {
        *(mem + bytes_read) = (char) data & 0xFF;
        bytes_read++;
        *(mem + bytes_read) = (char) ((data >> 8) & 0xFF);
        bytes_read++;
      } else {
        LOG(LOG_ERROR, "Only 16-bit / 8-bit config values supported!");
        return -1;
      }
    }

    if (mxt_write_register(mem, object_address, object_size) < 0)
    {
      LOG(LOG_ERROR, "Error writing to mxt!");
      return -1;
    }
  }
  return 0;
}
