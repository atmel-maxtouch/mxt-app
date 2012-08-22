//------------------------------------------------------------------------------
/// \file   usb_device.c
/// \brief  MXT device low level access via USB
/// \author Tim Culmer
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

#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include "../log.h"
#include "../dmesg.h"
#include "../info_block.h"
#include "../libmaxtouch.h"
#include "usb_device.h"

/* USB device configuration */
#define VENDOR_ID    0x03EB
#define ENDPOINT_1_IN  0x81
#define ENDPOINT_2_OUT 0x02

#define USB_TRANSFER_TIMEOUT 5000

#define REPORT_ID         0x01
#define READ_WRITE_CMD_ID 0x51
#define NO_OF_ADDR_BYTES     2
#define SIZE_OF_CMD_HEADER   6
#define MAX_CMD_DATA_LENGTH 58
#define MAX_RES_DATA_LENGTH 61

/* mXT command status codes */
#define READ_DATA_RETURNED  0x00
#define WRITE_COMPLETED     0x04

//******************************************************************************
/// \brief  Read/write memory map command packet format
typedef struct command_packet_tag {
  unsigned char usb_report_id;
  unsigned char command_id;
  unsigned char bytes_to_write;
  unsigned char bytes_to_read;
  unsigned short address_pointer;
  unsigned char write_data[MAX_CMD_DATA_LENGTH];
} command_packet;

//******************************************************************************
/// \brief  Read/write memory map response packet format
typedef struct response_packet_tag {
  unsigned char usb_report_id;
  unsigned char result;
  unsigned char bytes_read;
  unsigned char read_data[MAX_RES_DATA_LENGTH];
} response_packet;

//******************************************************************************
/// \brief  Device information
typedef struct usb_device_tag {
  bool library_initialised;
  libusb_context *context;
  bool device_connected;
  libusb_device_handle *device_handle;
  int ep1_in_max_packet_size;
  int interface;
} usb_device;

//******************************************************************************
/// \brief  Detected devices
usb_device gDevice;

static int read_packet(unsigned char *buf, int start_register, int count);
static int write_packet(unsigned char const *buf, int start_register, int count, bool ignore_response);

/* Function is only used in the LOG() macro so GCC thinks it is not used */
/* The "unused" attribute is used to suppress the warning */
__attribute__((unused)) static char * get_libusb_error_string(int libusb_error);


//******************************************************************************
/// \brief  Try to connect device
static void usb_scan_for_control_if(struct libusb_config_descriptor *config)
{
  int j,k,ret;
  char buf[128];
  const char control_if[] = "Atmel maXTouch Control";

  for (j = 0 ; j < config->bNumInterfaces; j++)
  {
    const struct libusb_interface *interface = &config->interface[j];
    for (k = 0; k < interface->num_altsetting; k++)
    {
      const struct libusb_interface_descriptor *altsetting = &interface->altsetting[k];

      if (altsetting->iInterface > 0)
      {
        ret = libusb_get_string_descriptor_ascii(gDevice.device_handle,
                altsetting->iInterface, (unsigned char *)buf, sizeof(buf));
        if (ret > 0)
        {
          if (!strncmp(buf, control_if, sizeof(control_if)))
          {
            LOG(LOG_DEBUG, "Found %s at interface %d altsetting %d",
              control_if, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);

            gDevice.interface = altsetting->bInterfaceNumber;
            return;
          }
        }
      }
    }
  }
}

//******************************************************************************
/// \brief  Scan configurations
/// \return 0 = success, negative for error
static void usb_scan_device_configs(libusb_device *dev, const struct libusb_device_descriptor desc)
{
  int i, ret;

  /* Scan through interfaces */
  for (i = 0; i < desc.bNumConfigurations; ++i)
  {
    struct libusb_config_descriptor *config;
    ret = libusb_get_config_descriptor(dev, i, &config);
    if (ret)
    {
      LOG(LOG_ERROR, "Couldn't get config description %d", i);
    }
    else
    {
      usb_scan_for_control_if(config);
    }
  }
}

//******************************************************************************
/// \brief  Try to connect device
/// \return 0 = success, negative for error
static int usb_connect_device(libusb_device *dev, const struct libusb_device_descriptor desc)
{
  int ret;

  ret = libusb_open(dev, &gDevice.device_handle);
  if (ret != LIBUSB_SUCCESS)
  {
    LOG(LOG_ERROR, "Unable to connect to device with VID=0x%04X PID=0x%04X"
                   " ret=%d", desc.idVendor, desc.idProduct, ret);
    return ret;
  }

  usb_scan_device_configs(dev, desc);

  if (gDevice.interface < 0)
  {
    LOG(LOG_WARN, "Did not find control interface");
  }

  /* Disconnect the kernel driver if it is active */
  if (libusb_kernel_driver_active(gDevice.device_handle, gDevice.interface) == 1)
  {
    LOG(LOG_VERBOSE, "Kernel driver is active - must be detached before claiming the interface");

    if (libusb_detach_kernel_driver(gDevice.device_handle, gDevice.interface) == 0)
    {
      LOG(LOG_VERBOSE, "Detached kernel driver");
    }
  }

  /* Claim the bInterfaceNumber 1 of the device */
  ret = libusb_claim_interface(gDevice.device_handle, gDevice.interface); if (ret != LIBUSB_SUCCESS)
  {
    LOG
    (
      LOG_ERROR,
      "Unable to claim bInterfaceNumber %d of the device, returned %s",
      gDevice.interface, get_libusb_error_string(ret)
    );
    return -1;
  }

  LOG(LOG_VERBOSE, "Claimed the USB interface");

  /* Get the maximum size of packets on endpoint 1 */
  ret = libusb_get_max_packet_size
  (
    libusb_get_device (gDevice.device_handle),
    ENDPOINT_1_IN
  );

  if (ret < LIBUSB_SUCCESS)
  {
    LOG
    (
      LOG_ERROR,
      "Unable to get maximum packet size on endpoint 1 IN, returned %s",
      get_libusb_error_string(ret)
    );
  }

  gDevice.ep1_in_max_packet_size = ret;
  LOG(LOG_VERBOSE, "Maximum packet size on endpoint 1 IN is %d bytes", gDevice.ep1_in_max_packet_size);

  gDevice.device_connected = true;
  LOG(LOG_INFO, "Registered USB device with VID=0x%04X PID=0x%04X Interface=%d",
      desc.idVendor, desc.idProduct, gDevice.interface);

  return 0;
}

//******************************************************************************
/// \brief  Scan for devices
/// \return 1 = device found, 0 = not found, negative for error
int usb_scan()
{
  int ret = -1;
  int count, i;
  struct libusb_device **devs;
  bool found;

  /* Initialise flags */
  gDevice.library_initialised = false;
  gDevice.device_connected = false;

  /* Initialise library */
  ret = libusb_init(&(gDevice.context));

  /* Was the library initialised successfully */
  if (ret != LIBUSB_SUCCESS)
  {
    LOG(LOG_ERROR, "Failed to initialise libusb");
    return ret;
  }

  gDevice.library_initialised = true;
  LOG(LOG_VERBOSE, "Initialised libusb");

  /* Set the debug level for the library */
  if (log_level < 3)
  {
    LOG(LOG_DEBUG, "Enabling libusb debug");
    libusb_set_debug(gDevice.context, 3);
  }

  count = libusb_get_device_list(gDevice.context, &devs);
  if (count <= 0)
  {
    LOG(LOG_ERROR, "Error %d enumerating devices", count);
    return count;
  }

  found = false;
  for (i = 0; i < count; i++)
  {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret != LIBUSB_SUCCESS)
    {
      LOG(LOG_WARN, "Error %d trying to retrieve descriptor", ret);
      continue;
    }

    if (desc.idVendor == VENDOR_ID)
    {
      if ((desc.idProduct >= 0x2126 && desc.idProduct <= 0x212D) ||
          (desc.idProduct >= 0x2135 && desc.idProduct <= 0x2138) ||
          (desc.idProduct >= 0x213A && desc.idProduct <= 0x21FC) ||
          (desc.idProduct >= 0x8000 && desc.idProduct <= 0x8FFF))
      {
        found = true;
      }
      else if (desc.idProduct == 0x6123)
      {
        LOG(LOG_WARN, "libmaxtouch does not yet support 5030 bridge chip");
      }
    }

    if (found)
    {
      LOG(LOG_VERBOSE, "Found VID=%04X PID=%04X", desc.idVendor, desc.idProduct);

      // Try to connect to device. Note this will only ever use first device
      // enumerated
      if (usb_connect_device(devs[i], desc) == 0)
      {
        break;
      }
    }
  }

  libusb_free_device_list(devs, 1);

  if (!gDevice.device_connected)
  {
    LOG(LOG_WARN, "Unable to find USB device");
    return 0;
  }
  else
  {
    return 1;
  }
}

//******************************************************************************
/// \brief  Release device
void usb_release()
{
  /* Are we connected to a device? */
  if (gDevice.device_connected)
  {
    libusb_release_interface(gDevice.device_handle, gDevice.interface);
    LOG(LOG_DEBUG, "Released the USB interface");

    libusb_close(gDevice.device_handle);
    LOG(LOG_INFO, "Disconnected from the device");
  }

  /* Is the library initialised? */
  if (gDevice.library_initialised)
  {
    libusb_exit(gDevice.context);
    LOG(LOG_DEBUG, "Exited from libusb");
  }
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return zero on success, negative error
int usb_reset_chip(bool bootloader_mode)
{
  int ret = -1;
  unsigned char write_value = RESET_COMMAND;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode)
  {
    write_value = BOOTLOADER_COMMAND;
  }

  /* Send write command to reset the chip */
  ret = write_packet
  (
    &write_value, command_processor_address + RESET_OFFSET, 1, true
  );

  if (ret != 0)
  {
    LOG(LOG_ERROR, "Reset of the chip unsuccessful");
    return -1;
  }

  LOG(LOG_INFO, "Forced a reset of the chip");
  usb_release();

  /* Chip will be unresponsive in bootloader mode so we cannot re-connect */
  if (!bootloader_mode)
  {
    /* We must wait for the chip to be ready to communicate again */
    LOG(LOG_DEBUG, "Waiting for the chip to re-connect");

    sleep(1);

    /* Rescan to connect to the chip again */
    ret = usb_scan();

    if (ret < 1)
    {
      LOG(LOG_ERROR, "Failed to re-connect to chip after reset");
      return -1;
    }
  }

  return 0;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return zero on success, negative error
int usb_read_register(unsigned char *buf, int start_register, int count)
{
  int ret = 0;

  int transferred_bytes = 0;
  int remaining_bytes = count;

  /* Read whole packets */
  while (remaining_bytes > MAX_RES_DATA_LENGTH && ret == 0)
  {
    ret = read_packet
    (
      buf + transferred_bytes, start_register + transferred_bytes,
      MAX_RES_DATA_LENGTH
    );

    transferred_bytes += MAX_RES_DATA_LENGTH;
    remaining_bytes -= MAX_RES_DATA_LENGTH;
  }

  if (ret == 0)
  {
    /* Read leftover bytes */
    ret = read_packet
    (
      buf + transferred_bytes, start_register + transferred_bytes,
      remaining_bytes
    );
  }

  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return zero on success, negative error
int usb_write_register(unsigned char const *buf, int start_register, int count)
{
  int ret = 0;

  int transferred_bytes = 0;
  int remaining_bytes = count;

  /* Write whole packets */
  while (remaining_bytes > MAX_CMD_DATA_LENGTH && ret == 0)
  {
    ret = write_packet
    (
      buf + transferred_bytes, start_register + transferred_bytes,
      MAX_CMD_DATA_LENGTH, false
    );

    transferred_bytes += MAX_CMD_DATA_LENGTH;
    remaining_bytes -= MAX_CMD_DATA_LENGTH;
  }

  if (ret == 0)
  {
    /* Write leftover bytes */
    ret = write_packet
    (
      buf + transferred_bytes, start_register + transferred_bytes,
      remaining_bytes, false
    );
  }

  return ret;
}

//******************************************************************************
/// \brief  Read a packet of data from the MXT chip
/// \return zero on success, negative error
static int read_packet(unsigned char *buf, int start_register, int count)
{
  int ret = -1;

  static const int command_size = SIZE_OF_CMD_HEADER;
  command_packet command;

  /* Try to read a whole packet - otherwise we get LIBUSB_ERROR_OVERFLOW error */
  int response_size = gDevice.ep1_in_max_packet_size;
  response_packet response;

  int bytes_transferred = 0;

  /* Check a device is present before trying to read from it */
  if (!gDevice.library_initialised || !gDevice.device_connected)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  /* Check input parameters */
  if (count > MAX_RES_DATA_LENGTH)
  {
    LOG(LOG_ERROR, "Cannot read %d bytes (max %d bytes)", count, MAX_RES_DATA_LENGTH);
    return ret;
  }

  LOG(LOG_VERBOSE, "Reading %d bytes starting from address 0x%04X", count, start_register);

  /* Command packet */
  command.usb_report_id = REPORT_ID;
  command.command_id = READ_WRITE_CMD_ID;
  command.bytes_to_write = NO_OF_ADDR_BYTES;
  command.bytes_to_read = count;
  command.address_pointer = start_register;

  /* Send command to request read */
  ret = libusb_interrupt_transfer
  (
    gDevice.device_handle, ENDPOINT_2_OUT, (unsigned char *)&command,
    command_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != command_size)
  {
    LOG
    (
      LOG_ERROR,
      "Read request failed - %d bytes transferred, returned %s",
      bytes_transferred, get_libusb_error_string(ret)
    );
    return -1;
  }

  /* Read response from read request */
  ret = libusb_interrupt_transfer
  (
    gDevice.device_handle, ENDPOINT_1_IN, (unsigned char *)&response,
    response_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != response_size)
  {
    LOG
    (
      LOG_ERROR,
      "Read response failed - %d bytes transferred, returned %s",
      bytes_transferred, get_libusb_error_string(ret)
    );
    return -1;
  }

  /* Check the result in the response */
  if (response.result != READ_DATA_RETURNED)
  {
    LOG
    (
      LOG_ERROR,
      "Wrong result in read response - expected 0x%02X got 0x%02X",
      READ_DATA_RETURNED, response.result
    );
    return -1;
  }

  /* Output the data read from the registers */
  (void)memcpy(buf, response.read_data, count);

  LOG(LOG_VERBOSE, "Registers read successfully");
  return 0;
}

//******************************************************************************
/// \brief  Write a packet of data to the MXT chip
/// \return zero on success, negative error
static int write_packet(unsigned char const *buf, int start_register, int count, bool ignore_response)
{
  int ret = -1;

  int command_size = SIZE_OF_CMD_HEADER + count;
  command_packet command;

  /* Try to read a whole packet - otherwise we get LIBUSB_ERROR_OVERFLOW error */
  int response_size = gDevice.ep1_in_max_packet_size;
  response_packet response;

  int bytes_transferred = 0;

  /* Check a device is present before trying to write to it */
  if (!gDevice.library_initialised || !gDevice.device_connected)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  /* Check input parameters */
  if (count > MAX_CMD_DATA_LENGTH)
  {
    LOG(LOG_ERROR, "Cannot write %d bytes (max %d bytes)", count, MAX_CMD_DATA_LENGTH);
    return -1;
  }

  LOG(LOG_VERBOSE, "Writing %d bytes from address 0x%04X", count, start_register);

  /* Command packet */
  command.usb_report_id = REPORT_ID;
  command.command_id = READ_WRITE_CMD_ID;
  command.bytes_to_write = NO_OF_ADDR_BYTES + count;
  command.bytes_to_read = 0;
  command.address_pointer = start_register;
  (void)memcpy(command.write_data, buf, count);

  /* Send command to request write */
  ret = libusb_interrupt_transfer
  (
    gDevice.device_handle, ENDPOINT_2_OUT, (unsigned char *)&command,
    command_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != command_size)
  {
    LOG
    (
      LOG_ERROR,
      "Write request failed - %d bytes transferred, returned %s",
      bytes_transferred, get_libusb_error_string(ret)
    );
    return -1;
  }

  /* Some write requests have no response e.g. chip reset */
  if (ignore_response)
  {
    LOG(LOG_VERBOSE, "Ignoring response command");
  }
  else
  {
    /* Read response from write request */
    ret = libusb_interrupt_transfer
    (
      gDevice.device_handle, ENDPOINT_1_IN, (unsigned char *)&response,
      response_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
    );

    if (ret != LIBUSB_SUCCESS || bytes_transferred != response_size)
    {
      LOG
      (
        LOG_ERROR,
        "Write response failed - %d bytes transferred, returned %s",
        bytes_transferred, get_libusb_error_string(ret)
      );
      return -1;
    }

    /* Check the result in the response */
    if (response.result != WRITE_COMPLETED)
    {
      LOG
      (
        LOG_ERROR,
        "Wrong result in write response - expected 0x%02X got 0x%02X",
        WRITE_COMPLETED, response.result
      );
      return -1;
    }
  }

  LOG(LOG_VERBOSE, "Registers written successfully");
  return 0;
}

//******************************************************************************
/// \brief  Converts a libusb error code into a string
/// \return Error string
static char * get_libusb_error_string(int libusb_error)
{
  switch (libusb_error)
  {
    case LIBUSB_SUCCESS:
      return "LIBUSB_SUCCESS";
    case LIBUSB_ERROR_IO:
      return "LIBUSB_ERROR_IO";
    case LIBUSB_ERROR_INVALID_PARAM:
      return "LIBUSB_ERROR_INVALID_PARAM";
    case LIBUSB_ERROR_ACCESS:
      return "LIBUSB_ERROR_ACCESS";
    case LIBUSB_ERROR_NO_DEVICE:
      return "LIBUSB_ERROR_NO_DEVICE";
    case LIBUSB_ERROR_NOT_FOUND:
      return "LIBUSB_ERROR_NOT_FOUND";
    case LIBUSB_ERROR_BUSY:
      return "LIBUSB_ERROR_BUSY";
    case LIBUSB_ERROR_TIMEOUT:
      return "LIBUSB_ERROR_TIMEOUT";
    case LIBUSB_ERROR_OVERFLOW:
      return "LIBUSB_ERROR_OVERFLOW";
    case LIBUSB_ERROR_PIPE:
      return "LIBUSB_ERROR_PIPE";
    case LIBUSB_ERROR_INTERRUPTED:
      return "LIBUSB_ERROR_INTERRUPTED";
    case LIBUSB_ERROR_NO_MEM:
      return "LIBUSB_ERROR_NO_MEM";
    case LIBUSB_ERROR_NOT_SUPPORTED:
      return "LIBUSB_ERROR_NOT_SUPPORTED";
    case LIBUSB_ERROR_OTHER:
      return "LIBUSB_ERROR_OTHER";
    default:
      return "unrecognised error code";
  }
}

