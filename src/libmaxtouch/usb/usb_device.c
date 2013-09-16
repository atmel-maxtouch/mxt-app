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
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "usb_device.h"

/* USB device configuration */
#define VENDOR_ID    0x03EB
#define ENDPOINT_1_IN  0x81
#define ENDPOINT_2_OUT 0x02

/* timeout in ms */
#define USB_TRANSFER_TIMEOUT 2000

#define REPORT_ID            0x01
#define IIC_DATA_1           0x51
#define CMD_READ_PINS        0x82
#define CMD_CONFIG           0x80
#define CMD_CONFIG_I2C_RETRY_ON_NAK (1 << 7)
#define CMD_FIND_IIC_ADDRESS 0xE0

/* mXT command status codes */
#define COMMS_STATUS_OK          0x00
#define COMMS_STATUS_DATA_NACK   0x01
#define COMMS_STATUS_ADDR_NACK   0x01
#define COMMS_STATUS_WRITE_OK    0x04

//******************************************************************************
/// \brief  Device information
typedef struct usb_device_t {
  bool library_initialised;
  libusb_context *context;
  bool device_connected;
  libusb_device_handle *device_handle;
  int ep1_in_max_packet_size;
  int interface;
  bool bootloader;
  int report_id;
  bool bridge_chip;
} usb_device;

//******************************************************************************
/// \brief  Detected devices
usb_device gDevice;

static int usb_transfer(void *cmd, int cmd_size, void *response, int response_size, bool ignore_response);
static int read_data(unsigned char *buf, uint16_t start_register, size_t count);
static int write_data(unsigned char const *buf, uint16_t start_register, size_t count, bool ignore_response);

//*****************************************************************************
/// \brief  Debug USB transfers
static void debug_usb(const unsigned char *data, uint8_t count, bool tx)
{
  int i;
  char hexbuf[5];
  char strbuf[256];

  // Reset string
  strbuf[0] = '\0';

  for (i = 0; i < count; i++) {
    snprintf(hexbuf, sizeof(hexbuf) - 1, "%02X ", data[i]);
    hexbuf[4] = '\0';
    strncat(strbuf, hexbuf, sizeof(strbuf) - 1);
  }

  // zero terminate
  strbuf[255] = '\0';

  LOG(LOG_VERBOSE, "%s: %s", tx ? "TX": "RX", strbuf);
}

//******************************************************************************
/// \brief Try to find descriptor of QRG interface
static int usb_scan_for_qrg_if(const struct libusb_device_descriptor *desc)
{
  int ret;
  char buf[128];
  const char qrg_if[] = "QRG-I/F";

  ret = libusb_get_string_descriptor_ascii(gDevice.device_handle,
                  desc->iProduct, (unsigned char *)buf, sizeof(buf));
  if (ret < 0)
  {
    return -1;
  }

  if (!strncmp(buf, qrg_if, sizeof(qrg_if)))
  {
    LOG(LOG_DEBUG, "Found %s", qrg_if);

    return 0;
  }
  else
  {
    return -1;
  }
}

//******************************************************************************
/// \brief  Try to find control interface
static int usb_scan_for_control_if(struct libusb_config_descriptor *config)
{
  int j,k,ret;
  char buf[128];
  const char control_if[] = "Atmel maXTouch Control";
  const char bootloader_if[] = "Atmel maXTouch Bootloader";

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
              buf, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);

            gDevice.bootloader = false;
            return altsetting->bInterfaceNumber;
          }
          else if (!strncmp(buf, bootloader_if, sizeof(bootloader_if)))
          {
            LOG(LOG_DEBUG, "Found %s at interface %d altsetting %d",
              buf, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);

            gDevice.bootloader = true;
            return altsetting->bInterfaceNumber;
          }
          else
          {
            LOG(LOG_DEBUG, "Ignoring %s at interface %d altsetting %d",
              buf, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);
          }
        }
      }
    }
  }

  return -1;
}

//******************************************************************************
/// \brief  Device is bootloader
/// \return true or false

bool usb_is_bootloader()
{
  return gDevice.bootloader;
}

//******************************************************************************
/// \brief  Scan configurations
/// \return interface number, negative for error
static int usb_scan_device_configs(libusb_device *dev,
                                    const struct libusb_device_descriptor *desc)
{
  int i, ret;

  if (gDevice.bridge_chip && desc->bNumConfigurations == 1)
  {
    return usb_scan_for_qrg_if(desc);
  }

  /* Scan through interfaces */
  for (i = 0; i < desc->bNumConfigurations; ++i)
  {
    struct libusb_config_descriptor *config;
    ret = libusb_get_config_descriptor(dev, i, &config);
    if (ret < 0)
    {
      LOG(LOG_ERROR, "Couldn't get config descriptor %d", i);
    }
    else
    {
      ret = usb_scan_for_control_if(config);
      if (ret >= 0)
      {
        // Found interface number
        return ret;
      }
    }
  }

  // not found
  return -1;
}

//******************************************************************************
/// \brief  Switch USB5030 to USB FS Bridge mode
/// \return zero on success, negative error
static int bridge_set_fs_mode(void)
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  int ret;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = 0;
  pkt[1] = 0xFA;
  pkt[2] = 0xE7;

  ret = usb_transfer(&pkt, sizeof(pkt), &pkt, sizeof(pkt), true);
  if (ret < 0)
    return ret;

  libusb_reset_device(gDevice.device_handle);

  return 0;
}

//******************************************************************************
/// \brief  Set the parameters for the comms mode on USB5030
/// \return zero on success, negative error
static int bridge_configure(void)
{
  int ret;
  unsigned char pkt[gDevice.ep1_in_max_packet_size];

  /* Command packet */
  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_CONFIG;
  /* 200kHz */
  pkt[1] = 0x20;
  pkt[2] = CMD_CONFIG_I2C_RETRY_ON_NAK;
  /* I2C retry delay */
  pkt[5] = 25 * 8;

  LOG(LOG_VERBOSE, "Sending CMD_CONFIG");

  ret = usb_transfer(&pkt, sizeof(pkt), &pkt, sizeof(pkt), false);

  return (ret < 0) ? ret : 0;
}

//******************************************************************************
/// \brief Hunt for I2C device using bridge chip
/// \return zero on success, negative error
static int bridge_find_i2c_address(void)
{
  int ret;
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  unsigned char response;

  /* Command packet */
  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_FIND_IIC_ADDRESS;

  LOG(LOG_VERBOSE, "Sending CMD_FIND_IIC_ADDRESS");

  ret = usb_transfer(&pkt, sizeof(pkt), &pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  response = pkt[1];

  if (response == 0x81)
  {
    LOG(LOG_ERROR, "No device found by bridge chip");
    return -1;
  }
  else
  {
    LOG(LOG_INFO, "Bridge found I2C device at 0x%02X", response);
    return 0;
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
    LOG(LOG_ERROR, "%s opening device with VID=0x%04X PID=0x%04X",
                   libusb_error_name(ret), desc.idVendor, desc.idProduct);
    return ret;
  }

  gDevice.interface = -1;

  ret = usb_scan_device_configs(dev, &desc);
  if (ret < 0)
  {
    LOG(LOG_WARN, "Did not find control interface");
    return ret;
  }
  else
  {
    gDevice.interface = ret;
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
  ret = libusb_claim_interface(gDevice.device_handle, gDevice.interface);
  if (ret != LIBUSB_SUCCESS)
  {
    LOG
    (
      LOG_ERROR,
      "Unable to claim bInterfaceNumber %d of the device, returned %s",
      gDevice.interface, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    LOG(LOG_VERBOSE, "Claimed the USB interface");
  }

  /* Get the maximum size of packets on endpoint 1 */
  ret = libusb_get_max_packet_size(libusb_get_device(gDevice.device_handle),
                                   ENDPOINT_1_IN);
  if (ret < LIBUSB_SUCCESS)
  {
    LOG(LOG_ERROR, "%s getting maximum packet size on endpoint 1 IN",
        libusb_error_name(ret));
    return -1;
  }

  gDevice.ep1_in_max_packet_size = ret;
  LOG(LOG_VERBOSE, "Maximum packet size on endpoint 1 IN is %d bytes",
      gDevice.ep1_in_max_packet_size);

  /* Configure bridge chip if necessary */
  if (gDevice.bridge_chip)
  {
    gDevice.report_id = 0;
    ret = bridge_set_fs_mode();
    if (ret < 0)
      return ret;

    ret = bridge_configure();
    if (ret < 0)
      return ret;

    ret = bridge_find_i2c_address();
    if (ret < 0)
      return ret;

  }
  else
  {
    gDevice.report_id = 1;
  }

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
    /* Level 3: informational messages are printed to stdout, warning and
     * error messages are printed to stderr */
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
      if (desc.idProduct == 0x211D ||
          (desc.idProduct >= 0x2126 && desc.idProduct <= 0x212D) ||
          (desc.idProduct >= 0x2135 && desc.idProduct <= 0x2139) ||
          (desc.idProduct >= 0x213A && desc.idProduct <= 0x21FC) ||
          (desc.idProduct >= 0x8000 && desc.idProduct <= 0x8FFF))
      {
        gDevice.bridge_chip = false;
        found = true;
      }
      else if (desc.idProduct == 0x6123)
      {
        LOG(LOG_DEBUG, "Found 5030 bridge chip");
        gDevice.bridge_chip = true;
        found = true;
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
    else
    {
      LOG(LOG_DEBUG, "Ignoring VID=%04X PID=%04X", desc.idVendor, desc.idProduct);
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
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND)
    return -1;

  /* The value written determines which mode the chip will boot into */
  if (bootloader_mode)
  {
    write_value = BOOTLOADER_COMMAND;
  }

  /* Send write command to reset the chip */
  ret = write_data
  (
    &write_value, t6_addr + RESET_OFFSET, 1, true
  );

  if (ret != 1)
  {
    LOG(LOG_ERROR, "Reset of the chip unsuccessful");
    return -1;
  }

  LOG(LOG_INFO, "Forced a reset of the chip");

  /* Chip will be unresponsive in bootloader mode so we cannot re-connect */
  if (!bootloader_mode)
  {
    usb_release();

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
int usb_read_register(unsigned char *buf, uint16_t start_register, size_t count)
{
  int ret;
  size_t bytes_read = 0;

  while (bytes_read < count)
  {
    ret = read_data(buf + bytes_read, start_register + bytes_read,
                    count - bytes_read);
    if (ret < 0)
      return ret;

    bytes_read += ret;
  }

  return 0;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return zero on success, negative error
int usb_write_register(unsigned char const *buf, uint16_t start_register, size_t count)
{
  int ret = 0;
  size_t bytes_written = 0;

  while (bytes_written < count)
  {
    ret = write_data(buf + bytes_written, start_register + bytes_written,
                     count - bytes_written, false);
    if (ret < 0)
      return ret;

    bytes_written += ret;
  }

  return 0;
}

//******************************************************************************
/// \brief  Read a packet of data from the MXT chip
/// \return number of bytes read, negative error
static int usb_transfer(void *cmd, int cmd_size, void *response,
                        int response_size, bool ignore_response)
{
  int bytes_transferred;
  int ret;

  /* Send command to request read */
  ret = libusb_interrupt_transfer
  (
    gDevice.device_handle, ENDPOINT_2_OUT, cmd,
    cmd_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != cmd_size)
  {
    LOG
    (
      LOG_ERROR,
      "Read request failed - %d bytes transferred, returned %s",
      bytes_transferred, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    debug_usb(cmd, cmd_size, true);
  }

  if (ignore_response)
  {
    LOG(LOG_VERBOSE, "Ignoring response command");
    return bytes_transferred;
  }

  /* Read response from read request */
  ret = libusb_interrupt_transfer
  (
    gDevice.device_handle, ENDPOINT_1_IN, response,
    response_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != response_size)
  {
    LOG
    (
      LOG_ERROR,
      "Read response failed - %d bytes transferred, returned %s",
      bytes_transferred, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    debug_usb(response, response_size, false);
  }

  return bytes_transferred;
}

//******************************************************************************
/// \brief  Read a packet of data from the MXT chip
/// \return number of bytes read, negative error
static int read_data(unsigned char *buf, uint16_t start_register, size_t count)
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  size_t cmd_size;
  size_t max_count;
  off_t response_ofs;
  int ret;

  /* Check a device is present before trying to read from it */
  if (!gDevice.library_initialised || !gDevice.device_connected)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  memset(&pkt, 0, sizeof(pkt));

  /* Command packet */
  if (gDevice.bridge_chip)
  {
    cmd_size = 5;
    max_count = gDevice.ep1_in_max_packet_size - cmd_size;

    if (count > max_count)
      count = max_count;

    pkt[0] = IIC_DATA_1;
    pkt[1] = 2;
    pkt[2] = count;
    pkt[3] = start_register & 0xFF;
    pkt[4] = (start_register & 0xFF00) >> 8;

    response_ofs = 0;
  }
  else
  {
    cmd_size = 6;
    max_count = gDevice.ep1_in_max_packet_size - cmd_size;

    if (count > max_count)
      count = max_count;

    pkt[0] = REPORT_ID;
    pkt[1] = IIC_DATA_1;
    pkt[2] = 2;
    pkt[3] = count;
    pkt[4] = start_register & 0xFF;
    pkt[5] = (start_register & 0xFF00) >> 8;

    response_ofs = 1;
  }

  LOG(LOG_VERBOSE, "Reading %" PRIuPTR " bytes starting from address %d",
      count, start_register);

  /* Command packet */

  ret = usb_transfer(&pkt, cmd_size, &pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  /* Check the result in the response */
  if (pkt[response_ofs] != COMMS_STATUS_OK)
  {
    LOG
    (
      LOG_ERROR,
      "Wrong result in read response - expected 0x%02X got 0x%02X",
      COMMS_STATUS_OK, pkt[response_ofs]
    );
    return -1;
  }

  /* Output the data read from the registers */
  (void)memcpy(buf, &pkt[response_ofs + 2], count);

  return count;
}

//******************************************************************************
/// \brief  Write a packet of data to the MXT chip
/// \return bytes written, negative error
static int write_data(unsigned char const *buf, uint16_t start_register,
                      size_t count, bool ignore_response)
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  int ret;
  size_t max_count;
  size_t cmd_size;
  int packet_size;
  off_t response_ofs;

  /* Check a device is present before trying to write to it */
  if (!gDevice.library_initialised || !gDevice.device_connected)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  memset(&pkt, 0, sizeof(pkt));

  /* Command packet */
  if (gDevice.bridge_chip)
  {
    cmd_size = 5;
    max_count = gDevice.ep1_in_max_packet_size - cmd_size;

    if (count > max_count)
      count = max_count;

    pkt[0] = IIC_DATA_1;
    pkt[1] = 2 + count;
    pkt[2] = 0;
    pkt[3] = start_register & 0xFF;
    pkt[4] = (start_register & 0xFF00) >> 8;

    response_ofs = 0;
  }
  else
  {
    cmd_size = 6;
    max_count = gDevice.ep1_in_max_packet_size - cmd_size;

    if (count > max_count)
      count = max_count;

    pkt[0] = REPORT_ID;
    pkt[1] = IIC_DATA_1;
    pkt[2] = 2 + count;
    pkt[3] = 0;
    pkt[4] = start_register & 0xFF;
    pkt[5] = (start_register & 0xFF00) >> 8;

    response_ofs = 1;
  }

  packet_size = cmd_size + count;

  (void)memcpy(pkt + cmd_size, buf, count);

  LOG(LOG_VERBOSE, "Writing %" PRIuPTR " bytes to address %d",
      count, start_register);

  ret = usb_transfer(pkt, packet_size, pkt, sizeof(pkt), ignore_response);
  if (ret < 0)
    return ret;

  /* Check the result in the response */
  if (!ignore_response && pkt[response_ofs] != COMMS_STATUS_WRITE_OK)
  {
    LOG
    (
      LOG_ERROR,
      "Wrong result in write response - expected 0x%02X got 0x%02X",
      COMMS_STATUS_WRITE_OK, pkt[response_ofs]
    );
    return -1;
  }

  return count;
}

//******************************************************************************
/// \brief  Read from bootloader
int usb_bootloader_read(unsigned char *buf, size_t count)
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  const off_t data_offset = 2;
  size_t max_count = gDevice.ep1_in_max_packet_size - data_offset;
  int ret;

  if (count > max_count)
    return -1;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = IIC_DATA_1;
  pkt[1] = 0x00;
  pkt[2] = (uint8_t)count;

  ret = usb_transfer(pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  /* Output the data read from the registers */
  (void)memcpy(buf, &pkt[data_offset], count);

  return 0;
}

//******************************************************************************
/// \brief  Write packet to bootloader
/// \return Numbers of bytes written or negative error
static int usb_bootloader_write_packet(unsigned char const *buf, size_t count)
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  int ret;
  const off_t data_offset = 3;
  size_t max_count = gDevice.ep1_in_max_packet_size - data_offset;

  if (count > max_count)
    count = max_count;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = IIC_DATA_1;
  pkt[1] = (uint8_t)count;
  pkt[2] = 0x00;

  (void)memcpy(&pkt[data_offset], buf, count);

  ret = usb_transfer(pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  return count;
}

//******************************************************************************
/// \brief  Write to bootloader
int usb_bootloader_write(unsigned char const *buf, size_t count)
{
  int ret;
  size_t bytes = 0;

  while (bytes < count)
  {
    ret = usb_bootloader_write_packet(buf + bytes, count - bytes);
    if (ret < 0)
      return ret;

    bytes += ret;
  }

  return 0;
}

//******************************************************************************
/// \brief Read CHG line
bool usb_read_chg()
{
  unsigned char pkt[gDevice.ep1_in_max_packet_size];
  bool chg;
  int ret;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_READ_PINS;

  ret = usb_transfer(pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  chg = pkt[2] & 0x4;

  LOG(LOG_VERBOSE, "CHG line %s", chg ? "HIGH" : "LOW");

  return chg;
}
