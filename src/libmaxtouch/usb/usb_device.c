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
#include <config.h>

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

#ifndef HAVE_LIBUSB_ERROR_NAME
//******************************************************************************
/// \brief Converts a libusb error code into a string
/// \return Error string
static const char * libusb_error_name(int errcode)
{
  switch (errcode)
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
#endif

//******************************************************************************
/// \brief  Read a packet of data from the MXT chip
/// \return number of bytes read, negative error
static int usb_transfer(struct mxt_device *mxt, void *cmd, int cmd_size,
                        void *response, int response_size, bool ignore_response)
{
  int bytes_transferred;
  int ret;

  /* Send command to request read */
  ret = libusb_interrupt_transfer
  (
    mxt->usb.handle, ENDPOINT_2_OUT, cmd,
    cmd_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != cmd_size)
  {
    mxt_err
    (
      mxt->ctx,
      "Read request failed - %d bytes transferred, returned %s",
      bytes_transferred, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "TX", cmd, cmd_size);
  }

  if (ignore_response)
  {
    mxt_verb(mxt->ctx, "Ignoring response command");
    return bytes_transferred;
  }

  /* Read response from read request */
  ret = libusb_interrupt_transfer
  (
    mxt->usb.handle, ENDPOINT_1_IN, response,
    response_size, &bytes_transferred, USB_TRANSFER_TIMEOUT
  );

  if (ret != LIBUSB_SUCCESS || bytes_transferred != response_size)
  {
    mxt_err
    (
      mxt->ctx,
      "Read response failed - %d bytes transferred, returned %s",
      bytes_transferred, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    mxt_log_buffer(mxt->ctx, LOG_VERBOSE, "RX", response, response_size);
  }

  return bytes_transferred;
}

//******************************************************************************
/// \brief  Read a packet of data from the MXT chip
/// \return number of bytes read, negative error
static int read_data(struct mxt_device *mxt, unsigned char *buf,
                     uint16_t start_register, size_t count)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  size_t cmd_size;
  size_t max_count;
  off_t response_ofs;
  int ret;

  /* Check a device is present before trying to read from it */
  if (!mxt->usb.device_connected)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return -1;
  }

  memset(&pkt, 0, sizeof(pkt));

  /* Command packet */
  if (mxt->usb.bridge_chip)
  {
    cmd_size = 5;
    max_count = mxt->usb.ep1_in_max_packet_size - cmd_size;

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
    max_count = mxt->usb.ep1_in_max_packet_size - cmd_size;

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

  mxt_verb(mxt->ctx, "Reading %" PRIuPTR " bytes starting from address %d",
      count, start_register);

  /* Command packet */

  ret = usb_transfer(mxt, &pkt, cmd_size, &pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  /* Check the result in the response */
  if (pkt[response_ofs] != COMMS_STATUS_OK)
  {
    mxt_err
    (
      mxt->ctx,
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
static int write_data(struct mxt_device *mxt, unsigned char const *buf,
                      uint16_t start_register, size_t count,
                      bool ignore_response)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  int ret;
  size_t max_count;
  size_t cmd_size;
  int packet_size;
  off_t response_ofs;

  /* Check a device is present before trying to write to it */
  if (!mxt->usb.device_connected)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return -1;
  }

  memset(&pkt, 0, sizeof(pkt));

  /* Command packet */
  if (mxt->usb.bridge_chip)
  {
    cmd_size = 5;
    max_count = mxt->usb.ep1_in_max_packet_size - cmd_size;

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
    max_count = mxt->usb.ep1_in_max_packet_size - cmd_size;

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

  mxt_verb(mxt->ctx, "Writing %" PRIuPTR " bytes to address %d",
      count, start_register);

  ret = usb_transfer(mxt, pkt, packet_size, pkt, sizeof(pkt), ignore_response);
  if (ret < 0)
    return ret;

  /* Check the result in the response */
  if (!ignore_response && pkt[response_ofs] != COMMS_STATUS_WRITE_OK)
  {
    mxt_err
    (
      mxt->ctx,
      "Wrong result in write response - expected 0x%02X got 0x%02X",
      COMMS_STATUS_WRITE_OK, pkt[response_ofs]
    );
    return -1;
  }

  return count;
}

//******************************************************************************
/// \brief Try to find descriptor of QRG interface
static int usb_scan_for_qrg_if(struct mxt_device *mxt)
{
  int ret;
  char buf[128];
  const char qrg_if[] = "QRG-I/F";

  ret = libusb_get_string_descriptor_ascii(mxt->usb.handle,
                  mxt->usb.desc.iProduct, (unsigned char *)buf, sizeof(buf));
  if (ret < 0)
  {
    return -1;
  }

  if (!strncmp(buf, qrg_if, sizeof(qrg_if)))
  {
    mxt_dbg(mxt->ctx, "Found %s", qrg_if);

    return 0;
  }
  else
  {
    return -1;
  }
}

//******************************************************************************
/// \brief  Try to find control interface
static int usb_scan_for_control_if(struct mxt_device *mxt,
                                   struct libusb_config_descriptor *config)
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
        ret = libusb_get_string_descriptor_ascii(mxt->usb.handle,
                altsetting->iInterface, (unsigned char *)buf, sizeof(buf));
        if (ret > 0)
        {
          if (!strncmp(buf, control_if, sizeof(control_if)))
          {
            mxt_dbg(mxt->ctx, "Found %s at interface %d altsetting %d",
              buf, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);

            mxt->usb.bootloader = false;
            return altsetting->bInterfaceNumber;
          }
          else if (!strncmp(buf, bootloader_if, sizeof(bootloader_if)))
          {
            mxt_dbg(mxt->ctx, "Found %s at interface %d altsetting %d",
              buf, altsetting->bInterfaceNumber, altsetting->bAlternateSetting);

            mxt->usb.bootloader = true;
            return altsetting->bInterfaceNumber;
          }
          else
          {
            mxt_verb(mxt->ctx, "Ignoring %s at interface %d altsetting %d",
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
bool usb_is_bootloader(struct mxt_device *mxt)
{
  return mxt->usb.bootloader;
}

//******************************************************************************
/// \brief  Scan configurations
/// \return interface number, negative for error
static int usb_scan_device_configs(struct mxt_device *mxt)
{
  int i, ret;

  if (mxt->usb.bridge_chip && mxt->usb.desc.bNumConfigurations == 1)
  {
    return usb_scan_for_qrg_if(mxt);
  }

  /* Scan through interfaces */
  for (i = 0; i < mxt->usb.desc.bNumConfigurations; ++i)
  {
    struct libusb_config_descriptor *config;
    ret = libusb_get_config_descriptor(mxt->usb.device, i, &config);
    if (ret < 0)
    {
      mxt_err(mxt->ctx, "Couldn't get config descriptor %d", i);
    }
    else
    {
      ret = usb_scan_for_control_if(mxt, config);
      libusb_free_config_descriptor(config);
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
static int bridge_set_fs_mode(struct mxt_device *mxt)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  int ret;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = 0;
  pkt[1] = 0xFA;
  pkt[2] = 0xE7;

  ret = usb_transfer(mxt, &pkt, sizeof(pkt), &pkt, sizeof(pkt), true);
  if (ret < 0)
    return ret;

  libusb_reset_device(mxt->usb.handle);

  return 0;
}

//******************************************************************************
/// \brief  Set the parameters for the comms mode on USB5030
/// \return zero on success, negative error
static int bridge_configure(struct mxt_device *mxt)
{
  int ret;
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];

  /* Command packet */
  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_CONFIG;
  /* 200kHz */
  pkt[1] = 0x20;
  pkt[2] = CMD_CONFIG_I2C_RETRY_ON_NAK;
  /* I2C retry delay */
  pkt[5] = 25 * 8;

  mxt_verb(mxt->ctx, "Sending CMD_CONFIG");

  ret = usb_transfer(mxt, &pkt, sizeof(pkt), &pkt, sizeof(pkt), false);

  return (ret < 0) ? ret : 0;
}

//******************************************************************************
/// \brief Hunt for I2C device using bridge chip
/// \return zero on success, negative error
static int bridge_find_i2c_address(struct mxt_device *mxt)
{
  int ret;
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  unsigned char response;

  /* Command packet */
  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_FIND_IIC_ADDRESS;

  mxt_verb(mxt->ctx, "Sending CMD_FIND_IIC_ADDRESS");

  ret = usb_transfer(mxt, &pkt, sizeof(pkt), &pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  response = pkt[1];

  if (response == 0x81)
  {
    mxt_err(mxt->ctx, "No device found by bridge chip");
    return -1;
  }
  else
  {
    mxt_info(mxt->ctx, "Bridge found I2C device at 0x%02X", response);
    return 0;
  }
}

//******************************************************************************
/// \brief  Find device by bus/number
static int usb_find_device(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt)
{
  int ret, count, i;
  struct libusb_device **devs;
  int usb_bus, usb_device;

  count = libusb_get_device_list(ctx->usb.libusb_ctx, &devs);
  if (count <= 0)
  {
    mxt_err(mxt->ctx, "Error %d enumerating devices", count);
    return count;
  }

  for (i = 0; i < count; i++)
  {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret != LIBUSB_SUCCESS)
    {
      mxt_warn(ctx, "Error %d trying to retrieve descriptor", ret);
      continue;
    }

    usb_bus = libusb_get_bus_number(devs[i]);
    usb_device = libusb_get_device_address(devs[i]);

    if (mxt->conn->usb.bus == usb_bus && mxt->conn->usb.device == usb_device)
    {
      if (desc.idProduct == 0x6123)
      {
        mxt->usb.bridge_chip = true;
        mxt_dbg(mxt->ctx, "Found 5030 bridge chip");
      } else {
        mxt->usb.bridge_chip = false;
        mxt_verb(mxt->ctx, "Found VID=%04X PID=%04X",
            desc.idVendor, desc.idProduct);
      }

      mxt->usb.device = devs[i];
      libusb_ref_device(mxt->usb.device);
      mxt->usb.desc = desc;
      ret = 0;
      goto free_device_list;
    }
  }

  ret = -1;

free_device_list:
  libusb_free_device_list(devs, 1);
  return ret;
}

//******************************************************************************
/// \brief  Initialise library if necessary
static int usb_initialise_libusb(struct libmaxtouch_ctx *ctx)
{
  int ret = -1;

  /* Skip if already initialised */
  if (ctx->usb.libusb_ctx)
    return 0;

  /* Initialise library */
  ret = libusb_init(&(ctx->usb.libusb_ctx));

  /* Was the library initialised successfully */
  if (ret != LIBUSB_SUCCESS)
  {
    mxt_err(ctx, "Failed to initialise libusb");
    return ret;
  }

  mxt_verb(ctx, "Initialised libusb");

  /* Set the debug level for the library */
  if (mxt_get_log_level(ctx) < LOG_DEBUG)
  {
    mxt_dbg(ctx, "Enabling libusb debug");
    /* Level 3: informational messages are printed to stdout, warning and
     * error messages are printed to stderr */
    libusb_set_debug(ctx->usb.libusb_ctx, 3);
  }

  return 0;
}

//******************************************************************************
/// \brief  Try to connect device
/// \return 0 = success, negative for error
int usb_open(struct mxt_device *mxt)
{
  int ret;

  ret = usb_initialise_libusb(mxt->ctx);
  if (ret < 0)
    return ret;

  ret = usb_find_device(mxt->ctx, mxt);
  if (ret != 0)
  {
    mxt_err(mxt->ctx, "Could not find device");
    return ret;
  }

  mxt_info(mxt->ctx, "Opening USB device:%03d-%03d VID=0x%04X PID=0x%04X",
           mxt->conn->usb.bus, mxt->conn->usb.device,
           mxt->usb.desc.idVendor, mxt->usb.desc.idProduct);

  ret = libusb_open(mxt->usb.device, &mxt->usb.handle);
  if (ret != LIBUSB_SUCCESS)
  {
    mxt_err(mxt->ctx, "%s opening USB device", libusb_error_name(ret));
    return ret;
  }

  mxt->usb.interface = -1;

  ret = usb_scan_device_configs(mxt);
  if (ret < 0)
  {
    mxt_warn(mxt->ctx, "Did not find control interface");
    return ret;
  }
  else
  {
    mxt->usb.interface = ret;
  }

  /* Disconnect the kernel driver if it is active */
  if (libusb_kernel_driver_active(mxt->usb.handle, mxt->usb.interface) == 1)
  {
    mxt_verb(mxt->ctx, "Kernel driver is active - must be detached before claiming the interface");

    if (libusb_detach_kernel_driver(mxt->usb.handle, mxt->usb.interface) == 0)
    {
      mxt_verb(mxt->ctx, "Detached kernel driver");
    }
  }

  /* Claim the bInterfaceNumber 1 of the device */
  ret = libusb_claim_interface(mxt->usb.handle, mxt->usb.interface);
  if (ret != LIBUSB_SUCCESS)
  {
    mxt_err
    (
      mxt->ctx,
      "Unable to claim bInterfaceNumber %d of the device, returned %s",
      mxt->usb.interface, libusb_error_name(ret)
    );
    return -1;
  }
  else
  {
    mxt_verb(mxt->ctx, "Claimed the USB interface");
  }

  /* Get the maximum size of packets on endpoint 1 */
  ret = libusb_get_max_packet_size(libusb_get_device(mxt->usb.handle),
                                   ENDPOINT_1_IN);
  if (ret < LIBUSB_SUCCESS)
  {
    mxt_err(mxt->ctx, "%s getting maximum packet size on endpoint 1 IN",
        libusb_error_name(ret));
    return -1;
  }

  mxt->usb.ep1_in_max_packet_size = ret;
  mxt_verb(mxt->ctx, "Maximum packet size on endpoint 1 IN is %d bytes",
      mxt->usb.ep1_in_max_packet_size);

  /* Configure bridge chip if necessary */
  if (mxt->usb.bridge_chip)
  {
    mxt->usb.report_id = 0;
    ret = bridge_set_fs_mode(mxt);
    if (ret < 0)
      return ret;

    ret = bridge_configure(mxt);
    if (ret < 0)
      return ret;

    ret = bridge_find_i2c_address(mxt);
    if (ret < 0)
      return ret;
  }
  else
  {
    mxt->usb.report_id = 1;
  }

  mxt->usb.device_connected = true;
  mxt_info(mxt->ctx, "Registered USB device with VID=0x%04X PID=0x%04X Interface=%d",
      mxt->usb.desc.idVendor, mxt->usb.desc.idProduct, mxt->usb.interface);

  return 0;
}

//******************************************************************************
/// \brief  Scan for supported devices on the USB bus
int usb_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn)
{
  int ret, count, i;
  struct libusb_device **devs;
  int usb_bus, usb_device;

  ret = usb_initialise_libusb(ctx);
  if (ret < 0)
    return ret;

  count = libusb_get_device_list(ctx->usb.libusb_ctx, &devs);
  if (count <= 0)
  {
    mxt_err(ctx, "Error %d enumerating devices", count);
    return count;
  }

  for (i = 0; i < count; i++)
  {
    struct libusb_device_descriptor desc;
    ret = libusb_get_device_descriptor(devs[i], &desc);
    if (ret != LIBUSB_SUCCESS)
    {
      mxt_warn(ctx, "Error %d trying to retrieve descriptor", ret);
      continue;
    }

    if (desc.idVendor == VENDOR_ID)
    {
      if (desc.idProduct == 0x211D ||
          (desc.idProduct >= 0x2126 && desc.idProduct <= 0x212D) ||
          (desc.idProduct >= 0x2135 && desc.idProduct <= 0x2139) ||
          (desc.idProduct >= 0x213A && desc.idProduct <= 0x21FC) ||
          (desc.idProduct >= 0x8000 && desc.idProduct <= 0x8FFF) ||
          (desc.idProduct == 0x6123))
      {
        usb_bus = libusb_get_bus_number(devs[i]);
        usb_device = libusb_get_device_address(devs[i]);

        if (ctx->query)
        {
          printf("usb:%03u-%03u Atmel %04X:%04X\n",
              usb_bus, usb_device,
              desc.idVendor, desc.idProduct);
        }
        else
        {
          struct mxt_conn_info *new_conn;
          ret = mxt_new_conn(&new_conn, E_USB);
          if (ret < 0)
            return ret;

          new_conn->usb.bus = usb_bus;
          new_conn->usb.device = usb_device;

          mxt_verb(ctx, "Found VID=%04X PID=%04X",
                   desc.idVendor, desc.idProduct);

          *conn = new_conn;
          ret = 1;
          goto free_device_list;
        }
      }
      else
      {
        mxt_verb(ctx, "Ignoring VID=%04X PID=%04X", desc.idVendor, desc.idProduct);
      }
    }
  }

  ret = 0;

free_device_list:
  libusb_free_device_list(devs, 1);
  return ret;
}

//******************************************************************************
/// \brief  Release device
void usb_release(struct mxt_device *mxt)
{
  /* Are we connected to a device? */
  if (mxt->usb.device_connected)
  {
    libusb_release_interface(mxt->usb.handle, mxt->usb.interface);
    mxt_dbg(mxt->ctx, "Released the USB interface");

    libusb_close(mxt->usb.handle);
    mxt_info(mxt->ctx, "Disconnected from the device");
    mxt->usb.handle = NULL;

    mxt->usb.device_connected = false;
  }

  if (mxt->usb.device)
  {
    libusb_unref_device(mxt->usb.device);
    mxt->usb.device = NULL;
  }
}

//******************************************************************************
/// \brief  Release USB library
int usb_close(struct libmaxtouch_ctx *ctx)
{
  /* Is the library initialised? */
  if (ctx->usb.libusb_ctx)
  {
    libusb_exit(ctx->usb.libusb_ctx);
    ctx->usb.libusb_ctx = NULL;
    mxt_dbg(ctx, "Exited from libusb");
  }

  return 0;
}

//******************************************************************************
/// \brief  Restart the maxtouch chip, in normal or bootloader mode
/// \return zero on success, negative error
int usb_reset_chip(struct mxt_device *mxt, bool bootloader_mode)
{
  int ret = -1;
  uint16_t t6_addr;
  unsigned char write_value = RESET_COMMAND;

  /* Obtain command processor's address */
  t6_addr = get_object_address(mxt, GEN_COMMANDPROCESSOR_T6, 0);
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
    mxt, &write_value, t6_addr + RESET_OFFSET, 1, true
  );

  if (ret != 1)
  {
    mxt_err(mxt->ctx, "Reset of the chip unsuccessful");
    return -1;
  }

  mxt_info(mxt->ctx, "Forced a reset of the chip");

  /* Chip will be unresponsive in bootloader mode so we cannot re-connect */
  if (!bootloader_mode)
  {
    usb_release(mxt);

    /* We must wait for the chip to be ready to communicate again */
    mxt_dbg(mxt->ctx, "Waiting for the chip to re-connect");

    sleep(1);

    /* Rescan to connect to the chip again */
    ret = usb_scan(mxt->ctx, &mxt->conn);

    if (ret < 1)
    {
      mxt_err(mxt->ctx, "Failed to re-connect to chip after reset");
      return -1;
    }
  }

  return 0;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return zero on success, negative error
int usb_read_register(struct mxt_device *mxt, unsigned char *buf,
                      uint16_t start_register, size_t count)
{
  int ret;
  size_t bytes_read = 0;

  while (bytes_read < count)
  {
    ret = read_data(mxt, buf + bytes_read, start_register + bytes_read,
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
int usb_write_register(struct mxt_device *mxt, unsigned char const *buf,
                       uint16_t start_register, size_t count)
{
  int ret = 0;
  size_t bytes_written = 0;

  while (bytes_written < count)
  {
    ret = write_data(mxt, buf + bytes_written, start_register + bytes_written,
                     count - bytes_written, false);
    if (ret < 0)
      return ret;

    bytes_written += ret;
  }

  return 0;
}

//******************************************************************************
/// \brief  Read from bootloader
int usb_bootloader_read(struct mxt_device *mxt, unsigned char *buf, size_t count)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  const off_t data_offset = 2;
  size_t max_count = mxt->usb.ep1_in_max_packet_size - data_offset;
  int ret;

  if (count > max_count)
    return -1;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = IIC_DATA_1;
  pkt[1] = 0x00;
  pkt[2] = (uint8_t)count;

  ret = usb_transfer(mxt, pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  /* Output the data read from the registers */
  (void)memcpy(buf, &pkt[data_offset], count);

  return 0;
}

//******************************************************************************
/// \brief  Write packet to bootloader
/// \return Numbers of bytes written or negative error
static int usb_bootloader_write_packet(struct mxt_device *mxt,
                                       unsigned char const *buf, size_t count)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  int ret;
  const off_t data_offset = 3;
  size_t max_count = mxt->usb.ep1_in_max_packet_size - data_offset;

  if (count > max_count)
    count = max_count;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = IIC_DATA_1;
  pkt[1] = (uint8_t)count;
  pkt[2] = 0x00;

  (void)memcpy(&pkt[data_offset], buf, count);

  ret = usb_transfer(mxt, pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  return count;
}

//******************************************************************************
/// \brief  Write to bootloader
int usb_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, size_t count)
{
  int ret;
  size_t bytes = 0;

  while (bytes < count)
  {
    ret = usb_bootloader_write_packet(mxt, buf + bytes, count - bytes);
    if (ret < 0)
      return ret;

    bytes += ret;
  }

  return 0;
}

//******************************************************************************
/// \brief Read CHG line
bool usb_read_chg(struct mxt_device *mxt)
{
  unsigned char pkt[mxt->usb.ep1_in_max_packet_size];
  bool chg;
  int ret;

  memset(&pkt, 0, sizeof(pkt));
  pkt[0] = CMD_READ_PINS;

  ret = usb_transfer(mxt, pkt, sizeof(pkt), pkt, sizeof(pkt), false);
  if (ret < 0)
    return ret;

  chg = pkt[2] & 0x4;

  mxt_verb(mxt->ctx, "CHG line %s", chg ? "HIGH" : "LOW");

  return chg;
}
