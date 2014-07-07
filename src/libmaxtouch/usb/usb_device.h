#pragma once
//------------------------------------------------------------------------------
/// \file   usb_device.h
/// \brief  Headers for MXT device low level access via USB
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

#ifdef ANDROID
#include <libusbdroid/code/src/LibUSBDroid/jni/libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#define USB_MAX_BUS_DEVICES  127

//******************************************************************************
/// \brief USB library context
struct usb_context
{
  libusb_context *libusb_ctx;
};

//******************************************************************************
/// \brief USB library context
struct usb_conn_info
{
  int bus;
  int device;
};

//******************************************************************************
/// \brief USB device information
struct usb_device
{
  bool device_connected;
  bool bridge_chip;
  libusb_device *device;
  libusb_device_handle *handle;
  struct libusb_device_descriptor desc;
  int ep1_in_max_packet_size;
  int interface;
  bool bootloader;
  int report_id;
};

int usb_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn);
int usb_open(struct mxt_device *mxt);
int usb_close(struct libmaxtouch_ctx *ctx);
void usb_release(struct mxt_device *mxt);
int usb_reset_chip(struct mxt_device *mxt, bool bootloader_mode);
int usb_read_register(struct mxt_device *mxt, unsigned char *buf, uint16_t start_register, size_t count);
int usb_write_register(struct mxt_device *mxt, unsigned char const *buf, uint16_t start_register, size_t count);
int usb_bootloader_read(struct mxt_device *mxt, unsigned char *buf, size_t count);
int usb_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, size_t count);
bool usb_is_bootloader(struct mxt_device *mxt);
int usb_read_chg(struct mxt_device *mxt, bool *value);
int usb_find_bus_devices(struct mxt_device *mxt, bool *device_list);
int usb_rediscover_device(struct mxt_device *mxt, bool *device_list);
