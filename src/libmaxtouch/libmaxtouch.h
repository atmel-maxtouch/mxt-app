#pragma once
//------------------------------------------------------------------------------
/// \file   libmaxtouch.h
/// \brief  headers for MXT device low level access
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

#include <stdbool.h>

struct libmaxtouch_ctx;
struct mxt_device;
struct mxt_conn_info;

#include "sysfs/sysfs_device.h"
#include "i2c_dev/i2c_dev_device.h"
#ifdef HAVE_LIBUSB
#include "usb/usb_device.h"
#endif
#include "info_block.h"

/* Address offsets for the command processor fields */
#define RESET_OFFSET        0
#define BACKUPNV_OFFSET     1
#define CALIBRATE_OFFSET    2

/* Values to write to the command processor fields */
#define RESET_COMMAND       0x01
#define BOOTLOADER_COMMAND  0xA5
#define BACKUPNV_COMMAND    0x55
#define CALIBRATE_COMMAND   0x01

/* Prefix for T5 messages */
#define MSG_PREFIX "MXT MSG:"

//******************************************************************************
/// \brief Device connection type
enum mxt_device_type {
  E_NONE,
  E_SYSFS,
  E_SYSFS_DEBUG_NG,
#ifdef HAVE_LIBUSB
  E_USB,
#endif
  E_I2C_DEV,
};

//******************************************************************************
/// \brief Libmaxtouch context
struct libmaxtouch_ctx
{
  bool query;

  union
  {
#ifdef HAVE_LIBUSB
    struct usb_context usb;
#endif
  };
};

//******************************************************************************
/// \brief Device connection parameters
struct mxt_conn_info
{
  enum mxt_device_type type;

  union
  {
    struct i2c_dev_conn_info i2c_dev;
    struct sysfs_conn_info sysfs;
#ifdef HAVE_LIBUSB
    struct usb_conn_info usb;
#endif
  };
};

//******************************************************************************
/// \brief Device context
struct mxt_device
{
  struct mxt_conn_info conn;
  struct libmaxtouch_ctx *ctx;
  struct info_block info_block;
  struct report_id_map *report_id_map;
  uint8_t *raw_info;

  union
  {
    struct sysfs_device sysfs;
#ifdef HAVE_LIBUSB
    struct usb_device usb;
#endif
    struct i2c_dev_device i2c_dev;
  };
};

int mxt_new(struct libmaxtouch_ctx **ctx);
int mxt_free(struct libmaxtouch_ctx *ctx);
int mxt_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info *conn, bool query);
int mxt_new_device(struct libmaxtouch_ctx *ctx, struct mxt_conn_info conn, struct mxt_device **mxt);
void mxt_free_device(struct mxt_device *mxt);
int mxt_get_info(struct mxt_device *mxt);
const char *mxt_get_input_event_file(struct mxt_device *mxt);
int mxt_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, int count);
int mxt_write_register(struct mxt_device *mxt, unsigned char const *buf, int start_register, int count);
int mxt_set_debug(struct mxt_device *mxt, bool debug_state);
bool mxt_get_debug(struct mxt_device *mxt);
int mxt_reset_chip(struct mxt_device *mxt, bool bootloader_mode);
int mxt_calibrate_chip(struct mxt_device *mxt);
int mxt_backup_config(struct mxt_device *mxt);
int mxt_load_config_file(struct mxt_device *mxt, const char *cfg_file);
int mxt_save_config_file(struct mxt_device *mxt, const char *cfg_file);
int mxt_get_msg_count(struct mxt_device *mxt);
char *mxt_get_msg_string(struct mxt_device *mxt);
int mxt_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf, size_t buflen);
int mxt_msg_reset(struct mxt_device *mxt);
int mxt_get_msg_poll_fd(struct mxt_device *mxt);
int mxt_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count);
int mxt_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count);
void mxt_msg_wait(struct mxt_device *mxt, int timeout_ms);
int mxt_new_i2c_dev(struct mxt_device **mxt, int adapter, int address);
