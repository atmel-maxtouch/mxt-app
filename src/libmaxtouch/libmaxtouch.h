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

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <signal.h>

struct libmaxtouch_ctx;
struct mxt_device;
struct mxt_conn_info;

#include "log.h"
#include "sysfs/sysfs_device.h"
#include "i2c_dev/i2c_dev_device.h"
#ifdef HAVE_LIBUSB
#include "usb/usb_device.h"
#endif
#include "hidraw/hidraw_device.h"
#include "info_block.h"

/* GEN_COMMANDPROCESSOR_T6 Register offsets from T6 base address */
#define MXT_T6_RESET_OFFSET      0x00
#define MXT_T6_BACKUPNV_OFFSET   0x01
#define MXT_T6_CALIBRATE_OFFSET  0x02
#define MXT_T6_REPORTALL_OFFSET  0x03
#define MXT_T6_RESERVED_OFFSET   0x04
#define MXT_T6_DIAGNOSTIC_OFFSET 0x05

/* Values to write to the command processor fields */
#define RESET_COMMAND       0x01
#define BOOTLOADER_COMMAND  0xA5
#define BACKUPNV_COMMAND    0x55
#define CALIBRATE_COMMAND   0x01

/* Prefix for T5 messages */
#define MSG_PREFIX "MXT MSG:"

/* Polling delay for continually polling messages */
#define MXT_MSG_POLL_DELAY_MS 10

/* Calibrate timeout */
#define MXT_CALIBRATE_TIMEOUT 10

//******************************************************************************
/// \brief Return codes
enum mxt_rc {
  MXT_SUCCESS = 0,                           /*!< Success */
  MXT_INTERNAL_ERROR = 1,                    /*!< Internal error/assert */
  MXT_ERROR_IO = 2,                          /*!< Input/output error */
  MXT_ERROR_NO_MEM = 3,                      /*!< Memory allocation failure */
  MXT_ERROR_TIMEOUT = 4,                     /*!< Timeout */
  MXT_ERROR_NO_DEVICE = 5,                   /*!< Could not find a device or device went away */
  MXT_ERROR_ACCESS = 6,                      /*!< Permission denied */
  MXT_ERROR_NOT_SUPPORTED = 7,               /*!< Operation not allowed for this device type */
  MXT_ERROR_INTERRUPTED = 8,                 /*!< Interrupt function call */
  MXT_ERROR_OBJECT_NOT_FOUND = 9,            /*!< Object not available on device */
  MXT_ERROR_NO_MESSAGE = 10,                 /*!< Received unexpected invalid message from message processor */
  MXT_ERROR_SELF_TEST_INVALID = 11,          /*!< Self test invalid test command */
  MXT_ERROR_SELF_TEST_ANALOG = 12,           /*!< Self test AVdd Analog power is not present */
  MXT_ERROR_SELF_TEST_PIN_FAULT = 13,        /*!< Self test Pin fault */
  MXT_ERROR_SELF_TEST_AND_GATE = 14,         /*!< Self test AND Gate Fault */
  MXT_ERROR_SELF_TEST_SIGNAL_LIMIT = 15,     /*!< Self test Signal limit fault */
  MXT_ERROR_SELF_TEST_GAIN = 16,             /*!< Self test Gain error */
  MXT_ERROR_INFO_CHECKSUM_MISMATCH = 17,     /*!< Information block checksum error */
  MXT_ERROR_BOOTLOADER_UNLOCKED = 18,        /*!< Bootloader already unlocked */
  MXT_ERROR_BOOTLOADER_FRAME_CRC_FAIL = 19,  /*!< Bootloader CRC failure (transmission failure) */
  MXT_ERROR_FILE_FORMAT = 20,                /*!< File format error */
  MXT_FIRMWARE_UPDATE_NOT_REQUIRED = 21,     /*!< Device firmware already required version */
  MXT_ERROR_BOOTLOADER_NO_ADDRESS = 22,      /*!< Could not identify bootloader address */
  MXT_ERROR_FIRMWARE_UPDATE_FAILED = 23,     /*!< Version on device did not match version given after bootloading operation */
  MXT_ERROR_RESET_FAILURE = 24,              /*!< Device did not reset */
  MXT_ERROR_UNEXPECTED_DEVICE_STATE = 25,    /*!< Device in unexpected state */
  MXT_ERROR_BAD_INPUT = 26,                  /*!< Incorrect command line parameters or menu input given */
  MXT_ERROR_PROTOCOL_FAULT = 27,             /*!< Bridge TCP protocol parse error */
  MXT_ERROR_CONNECTION_FAILURE = 28,         /*!< Bridge connection error */
  MXT_ERROR_SERIAL_DATA_FAILURE = 29,        /*!< Serial data download failed */
  MXT_ERROR_NOENT = 30,                      /*!< No such file or directory */
  MXT_ERROR_SELFCAP_TUNE = 31,               /*!< Error processing self cap command */
  MXT_MSG_CONTINUE = 32,                     /*!< Continue processing messages */
  MXT_ERROR_RESERVED = 33,                   /*!< Reserved value */
  MXT_DEVICE_IN_BOOTLOADER = 34,             /*!< Device is in bootloader mode */
};

//******************************************************************************
/// \brief Device connection type
enum mxt_device_type {
  E_SYSFS,
#ifdef HAVE_LIBUSB
  E_USB,
#endif
  E_I2C_DEV,
  E_HIDRAW,
};

//******************************************************************************
/// \brief Libmaxtouch context
struct libmaxtouch_ctx
{
  bool query;
  bool query_found_device;
  enum mxt_log_level log_level;

  void (*log_fn)(struct libmaxtouch_ctx *ctx, enum mxt_log_level level,
                 const char *format, va_list args);

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
  int refcount;

  union
  {
    struct i2c_dev_conn_info i2c_dev;
    struct hidraw_conn_info hidraw;
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
  struct mxt_conn_info *conn;
  struct libmaxtouch_ctx *ctx;
  struct mxt_info info;
  struct mxt_report_id_map *report_id_map;
  char msg_string[255];

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
int mxt_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn, bool query);
int mxt_new_conn(struct mxt_conn_info **conn, enum mxt_device_type type);
struct mxt_conn_info *mxt_ref_conn(struct mxt_conn_info *conn);
struct mxt_conn_info *mxt_unref_conn(struct mxt_conn_info *conn);
int mxt_new_device(struct libmaxtouch_ctx *ctx, struct mxt_conn_info *conn, struct mxt_device **mxt);
void mxt_set_log_fn(struct libmaxtouch_ctx *ctx, void (*log_fn)(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, va_list args));
void mxt_free_device(struct mxt_device *mxt);
int mxt_get_info(struct mxt_device *mxt);
int mxt_read_register(struct mxt_device *mxt, uint8_t *buf, int start_register, int count);
int mxt_write_register(struct mxt_device *mxt, uint8_t const *buf, int start_register, int count);
int mxt_set_debug(struct mxt_device *mxt, bool debug_state);
int mxt_get_debug(struct mxt_device *mxt, bool *value);
int mxt_reset_chip(struct mxt_device *mxt, bool bootloader_mode);
int mxt_calibrate_chip(struct mxt_device *mxt);
int mxt_backup_config(struct mxt_device *mxt, uint8_t backup_command);
int mxt_load_config_file(struct mxt_device *mxt, const char *cfg_file);
int mxt_save_config_file(struct mxt_device *mxt, const char *filename);
int mxt_zero_config(struct mxt_device *mxt);
int mxt_get_msg_count(struct mxt_device *mxt, int *count);
char *mxt_get_msg_string(struct mxt_device *mxt);
int mxt_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf, size_t buflen, int *count);
int mxt_msg_reset(struct mxt_device *mxt);
int mxt_get_msg_poll_fd(struct mxt_device *mxt);
int mxt_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count);
int mxt_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count);
int mxt_msg_wait(struct mxt_device *mxt, int timeout_ms);
int mxt_errno_to_rc(int errno_in);
int mxt_report_all(struct mxt_device *mxt);

#ifdef __cplusplus
}
#endif
