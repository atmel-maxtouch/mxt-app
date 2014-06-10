#pragma once
//------------------------------------------------------------------------------
/// \file   mxt_app.h
/// \brief  mxt-app header file
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2012 Atmel Corporation. All rights reserved.
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

#include <signal.h>

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
           _a < _b ? _a : _b; })

/* T6 Debug Diagnostics Commands */
#define PAGE_UP           0x01
#define PAGE_DOWN         0x02
#define DELTAS_MODE       0x10
#define REFS_MODE         0x11
#define SELF_CAP_SIGNALS  0xF5
#define SELF_CAP_DELTAS   0xF7
#define SELF_CAP_REFS     0xF8

/* T25 Self Test Commands */
#define SELF_TEST_ANALOG       0x01
#define SELF_TEST_PIN_FAULT    0x11
#define SELF_TEST_PIN_FAULT_2  0x12
#define SELF_TEST_AND_GATE     0x13
#define SELF_TEST_SIGNAL_LIMIT 0x17
#define SELF_TEST_GAIN         0x20
#define SELF_TEST_OFFSET       0x21
#define SELF_TEST_ALL          0xFE
#define SELF_TEST_INVALID      0xFD
#define SELF_TEST_TIMEOUT      0xFC

/* Message Timeout Options */
#define MSG_NO_WAIT            0
#define MSG_CONTINUOUS         -1

//******************************************************************************
/// \brief Commands for mxt-app
typedef enum mxt_app_cmd_t {
  CMD_NONE,
  CMD_QUERY,
  CMD_INFO,
  CMD_TEST,
  CMD_WRITE,
  CMD_READ,
  CMD_GOLDEN_REFERENCES,
  CMD_BRIDGE_CLIENT,
  CMD_BRIDGE_SERVER,
  CMD_SERIAL_DATA,
  CMD_FLASH,
  CMD_RESET,
  CMD_RESET_BOOTLOADER,
  CMD_BACKUP,
  CMD_CALIBRATE,
  CMD_DEBUG_DUMP,
  CMD_LOAD_CFG,
  CMD_SAVE_CFG,
  CMD_MESSAGES,
  CMD_SELF_CAP_TUNE_CONFIG,
  CMD_SELF_CAP_TUNE_NVRAM,
} mxt_app_cmd;

//******************************************************************************
/// \brief T37 Diagnostic Data context object
struct t37_ctx {
  struct mxt_device *mxt;
  struct libmaxtouch_ctx *lc;

  bool self_cap;

  int x_size;
  int y_size;

  int num_data_values;
  int num_passes;
  int pages;
  int stripe_width;
  int stripe_starty;
  int stripe_endy;
  uint8_t page_size;
  uint8_t mode;

  int diag_cmd_addr;
  int t37_addr;
  int t37_size;
  uint8_t t111_instances;

  uint16_t frame;
  int pass;
  int page;
  int x_ptr;
  int y_ptr;

  uint8_t *page_buf;
  uint16_t *data_buf;

  FILE *hawkeye;
};

int mxt_flash_firmware(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, const char *filename, const char *new_version, struct mxt_conn_info *conn);
int mxt_socket_server(struct mxt_device *mxt, uint16_t port);
int mxt_socket_client(struct mxt_device *mxt, char *ip_address, uint16_t port);
int mxt_debug_dump(struct mxt_device *mxt, int mode, const char *csv_file, uint16_t frames);
void mxt_dd_menu(struct mxt_device *mxt);
int mxt_store_golden_refs(struct mxt_device *mxt);
int mxt_menu(struct mxt_device *mxt);
uint8_t self_test_menu(struct mxt_device *mxt);
int run_self_tests(struct mxt_device *mxt, uint8_t cmd);
int mxt_serial_data_upload(struct mxt_device *mxt, const char *filename, uint16_t datatype);
int print_raw_messages(struct mxt_device *mxt, int timeout, uint16_t object_type);
int print_raw_messages_t44(struct mxt_device *mxt);
void print_t6_status(uint8_t status);
int mxt_self_cap_tune(struct mxt_device *mxt, mxt_app_cmd cmd);
int mxt_read_messages(struct mxt_device *mxt, int timeout_seconds, void *context, int (*msg_func)(struct mxt_device *mxt, uint8_t *msg, void *context, uint8_t size));
int mxt_debug_dump_frame(struct t37_ctx *ctx);
int mxt_debug_dump_initialise(struct t37_ctx *ctx);
void mxt_init_sigint_handler(struct mxt_device *mxt, struct sigaction sa);
void mxt_release_sigint_handler(struct mxt_device *mxt, struct sigaction sa);
int mxt_get_sigint_flag(void);
