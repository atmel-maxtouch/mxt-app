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

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
           _a < _b ? _a : _b; })

/* Object offsets */
#define T100_XORIGIN_OFFSET    0x08
#define T100_YORIGIN_OFFSET    0x13
#define T100_YSIZE_OFFSET      0x14
#define T100_XSIZE_OFFSET      0x09
#define T9_XORIGIN_OFFSET      0x01
#define T9_YORIGIN_OFFSET      0x02
#define T9_YSIZE_OFFSET        0x04
#define T9_XSIZE_OFFSET        0x03

/* T15 Key Object offsets */
#define T15_CTRL_OFFSET			0x00
#define T15_XORIGIN_OFFSET		0x01
#define T15_YORIGIN_OFFSET		0x02
#define T15_XSIZE_OFFSET		0x03
#define T15_YSIZE_OFFSET		0x04

/* T6 Debug Diagnostics Commands */
#define PAGE_UP           0x01
#define PAGE_DOWN         0x02
#define DELTAS_MODE       0x10
#define REFS_MODE         0x11
#define KEY_DELTAS_MODE	  0x17
#define KEY_REFS_MODE     0x18
#define KEY_SIGS_MODE	    0x19
#define KEY_RAW_SIGS_MODE 0x20
#define SELF_CAP_SIGNALS  0xF5
#define SELF_CAP_DELTAS   0xF7
#define SELF_CAP_REFS     0xF8
#define AST_DELTAS        0xFB
#define AST_REFS          0xFC

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

/* T10 Status field  */
#define OND_TEST_ALL_PASS      0x31
#define OND_TEST_FAILED        0x32
#define OND_TEST_INVALID       0x3F
#define POST_TEST_ALL_PASS     0x11
#define POST_TEST_FAILED       0x12
#define BIST_TEST_ALL_PASS     0x21
#define BIST_TEST_FAILED       0x22
#define BIST_TEST_OVERRUN      0x23

/* T10 Commands */
#define OND_POWER_TEST         0x31
#define OND_PIN_FAULT_TEST     0x32
#define OND_SIGNAL_LIMIT_TEST  0x33
#define OND_RUN_ALL_TEST       0x3E

/* T10 Test Field Errors */

#define CLOCK_FAILURE          0x02
#define FLASH_MEM_FAILURE      0x03
#define RAM_MEM_FAILURE        0x04
#define CTE_TEST_FAILURE       0x05
#define SIGNAL_LIMIT_FAILURE   0x06
#define POWER_TEST_FAILURE     0x07
#define PIN_FAULT_FAILURE      0x08

/* SELF Test Errors */

#define PIN_FAULT_TO_PWR       0x01
#define PIN_FAULT_TO_GND       0x02
#define PIN_SHORT_LOW          0x03
#define PIN_SHORT_HIGH         0x04
#define INIT_HIGH_VOLT         0x07

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
  CMD_OD_TEST,
  CMD_WRITE,
  CMD_READ,
  CMD_GOLDEN_REFERENCES,
  CMD_BRIDGE_CLIENT,
  CMD_BRIDGE_SERVER,
  CMD_SERIAL_DATA,
  CMD_FLASH,
  CMD_RESET,
  CMD_RESET_BOOTLOADER,
  CMD_BOOTLOADER_VERSION,
  CMD_BACKUP,
  CMD_CALIBRATE,
  CMD_DEBUG_DUMP,
  CMD_LOAD_CFG,
  CMD_SAVE_CFG,
  CMD_MESSAGES,
  CMD_SELF_CAP_TUNE_CONFIG,
  CMD_SELF_CAP_TUNE_NVRAM,
  CMD_ZERO_CFG,
  CMD_BROKEN_LINE,
  CMD_SENSOR_VARIANT,
  CMD_CRC_CHECK,
} mxt_app_cmd;

//******************************************************************************
/// \brief Signal handler semaphore
extern volatile sig_atomic_t mxt_sigint_rx;

struct t37_diagnostic_data;
struct mxt_conn_info;

//******************************************************************************
/// \brief T37 Diagnostic Data context object
struct t37_ctx {
  struct mxt_device *mxt;
  struct libmaxtouch_ctx *lc;

  bool self_cap;
  bool active_stylus;
  bool t15_keyarray;
  bool fformat;

  int x_size;
  int y_size;

  int data_values;
  int passes;
  int pages_per_pass;
  int stripe_width;
  int stripe_starty;
  int stripe_endy;
  uint8_t page_size;
  uint8_t mode;

  int diag_cmd_addr;
  int t37_addr;
  int t37_size;
  uint8_t t111_instances;
  uint8_t t107_instances;
  uint8_t t15_instances;
  uint8_t t100_instances;
  uint8_t t9_instances;
  uint8_t instance;
  uint8_t file_attr;

  uint16_t frame;
  int pass;
  int page;
  int x_ptr;
  int y_ptr;

  double mean;
  double variance;
  double std_dev;

  struct t37_diagnostic_data *t37_buf;
  uint16_t *data_buf;
  uint16_t *temp_buf;
  uint8_t *key_buf;

  FILE *hawkeye;
};

//******************************************************************************
/// \brief Touchscreen info context
struct mxt_touchscreen_info {
  uint16_t instance_addr;
  uint8_t xorigin;
  uint8_t yorigin;
  uint8_t xsize;
  uint8_t ysize;
};

//******************************************************************************
/// \brief Touchscreen info context
struct mxt_t15_info {
  uint16_t t15_instance_addr;
  uint8_t t15_xorigin;
  uint8_t t15_yorigin;
  uint8_t t15_xsize;
  uint8_t t15_ysize;
  uint8_t t15_enable;
};



int mxt_flash_firmware(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, const char *filename, const char *new_version, struct mxt_conn_info *conn);
int mxt_socket_server(struct mxt_device *mxt, uint16_t port);
int mxt_socket_client(struct mxt_device *mxt, char *ip_address, uint16_t port);
int mxt_debug_dump(struct mxt_device *mxt, int mode, const char *csv_file, uint16_t frames, uint16_t obj_inst, uint16_t format, uint8_t attr);
void mxt_dd_menu(struct mxt_device *mxt);
void mxt_dd_menu2(struct mxt_device *mxt, char selection);
void mxt_mutual_menu(struct mxt_device *mxt, char selection);
void mxt_self_menu(struct mxt_device *mxt, char selection);
void mxt_stylus_menu(struct mxt_device *mxt, char selection);
void mxt_key_array_menu(struct mxt_device *mxt, char selection);
int mxt_store_golden_refs(struct mxt_device *mxt);
int mxt_menu(struct mxt_device *mxt);
uint8_t self_test_main_menu(struct mxt_device *mxt);
int run_self_tests(struct mxt_device *mxt, uint8_t cmd, bool type);
uint8_t self_test_t10_menu(struct mxt_device *mxt);
int print_raw_messages(struct mxt_device *mxt, int timeout, uint16_t object_type);
int print_raw_messages_t44(struct mxt_device *mxt);
void print_t6_status(uint8_t status);
int mxt_self_cap_tune(struct mxt_device *mxt, mxt_app_cmd cmd);
int mxt_read_diagnostic_data_frame(struct mxt_device *mxt, struct t37_ctx *ctx);
int mxt_debug_dump_initialise(struct mxt_device *mxt, struct t37_ctx *ctx);
sig_atomic_t mxt_get_sigint_flag(void);
int mxt_read_messages_sigint(struct mxt_device *mxt, int timeout_seconds, void *context, int (*msg_func)(struct mxt_device *mxt, uint8_t *msg, void *context, uint8_t size));
int mxt_bootloader_version(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, struct mxt_conn_info *conn);
int disable_gr(struct mxt_device *mxt);
int16_t get_value(struct t37_ctx *ctx, int x, int y);
int debug_frame_calc_stats(struct t37_ctx *ctx);
int debug_frame_normalise(struct t37_ctx *ctx);
int mxt_read_touchscreen_info(struct mxt_device *mxt, struct mxt_touchscreen_info **mxt_ts_info);
int mxt_read_t15_key_info(struct mxt_device *mxt, struct mxt_t15_info **mxt_key_info);
float reference_no_offset(float val);
int mxt_free_run_mode(struct mxt_device *mxt);
int mxt_disable_touch(struct mxt_device *mxt);
int debug_frame(struct t37_ctx *ctx);
