//------------------------------------------------------------------------------
/// \file   mxt_app.c
/// \brief  Command line tool for Atmel maXTouch chips.
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

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"
#include "serial_data.h"
#include "libmaxtouch/msg.h"

#include "broken_line.h"
#include "sensor_variant.h"
#include "mxt_app.h"
#include "freq_sweep.h"

#define BUF_SIZE 1024

//******************************************************************************
/// \brief Initialize mXT device and read the info block
/// \return #mxt_rc
static int mxt_init_chip(struct libmaxtouch_ctx *ctx, struct mxt_device **mxt,
                         struct mxt_conn_info **conn)
{
  int ret;

  ret = mxt_scan(ctx, conn, false);
  
  if (ret == MXT_ERROR_NO_DEVICE) {
    mxt_err(ctx, "Unable to find a device");
    return ret;
  } else if (ret) {
    mxt_err(ctx, "Failed to find device");
    return ret;
  }

  ret = mxt_new_device(ctx, *conn, mxt);
  if (ret)
    return ret;

#ifdef HAVE_LIBUSB
  if ((*mxt)->conn->type == E_USB && usb_is_bootloader(*mxt)) {
    mxt_free_device(*mxt);
    mxt_err(ctx, "USB device in bootloader mode");
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }
#endif
  ret = mxt_get_info(*mxt);
  if (ret)
    return ret;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Print usage for mxt-app
static void print_usage(char *prog_name)
{
  fprintf(stderr, "Command line tool for Atmel maXTouch chips version: %s\n\n"
          "Usage: %s [command] [options]\n\n"
          "When run with no options, access menu interface.\n\n"
          "General commands:\n"
          "  -h [--help]                : display this help and exit\n"
          "  -i [--info]                : print device information\n"
          "  -M [--messages] [TIMEOUT]  : print the messages (for TIMEOUT seconds)\n"
          "  -F [--msg-filter] TYPE     : message filtering by object TYPE\n"
          "  --reset                    : reset device\n"
          "  --calibrate                : send calibrate command\n"
          "  -g                         : store golden references\n"
          "  --self-cap-tune-config     : tune self capacitance settings to config\n"
          "  --self-cap-tune-nvram      : tune self capacitance settings to NVRAM\n"
          "  --version                  : print version\n"
          "  --block-size BLOCKSIZE     : set the maximum block size used for i2c transfers (default %d)\n"
          "\n"
          "Configuration file commands:\n"
          "  --load FILE                : upload cfg from FILE in .xcfg or OBP_RAW format\n"
          "  --save FILE                : save cfg to FILE in .xcfg or OBP_RAW format\n"
          "  --format N                 : save in specific format N - 0 or 3\n"
          "  --backup[=COMMAND]         : backup configuration to NVRAM\n"
          "  --checksum FILE            : verify .xcfg or OBP_RAW file config checksum\n"
          "\n"
          "Register read/write commands:\n"
          "  -R [--read]                : read from object\n"
          "  -W [--write]               : write to object\n"
          "  -n [--count] COUNT         : read/write COUNT registers\n"
          "  -f [--format]              : format register output\n"
          "  -I [--instance] INSTANCE   : select object INSTANCE\n"
          "  -r [--register] REGISTER   : start at REGISTER (offset when used with TYPE)\n"
          "  -T [--type] TYPE           : select object TYPE\n"
          "  --zero                     : zero all configuration settings\n"
          "\n"
          "TCP socket commands:\n"
          "  -C [--bridge-client] HOST  : connect over TCP to HOST\n"
          "  -S [--bridge-server]       : start TCP socket server\n"
          "  -p [--port] PORT           : TCP port (default 4000)\n"
          "\n"
          "Bootloader commands:\n"
          "  --bootloader-version       : query bootloader version\n"
          "  --flash FIRMWARE           : send FIRMWARE to bootloader\n"
          "  --reset-bootloader         : reset device in bootloader mode\n"
          "  --firmware-version VERSION : check firmware VERSION "
          "before and after flash\n"
          "\n"
          "T68 Serial Data commands:\n"
          "  --t68-file FILE            : upload FILE\n"
          "  --t68-datatype DATATYPE    : select DATATYPE\n"
          "\n"
          "T25 Self Test commands:\n"
          "  -t [--test]                : run all self tests\n"
          "  -tXX [--test=XX]           : run individual test, write XX to CMD register\n"
          "\n"
          "T10 On-Demand Test command:\n"
          "  --odtest                   : run all on-demand self tests\n"
          "\n"
          "T37 Diagnostic Data commands:\n"
          "  --debug-dump FILE          : capture diagnostic data to FILE\n"
          "  --frames N                 : capture N frames of data\n"
          "  --instance INSTANCE        : select object INSTANCE\n"
	  "  --format 0/1               : capture using format 0 or 1\n"
          "  --references               : capture references data\n"
          "  --self-cap-signals         : capture self cap signals\n"
          "  --self-cap-deltas          : capture self cap deltas\n"
          "  --self-cap-refs            : capture self cap references\n"
	  "  --key-array-deltas         : capture key array deltas\n"
	  "  --key-array-refs           : capture key array references\n"
	  "  --key-array-signals        : capture key array signals\n"
          "  --active-stylus-deltas     : capture active stylus deltas\n"
          "  --active-stylus-refs       : capture active stylus references\n"
          "\n"
          "T72/T108 Frequency Sweep Tool:\n"
          "  --freq-sweep FILE          : input test parameter file\n"
          "  -o [--output] FILE         : output file for sweep results\n"
          "\n"
          "Broken line detection commands:\n"
          "  --broken-line              : run broken line detection\n"
          "  --dualx                    : X lines are double connected\n"
          "  --x-center-threshold N     : set X line center threshold to N percent\n"
          "  --x-border-threshold N     : set X line border threshold to N percent\n"
          "  --y-center-threshold N     : set Y line center threshold to N percent\n"
          "  --y-border-threshold N     : set Y line border threshold to N percent\n"
          "  --pattern PATTERN          : sensor PATTERN (ITO or XSense)\n"
          "\n"
          "Sensor Variant algorithm commands:\n"
          "  --sensor-variant           : Perform the Sensor Variant algorithm\n"
          "  --dualx                    : X lines are double connected\n"
          "  --fail-if-any              : Fail the Sensor Variant test on any defects\n"
          "  --max-defects N            : Maximum No. of continuious defects\n"
          "  --upper-limit N            : Upper limit for regression, in %%\n"
          "  --lower-limit N            : Lower limit for regression, in %%\n"
          "  --matrix-size N            : The allowed matrix size\n"
          "\n"
          "Device connection options:\n"
          "  -q [--query]               : scan for devices\n"
          "  -d [--device] DEVICESTRING : DEVICESTRING as output by --query\n\n"
          "  Examples:\n"
          "  -d i2c-dev:ADAPTER:ADDRESS : raw i2c device, eg \"i2c-dev:2-004a\"\n"
#ifdef HAVE_LIBUSB
          "  -d usb:BUS-DEVICE          : USB device, eg \"usb:001-003\"\n"
          "  -d usb:BUS-DEVICE-ADDRESS  : USB device, eg \"usb:001-003-4a\"\n"
#endif
          "  -d sysfs:PATH              : sysfs interface\n"
          "  -d hidraw:PATH             : HIDRAW device, eg \"hidraw:/dev/hidraw0\"\n"
          "\n"
#ifdef HAVE_LIBUSB
          "5030 Bridge Board commands:\n"
          "  --bridge-config 0xnn       : 0xnn, slave address of maXTouch chip, save address to EEPROM\n"
          "  --switch-fast              : Switches to QRG I/F mode, no USBHID reports\n"
          "  --switch-parallel          : Switches to HID Parallel Digitizer mode\n"
#endif
          "Debug options:\n"
          "  -v [--verbose] LEVEL       : set debug level\n",
          MXT_VERSION, prog_name, I2C_DEV_MAX_BLOCK);
}

//******************************************************************************
/// \brief Main function for mxt-app
int main (int argc, char *argv[])
{
  int ret;
  int c;
  int msgs_timeout = MSG_CONTINUOUS;
  bool msgs_enabled = false;
  uint8_t backup_cmd = BACKUPNV_COMMAND;
  unsigned char self_test_cmd = SELF_TEST_ALL;
  unsigned char ondemand_test_cmd = OND_RUN_ALL_TEST;
  uint16_t address = 0;
  uint16_t count = 0;
  struct mxt_conn_info *conn = NULL;
  uint16_t object_type = 0;
  uint16_t msg_filter_type = 0;
  uint8_t instance = 0;
  uint8_t verbose = 2;
  uint16_t t37_frames = 1;
  uint8_t t37_file_attr = 0;   /* 0 - write, 1 - append */
  uint8_t t37_mode = DELTAS_MODE;
  uint8_t bi2c_addr = 0x4a;
  uint8_t format = 0;
  uint16_t port = 4000;
  int i2c_block_size = I2C_DEV_MAX_BLOCK;
  uint8_t t68_datatype = 1;
  unsigned char databuf;
  char strbuf2[BUF_SIZE];
  char strbuf[BUF_SIZE];
  bool dualx = false;
  struct broken_line_options bl_opts = {0};
  bl_opts.pattern = BROKEN_LINE_PATTERN_ITO;
  bl_opts.x_center_threshold = BROKEN_LINE_DEFAULT_THRESHOLD;
  bl_opts.x_border_threshold = BROKEN_LINE_DEFAULT_THRESHOLD;
  bl_opts.y_center_threshold = BROKEN_LINE_DEFAULT_THRESHOLD;
  bl_opts.y_border_threshold = BROKEN_LINE_DEFAULT_THRESHOLD;
  struct sensor_variant_options sv_opts = {0};
  struct freq_sweep_options fs_opts = {0};
  fs_opts.freq_start = 1;
  fs_opts.freq_end = 255;
  sv_opts.max_defects = 0;
  sv_opts.upper_limit = 15;
  sv_opts.lower_limit = 15;
  sv_opts.matrix_size = 0;
  strbuf[0] = '\0';
  strbuf2[0] = '\0';
  mxt_app_cmd cmd = CMD_NONE;

  while (1) {
    int option_index = 0;

    static struct option long_options[] = {
      {"backup",           optional_argument, 0, 0},
      {"block-size",       required_argument, 0, 0},
      {"bootloader-version", no_argument,     0, 0},
      {"bridge-client",    required_argument, 0, 'C'},
      {"calibrate",        no_argument,       0, 0},
      {"checksum",         required_argument, 0, 0},
      {"debug-dump",       required_argument, 0, 0},
      {"device",           required_argument, 0, 'd'},
      {"freq-sweep",       required_argument, 0, 0},
      {"t68-file",         required_argument, 0, 0},
      {"t68-datatype",     required_argument, 0, 0},
      {"msg-filter",       required_argument, 0, 'F'},
      {"format",           required_argument, 0, 'f'},
      {"flash",            required_argument, 0, 0},
      {"firmware-version", required_argument, 0, 0},
      {"frames",           required_argument, 0, 0},
      {"help",             no_argument,       0, 'h'},
      {"info",             no_argument,       0, 'i'},
      {"instance",         required_argument, 0, 'I'},
      {"file-attr",        required_argument, 0, 0},
      {"load",             required_argument, 0, 0},
      {"save",             required_argument, 0, 0},
      {"messages",         optional_argument, 0, 'M'},
      {"broken-line",      no_argument,       0, 0},
      {"dualx",            no_argument,       0, 0},
      {"x-center-threshold",  required_argument, 0,0},
      {"x-border-threshold",  required_argument, 0,0},
      {"y-center-threshold",  required_argument, 0,0},
      {"y-border-threshold",  required_argument, 0,0},
      {"sensor-variant",      no_argument,       0, 0},
      {"fail-if-any",         no_argument,       0, 0},
      {"matrix-size",         required_argument, 0,0},
      {"max-defects",      required_argument, 0,  0},
      {"upper-limit",      required_argument, 0,  0},
      {"lower-limit",      required_argument, 0,  0},
      {"pattern",          required_argument, 0,  0},
      {"count",            required_argument, 0, 'n'},
      {"output",           required_argument, 0, 'o'},
      {"port",             required_argument, 0, 'p'},
      {"query",            no_argument,       0, 'q'},
      {"read",             no_argument,       0, 'R'},
      {"reset",            no_argument,       0, 0},
      {"reset-bootloader", no_argument,       0, 0},
      {"register",         required_argument, 0, 'r'},
      {"references",       no_argument,       0, 0},
      {"self-cap-tune-config", no_argument,       0, 0},
      {"self-cap-tune-nvram",  no_argument,       0, 0},
      {"self-cap-signals", no_argument,       0, 0},
      {"self-cap-deltas",  no_argument,       0, 0},
      {"self-cap-refs",    no_argument,       0, 0},
      {"key-array-deltas",    no_argument,       0, 0},
      {"key-array-refs",    no_argument,       0, 0},
      {"key-array-signals",    no_argument,       0, 0},
      {"active-stylus-deltas",  no_argument,       0, 0},
      {"active-stylus-refs",    no_argument,       0, 0},
      {"bridge-server",    no_argument,       0, 'S'},
      {"test",             optional_argument, 0, 't'},
      {"odtest",           no_argument,       0, 0},
      {"type",             required_argument, 0, 'T'},
      {"verbose",          required_argument, 0, 'v'},
      {"version",          no_argument,       0, 0},
      {"write",            no_argument,       0, 'W'},
      {"zero",             no_argument,       0, 0},
      {"switch-parallel",  optional_argument, 0, 0},
      {"switch-fast",      optional_argument, 0, 0},
      {"bridge-config",  required_argument, 0, 0},
      {0,                  0,                 0, 0}
    };

    c = getopt_long(argc, argv,
                    "C:d:D:f:F:ghiI:M::m:n:o:p:qRr:St::T:v:W",
                    long_options, &option_index);

    if (c == -1)
      break;

    switch (c) {
    case 0:
      if (!strcmp(long_options[option_index].name, "t68-file")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SERIAL_DATA;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "t68-datatype")) {
        t68_datatype = strtol(optarg, NULL, 0);
      } else if (!strcmp(long_options[option_index].name, "flash")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_FLASH;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "backup")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_BACKUP;
          if (optarg) {
            ret = mxt_convert_hex(optarg, &databuf, &count, sizeof(databuf));
            if (ret || count == 0) {
              fprintf(stderr, "Hex convert error\n");
              ret = MXT_ERROR_BAD_INPUT;
            }
            backup_cmd = databuf;
          }
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "calibrate")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_CALIBRATE;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "debug-dump")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_DEBUG_DUMP;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "freq-sweep")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_FREQ_SWEEP;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "broken-line")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_BROKEN_LINE;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "dualx")) {
        dualx = true;
      } else if (!strcmp(long_options[option_index].name, "x-center-threshold")) {
        if (optarg) {
          bl_opts.x_center_threshold = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "x-border-threshold")) {
        if (optarg) {
          bl_opts.x_border_threshold = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "y-center-threshold")) {
        if (optarg) {
          bl_opts.y_center_threshold = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "y-border-threshold")) {
        if (optarg) {
          bl_opts.y_border_threshold = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "pattern")) {
        strncpy(strbuf, optarg, sizeof(strbuf));
        strbuf[sizeof(strbuf) - 1] = '\0';

        if (!strcasecmp(strbuf, "xsense"))
          bl_opts.pattern = BROKEN_LINE_PATTERN_XSENSE;
        else
          bl_opts.pattern = BROKEN_LINE_PATTERN_ITO;
      } else if (!strcmp(long_options[option_index].name, "sensor-variant")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SENSOR_VARIANT;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "fail-if-any")) {
        if (optarg) {
          sv_opts.max_defects = 0;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "max-defects")) {
        if (optarg) {
          sv_opts.max_defects = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "upper-limit")) {
        if (optarg) {
          sv_opts.upper_limit = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "lower-limit")) {
        if (optarg) {
          sv_opts.lower_limit = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "matrix-size")) {
        if (optarg) {
          sv_opts.matrix_size = strtol(optarg, NULL, 0);
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "reset")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_RESET;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        } 
      } else if (!strcmp(long_options[option_index].name, "odtest")) { 
        if (cmd == CMD_NONE) {
          if (optarg) {
            ret = mxt_convert_hex(optarg, &databuf, &count, sizeof(databuf));
              if (ret) {
                fprintf(stderr, "Hex convert error\n");
                ret = MXT_ERROR_BAD_INPUT;
              } else {
                ondemand_test_cmd = databuf;
              } 
          }
            cmd = CMD_OD_TEST;
          } else {
              print_usage(argv[0]);
              return MXT_ERROR_BAD_INPUT;
          }
      } else if (!strcmp(long_options[option_index].name, "self-cap-tune-config")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SELF_CAP_TUNE_CONFIG;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "self-cap-tune-nvram")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SELF_CAP_TUNE_NVRAM;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "load")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_LOAD_CFG;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "save")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SAVE_CFG;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "reset-bootloader")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_RESET_BOOTLOADER;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "bootloader-version")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_BOOTLOADER_VERSION;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "switch-parallel")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SWITCH_PARALLEL;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "switch-fast")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_SWITCH_FAST;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } 
#ifdef HAVE_LIBUSB
        else if (!strcmp(long_options[option_index].name, "bridge-config")) {
        if (cmd == CMD_NONE) {        
          cmd = CMD_BRIDGE_CONFIG;
          if (sscanf(optarg, "%x", &conn->usb.b_i2c_addr) != 1) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            print_usage(argv[0]);
            return MXT_ERROR_BAD_INPUT;
          }
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } 
#endif
      else if (!strcmp(long_options[option_index].name, "checksum")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_CRC_CHECK;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "zero")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_ZERO_CFG;
        } else {
          print_usage(argv[0]);
          return MXT_ERROR_BAD_INPUT;
        }
      } else if (!strcmp(long_options[option_index].name, "firmware-version")) {
        strncpy(strbuf2, optarg, sizeof(strbuf2));
      } else if (!strcmp(long_options[option_index].name, "frames")) {
        t37_frames = strtol(optarg, NULL, 0);
      } else if (!strcmp(long_options[option_index].name, "file-attr")) {
        t37_file_attr = strtol(optarg, NULL, 0);
      } else if (!strcmp(long_options[option_index].name, "references")) {
        t37_mode = REFS_MODE;
      } else if (!strcmp(long_options[option_index].name, "self-cap-signals")) {
        t37_mode = SELF_CAP_SIGNALS;
      } else if (!strcmp(long_options[option_index].name, "self-cap-refs")) {
        t37_mode = SELF_CAP_REFS;
      } else if (!strcmp(long_options[option_index].name, "self-cap-deltas")) {
        t37_mode = SELF_CAP_DELTAS;
      } else if (!strcmp(long_options[option_index].name, "key-array-deltas")) {
        t37_mode = KEY_DELTAS_MODE;
      } else if (!strcmp(long_options[option_index].name, "key-array-refs")) {
        t37_mode = KEY_REFS_MODE;		  
      } else if (!strcmp(long_options[option_index].name, "key-array-signals")) {
        t37_mode = KEY_SIGS_MODE;		   
      } else if (!strcmp(long_options[option_index].name, "key-array-raw_sigs")) {
        t37_mode = KEY_RAW_SIGS_MODE;      
      } else if (!strcmp(long_options[option_index].name, "active-stylus-deltas")) {
        t37_mode = AST_DELTAS;
      } else if (!strcmp(long_options[option_index].name, "active-stylus-refs")) {
        t37_mode = AST_REFS;
      } else if (!strcmp(long_options[option_index].name, "block-size")) {
        i2c_block_size = atoi(optarg);
      } else if (!strcmp(long_options[option_index].name, "version")) {
        printf("mxt-app %s%s\n", MXT_VERSION, ENABLE_DEBUG ? " DEBUG":"");
        return MXT_SUCCESS;
      } else {
        fprintf(stderr, "Unknown option %s\n",
                long_options[option_index].name);
      }
      break;

    case 'd':
      if (optarg) {
        if (!strncmp(optarg, "i2c-dev:", 8)) {
          ret = mxt_new_conn(&conn, E_I2C_DEV);
          if (ret)
            return ret;

          if (sscanf(optarg, "i2c-dev:%d-%x",
                     &conn->i2c_dev.adapter, &conn->i2c_dev.address) != 2) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }
        } else if (!strncmp(optarg, "sysfs_i2c:", 10)) {
          ret = mxt_new_conn(&conn, E_SYSFS_I2C);
          if (ret)
            return ret;

          if (sscanf(optarg, "sysfs_i2c:%d-%x",
                     &conn->sysfs.i2c_bus, &conn->sysfs.i2c_addr) != 2) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }

          conn->sysfs.path = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!conn->sysfs.path) {
            fprintf(stderr, "calloc failure\n");
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }

          memcpy(conn->sysfs.path, optarg + 10, strlen(optarg) - 10);
        } else if (!strncmp(optarg, "sysfs_spi:", 10)) {
          ret = mxt_new_conn(&conn, E_SYSFS_SPI);
          if (ret)
            return ret;

          if (sscanf(optarg, "sysfs_spi:%d-%d",
                     &conn->sysfs.spi_bus, &conn->sysfs.spi_cs) != 2) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }
          
          conn->sysfs.path = (char *)calloc(strlen(optarg) + 255, sizeof(char));

          if (!conn->sysfs.path) {
              fprintf(stderr, "calloc failure\n");
              conn = mxt_unref_conn(conn);
              return MXT_ERROR_NO_MEM;
            }

          memcpy(conn->sysfs.path, optarg, strlen(optarg));
        }
#ifdef HAVE_LIBUSB
        else if (!strncmp(optarg, "usb:", 4)) {
          ret = mxt_new_conn(&conn, E_USB);
          if (ret)
            return ret;

          if (sscanf(optarg, "usb:%d-%d-%x", &conn->usb.bus, &conn->usb.device, 
                &conn->usb.b_i2c_addr) != 3) {
            if (sscanf(optarg, "usb:%d-%d", &conn->usb.bus, &conn->usb.device) != 2) {
                fprintf(stderr, "Invalid device string %s\n", optarg);
                conn = mxt_unref_conn(conn);
                return MXT_ERROR_NO_MEM;
            }
          }
        }
#endif
        else if (!strncmp(optarg, "hidraw:", 7)) {
          ret = mxt_new_conn(&conn, E_HIDRAW);
          if (ret)
            return ret;

          conn->hidraw.report_id = HIDRAW_REPORT_ID;

          if (sscanf(optarg, "hidraw:%s", conn->hidraw.node) != 1) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }
        } else {
          fprintf(stderr, "Invalid device string %s\n", optarg);
          conn = mxt_unref_conn(conn);
          return MXT_ERROR_BAD_INPUT;
        }
      }
      break;

    case 'C':
      if (cmd == CMD_NONE) {
        cmd = CMD_BRIDGE_CLIENT;
        strncpy(strbuf, optarg, sizeof(strbuf));
        strbuf[sizeof(strbuf) - 1] = '\0';
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'g':
      if (cmd == CMD_NONE) {
        cmd = CMD_GOLDEN_REFERENCES;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'h':
      print_usage(argv[0]);
      return MXT_SUCCESS;

    case 'f':
      if (optarg) {
        format = strtol(optarg, NULL, 0);
      }
      break;

    case 'I':
      if (optarg) {
        instance = strtol(optarg, NULL, 0);
      }
      break;

    case 'M':
      msgs_enabled = true;
      if (cmd == CMD_NONE) {
        cmd = CMD_MESSAGES;
      }
      if (optarg)
        msgs_timeout = strtol(optarg, NULL, 0);
      break;

    case 'F':
      if (optarg) {
        msg_filter_type = strtol(optarg, NULL, 0);
      }
      break;

    case 'o':
      if (optarg) {
        strncpy(strbuf2, optarg, sizeof(strbuf2));
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'n':
      if (optarg) {
        count = strtol(optarg, NULL, 0);
      }
      break;

    case 'p':
      if (optarg) {
        port = strtol(optarg, NULL, 0);
      }
      break;

    case 'q':
      if (cmd == CMD_NONE) {
        cmd = CMD_QUERY;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'r':
      if (optarg) {
        address = strtol(optarg, NULL, 0);
      }
      break;

    case 'R':
      if (cmd == CMD_NONE) {
        cmd = CMD_READ;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'S':
      if (cmd == CMD_NONE) {
        cmd = CMD_BRIDGE_SERVER;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'T':
      if (optarg) {
        object_type = strtol(optarg, NULL, 0);
      }
      break;

    case 'i':
      if (cmd == CMD_NONE) {
        cmd = CMD_INFO;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 't':
      if (cmd == CMD_NONE) {
        if (optarg) {
          ret = mxt_convert_hex(optarg, &databuf, &count, sizeof(databuf));
          if (ret) {
            fprintf(stderr, "Hex convert error\n");
            ret = MXT_ERROR_BAD_INPUT;
          } else {
            self_test_cmd = databuf;
          }
        }
        cmd = CMD_TEST;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    case 'v':
      if (optarg) {
        verbose = strtol(optarg, NULL, 0);
      }
      break;

    case 'W':
      if (cmd == CMD_NONE) {
        cmd = CMD_WRITE;
      } else {
        print_usage(argv[0]);
        return MXT_ERROR_BAD_INPUT;
      }
      break;

    default:
      /* Output newline to create space under getopt error output */
      fprintf(stderr, "\n\n");
      print_usage(argv[0]);
      return MXT_ERROR_BAD_INPUT;
    }
  }

  struct mxt_device *mxt = NULL;
  struct libmaxtouch_ctx *ctx;

  ret = mxt_new(&ctx);
  if (ret) {
    mxt_err(ctx, "Failed to init libmaxtouch");
    return ret;
  }

  /* Set debug level */
  mxt_set_log_level(ctx, verbose);
  mxt_verb(ctx, "verbose:%u", verbose);

  /* Update the i2c block size */
  if (i2c_block_size != I2C_DEV_MAX_BLOCK) {
    mxt_verb(ctx, "Setting i2c_block_size from %d to %d", ctx->i2c_block_size, i2c_block_size);
    ctx->i2c_block_size = i2c_block_size;
  }

  if (cmd == CMD_WRITE || cmd == CMD_READ) {
    mxt_verb(ctx, "instance:%u", instance);
    mxt_verb(ctx, "count:%u", count);
    mxt_verb(ctx, "address:%u", address);
    mxt_verb(ctx, "object_type:%u", object_type);
    mxt_verb(ctx, "format:%s", format ? "true" : "false");
  }

  if (cmd == CMD_QUERY) {
    ret = mxt_scan(ctx, &conn, true);
    goto free;


  /* Initialization of chip, scan new device */
  } else if (cmd != CMD_FLASH && cmd != CMD_BOOTLOADER_VERSION) {
    ret = mxt_init_chip(ctx, &mxt, &conn);
    if (ret && cmd != CMD_CRC_CHECK )
      goto free;

    if (mxt)
      mxt_set_debug(mxt, true);
  }

  switch (cmd) {
  case CMD_WRITE:
    mxt_verb(ctx, "Write command");
    ret = mxt_handle_write_cmd(mxt, object_type, count, instance, address,
                               argc, argv);
    if (ret == MXT_ERROR_BAD_INPUT)
      goto free;

    mxt_info(ctx, "Write completed");
    break;

  case CMD_READ:
    mxt_verb(ctx, "Read command");
    ret = mxt_read_object(mxt, object_type, instance, address, count, format);
       
    mxt_info(ctx, "Read completed");
    break;

  case CMD_INFO:
    mxt_verb(ctx, "CMD_INFO");
    mxt_print_info_block(mxt);
    mxt_print_config_crc(mxt);
    ret = MXT_SUCCESS;
    break;

  case CMD_GOLDEN_REFERENCES:
    mxt_verb(ctx, "CMD_GOLDEN_REFERENCES");
    ret = mxt_store_golden_refs(mxt);
    break;

  case CMD_BRIDGE_SERVER:
    mxt_verb(ctx, "CMD_BRIDGE_SERVER");
    mxt_verb(ctx, "port:%u", port);
    ret = mxt_socket_server(mxt, port);
    break;

  case CMD_BRIDGE_CLIENT:
    mxt_verb(ctx, "CMD_BRIDGE_CLIENT");
    ret = mxt_socket_client(mxt, strbuf, port);
    break;

  case CMD_SERIAL_DATA:
    mxt_verb(ctx, "CMD_SERIAL_DATA");
    mxt_verb(ctx, "t68_datatype:%u", t68_datatype);
    ret = mxt_serial_data_upload(mxt, strbuf, t68_datatype);
    break;

  case CMD_TEST:
    mxt_verb(ctx, "CMD_TEST");
    ret = run_self_tests(mxt, self_test_cmd, 0);
    break;

  case CMD_OD_TEST:
    mxt_verb(ctx, "CMD_OD_TEST");
    ret = run_self_tests(mxt, ondemand_test_cmd, 1);
    break;

  case CMD_FLASH:
    mxt_verb(ctx, "CMD_FLASH");
    ret = mxt_flash_firmware(ctx, mxt, strbuf, strbuf2, conn);
    break;

  case CMD_RESET:
    mxt_verb(ctx, "CMD_RESET");
    ret = mxt_reset_chip(mxt, false, 0);
    break;

  case CMD_BROKEN_LINE:
    if (dualx)
      bl_opts.dualx = dualx;
    mxt_verb(ctx, "CMD_BROKEN_LINE");
    ret = mxt_broken_line(mxt, &bl_opts);
    break;

  case CMD_SENSOR_VARIANT:
    if (dualx)
      sv_opts.dualx = dualx;
    mxt_verb(ctx, "CMD_SENSOR_VARIANT");
    ret = mxt_sensor_variant(mxt, &sv_opts);
    break;

  case CMD_RESET_BOOTLOADER:
    mxt_verb(ctx, "CMD_RESET_BOOTLOADER");
    ret = mxt_reset_chip(mxt, true, 0);
    break;

  case CMD_BOOTLOADER_VERSION:
    mxt_verb(ctx, "CMD_RESET_BOOTLOADER");
    ret = mxt_bootloader_version(ctx, mxt, conn);
    break;
#ifdef HAVE_LIBUSB
  case CMD_SWITCH_PARALLEL:
    mxt_verb(ctx, "CMD_SWITCH_PARALLEL");
    ret = usb_switch_parallel_mode(mxt, conn);
    break;

  case CMD_SWITCH_FAST:
    mxt_verb(ctx, "CMD_SWITCH_FAST");
    ret = usb_switch_fast_mode(mxt, conn);
    break;

  case CMD_BRIDGE_CONFIG:
    mxt_verb(ctx, "CMD_BRIDGE_CONFIG");
    ret = bridge_configure(mxt);
    break;
#endif

  case CMD_MESSAGES:
    // Messages handled after switch
    break;

  case CMD_BACKUP:
    mxt_verb(ctx, "CMD_BACKUP");
    ret = mxt_backup_config(mxt, backup_cmd);
    break;

  case CMD_CALIBRATE:
    mxt_verb(ctx, "CMD_CALIBRATE");
    ret = mxt_calibrate_chip(mxt);
    break;

  case CMD_DEBUG_DUMP:
    mxt_verb(ctx, "CMD_DEBUG_DUMP");
    mxt_verb(ctx, "mode:%u", t37_mode);
    mxt_verb(ctx, "frames:%u", t37_frames);
    mxt_verb(ctx, "file_attr:%u", t37_file_attr);
    ret = mxt_debug_dump(mxt, t37_mode, strbuf, t37_frames, instance, format, t37_file_attr);
    break;

  case CMD_FREQ_SWEEP:
    mxt_verb(ctx, "CMD_FREQ_SWEEP");
    mxt_verb(ctx, "filename: %s", strbuf);
    ret = mxt_freq_sweep(mxt, strbuf, strbuf2, &fs_opts);
    break;

  case CMD_ZERO_CFG:
    mxt_verb(ctx, "CMD_ZERO_CFG");
    ret = mxt_zero_config(mxt);
    if (ret)
      mxt_err(ctx, "Error zeroing all configuration settings");
    break;

  case CMD_LOAD_CFG:
    mxt_verb(ctx, "CMD_LOAD_CFG");
    mxt_verb(ctx, "filename:%s", strbuf);
    ret = mxt_load_config_file(mxt, strbuf);
    if (ret) {
      mxt_err(ctx, "Error loading the configuration");
    } else {
      mxt_info(ctx, "Configuration loaded");
    }
    break;

  case CMD_SAVE_CFG:
    mxt_verb(ctx, "CMD_SAVE_CFG");
    mxt_verb(ctx, "filename:%s", strbuf);
    mxt_verb(ctx, "format %d", format);
    ret = mxt_save_config_file(mxt, strbuf, format);
    break;

  case CMD_SELF_CAP_TUNE_CONFIG:
  case CMD_SELF_CAP_TUNE_NVRAM:
    mxt_verb(ctx, "CMD_SELF_CAP_TUNE");
    ret = mxt_self_cap_tune(mxt, cmd);
    break;

  case CMD_CRC_CHECK:
    mxt_verb(ctx, "CMD_CRC_CHECK");
    mxt_verb(ctx, "filename:%s", strbuf);
    ret = mxt_checkcrc(mxt, strbuf);
    break;

  case CMD_NONE:
  default:
    mxt_verb(ctx, "cmd: %d", cmd);
    mxt_set_log_fn(ctx, mxt_log_stdout);

    if (verbose <= 2)
      mxt_set_log_level(ctx, 2);

    ret = mxt_menu(mxt);
    break;
  }

  if (cmd == CMD_MESSAGES || (msgs_enabled && ret == MXT_SUCCESS)) {
    mxt_verb(ctx, "CMD_MESSAGES");
    mxt_verb(ctx, "msgs_timeout:%d", msgs_timeout);
    // Support message filtering with -T
    if (cmd == CMD_MESSAGES && !msg_filter_type)
      msg_filter_type = object_type;

    ret = print_raw_messages(mxt, msgs_timeout, msg_filter_type);
  }

  if (cmd != CMD_FLASH && cmd != CMD_BOOTLOADER_VERSION && mxt) {
    mxt_set_debug(mxt, false);
    mxt_free_device(mxt);
    mxt_unref_conn(conn);
  }

free:
  mxt_free(ctx);

  return ret;
}
