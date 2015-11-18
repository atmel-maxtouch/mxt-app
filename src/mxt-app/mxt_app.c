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

#include "mxt_app.h"

#define BUF_SIZE 1024


//******************************************************************************
/// \brief Initialize mXT device and read the info block
/// \return #mxt_rc
static int mxt_init_chip(struct libmaxtouch_ctx *ctx, struct mxt_device **mxt,
                         struct mxt_conn_info **conn)
{
  int ret;

  if (!*conn) {
    ret = mxt_scan(ctx, conn, false);
    if (ret == MXT_ERROR_NO_DEVICE) {
      mxt_err(ctx, "Unable to find a device");
      return ret;
    } else if (ret) {
      mxt_err(ctx, "Failed to find device");
      return ret;
    }
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
          "  --reset-bootloader         : reset device in bootloader mode\n"
          "  --calibrate                : send calibrate command\n"
          "  --backup[=COMMAND]         : backup configuration to NVRAM\n"
          "  -g                         : store golden references\n"
          "  --self-cap-tune-config     : tune self capacitance settings to config\n"
          "  --self-cap-tune-nvram      : tune self capacitance settings to NVRAM\n"
          "  --version                  : print version\n"
          "\n"
          "Configuration file commands:\n"
          "  --load FILE                : upload cfg from FILE in .xcfg or OBP_RAW format\n"
          "  --save FILE                : save cfg to FILE in .xcfg or OBP_RAW format\n"
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
          "T37 Diagnostic Data commands:\n"
          "  --debug-dump FILE          : capture diagnostic data to FILE\n"
          "  --frames N                 : capture N frames of data\n"
          "  --references               : capture references data\n"
          "  --self-cap-signals         : capture self cap signals\n"
          "  --self-cap-deltas          : capture self cap deltas\n"
          "  --self-cap-refs            : capture self cap references\n"
          "\n"
          "Device connection options:\n"
          "  -q [--query]               : scan for devices\n"
          "  -d [--device] DEVICESTRING : DEVICESTRING as output by --query\n\n"
          "  Examples:\n"
          "    -d i2c-dev:ADAPTER:ADDRESS : raw i2c device, eg \"i2c-dev:2-004a\"\n"
#ifdef HAVE_LIBUSB
          "    -d usb:BUS-DEVICE          : USB device, eg \"usb:001-003\"\n"
#endif
          "    -d sysfs:PATH              : sysfs interface\n"
          "    -d hidraw:PATH             : HIDRAW device, eg \"hidraw:/dev/hidraw0\"\n"
          "\n"
          "Debug options:\n"
          "  -v [--verbose] LEVEL       : set debug level\n",
          MXT_VERSION, prog_name);
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
  uint16_t address = 0;
  uint16_t count = 0;
  struct mxt_conn_info *conn = NULL;
  uint16_t object_type = 0;
  uint16_t msg_filter_type = 0;
  uint8_t instance = 0;
  uint8_t verbose = 2;
  uint16_t t37_frames = 1;
  uint8_t t37_mode = DELTAS_MODE;
  bool format = false;
  uint16_t port = 4000;
  uint8_t t68_datatype = 1;
  unsigned char databuf[BUF_SIZE];
  char strbuf2[BUF_SIZE];
  char strbuf[BUF_SIZE];
  strbuf[0] = '\0';
  strbuf2[0] = '\0';
  mxt_app_cmd cmd = CMD_NONE;

  while (1) {
    int option_index = 0;

    static struct option long_options[] = {
      {"backup",           optional_argument, 0, 0},
      {"bootloader-version", no_argument,     0, 0},
      {"bridge-client",    required_argument, 0, 'C'},
      {"calibrate",        no_argument,       0, 0},
      {"checksum",         required_argument, 0, 0},
      {"debug-dump",       required_argument, 0, 0},
      {"device",           required_argument, 0, 'd'},
      {"t68-file",         required_argument, 0, 0},
      {"t68-datatype",     required_argument, 0, 0},
      {"msg-filter",       required_argument, 0, 'F'},
      {"format",           no_argument,       0, 'f'},
      {"flash",            required_argument, 0, 0},
      {"firmware-version", required_argument, 0, 0},
      {"frames",           required_argument, 0, 0},
      {"help",             no_argument,       0, 'h'},
      {"info",             no_argument,       0, 'i'},
      {"instance",         required_argument, 0, 'I'},
      {"load",             required_argument, 0, 0},
      {"save",             required_argument, 0, 0},
      {"messages",         optional_argument, 0, 'M'},
      {"count",            required_argument, 0, 'n'},
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
      {"bridge-server",    no_argument,       0, 'S'},
      {"test",             optional_argument, 0, 't'},
      {"type",             required_argument, 0, 'T'},
      {"verbose",          required_argument, 0, 'v'},
      {"version",          no_argument,       0, 0},
      {"write",            no_argument,       0, 'W'},
      {"zero",             no_argument,       0, 0},
      {0,                  0,                 0,  0 }
    };

    c = getopt_long(argc, argv,
                    "C:d:D:fF:ghiI:M::m:n:p:qRr:St::T:v:W",
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
            ret = mxt_convert_hex(optarg, &databuf[0], &count, sizeof(databuf));
            if (ret || count == 0) {
              fprintf(stderr, "Hex convert error\n");
              ret = MXT_ERROR_BAD_INPUT;
            }
            backup_cmd = databuf[0];
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
      } else if (!strcmp(long_options[option_index].name, "reset")) {
        if (cmd == CMD_NONE) {
          cmd = CMD_RESET;
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
      } else if (!strcmp(long_options[option_index].name, "checksum")) {
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
      } else if (!strcmp(long_options[option_index].name, "references")) {
        t37_mode = REFS_MODE;
      } else if (!strcmp(long_options[option_index].name, "self-cap-signals")) {
        t37_mode = SELF_CAP_SIGNALS;
      } else if (!strcmp(long_options[option_index].name, "self-cap-refs")) {
        t37_mode = SELF_CAP_REFS;
      } else if (!strcmp(long_options[option_index].name, "self-cap-deltas")) {
        t37_mode = SELF_CAP_DELTAS;
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
        } else if (!strncmp(optarg, "sysfs:", 6)) {
          ret = mxt_new_conn(&conn, E_SYSFS);
          if (ret)
            return ret;

          conn->sysfs.path = (char *)calloc(strlen(optarg) + 1, sizeof(char));
          if (!conn->sysfs.path) {
            fprintf(stderr, "malloc failure\n");
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
          }

          memcpy(conn->sysfs.path, optarg + 6, strlen(optarg) - 6);
        }
#ifdef HAVE_LIBUSB
        else if (!strncmp(optarg, "usb:", 4)) {
          ret = mxt_new_conn(&conn, E_USB);
          if (ret)
            return ret;

          if (sscanf(optarg, "usb:%d-%d", &conn->usb.bus, &conn->usb.device) != 2) {
            fprintf(stderr, "Invalid device string %s\n", optarg);
            conn = mxt_unref_conn(conn);
            return MXT_ERROR_NO_MEM;
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
      format = true;
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
          ret = mxt_convert_hex(optarg, &databuf[0], &count, sizeof(databuf));
          if (ret) {
            fprintf(stderr, "Hex convert error\n");
            ret = MXT_ERROR_BAD_INPUT;
          } else {
            self_test_cmd = databuf[0];
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

  /* Debug does not work until mxt_set_verbose() is called */
  mxt_info(ctx, "Version:%s", MXT_VERSION);

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
                               databuf, sizeof(databuf), argc, argv);
    if (ret == MXT_ERROR_BAD_INPUT)
      goto free;
    break;

  case CMD_READ:
    mxt_verb(ctx, "Read command");
    ret = mxt_read_object(mxt, object_type, instance, address, count, format);
    break;

  case CMD_INFO:
    mxt_verb(ctx, "CMD_INFO");
    mxt_print_info_block(mxt);
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
    ret = run_self_tests(mxt, self_test_cmd);
    break;

  case CMD_FLASH:
    mxt_verb(ctx, "CMD_FLASH");
    ret = mxt_flash_firmware(ctx, mxt, strbuf, strbuf2, conn);
    break;

  case CMD_RESET:
    mxt_verb(ctx, "CMD_RESET");
    ret = mxt_reset_chip(mxt, false);
    break;

  case CMD_RESET_BOOTLOADER:
    mxt_verb(ctx, "CMD_RESET_BOOTLOADER");
    ret = mxt_reset_chip(mxt, true);
    break;

  case CMD_BOOTLOADER_VERSION:
    mxt_verb(ctx, "CMD_RESET_BOOTLOADER");
    ret = mxt_bootloader_version(ctx, mxt, conn);
    break;

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
    ret = mxt_debug_dump(mxt, t37_mode, strbuf, t37_frames);
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

      ret = mxt_backup_config(mxt, backup_cmd);
      if (ret) {
        mxt_err(ctx, "Error backing up");
      } else {
        mxt_info(ctx, "Configuration backed up");

        ret = mxt_reset_chip(mxt, false);
        if (ret) {
          mxt_err(ctx, "Error resetting");
        } else {
          mxt_info(ctx, "Chip reset");
        }
      }
    }
    break;

  case CMD_SAVE_CFG:
    mxt_verb(ctx, "CMD_SAVE_CFG");
    mxt_verb(ctx, "filename:%s", strbuf);
    ret = mxt_save_config_file(mxt, strbuf);
    break;

  case CMD_SELF_CAP_TUNE_CONFIG:
  case CMD_SELF_CAP_TUNE_NVRAM:
    mxt_verb(ctx, "CMD_SELF_CAP_TUNE");
    ret = mxt_self_cap_tune(mxt, cmd);
    break;

  case CMD_CRC_CHECK:
    mxt_verb(ctx, "CMD_CRC_CHECK");
    mxt_verb(ctx, "filename:%s", strbuf);
    ret = mxt_checkcrc(ctx, mxt, strbuf);
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
