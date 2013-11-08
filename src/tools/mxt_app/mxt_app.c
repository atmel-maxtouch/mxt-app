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

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"

#include "mxt_app.h"

#define BUF_SIZE 1024

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
} mxt_app_cmd;

//******************************************************************************
/// \brief Initialize mXT device and read the info block
static int mxt_init_chip(struct libmaxtouch_ctx *ctx, struct mxt_device **mxt,
                         struct mxt_conn_info conn)
{
  int ret;

  if (conn.type == E_NONE)
  {
    ret = mxt_scan(ctx, &conn, false);
    if (ret == 0)
    {
      LOG(LOG_ERROR, "Unable to find a device");
      return -1;
    }
    else if (ret < 0)
    {
      LOG(LOG_ERROR, "Failed to find device");
      return -1;
    }
  } else {
    LOG(LOG_DEBUG, "Connection parameters given");
  }

  if (mxt_open(ctx, conn, mxt) < 0)
  {
    LOG(LOG_ERROR, "Failed to open device");
    return -1;
  }

  if (mxt_get_info(*mxt) < 0)
  {
    LOG(LOG_ERROR, "Failed to read information block");
    return -1;
  }

  return 0;
}

//******************************************************************************
/// \brief Print usage for mxt-app
static void print_usage(char *prog_name)
{
  fprintf(stderr, "Command line tool for Atmel maXTouch chips version: %s\n\n"
                  "Usage: %s [command] [args]\n\n"
                  "When run with no options, access menu interface.\n\n"
                  "Available commands:\n"
                  "  -h [--help]                : display this help and exit\n"
                  "  -q [--query]               : scan for devices\n"
                  "  -i [--info]                : print device information\n"
                  "  -R [--read]                : read from object\n"
                  "  -t [--test]                : run all self tests\n"
                  "  -W [--write]               : write to object\n"
                  "  --flash FIRMWARE           : send FIRMWARE to bootloader\n"
                  "  --reset                    : reset device\n"
                  "  --reset-bootloader         : reset device in bootloader mode\n"
                  "  --backup                   : backup configuration to NVRAM\n"
                  "  --calibrate                : send calibrate command\n"
                  "  --debug-dump FILE          : capture diagnostic data to FILE\n"
                  "  --load FILE                : upload config from FILE\n"
                  "  --save FILE                : save config to FILE\n"
                  "  -g                         : store golden references\n"
                  "  --version                  : print version\n"
                  "\n"
                  "Valid options:\n"
                  "  -n [--count] COUNT         : read/write COUNT registers\n"
                  "  -f [--format]              : format register output\n"
                  "  -I [--instance] INSTANCE   : select object INSTANCE\n"
                  "  -r [--register] REGISTER   : start at REGISTER\n"
                  "  -T [--type] TYPE           : select object TYPE\n"
                  "  -v [--verbose] LEVEL       : print additional debug\n"
                  "\n"
                  "For TCP socket:\n"
                  "  -C [--bridge-client] HOST  : connect over TCP to HOST\n"
                  "  -S [--bridge-server]       : start TCP socket server\n"
                  "  -p [--port] PORT           : TCP port (default 4000)\n"
                  "\n"
                  "For bootloader mode:\n"
                  "  --firmware-version VERSION : Check firmware VERSION "
                                                 "before and after flash\n"
                  "\n"
                  "For T37 diagnostic data:\n"
                  "  --frames N                 : Capture N frames of data\n"
                  "  --references               : Dump references data\n"
                  "\n"
                  "To specify device\n"
                  "  -d [--device] DEVICESTRING : DEVICESTRING as output by --query\n\n"
                  "   Examples:\n"
                  "  -d i2c-dev:ADAPTER:ADDRESS : raw i2c device, eg \"i2c-dev:2-004a\"\n"
                  "  -d usb:BUS-DEVICE          : USB device, eg \"usb:001-003\"\n"
                  "  -d sysfs:PATH              : sysfs interface\n"
                  "\n"
                  "For T68 serial data:\n"
                  "  --t68-file FILE            : Upload FILE\n"
                  "  --t68-datatype DATATYPE    : Select DATATYPE\n"
                  "\n"
                  "Examples:\n"
                  "  %s -R -n7 -r0      : Read info block\n"
                  "  %s -R -T9 --format : Read T9 object, formatted output\n"
                  "  %s -W -T7 0000     : Zero first two bytes of T7\n"
                  "  %s --test          : run self tests\n",
                  __GIT_VERSION,
                  prog_name, prog_name, prog_name, prog_name, prog_name);
}

//******************************************************************************
/// \brief Main function for mxt-app
int main (int argc, char *argv[])
{
  int ret;
  int c;
  uint16_t address = 0;
  uint16_t object_address = 0;
  uint16_t count = 0;
  struct mxt_conn_info conn = { .type = E_NONE };
  uint16_t object_type = 0;
  uint8_t instance = 0;
  uint8_t verbose = 0;
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

  while (1)
  {
    int option_index = 0;

    static struct option long_options[] = {
      {"backup",           no_argument,       0, 0},
      {"bridge-client",    required_argument, 0, 'C'},
      {"calibrate",        no_argument,       0, 0},
      {"debug-dump",       required_argument, 0, 0},
      {"device",           required_argument, 0, 'd'},
      {"t68-file",         required_argument, 0, 0},
      {"t68-datatype",     required_argument, 0, 0},
      {"format",           no_argument,       0, 'f'},
      {"flash",            required_argument, 0, 0},
      {"firmware-version", required_argument, 0, 0},
      {"frames",           required_argument, 0, 0},
      {"help",             no_argument,       0, 'h'},
      {"info",             no_argument,       0, 'i'},
      {"instance",         required_argument, 0, 'I'},
      {"load",             required_argument, 0, 0},
      {"save",             required_argument, 0, 0},
      {"count",            required_argument, 0, 'n'},
      {"port",             required_argument, 0, 'p'},
      {"query",            no_argument,       0, 'q'},
      {"read",             no_argument,       0, 'R'},
      {"reset",            no_argument,       0, 0},
      {"reset-bootloader", no_argument,       0, 0},
      {"register",         required_argument, 0, 'r'},
      {"references",       no_argument,       0, 0},
      {"bridge-server",    no_argument,       0, 'S'},
      {"test",             no_argument,       0, 't'},
      {"type",             required_argument, 0, 'T'},
      {"verbose",          required_argument, 0, 'v'},
      {"version",          no_argument,       0, 0},
      {"write",            no_argument,       0, 'W'},
      {0,                  0,                 0,  0 }
    };

    c = getopt_long(argc, argv, "C:d:D:fghiI:n:p:qRr:StT:v:W", long_options, &option_index);

    if (c == -1)
      break;

    switch (c)
    {
      case 0:
        if (!strcmp(long_options[option_index].name, "t68-file"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_SERIAL_DATA;
            strncpy(strbuf, optarg, sizeof(strbuf));
            strbuf[sizeof(strbuf) - 1] = '\0';
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "t68-datatype"))
        {
          t68_datatype = strtol(optarg, NULL, 0);
        }
        else if (!strcmp(long_options[option_index].name, "flash"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_FLASH;
            strncpy(strbuf, optarg, sizeof(strbuf));
            strbuf[sizeof(strbuf) - 1] = '\0';
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "backup"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_BACKUP;
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "calibrate"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_CALIBRATE;
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "debug-dump"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_DEBUG_DUMP;
            strncpy(strbuf, optarg, sizeof(strbuf));
            strbuf[sizeof(strbuf) - 1] = '\0';
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "reset"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_RESET;
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "load"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_LOAD_CFG;
            strncpy(strbuf, optarg, sizeof(strbuf));
            strbuf[sizeof(strbuf) - 1] = '\0';
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "save"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_SAVE_CFG;
            strncpy(strbuf, optarg, sizeof(strbuf));
            strbuf[sizeof(strbuf) - 1] = '\0';
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "reset-bootloader"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_RESET_BOOTLOADER;
          } else {
            print_usage(argv[0]);
            return -1;
          }
        }
        else if (!strcmp(long_options[option_index].name, "firmware-version"))
        {
          strncpy(strbuf2, optarg, sizeof(strbuf2));
        }
        else if (!strcmp(long_options[option_index].name, "frames"))
        {
          t37_frames = strtol(optarg, NULL, 0);
        }
        else if (!strcmp(long_options[option_index].name, "references"))
        {
          t37_mode = REFS_MODE;
        }
        else if (!strcmp(long_options[option_index].name, "version"))
        {
          LOG(LOG_INFO, "mxt-app %s", __GIT_VERSION);
          return 0;
        }
        else
        {
          fprintf(stderr, "Unknown option %s\n",
                  long_options[option_index].name);
        }
        break;

      case 'd':
        if (optarg) {
          if (!strncmp(optarg, "i2c-dev:", 8))
          {
            if (sscanf(optarg, "i2c-dev:%d-%x",
                  &conn.i2c_dev.adapter, &conn.i2c_dev.address) != 2)
            {
              LOG(LOG_ERROR, "Invalid device string %s", optarg);
              return -1;
            }

            conn.type = E_I2C_DEV;
            LOG(LOG_INFO, "i2c-dev device: %d-%04x",
                conn.i2c_dev.adapter, conn.i2c_dev.address);
          }
#ifdef HAVE_LIBUSB
          else if (!strncmp(optarg, "usb:", 4))
          {
            if (sscanf(optarg, "usb:%d-%d", &conn.usb.bus, &conn.usb.device) != 2)
            {
              LOG(LOG_ERROR, "Invalid device string %s", optarg);
              return -1;
            }

            conn.type = E_USB;
            LOG(LOG_INFO, "USB device:%03d-%03d", conn.usb.bus, conn.usb.device);
          }
#endif
        }
        break;

      case 'C':
        if (cmd == CMD_NONE) {
          cmd = CMD_BRIDGE_CLIENT;
          strncpy(strbuf, optarg, sizeof(strbuf));
          strbuf[sizeof(strbuf) - 1] = '\0';
        } else {
          print_usage(argv[0]);
          return -1;
        }
        break;

      case 'g':
        if (cmd == CMD_NONE) {
          cmd = CMD_GOLDEN_REFERENCES;
        } else {
          print_usage(argv[0]);
          return -1;
        }
        break;

      case 'h':
        print_usage(argv[0]);
        return 0;

      case 'f':
        format = true;
        break;

      case 'I':
        if (optarg) {
          instance = strtol(optarg, NULL, 0);
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
          return -1;
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
          return -1;
        }
        break;

      case 'S':
        if (cmd == CMD_NONE) {
          cmd = CMD_BRIDGE_SERVER;
        } else {
          print_usage(argv[0]);
          return -1;
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
          return -1;
        }
        break;

      case 't':
        if (cmd == CMD_NONE) {
          cmd = CMD_TEST;
        } else {
          print_usage(argv[0]);
          return -1;
        }
        break;

      case 'v':
        if (optarg) {
          verbose = strtol(optarg, NULL, 0);
          mxt_set_verbose(verbose);
          LOG(LOG_VERBOSE, "verbose:%u", verbose);
        }
        break;

      case 'W':
        if (cmd == CMD_NONE) {
          cmd = CMD_WRITE;
        } else {
          print_usage(argv[0]);
          return -1;
        }
        break;

      default:
        /* Output newline to create space under getopt error output */
        fprintf(stderr, "\n\n");
        print_usage(argv[0]);
        return -1;
    }
  }

  struct mxt_device *mxt;
  struct libmaxtouch_ctx *ctx;

  ret = mxt_init(&ctx);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to init libmaxtouch");
    return ret;
  }

  /* Debug does not work until mxt_set_verbose() is called */
  LOG(LOG_DEBUG, "Version:%s", __GIT_VERSION);

  if (cmd == CMD_WRITE || cmd == CMD_READ)
  {
    LOG(LOG_VERBOSE, "instance:%u", instance);
    LOG(LOG_VERBOSE, "count:%u", count);
    LOG(LOG_VERBOSE, "address:%u", address);
    LOG(LOG_VERBOSE, "object_type:%u", object_type);
    LOG(LOG_VERBOSE, "format:%s", format ? "true" : "false");
  }

  /* initialise chip, bootloader mode handles this itself */
  if (cmd == CMD_QUERY)
  {
    return mxt_scan(ctx, &conn, true);
  }
  else if (cmd != CMD_FLASH)
  {
    ret = mxt_init_chip(ctx, &mxt, conn);
    if (ret < 0)
      return ret;

    /*! Turn on kernel dmesg output of MSG */
    mxt_set_debug(mxt, true);
  }

  switch (cmd) {
    case CMD_WRITE:
      LOG(LOG_VERBOSE, "Write command");

      if (object_type > 0) {
        object_address = get_object_address(mxt, object_type, instance);
        if (object_address == OBJECT_NOT_FOUND) {
          fprintf(stderr, "No such object\n");
          ret = -1;
          break;
        }

        LOG(LOG_VERBOSE, "T%u address:%u offset:%u", object_type,
            object_address, address);
        address = object_address + address;

        if (count == 0) {
          count = get_object_size(mxt, object_type);
        }
      } else if (count == 0) {
        fprintf(stderr, "Not enough arguments!\n");
        return -1;
      }

      if (optind != (argc - 1)) {
        fprintf(stderr, "Must give hex input\n");
        return -1;
      }

      ret = mxt_convert_hex(argv[optind], &databuf[0], &count, sizeof(databuf));
      if (ret < 0) {
        fprintf(stderr, "Hex convert error\n");
      } else {
        ret = mxt_write_register(mxt, &databuf[0], address, count);
        if (ret < 0) {
          fprintf(stderr, "Write error\n");
        }
      }
      break;

    case CMD_READ:
      LOG(LOG_VERBOSE, "Read command");
      ret = read_object(mxt, object_type, instance, address, count, format);
      break;

    case CMD_INFO:
      LOG(LOG_VERBOSE, "CMD_INFO");
      print_info_block(mxt);
      break;

    case CMD_GOLDEN_REFERENCES:
      LOG(LOG_VERBOSE, "CMD_GOLDEN_REFERENCES");
      ret = mxt_store_golden_refs(mxt);
      break;

    case CMD_BRIDGE_SERVER:
      LOG(LOG_VERBOSE, "CMD_BRIDGE_SERVER");
      LOG(LOG_VERBOSE, "port:%u", port);
      ret = mxt_socket_server(mxt, port);
      break;

    case CMD_BRIDGE_CLIENT:
      LOG(LOG_VERBOSE, "CMD_BRIDGE_CLIENT");
      ret = mxt_socket_client(mxt, strbuf, port);
      break;

    case CMD_SERIAL_DATA:
      LOG(LOG_VERBOSE, "CMD_SERIAL_DATA");
      LOG(LOG_VERBOSE, "t68_datatype:%u", t68_datatype);
      ret = mxt_serial_data_upload(mxt, strbuf, t68_datatype);
      break;

    case CMD_TEST:
      LOG(LOG_VERBOSE, "CMD_TEST");
      ret = run_self_tests(mxt, SELF_TEST_ALL);
      break;

    case CMD_FLASH:
      LOG(LOG_VERBOSE, "CMD_FLASH");
      ret = mxt_flash_firmware(ctx, mxt, strbuf, strbuf2, conn);
      break;

    case CMD_RESET:
      LOG(LOG_VERBOSE, "CMD_RESET");
      ret = mxt_reset_chip(mxt, false);
      break;

    case CMD_RESET_BOOTLOADER:
      LOG(LOG_VERBOSE, "CMD_RESET_BOOTLOADER");
      ret = mxt_reset_chip(mxt, true);
      break;

    case CMD_BACKUP:
      LOG(LOG_VERBOSE, "CMD_BACKUP");
      ret = mxt_backup_config(mxt);
      break;

    case CMD_CALIBRATE:
      LOG(LOG_VERBOSE, "CMD_CALIBRATE");
      ret = mxt_calibrate_chip(mxt);
      break;

    case CMD_DEBUG_DUMP:
      LOG(LOG_VERBOSE, "CMD_DEBUG_DUMP");
      LOG(LOG_VERBOSE, "mode:%u", t37_mode);
      LOG(LOG_VERBOSE, "frames:%u", t37_frames);
      ret = mxt_debug_dump(mxt, t37_mode, strbuf, t37_frames);
      break;

    case CMD_LOAD_CFG:
      LOG(LOG_VERBOSE, "CMD_LOAD_CFG");
      LOG(LOG_VERBOSE, "filename:%s", strbuf);
      ret = mxt_load_config_file(mxt, strbuf);
      if (ret < 0)
      {
        LOG(LOG_ERROR, "Error loading the configuration");
      }
      else
      {
        LOG(LOG_INFO, "Configuration loaded");

        ret = mxt_backup_config(mxt);
        if (ret < 0)
        {
          LOG(LOG_ERROR, "Error backing up");
        }
        else
        {
          LOG(LOG_INFO, "Configuration backed up");

          ret = mxt_reset_chip(mxt, false);
          if (ret < 0)
          {
            LOG(LOG_ERROR, "Error resetting");
          }
          else
          {
            LOG(LOG_INFO, "Chip reset");
          }
        }
      }
      break;

    case CMD_SAVE_CFG:
      LOG(LOG_VERBOSE, "CMD_SAVE_CFG");
      LOG(LOG_VERBOSE, "filename:%s", strbuf);
      ret = mxt_save_config_file(mxt, strbuf);
      break;

    case CMD_NONE:
    default:
      LOG(LOG_VERBOSE, "cmd: %d", cmd);
      ret = mxt_menu(mxt);
      break;
  }

  if (cmd != CMD_FLASH)
  {
    mxt_set_debug(mxt, false);
    mxt_release(mxt);
  }

  mxt_close(ctx);

  return ret;
}
