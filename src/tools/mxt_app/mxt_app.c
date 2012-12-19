//------------------------------------------------------------------------------
/// \file   mxt_app.c
/// \brief  Command line tool for Atmel maXTouch chips.
/// \author Srivalli Ineni & Iiro Valkonen.
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
#include "libmaxtouch/i2c_dev/i2c_dev_device.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"

#include "touch_app.h"
#include "self_test.h"
#include "gr.h"
#include "bridge.h"
#include "serial_data.h"
#include "bootloader.h"

#define BUF_SIZE 1024

//******************************************************************************
/// \brief Commands for mxt-app
typedef enum mxt_app_cmd_tag {
  CMD_NONE,
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
  CMD_BACKUP
} mxt_app_cmd;

//******************************************************************************
/// \brief Load config from file
static void load_config(void)
{
   char cfg_file[255];

   /* Load config file */
   printf("Give cfg file name: ");
   if (scanf("%255s", cfg_file) != 1) {
     printf("Input parse error\n");
     return;
   }

   printf("Trying to open %s...\n", cfg_file);

   if (mxt_load_config_file(cfg_file) == 0)
   {
      printf("Successfully uploaded the configuration file\n");
   }
   else
   {
      printf("Failed to upload the configuration\n");
   }
}

//******************************************************************************
/// \brief Save config to file
static void save_config(void)
{
   char cfg_file[255];

   /* Save config file */
   printf("Give cfg file name: ");
   if (scanf("%255s", cfg_file) != 1) {
     printf("Input parse error\n");
     return;
   }

   if (mxt_save_config_file(cfg_file) == 0)
   {
      printf("Successfully saved configuration to file\n");
   }
   else
   {
      printf("Failed to save configuration\n");
   }
}

//******************************************************************************
/// \brief Read objects according to the input value
static void read_object_command(void)
{
   int obj_num;

   printf("Objects on the chip:\n");

   while(1)
   {
      print_objs();

      printf("Enter the object number to read the object's "
             "field values; Enter 255 to return to main menu\n");
      if (scanf("%d",&obj_num) != 1) {
        printf("Input parse error\n");
        return;
      };

      if ((obj_num >= 0) && (obj_num < 255))
      {
         read_object(obj_num, 0, 0, 0, true);
      }
      else if (obj_num == 255)
      {
         break;
      }
      else
      {
         printf("Please enter a valid object number\n");
         printf("Coming out of objects space...\n");
         break;
      }
   }
}

//******************************************************************************
/// \brief Write objects
static void write_object_command(void)
{
   int obj_num;

   printf("Objects on the chip:\n");
   while(1)
   {
     print_objs();
     printf("Enter the object number to modify the object's "
       "field values; or 255 to return to main menu\n");
     if (scanf("%d",&obj_num) != 1)
     {
       printf("Input parse error\n");
       return;
     }

     if((obj_num >= 0) && (obj_num < 255))
     {
       write_to_object(obj_num);
     }
     else if(obj_num == 255)
     {
       break;
     }
     else
     {
       printf("Please enter a valid object number\n");
       printf("Coming out of objects space...\n");
       break;
     }
   }
}

//******************************************************************************
/// \brief Handle command
static bool mxt_app_command(char selection)
{
   bool exit_loop = false;

   switch(selection)
   {
   case 'l':
      load_config();
      break;
   case 's':
      save_config();
   case 'i':
      /* Print info block */
      printf("Reading info block.....\n");
      print_info_block();
      break;
   case 'd':
      read_object_command();
      break;
    case 'w':
      write_object_command();
      break;
    case 'f':
      /* Run the self-test */
      self_test_handler();
      break;
    case 'b':
      /* Backup the config data */
      if (mxt_backup_config() == 0)
      {
        printf("Settings successfully backed up to non-volatile memory\n");
      }
      else
      {
        printf("Failed to back up settings\n");
      }
      break;
    case 'r':
      /* Reset the chip */
      if (mxt_reset_chip(false) == 0)
      {
        printf("Successfully forced a reset of the device\n");
      }
      else
      {
        printf("Failed to force a reset\n");
      }
      break;
    case 'a':
      /* Calibrate the device*/
      if (mxt_calibrate_chip() == 0)
      {
        printf("Successfully performed a global recalibration on all channels\n");
      }
      else
      {
        printf("Failed to perform a global recalibration\n");
      }
      break;
    case 'e':
      /* Read the events generated */
      event_printer();
      break;
    case 'm':
      /* Display raw messages */
      print_raw_messages();
      break;
    case 'q':
      printf("Quitting the maxtouch application\n");
      exit_loop = 1;
      break;
    default:
      printf("Invalid menu option\n");
      break;
  }

  return exit_loop;
}

//******************************************************************************
/// \brief Menu function for mxt-app
static int mxt_menu(void)
{
   char menu_input;
   bool exit_loop = false;

   printf("Command line tool for Atmel maXTouch chips version: %s\n\n",
          __GIT_VERSION);

   while(!exit_loop)
   {
     printf("Select one of the options:\n\n"
       "Enter L:   (L)oad config file\n"
       "Enter S:   (S)ave config file\n"
       "Enter I:   Read (I)nfo block\n"
       "Enter D:   Rea(D) individual object config\n"
       "Enter W:   (W)rite individual object\n"
       "Enter F:   Run sel(F)-test\n"
       "Enter B:   (B)ackup the config data to NVM\n"
       "Enter R:   (R)eset the maxtouch device\n"
       "Enter A:   C(A)librate the maxtouch device\n"
       "Enter E:   Display the input (E)vents from the device\n"
       "Enter M:   Display raw (M)essages\n"
       "Enter Q:   (Q)uit the application\n");

     if (scanf("%1s", &menu_input) == 1)
       exit_loop = mxt_app_command(menu_input);
   }

   return 0;
}

//******************************************************************************
/// \brief Initialize mXT device and read the info block
static int mxt_init_chip(int adapter, int address)
{
  int ret;

  if (adapter >= 0 && address > 0)
  {
    ret = i2c_dev_set_address(adapter, address);
    if (ret < 0)
    {
      printf("Failed to init device - exiting the application\n");
      return -1;
    }
  }
  else
  {
    ret = mxt_scan();
    if (ret == 0)
    {
      printf("Unable to find any maXTouch devices - exiting the application\n");
      return -1;
    }
    else if (ret < 0)
    {
      printf("Failed to init device - exiting the application\n");
      return -1;
    }
  }

  if (mxt_get_info() < 0)
  {
    printf("Error reading info block, exiting...\n");
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
                  "  -R [--read]                : read from object\n"
                  "  -t [--test]                : run all self tests\n"
                  "  -W [--write]               : write to object\n"
                  "  --flash                    : send firmware to bootloader\n"
                  "  --reset                    : reset device\n"
                  "  --reset-bootloader         : reset device in bootloader mode\n"
                  "  --backup                   : backup configuration to NVRAM\n"
                  "  -g                         : store golden references\n"
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
                  "For i2c-dev and bootloader mode:\n"
                  "  -d [--i2c-adapter] ADAPTER : i2c adapter, eg \"2\"\n"
                  "  -a [--i2c-address] ADDRESS : i2c address, eg \"4a\"\n"
                  "\n"
                  "For T68 serial data:\n"
                  "  --t68-file FILE            : Upload FILE\n"
                  "  --t68-datatype DATATYPE    : Select DATATYPE\n"
                  "\n"
                  "Examples:\n"
                  "  %s -R -n7 -r0              : Read info block\n"
                  "  %s -R -T9 --format         : Read T9 object, formatted output\n"
                  "  %s -W -T38 000000          : Zero first three bytes of T38\n"
                  "  %s --test                  : run self tests\n",
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
  uint8_t count = 0;
  int i2c_address = -1;
  int i2c_adapter = -1;
  uint16_t object_type = 0;
  uint8_t instance = 0;
  uint8_t verbose = 0;
  bool format = false;
  uint16_t port = 4000;
  uint8_t t68_datatype = 1;
  unsigned char databuf[BUF_SIZE];
  char strbuf2[BUF_SIZE];
  char strbuf[BUF_SIZE];
  strbuf[0] = '\0';
  strbuf2[0] = '\0';
  mxt_app_cmd cmd = CMD_NONE;

  LOG(LOG_DEBUG, "Decoding cmd arguments");

  while (1)
  {
    int option_index = 0;

    static struct option long_options[] = {
      {"i2c-address",  required_argument, 0, 'a'},
      {"backup",        no_argument,      0, 0},
      {"bridge-client",required_argument, 0, 'C'},
      {"i2c-adapter",  required_argument, 0, 'd'},
      {"t68-file",     required_argument, 0, 0},
      {"t68-datatype", required_argument, 0, 0},
      {"format",       no_argument,       0, 'f'},
      {"flash",        required_argument, 0, 0},
      {"firmware-version", required_argument, 0, 0},
      {"help",         no_argument,       0, 'h'},
      {"instance",     required_argument, 0, 'I'},
      {"count",        required_argument, 0, 'n'},
      {"port",         required_argument, 0, 'p'},
      {"read",         no_argument,       0, 'R'},
      {"reset",        no_argument,       0, 0},
      {"reset-bootloader", no_argument,       0, 0},
      {"register",     required_argument, 0, 'r'},
      {"bridge-server",no_argument,       0, 'S'},
      {"test",         no_argument,       0, 't'},
      {"type",         required_argument, 0, 'T'},
      {"verbose",      required_argument, 0, 'v'},
      {"write",        no_argument,       0, 'W'},
      {0,              0,                 0,  0 }
    };

    c = getopt_long(argc, argv, "a:C:d:D:fghI:n:p:Rr:StT:v:W", long_options, &option_index);

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
        else if (!strcmp(long_options[option_index].name, "reset"))
        {
          if (cmd == CMD_NONE) {
            cmd = CMD_RESET;
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
        else
        {
          printf("Unknown option %s\n", long_options[option_index].name);
        }
        break;

      case 'a':
        if (optarg) {
          i2c_address = strtol(optarg, NULL, 16);
        }
        break;

      case 'd':
        if (optarg) {
          i2c_adapter = strtol(optarg, NULL, 0);
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
        printf("\n\n");
        print_usage(argv[0]);
        return -1;
    }
  }

  LOG(LOG_DEBUG, "Version:%s", __GIT_VERSION);
  LOG(LOG_DEBUG, "cmd:%u", cmd);
  LOG(LOG_DEBUG, "i2c_address:%u", i2c_address);
  LOG(LOG_DEBUG, "i2c_adapter:%u", i2c_adapter);
  LOG(LOG_DEBUG, "format:%s", format ? "true" : "false");
  LOG(LOG_DEBUG, "instance:%u", instance);
  LOG(LOG_DEBUG, "count:%u", count);
  LOG(LOG_DEBUG, "address:%u", address);
  LOG(LOG_DEBUG, "object_type:%u", object_type);
  LOG(LOG_DEBUG, "verbose:%u", verbose);
  LOG(LOG_DEBUG, "port:%u", port);
  LOG(LOG_DEBUG, "t68_datatype:%u", t68_datatype);

  /* initialise chip, bootloader mode handles this itself */
  if (cmd != CMD_FLASH)
  {
    ret = mxt_init_chip(i2c_adapter, i2c_address);
    if (ret < 0)
      return ret;

    /*! Turn on kernel dmesg output of MSG */
    mxt_set_debug(true);
  }

  switch (cmd) {
    case CMD_WRITE:
      LOG(LOG_DEBUG, "Write command");

      if (object_type > 0) {
        object_address = get_object_address(object_type, instance);
        if (object_address == OBJECT_NOT_FOUND) {
          printf("No such object\n");
          ret = -1;
          break;
        }

        LOG(LOG_DEBUG, "T%u address:%u offset:%u", object_type,
            object_address, address);
        address = object_address + address;

        if (count == 0) {
          count = get_object_size(object_type);
        }
      } else if (count == 0) {
        printf("Not enough arguments!\n");
        return -1;
      }

      if (optind != (argc - 1)) {
        printf("Must give hex input\n");
        return -1;
      }

      ret = mxt_convert_hex(argv[optind], &databuf[0], &count, sizeof(databuf));
      if (ret < 0) {
        printf("Hex convert error\n");
      } else {
        ret = mxt_write_register(&databuf[0], address, count);
        if (ret < 0) {
          printf("Write error\n");
        }
      }
      break;

    case CMD_READ:
      LOG(LOG_DEBUG, "Read command");
      ret = read_object(object_type, instance, address, count, format);
      break;

    case CMD_GOLDEN_REFERENCES:
      ret = mxt_store_golden_refs();
      break;

    case CMD_BRIDGE_SERVER:
      ret = mxt_socket_server(port);
      break;

    case CMD_BRIDGE_CLIENT:
      ret = mxt_socket_client(strbuf, port);
      break;

    case CMD_SERIAL_DATA:
      ret = mxt_serial_data_upload(strbuf, t68_datatype);
      break;

    case CMD_TEST:
      LOG(LOG_DEBUG, "Running all tests");
      ret = run_self_tests(SELF_TEST_ALL);
      break;

    case CMD_FLASH:
      ret = mxt_flash_firmware(strbuf, strbuf2, i2c_adapter, i2c_address);
      break;

    case CMD_RESET:
      ret = mxt_reset_chip(false);
      break;

    case CMD_RESET_BOOTLOADER:
      ret = mxt_reset_chip(true);
      break;

    case CMD_BACKUP:
      ret = mxt_backup_config();
      break;

    case CMD_NONE:
    default:
      ret = mxt_menu();
      break;
  }

  mxt_set_debug(false);
  mxt_release();

  return ret;
}
