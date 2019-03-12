//------------------------------------------------------------------------------
/// \file   menu.c
/// \brief  Menu functions for mxt-app
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

#include <stdio.h>
#ifndef ANDROID
#include <stdio_ext.h>
#endif
#include <ctype.h>
#include <inttypes.h>
#include <errno.h>
#include <string.h>
#include <malloc.h>
#include <stdlib.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/msg.h"

#include "mxt_app.h"

//******************************************************************************
/// \brief Load config from file
static void load_config(struct mxt_device *mxt)
{
  char cfg_file[255];

  /* Load config file */
  printf("Give cfg file name: ");
  if (scanf("%255s", cfg_file) != 1) {
    printf("Input parse error\n");
    return;
  }

  printf("Trying to open %s...\n", cfg_file);

  if (mxt_load_config_file(mxt, cfg_file) == MXT_SUCCESS) {
    printf("Successfully uploaded the configuration file\n");
  } else {
    printf("Failed to upload the configuration\n");
  }
}

//******************************************************************************
/// \brief Save config to file
static void save_config(struct mxt_device *mxt)
{
  char cfg_file[255];

  /* Save config file */
  printf("Give cfg file name: ");
  if (scanf("%255s", cfg_file) != 1) {
    printf("Input parse error\n");
    return;
  }

  if (mxt_save_config_file(mxt, cfg_file) == MXT_SUCCESS) {
    printf("Successfully saved configuration to file\n");
  } else {
    printf("Failed to save configuration\n");
  }
}

//******************************************************************************
/// \brief Flash firmware to chip
static void flash_firmware_command(struct mxt_device *mxt)
{
  char fw_file[255];
  struct mxt_conn_info *conn = NULL;

  /* Save config file */
  printf("Give firmware .enc file name: ");
  if (scanf("%255s", fw_file) != 1) {
    printf("Input parse error\n");
    return;
  }

  mxt_flash_firmware(mxt->ctx, mxt, fw_file, "", conn);
}


//******************************************************************************
/// \brief Read objects according to the input value
static void read_object_command(struct mxt_device *mxt)
{
  uint16_t obj_num;
  uint8_t instance = 0;

  while(1) {
    printf("Enter the object number to read or 0 to finish\n");
    if (scanf("%" SCNu16, &obj_num) != 1) {
      printf("Input parse error\n");
      return;
    };

    if (obj_num == 0)
      return;

    if (MXT_INSTANCES(mxt->info.objects[obj_num]) > 1) {
      printf("Enter object instance\n");
      if (scanf("%" SCNu8, &instance) != 1) {
        printf("Input parse error\n");
        return;
      }
    }

    mxt_read_object(mxt, obj_num, instance, 0, 0, true);
  }
}

//******************************************************************************
/// \brief Menu function to write values to object
static void write_to_object(struct mxt_device *mxt, int obj_num, uint8_t instance)
{
  uint8_t obj_tbl_num, i;
  uint8_t *buffer;
  uint16_t start_position;
  int yn;
  uint8_t value;
  uint8_t size;

  obj_tbl_num = mxt_get_object_table_num(mxt, obj_num);
  if (obj_tbl_num == 255) {
    printf("Object not found\n");
    return;
  }

  buffer = (uint8_t *)calloc(MXT_SIZE(mxt->info.objects[obj_tbl_num]), sizeof(char));
  if (buffer == NULL) {
    mxt_err(mxt->ctx, "Memory error");
    return;
  }

  const char *obj_name = mxt_get_object_name(mxt->info.objects[obj_tbl_num].type);
  if (obj_name)
    printf("%s:\n", obj_name);
  else
    printf("UNKNOWN_T%d:\n", mxt->info.objects[obj_tbl_num].type);

  start_position = mxt_get_start_position(mxt->info.objects[obj_tbl_num], instance);
  size = mxt_get_object_size(mxt, obj_num);

  mxt_read_register(mxt, buffer, start_position, size);

  for(i = 0; i < size; i++) {
    printf("Object element %d =\t %d\n",i, *(buffer+i));
    printf("Do you want to change this value? (1 for yes/2 for no)");
    if (scanf("%d", &yn) != 1) {
      printf("Input error\n");
      return;
    }
    if (yn == 1) {
      printf("Enter the value to be written to object element %d\t :", i);
      if (scanf("%" SCNu8, &value) != 1) {
        printf("Input error\n");
        return;
      }
      *(buffer+i) = value;
      printf("Wrote %d\n", value);
    }
  }

  mxt_write_register(mxt, buffer, start_position, size);
}

//******************************************************************************
/// \brief Write objects
static void write_object_command(struct mxt_device *mxt)
{
  uint16_t obj_num;
  uint8_t instance = 0;

  while(1) {
    printf("Enter the object number to write or 0 to finish\n");
    if (scanf("%" SCNu16, &obj_num) != 1) {
      printf("Input parse error\n");
      return;
    }

    if (obj_num == 0)
      return;

    if (MXT_INSTANCES(mxt->info.objects[obj_num]) > 1) {
      printf("Enter object instance\n");
      if (scanf("%" SCNu8, &instance) != 1) {
        printf("Input parse error\n");
        return;
      }
    }

    write_to_object(mxt, obj_num, instance);
  }
}

//******************************************************************************
/// \brief Print Messages
static void print_messages_command(struct mxt_device *mxt)
{
  int msgs_timeout = MSG_CONTINUOUS;
  char tmp_buf[8];

  /* Flush stdin */
#ifndef ANDROID
  __fpurge(stdin);
#else
  fpurge(stdin);
#endif

  printf("Enter the messages timeout period in seconds. [Default: Run continually]\n");

  fgets(tmp_buf, sizeof(tmp_buf), stdin);
  if (sscanf(tmp_buf, "%d", &msgs_timeout) == EOF)
    printf("Please Press Ctrl-C to Return to Main Menu.\n");

  print_raw_messages(mxt, msgs_timeout, 0);

}

//******************************************************************************
/// \brief Handle command
static bool mxt_app_command(struct mxt_device *mxt, char selection)
{
  bool exit_loop = false;

  switch(selection) {
  case 'l':
    load_config(mxt);
    break;
  case 's':
    save_config(mxt);
  case 'i':
    /* Print info block */
    printf("Reading info block.....\n");
    mxt_print_info_block(mxt);
    /* Print config crc */
    printf("Reading config crc.....\n\n");
    mxt_print_config_crc(mxt);
    break;
  case 'd':
    read_object_command(mxt);
    break;
  case 'w':
    write_object_command(mxt);
    break;
  case 'f':
    flash_firmware_command(mxt);
    break;
  case 't':
    /* Run the self-test */
    self_test_menu(mxt);
    break;
  case 'b':
    /* Backup the config data */
    if (mxt_backup_config(mxt, BACKUPNV_COMMAND) == MXT_SUCCESS) {
      printf("Settings successfully backed up to non-volatile memory\n");
    } else {
      printf("Failed to back up settings\n");
    }
    break;
  case 'r':
    /* Reset the chip */
    if (mxt_reset_chip(mxt, false) == MXT_SUCCESS) {
      printf("Successfully forced a reset of the device\n");
    } else {
      printf("Failed to force a reset\n");
    }
    break;
  case 'c':
    /* Calibrate the device*/
    if (mxt_calibrate_chip(mxt) == MXT_SUCCESS) {
      printf("Successfully performed a global recalibration on all channels\n");
    } else {
      printf("Failed to perform a global recalibration\n");
    }
    break;
  case 'm':
    /* Display raw messages */
    print_messages_command(mxt);
    break;
  case 'u':
    mxt_dd_menu(mxt);
    break;
  case 'q':
    printf("Quit\n");
    exit_loop = true;
    break;
  default:
    printf("Invalid menu option\n");
    exit_loop = true;
    break;
  }

  return exit_loop;
}

//******************************************************************************
/// \brief Menu function for mxt-app
int mxt_menu(struct mxt_device *mxt)
{
  unsigned char menu_input;
  bool exit_loop = false;
  int ret;

  printf("Command line tool for Atmel maXTouch chips version: %s\n\n",
         MXT_VERSION);

  while(!exit_loop) {
    printf("Select one of the options:\n\n"
           "Enter L:   (L)oad config file\n"
           "Enter S:   (S)ave config file\n"
           "Enter I:   Read (I)nfo block and Config CRC\n"
           "Enter D:   Rea(D) individual object config\n"
           "Enter W:   (W)rite individual object\n"
           "Enter T:   Run sel(T)-test\n"
           "Enter F:   (F)lash firmware to chip\n"
           "Enter B:   (B)ackup the config data to NVM\n"
           "Enter R:   (R)eset the maxtouch device\n"
           "Enter C:   (C)alibrate the maxtouch device\n"
           "Enter M:   Display raw (M)essages\n"
           "Enter U:   D(U)mp Diagnostic data\n"
           "Enter Q:   (Q)uit the application\n");

    ret = scanf("%1s", &menu_input);
    if (ret == 1) {
      /* force lower case */
      menu_input = tolower(menu_input);

      exit_loop = mxt_app_command(mxt, menu_input);
    } else if (ret == EOF) {
      fprintf(stderr, "Error %s\n", strerror(errno));
      return MXT_ERROR_BAD_INPUT;
    }
  }

  return MXT_SUCCESS;
}
