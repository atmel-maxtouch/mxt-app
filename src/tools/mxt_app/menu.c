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
#include <ctype.h>
#include <inttypes.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"

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

  if (mxt_load_config_file(mxt, cfg_file) == 0)
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
static void save_config(struct mxt_device *mxt)
{
  char cfg_file[255];

  /* Save config file */
  printf("Give cfg file name: ");
  if (scanf("%255s", cfg_file) != 1) {
    printf("Input parse error\n");
    return;
  }

  if (mxt_save_config_file(mxt, cfg_file) == 0)
  {
    printf("Successfully saved configuration to file\n");
  }
  else
  {
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

  while(1)
  {
    printf("Enter the object number to read or 0 to finish\n");
    if (scanf("%" SCNu16, &obj_num) != 1) {
      printf("Input parse error\n");
      return;
    };

    if (obj_num == 0)
      return;

    if (mxt->info_block.objects[obj_num].instances > 0) {
      printf("Enter object instance\n");
      if (scanf("%" SCNu8, &instance) != 1) {
        printf("Input parse error\n");
        return;
      }
    }

    read_object(mxt, obj_num, instance, 0, 0, true);
  }
}

//******************************************************************************
/// \brief Write objects
static void write_object_command(struct mxt_device *mxt)
{
  uint16_t obj_num;
  uint8_t instance = 0;

  while(1)
  {
    printf("Enter the object number to write or 0 to finish\n");
    if (scanf("%" SCNu16, &obj_num) != 1)
    {
      printf("Input parse error\n");
      return;
    }

    if (obj_num == 0)
      return;

    if (mxt->info_block.objects[obj_num].instances > 0) {
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
/// \brief Handle command
static bool mxt_app_command(struct mxt_device *mxt, char selection)
{
  bool exit_loop = false;

  switch(selection)
  {
    case 'l':
      load_config(mxt);
      break;
    case 's':
      save_config(mxt);
    case 'i':
      /* Print info block */
      printf("Reading info block.....\n");
      print_info_block(mxt);
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
      if (mxt_backup_config(mxt) == 0)
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
      if (mxt_reset_chip(mxt, false) == 0)
      {
        printf("Successfully forced a reset of the device\n");
      }
      else
      {
        printf("Failed to force a reset\n");
      }
      break;
    case 'c':
      /* Calibrate the device*/
      if (mxt_calibrate_chip(mxt) == 0)
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
      event_printer(mxt);
      break;
    case 'm':
      /* Display raw messages */
      print_raw_messages(mxt);
      break;
    case 'u':
      mxt_dd_menu(mxt);
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
int mxt_menu(struct mxt_device *mxt)
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
        "Enter T:   Run sel(T)-test\n"
        "Enter F:   (F)lash firmware to chip\n"
        "Enter B:   (B)ackup the config data to NVM\n"
        "Enter R:   (R)eset the maxtouch device\n"
        "Enter C:   (C)alibrate the maxtouch device\n"
        "Enter E:   Display the input (E)vents from the device\n"
        "Enter M:   Display raw (M)essages\n"
        "Enter U:   D(U)mp Diagnostic data\n"
        "Enter Q:   (Q)uit the application\n");

    if (scanf("%1s", &menu_input) == 1)
    {
      /* force lower case */
      menu_input = tolower(menu_input);

      exit_loop = mxt_app_command(mxt, menu_input);
    }
  }

  return 0;
}
