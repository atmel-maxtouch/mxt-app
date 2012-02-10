//------------------------------------------------------------------------------
/// \file   config_loader.c
/// \brief  Configuration loader tool
/// \author Tim Culmer
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
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libmaxtouch/libmaxtouch.h"

#define VERSION "1.0"

static void display_usage(void);

/*!
 * @brief  Entry point for the config_loader utility.
 * @return Zero on success, negative for error.
 */
int main (int argc, char *argv[])
{
  bool override_checking = false;
  struct stat file_info;
  char *filename = NULL;

  printf("Config loader tool v. %s for Atmel maXTouch chips\n\n", VERSION);

  /* Parse input arguments */
  if (argc > 1)
  {
    filename = argv[1];

    if (argc > 2 && strcmp(argv[2], "-f") == 0)
    {
      override_checking = true;
    }
  }
  else
  {
    display_usage();
    return -1;
  }

  /* Check configuration file exists */
  if (stat(filename, &file_info) != 0)
  {
    printf("Specified file does not exist\n\n");
    display_usage();
    return -1;
  }

  /* Find an mXT device and read the info block */
  switch (mxt_scan())
  {
    case 1:
      /* Device found - continue */
      break;
    case 0:
      printf("Could not find a device, exiting...\n");
      return -1;
    default:
      printf("Error initializing, exiting...\n");
      return -1;
  }

  if (mxt_get_info() < 0)
  {
    printf("Error reading info\n");
    return -1;
  }

  /* Load and back-up the configuration file */
  if (mxt_load_config_file(filename, override_checking) < 0)
  {
    printf("Error loading the configuration, exiting...\n");
    mxt_release();
    return -1;
  }
  printf("Configuration loaded\n");

  if (mxt_backup_config() < 0)
  {
    printf("Error backing up, exiting...\n");
    mxt_release();
    return -1;
  }
  printf("Configuration backed up\n");

  /* Reset the chip to apply new configuration */
  if (mxt_reset_chip(false) < 0)
  {
    printf("Error reseting, exiting...\n");
    mxt_release();
    return -1;
  }
  printf("Chip reset\n");
  mxt_release();

  printf("\nConfiguration file loader ran successfully\n");

  return 0;
}

/*!
 * @brief  Display usage information for the config_loader utility.
 */
static void display_usage(void)
{
  printf
  (
    "Usage: ./config_loader filename [-f]\n"
    "filename  Filename of configuration to be uploaded to the mXT chip.\n"
    "\n"
    "Options:\n"
    "  -f  Force utility to use config file. Overrides checking.\n"
  );
}

