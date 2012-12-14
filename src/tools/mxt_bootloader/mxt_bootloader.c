//------------------------------------------------------------------------------
/// \file   mxt_bootloader.c
/// \brief  Bootloader tool for maxtouch chips
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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/i2c_dev/i2c_dev_device.h"
#include "libmaxtouch/sysfs/sysfs_device.h"

#define MXT_UNLOCK_CMD_MSB      0xaa
#define MXT_UNLOCK_CMD_LSB      0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD 0xc0 /* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA   0x80 /* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK      0x02
#define MXT_FRAME_CRC_FAIL       0x03
#define MXT_FRAME_CRC_PASS       0x04
#define MXT_APP_CRC_FAIL         0x40 /* valid 7 6 bit only */
#define MXT_BOOT_STATUS_MASK     0x3f

#define FIRMWARE_BUFFER_SIZE     1024

#define MXT_RESET_TIME           2
#define MXT_BOOTLOADER_DELAY     20000

bool have_bootloader_version = false;
bool extended_id_mode = false;

static void display_usage(void);

static int unlock_bootloader(void)
{
  unsigned char buf[2];

  buf[0] = MXT_UNLOCK_CMD_LSB;
  buf[1] = MXT_UNLOCK_CMD_MSB;

  usleep(MXT_BOOTLOADER_DELAY);
  return i2c_dev_bootloader_write(buf, sizeof(buf));
}

static int mxt_check_bootloader(unsigned int state)
{
    unsigned char buf[3];
    unsigned char val;
    unsigned char bootloader_id;
    unsigned char bootloader_version;

recheck:
    usleep(MXT_BOOTLOADER_DELAY);

    if ((!have_bootloader_version) && extended_id_mode && (state == MXT_WAITING_FRAME_DATA))
    {
       LOG(LOG_INFO, "Attempting to retrieve bootloader version");
       if (i2c_dev_bootloader_read(&buf[0], 3) != 0) {
           return -1;
       }

       val = buf[0];
       bootloader_id = buf[1];
       bootloader_version = buf[2];

       LOG(LOG_INFO, "Bootloader ID:%d Version:%d",
         bootloader_id, bootloader_version);

       have_bootloader_version = true;
    }
    else
    {
      if (i2c_dev_bootloader_read(&val, 1) != 0) {
        return -1;
      }
    }

    switch (state) {
    case MXT_WAITING_BOOTLOAD_CMD:
        bootloader_id = val & MXT_BOOT_STATUS_MASK;
        val &= ~MXT_BOOT_STATUS_MASK;

        if (val == MXT_APP_CRC_FAIL) {
            LOG(LOG_INFO, "CRC check for the currently stored application code failed");
            goto recheck;
        }

        break;
    case MXT_WAITING_FRAME_DATA:
        val &= ~MXT_BOOT_STATUS_MASK;
        break;
    case MXT_FRAME_CRC_PASS:
        if (val == MXT_FRAME_CRC_CHECK)
            goto recheck;
        break;
    default:
        return -2;
    }

    if (val != state) {
        LOG(LOG_ERROR, "Invalid bootloader mode state %X", val);
        printf("ERROR: Invalid bootloader mode\n");
        return -2;
    }

    if (!have_bootloader_version
        && state == MXT_WAITING_BOOTLOAD_CMD)
    {
      if (bootloader_id | 0x20)
      {
        LOG(LOG_INFO, "Bootloader using extended ID mode");
        extended_id_mode = true;
      }
      else
      {
        bootloader_id &= 0x1f;
        LOG(LOG_INFO, "Bootloader ID:%d", bootloader_id);
        have_bootloader_version = true;
      }
    }

    return 0;
}

static int get_hex_value(FILE *fp, unsigned char *ptr)
{
  char str[] = "00\0";
  int val;
  int ret;

  str[0] = fgetc(fp);
  str[1] = fgetc(fp);

  if (feof(fp)) return EOF;

  ret = sscanf(str, "%x", &val);

  *ptr =  val;

  return ret;
}

/*!
 * @brief  Entry point for the bootloader utility.
 * @return Zero on success, negative for error.
 */
int main (int argc, char *argv[])
{
  int ret;
  int frame_size = 0;
  int frame;
  int frame_retry = 0;
  unsigned char buffer[FIRMWARE_BUFFER_SIZE];
  int i;
  char *filename;
  FILE *fp;
  int adapter;
  int appmode_address = 0;
  int bootloader_address = 0;
  char *build;
  char curr_version[MXT_FW_VER_LEN];

  mxt_set_verbose(5);

  setbuf(stdout, NULL);

  printf("Bootloader tool for Atmel maXTouch chips version: %s\n\n",
         __GIT_VERSION);

  /* Parse input arguments */
  if (argc == 3)
  {
    filename = argv[1];
    LOG(LOG_INFO, "Filename %s", filename);

    build = argv[2];
    LOG(LOG_INFO, "Firmware build %s", build);
  }
  else
  {
    display_usage();
    return -1;
  }

  LOG(LOG_INFO, "Opening firmware file %s...", filename);

  fp = fopen(filename, "r");

  if (fp == NULL)
  {
    LOG(LOG_ERROR, "Error opening %s!", filename);
    printf("ERROR: cannot open firmware file\n");
    return -1;
  }

  /* Try to discover a maxtouch device under sysfs */
  ret = mxt_scan();

  if (ret == 1)
  {
    printf("Chip detected\n");
    LOG(LOG_INFO, "Switching to i2c-dev mode");

    adapter = sysfs_get_i2c_adapter();
    appmode_address = sysfs_get_i2c_address();
    i2c_dev_set_address(adapter, appmode_address);

    ret = mxt_get_info();
    if (ret) {
      printf("ERROR: could not read info block!\n");
    } else {
      mxt_get_firmware_version((char *)&curr_version);
      LOG(LOG_INFO, "Old firmware version: %s", curr_version);
      if (!strcmp((char *)&curr_version, build)) {
        printf("Firmware already correct version %s, exiting\n", build);
        return -1;
      }

      printf("Resetting chip\n");
      LOG(LOG_INFO, "Attempting to reset chip");

      /* Change to the bootloader mode */
      ret = mxt_reset_chip(true);

      if (ret < 0)
      {
        LOG(LOG_ERROR, "Reset failure - aborting");
        printf("ERROR: could not reset chip\n");
        return -1;
      }
      else
      {
        sleep(MXT_RESET_TIME);
      }
    }

    switch (appmode_address)
    {
      case 0x4a:
      case 0x4b:
        /* 1188S/1664S use different scheme */
        if (info_block.id->family_id == 0xa2)
        {
          bootloader_address = appmode_address - 0x24;
          break;
        }
        /* Fall through for normal case */
      case 0x4c:
      case 0x4d:
      case 0x5a:
      case 0x5b:
        bootloader_address = appmode_address - 0x26;
        break;
      default:
        printf("Bootloader address not found!\n");
        return -1;
    }

    /* Change to slave address of bootloader */
    i2c_dev_release();
    i2c_dev_set_address(adapter, bootloader_address);
  } else {
    LOG(LOG_INFO, "Could not find a device");
    return -1;
  }

  if (mxt_check_bootloader(MXT_WAITING_BOOTLOAD_CMD)< 0) {
    LOG(LOG_ERROR, "Bootloader not found");
    printf("ERROR: Bootloader not found!\n");
    return -1;
  }

  printf("Bootloader found\n");
  LOG(LOG_INFO, "Unlocking bootloader");

  if (unlock_bootloader() < 0) {
    LOG(LOG_ERROR, "Failure to unlock bootloader");
    printf("ERROR: Bootloader not unlocked!\n");
    return -1;
  }

  printf("Bootloader unlocked, flashing...\n");

  frame = 1;

  while (!feof(fp)) {
    if (frame_retry == 0) {
        if (get_hex_value(fp, &buffer[0]) == EOF) {
            LOG(LOG_INFO, "End of file");
            break;
        }

        if (get_hex_value(fp, &buffer[1]) == EOF) {
            LOG(LOG_ERROR, "Unexpected end of firmware file");
            printf("ERROR: Unexpected end of firmware file!\n");
            return -1;
        }

        frame_size = (buffer[0] << 8) | buffer[1];

        LOG(LOG_DEBUG, "Frame %d: size %d", frame, frame_size);

        /* We should add 2 at frame size as the the firmware data is not
         * included the CRC bytes.
         */
        frame_size += 2;

        if (frame_size > FIRMWARE_BUFFER_SIZE) {
                LOG(LOG_ERROR, "Frame too big");
                printf("ERROR: Frame too big!\n");
                return -1;
            }

        for (i = 2; i < frame_size; i++)
        {
            ret = get_hex_value(fp, &buffer[i]);

            if (ret == EOF)
            {
                LOG(LOG_ERROR, "Unexpected end of firmware file");
                printf("ERROR: Unexpected end of firmware file!\n");
                return -1;
            }
        }
    }

    if (mxt_check_bootloader(MXT_WAITING_FRAME_DATA) < 0) {
        LOG(LOG_ERROR, "Unexpected bootloader state");
        return -1;
    }

    /* Write one frame to device */
    usleep(MXT_BOOTLOADER_DELAY);
    i2c_dev_bootloader_write(buffer, frame_size);

    // Check CRC
    LOG(LOG_DEBUG, "Checking CRC");
    ret = mxt_check_bootloader(MXT_FRAME_CRC_PASS);
    if (ret) {
        if (frame_retry > 0) {
          LOG(LOG_ERROR, "Frame %d: CRC fail, aborting", frame);
          printf("ERROR: CRC fail, aborting\n");
          return -1;
        } else {
          frame_retry++;
          LOG(LOG_ERROR, "Frame %d: CRC fail, retry %d", frame, frame_retry);
          printf("  Frame %d retry\n", frame);
        }
    } else {
        LOG(LOG_INFO, "Frame %d: Uploaded %d bytes, CRC pass", frame, frame_size);
        frame++;
        if (frame % 20 == 0) {
          printf("  Frame %d\n", frame);
        }
    }
  }

  fclose(fp);

  printf("Done\n");

  sleep(MXT_RESET_TIME);

  i2c_dev_release();
  i2c_dev_set_address(adapter, appmode_address);
  ret = mxt_get_info();

  if (ret == 0)
  {
    mxt_get_firmware_version((char *)&curr_version);
    if (!strcmp(curr_version, build))
    {
       printf("SUCCESS - version verified\n");
       return 0;
    }
    else
    {
       printf("FAILURE - detected version is %s\n", curr_version);
       return -1;
    }
  }
  else
  {
       printf("FAILURE - chip did not reset\n");
       return -1;
  }
}

/*!
 * @brief  Display usage information for the config_loader utility.
 */
static void display_usage()
{
  printf
  (
    "Usage: ./bootloader filename build\n"
    "filename  Filename of firmware to be uploaded to the mXT chip.\n"
  );
}

