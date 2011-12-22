//------------------------------------------------------------------------------
/// \file   touch_app.c
/// \brief  Linux application demo for Atmel chips using the Object Based Project protocol
/// \author Iiro Valkonen
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

/*----------------------------------------------------------------------------
  include files
  ----------------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

#include "touch_app.h"

#include "libmaxtouch/dmesg.h"
#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"

#include <linux/input.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>

/*----------------------------------------------------------------------------
  Function definitions
  ----------------------------------------------------------------------------*/

void event_printer()
{
  FILE *fp;
  struct input_event event;
  char *filename = mxt_get_input_event_file();

  if (filename == NULL)
  {
    printf("Unable to get input event file - aborting event printer\n");
    return;
  }

  printf("Opening %s...\n", filename);

  fp = fopen(filename, "r");

  if (fp == NULL)
  {
    fprintf(stderr, "Error opening %s\n", filename);
    if (errno == EACCES)
    {
      fprintf(stderr, "Permission denied - try running the utility as root\n");
    }
    exit(-1);
  }

  while(1)
  {
    fread(&event, 1, sizeof(struct input_event), fp);
    switch(event.type)
    {
      case EV_SYN:
        printf("Synchronisation_Event:\t");
        if(event.code == SYN_REPORT)
        {
          printf("SYN_REPORT\t");
          printf("Value = %d\n", event.value);
        }
        break;
      case EV_ABS:
        printf("Absolute Event\t\t");
        if(event.code == ABS_X)
        {
          printf("Absolute X\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_Y)
        {
          printf("Absolute Y\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_TOOL_WIDTH)
        {
          printf("Tool width\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_PRESSURE)
        {
          printf("Pressure\t");
          printf("Value = %d\n", event.value);
        }
#ifdef ABS_MT_SLOT
        else if(event.code == ABS_MT_SLOT)
        {
          printf("MT slot\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_MT_TOUCH_MAJOR)
        {
          printf("MT Touch Major\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_MT_POSITION_X)
        {
          printf("MT Touch X position\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_MT_POSITION_Y)
        {
          printf("MT Touch Y position\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_MT_PRESSURE)
        {
          printf("MT Pressure\t");
          printf("Value = %d\n", event.value);
        }
        else if(event.code == ABS_MT_TRACKING_ID)
        {
          printf("MT Tracking ID\t");
          printf("Value = %d\n", event.value);
        }
#endif
        else
        {
          printf("Event Code = 0x%02X\t", event.code);
          printf("Value = %d\n", event.value);
        }
        break;
      case EV_KEY:
        printf("Key Event \t\t");
        if(event.code == BTN_TOUCH)
        {
          printf("Button Touch\t");
          printf("Value = %d\n", event.value);
        }
        break;
      default:
        printf("Event Type=%d\t", event.type);
        printf("Code=%d\t", event.code);
        printf("Value = %d\n", event.value);
        break;
    }
  }
  fclose(fp);
}

uint8_t self_test_handler()
{
  int self_test;
  uint16_t self_test_address;
  uint8_t cmd;

  self_test_address = get_object_address(SPT_SELFTEST_T25, 0);

  while(1)
  {
    printf("Self-test menu:\n\
      Enter 1 for running Analog power test\n\
      Enter 2 for running Pin fault test\n\
      Enter 3 for running Signal Limit test\n\
      Enter 4 for running Gain test\n\
      Enter 5 for running all the above tests\n\
      Enter 255 to get out of the self-test menu\n");
    scanf("%d", &self_test);
    switch(self_test)
    {
      case 1:
        cmd = 0x01;
        break;
      case 2:
        cmd = 0x11;
        break;
      case 3:
        cmd = 0x17;
        break;
      case 4:
        cmd = 0x20;
        break;
      case 5:
        cmd = 0xFE;
        break;
      case 255:
        return 1;
      default:
        printf("Invalid option\n");
        break;
    }

    if(0 == mxt_write_register(&cmd, self_test_address + 1, 1))
    {
      printf("Write command successful\n");
    }
    else
    {
      printf("Write command unsuccessful\n");
    }
  }

}


int print_raw_messages(){

  int count, i;

  /* Get the number of new messages */
  count = mxt_get_debug_messages();

  /* Print any new messages */
  if (count == 0)
  {
    printf("(no messages)\n");
  }
  else if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      printf("%s\n", (char *)mxt_retrieve_message());
      fflush(stdout);
    }
  }
  else
  {
    /* Error reading messages */
    return -1;
  }

  return 0;

}
