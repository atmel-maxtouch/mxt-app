//------------------------------------------------------------------------------
/// \file   self_test.c
/// \brief  Self test functions
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"

#include "self_test.h"

//******************************************************************************
/// \brief Handle messages from the self test object
static void self_test_handle_messages(void)
{
   bool done = false;
   uint16_t count, i, byte;
   time_t now;
   time_t start_time = time(NULL);
   static const uint8_t TIMEOUT = 10; // seconds
   uint8_t buf[10];
   size_t len;
   unsigned int object_type;

   while (!done)
   {
      now = time(NULL);
      if ((now - start_time) > TIMEOUT)
      {
         printf("Timeout\n");
         return;
      }

      count = mxt_get_msg_count();

      if (count > 0)
      {
         for (i = 0; i < count; i++)
         {
            len = mxt_get_msg_bytes(buf, sizeof(buf));

            if (len > 0)
            {
               object_type = report_id_to_type(buf[0]);

               printf("Received message from T%u\n", object_type);
               
               if (object_type == SPT_SELFTEST_T25)
               {
                  for (byte = 1; byte < len; byte++)
                  {
                     printf("%02X ", buf[byte]);
                  }
                  printf("\n\n");

                  switch (buf[1])
                  {
                  case SELF_TEST_ALL:
                     printf("PASS: All tests passed\n");
                     break;
                  case SELF_TEST_INVALID:
                     printf("FAIL: Invalid test command\n");
                     break;
                  case SELF_TEST_ANALOG:
                     printf("FAIL: AVdd is not present\n");
                     break;
                  case SELF_TEST_PIN_FAULT:
                     printf("FAIL: Pin fault\n");
                     break;
                  case SELF_TEST_SIGNAL_LIMIT:
                     printf("FAIL: Signal limit fault\n");
                     break;
                  case SELF_TEST_GAIN:
                     printf("FAIL: Gain error\n");
                     break;
                  default:
                     printf("FAIL: Unrecognised error\n");
                     break;
                  }

                  done = true;
               }
            }
         }
      }

      sleep(1);
   }
}

//******************************************************************************
/// \brief Print T25 limits for each enabled touch object
static void print_t25_limits(uint16_t t25_addr)
{
   int i;
   object_t element;
   int touch_object = 0;
   uint8_t buf[4];
   uint16_t upsiglim;
   uint16_t losiglim;
   int instance;

   for (i = 0; i < info_block.id->num_declared_objects; i++)
   {
      element = info_block.objects[i];

      switch (element.object_type)
      {
      case TOUCH_MULTITOUCHSCREEN_T9:
      case TOUCH_SINGLETOUCHSCREEN_T10:
      case TOUCH_XSLIDER_T11:
      case TOUCH_YSLIDER_T12:
      case TOUCH_XWHEEL_T13:
      case TOUCH_YWHEEL_T14:
      case TOUCH_KEYARRAY_T15:
      case TOUCH_PROXIMITY_T23:
      case TOUCH_KEYSET_T31:
      case TOUCH_XSLIDERSET_T32:
         for (instance = 0; (instance < element.instances + 1); instance++)
         {
            mxt_read_register((uint8_t *)&buf, get_start_position(element), 1);

            printf("%s[%d] %s\n",
                   objname(element.object_type),
                   instance,
                   buf[0] & 0x01 ? "enabled":"disabled");

            mxt_read_register((uint8_t *)&buf,
               t25_addr + 2 + touch_object * 4, 4);

            upsiglim = (uint16_t)((buf[1] << 8u) | buf[0]);
            losiglim = (uint16_t)((buf[3] << 8u) | buf[2]);

            printf("  UPSIGLIM:%d\n", upsiglim);
            printf("  LOSIGLIM:%d\n\n", losiglim);

            touch_object++;
         }
         break;
      default:
         break;
      }
   }
   
}

//******************************************************************************
/// \brief Disable noise suppression objects
static void disable_noise_suppression(void)
{
   uint16_t addr;
   uint8_t disable = 0;

   addr = get_object_address(PROCG_NOISESUPPRESSION_T22, 0);
   if (addr != OBJECT_NOT_FOUND)
   {
      mxt_write_register(&disable, addr, 1);
   }

   addr = get_object_address(PROCG_NOISESUPPRESSION_T48, 0);
   if (addr != OBJECT_NOT_FOUND)
   {
      mxt_write_register(&disable, addr, 1);
   }

   addr = get_object_address(PROCG_NOISESUPPRESSION_T54, 0);
   if (addr != OBJECT_NOT_FOUND)
   {
      mxt_write_register(&disable, addr, 1);
   }

   addr = get_object_address(PROCG_NOISESUPPRESSION_T62, 0);
   if (addr != OBJECT_NOT_FOUND)
   {
      mxt_write_register(&disable, addr, 1);
   }
}

//******************************************************************************
/// \brief Run self test
void run_self_tests(uint8_t cmd)
{
   uint16_t t25_addr;
   uint8_t enable = 3;

   mxt_msg_reset();

   // Enable self test object & reporting
   t25_addr = get_object_address(SPT_SELFTEST_T25, 0);
   printf("Enabling self test object\n");
   mxt_write_register(&enable, t25_addr, 1);

   printf("Disabling noise suppression\n\n");
   disable_noise_suppression();

   print_t25_limits(t25_addr);

   printf("Running tests\n");
   mxt_write_register(&cmd, t25_addr + 1, 1);

   self_test_handle_messages();
}

//******************************************************************************
/// \brief Run self test
uint8_t self_test_handler()
{
   int self_test;
   uint8_t cmd;

   while(1)
   {
      cmd = 0;

      printf("Self-test menu:\n\
      Enter 1 for running Analog power test\n\
      Enter 2 for running Pin fault test\n\
      Enter 3 for running Signal Limit test\n\
      Enter 4 for running Gain test\n\
      Enter 5 for running all the above tests\n\
      Enter 255 to get out of the self-test menu\n");

      if (scanf("%d", &self_test) != 1)
      {
        printf("Input error\n");
        return -1;
      }

      switch(self_test)
      {
      case 1:
        cmd = SELF_TEST_ANALOG;
        break;
      case 2:
        cmd = SELF_TEST_PIN_FAULT;
        break;
      case 3:
        cmd = SELF_TEST_SIGNAL_LIMIT;
        break;
      case 4:
        cmd = SELF_TEST_GAIN;
        break;
      case 5:
        cmd = SELF_TEST_ALL;
        break;
      case 255:
        return 1;
      default:
        printf("Invalid option\n");
        break;
      }

      if (cmd > 0)
      {
        run_self_tests(cmd);
      }
   }
}
