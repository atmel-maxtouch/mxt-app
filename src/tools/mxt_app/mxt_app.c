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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>

#include "libmaxtouch/libmaxtouch.h"
#include "touch_app.h"
#include "utilfuncs.h"

/* \brief Defines CHANGE line active mode. */
#define CHANGELINE_ASSERTED 0
#define MESSAGE_READ_OK             1u
#define MESSAGE_READ_FAILED         2u

/*! \brief Returns the changeline state. */
uint8_t read_changeline(void);

int main (int argc, char *argv[])
{
   char main_menu[255];
   int obj_num, exit_loop, ret;
   char cfg_file[255];

   printf("Command line tool for Atmel maXTouch chips\n");

   /*! Find an mXT device and read the info block */
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

   if (mxt_get_info() < 0)
   {
     printf("Error reading info block, exiting...\n");
     return -1;
   }

   exit_loop = 0;
   while(!exit_loop)
   {
     printf("\nSelect one of the options:\n\n"
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
       "Enter O:   Set to b(O)otloader mode and quit\n"
       "Enter Q:   (Q)uit the application\n");
     scanf("%255s", main_menu);
     switch(tolower(main_menu[0]))
     {
       case 'l':
         /* Load config file */
         printf("Give cfg file name: ");
         scanf("%s", cfg_file);
         printf("Trying to open %s...\n", cfg_file);
         if (mxt_load_config_file(cfg_file, false) == 0)
         {
           printf("Successfully uploaded the configuration file\n");
         }
         else
         {
           printf("Failed to upload the configuration\n");
         }
         break;
      case 's':
         /* Save config file */
         printf("Give cfg file name: ");
         scanf("%s", cfg_file);
         if (mxt_save_config_file(cfg_file) == 0)
         {
           printf("Successfully saved configuration to file\n");
         }
         else
         {
           printf("Failed to save configuration\n");
         }
         break;
      case 'i':
         /* Print info block */
         printf("Reading info block.....\n");
         print_info_block();
         break;
      case 'd':
         /* Read objects according to the input value */
         printf("Objects on the chip: \n");
         while(1)
         {
           print_objs();

           printf("Enter the object number to read the object's "
             "field values; Enter 255 to return to main menu \n");
           scanf("%d",&obj_num);

           if((obj_num >= 0) && (obj_num < 255))
           {
             read_object(obj_num);
           }
           else if(obj_num == 255)
           {
             break;
           }
           else
           {
             printf("Please enter a valid object number\n");
             printf("Coming out of objects space... \n");
             break;
           }
         }
         break;
      case 'w':
        printf("Objects on the chip: \n");
        while(1)
        {
          print_objs();
          printf("Enter the object number to modify the object's "
            "field values; or 255 to return to main menu\n");
          scanf("%d",&obj_num);

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
        if (mxt_get_device_type() == E_USB)
        {
          printf("to be implemented...\n\n");
        }
        else
        {
          print_raw_messages();
        }
        break;
      case 'o':
        /* Restart the chip in bootlader mode, and exit */
        if (mxt_reset_chip(true) == 0)
        {
          printf
          (
            "Successfully restarted the device in bootloader mode...\n"
            "...quitting the maxtouch application\n"
          );
          exit_loop = 1;
        }
        else
        {
          printf("Failed to restart the device in bootloader mode\n");
        }
        break;
      case 'q':
        printf("Quitting the maxtouch application\n");
        mxt_release();
        exit_loop = 1;
        break;
      default:
        printf("Invalid menu option\n");
        break;
    }
  }

  return 0;
}

