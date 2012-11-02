//------------------------------------------------------------------------------
/// \file   utilfuncs.c
/// \brief  Utility functions for Linux maXTtouch app.
/// \author Iiro Valkonen.
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
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/info_block_driver.h"
#include "utilfuncs.h"


void print_info_block()
{
  int i;

  /* Show the Version Info */
  printf("Family ID:         0x%02X\n", info_block.id->family_id);
  printf("Variant ID:        0x%02X\n", info_block.id->variant_id);
  printf("Version:           %d.%d\n",
         (info_block.id->version & 0xF0) >> 4, (info_block.id->version & 0x0F));
  printf("Build:             0x%02X\n", info_block.id->build);
  printf("Matrix X Size:     %d\n", info_block.id->matrix_x_size);
  printf("Matrix Y Size:     %d\n", info_block.id->matrix_y_size);
  printf("Number of objects: %d\n", info_block.id->num_declared_objects);
  printf("\n");

  /* Show the object table */
  printf("%s: OBJECT TABLE\n", __func__);
  for (i = 0; i < info_block.id->num_declared_objects; i++)
  {
    printf("Object Type:\t\t %s\n", objname(info_block.objects[i].object_type));
    printf("Address:\t\t %d\n", get_start_position(info_block.objects[i]));
    printf("Size:\t\t\t %d\n", info_block.objects[i].size);
    printf("Instances:\t\t %d\n", info_block.objects[i].instances);
    printf("ReportIDs:\t\t %d\n", info_block.objects[i].num_report_ids);
    printf("\n");
  }

  /* Show the CRC */
  printf("Information Block Checksum:\t\t 0x%06X \n",
    (info_block.crc->CRC_hi<<16u) | (info_block.crc->CRC));
}

const char *objname(uint8_t objtype)
{
  switch(objtype)
  {
    case DEBUG_DELTAS_T2:
      return("DEBUG_DELTAS_T2");
    case DEBUG_REFERENCES_T3:
      return("DEBUG_REFERENCES_T3");
    case DEBUG_SIGNALS_T4:
      return("DEBUG_SIGNALS_T4");
    case GEN_MESSAGEPROCESSOR_T5:
      return("GEN_MESSAGEPROCESSOR_T5");
    case GEN_COMMANDPROCESSOR_T6:
      return("GEN_COMMANDPROCESSOR_T6");
    case GEN_POWERCONFIG_T7:
      return("GEN_POWERCONFIG_T7");
    case GEN_ACQUISITIONCONFIG_T8:
      return("GEN_ACQUISITIONCONFIG_T8");
    case TOUCH_MULTITOUCHSCREEN_T9:
      return("TOUCH_MULTITOUCHSCREEN_T9");
    case TOUCH_SINGLETOUCHSCREEN_T10:
      return("TOUCH_SINGLETOUCHSCREEN_T10");
    case TOUCH_XSLIDER_T11:
      return("TOUCH_XSLIDER_T11");
    case TOUCH_YSLIDER_T12:
      return("TOUCH_YSLIDER_T12");
    case TOUCH_XWHEEL_T13:
      return("TOUCH_XWHEEL_T13");
    case TOUCH_YWHEEL_T14:
      return("TOUCH_YWHEEL_T14");
    case TOUCH_KEYARRAY_T15:
      return("TOUCH_KEYARRAY_T15");
    case PROCG_SIGNALFILTER_T16:
      return("PROCG_SIGNALFILTER_T16");
    case PROCI_LINEARIZATIONTABLE_T17:
      return("PROCI_LINEARIZATIONTABLE_T17");
    case SPT_COMMSCONFIG_T18:
      return("SPT_COMMSCONFIG_T18");
    case SPT_GPIOPWM_T19:
      return("SPT_GPIOPWM_T19");
    case PROCI_GRIPFACESUPPRESSION_T20:
      return("PROCI_GRIPFACESUPPRESSION_T20");
    case PROCG_NOISESUPPRESSION_T22:
      return("PROCG_NOISESUPPRESSION_T22");
    case TOUCH_PROXIMITY_T23:
      return("TOUCH_PROXIMITY_T23");
    case PROCI_ONETOUCHGESTUREPROCESSOR_T24:
      return("PROCI_ONETOUCHGESTUREPROCESSOR_T24");
    case SPT_SELFTEST_T25:
      return("SPT_SELFTEST_T25");
    case DEBUG_CTERANGE_T26:
      return("DEBUG_CTERANGE_T26");
    case PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
      return("PROCI_TWOTOUCHGESTUREPROCESSOR_T27");
    case SPT_CTECONFIG_T28:
      return("SPT_CTECONFIG_T28");
    case SPT_GPI_T29:
      return("SPT_GPI_T29");
    case SPT_GATE_T30:
      return("SPT_GATE_T30");
    case TOUCH_KEYSET_T31:
      return("TOUCH_KEYSET_T31");
    case TOUCH_XSLIDERSET_T32:
      return("TOUCH_XSLIDERSET_T32");
    case GEN_MESSAGEBLOCK_T34:
      return("GEN_MESSAGEBLOCK_T34");
    case SPT_PROTOTYPE_T35:
      return("SPT_PROTOTYPE_T35");
    case DEBUG_DIAGNOSTIC_T37:
      return("DEBUG_DIAGNOSTIC_T37");
    case SPT_USERDATA_T38:
      return("SPT_USERDATA_T38");
    case PROCI_GRIPSUPPRESSION_T40:
      return("PROCI_GRIPSUPPRESSION_T40");
    case PROCI_PALMSUPPRESSION_T41:
      return("PROCI_PALMSUPPRESSION_T41");
    case PROCI_TOUCHSUPPRESSION_T42:
      return("PROCI_TOUCHSUPPRESSION_T42");
    case SPT_DIGITIZER_T43:
      return("SPT_DIGITIZER_T43");
    case SPT_MESSAGECOUNT_T44:
      return("SPT_MESSAGECOUNT_T44");
    case PROCI_VIRTUALKEY_T45:
      return("PROCI_VIRTUALKEY_T45");
    case SPT_CTECONFIG_T46:
      return("SPT_CTECONFIG_T46");
    case PROCI_STYLUS_T47:
      return("PROCI_STYLUS_T47");
    case PROCG_NOISESUPPRESSION_T48:
      return("PROCG_NOISESUPPRESSION_T48");
    case GEN_DUALPULSE_T49:
      return("GEN_DUALPULSE_T49");
    case SPT_SONY_CUSTOM_T51:
      return("SPT_SONY_CUSTOM_T51");
    case TOUCH_PROXKEY_T52:
      return("TOUCH_PROXKEY_T52");
    case GEN_DATASOURCE_T53:
      return("GEN_DATASOURCE_T53");
    case PROCG_NOISESUPPRESSION_T54:
      return("PROCG_NOISESUPPRESSION_T54");
    case PROCI_ADAPTIVETHRESHOLD_T55:
      return("PROCI_ADAPTIVETHRESHOLD_T55");
    case PROCI_SHIELDLESS_T56:
      return("PROCI_SHIELDLESS_T56");
    case PROCI_EXTRATOUCHSCREENDATA_T57:
      return("PROCI_EXTRATOUCHSCREENDATA_T57");
    case SPT_EXTRANOISESUPCTRLS_T58:
      return("SPT_EXTRANOISESUPCTRLS_T58");
    case SPT_FASTDRIFT_T59:
      return("SPT_FASTDRIFT_T59");
    case SPT_TIMER_T61:
      return("SPT_TIMER_T61");
    case PROCG_NOISESUPPRESSION_T62:
      return("PROCG_NOISESUPPRESSION_T62");
    case PROCI_ACTIVESTYLUS_T63:
      return("PROCI_ACTIVESTYLUS_T63");
    case SPT_REFERENCERELOAD_T64:
      return("SPT_REFERENCERELOAD_T64");
    case PROCI_LENSBENDING_T65:
      return("PROCI_LENSBENDING_T65");
    case SPT_GOLDENREFERENCES_T66:
      return("SPT_GOLDENREFERENCES_T66");
    case PROCI_CUSTOMGESTUREPROCESSOR_T67:
      return("PROCI_CUSTOMGESTUREPROCESSOR_T67");
    case SERIAL_DATA_COMMAND_T68:
      return("SERIAL_DATA_COMMAND_T68");
    case PROCI_PALMGESTUREPROCESSOR_T69:
      return("PROCI_PALMGESTUREPROCESSOR_T69");
    case SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
      return("SPT_DYNAMICCONFIGURATIONCONTROLLER_T70");
    case SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
      return("SPT_DYNAMICCONFIGURATIONCONTAINER_T71");
    case PROCG_NOISESUPPRESSION_T72:
      return("PROCG_NOISESUPPRESSION_T72");
    case PROCI_ZONEINDICATION_T73:
      return("PROCI_ZONEINDICATION_T73");
    case PROCG_SIMPLEGESTUREPROCESSOR_T74:
      return("PROCG_SIMPLEGESTUREPROCESSOR_T74");
    case MOTION_SENSING_OBJECT_T75:
      return("MOTION_SENSING_OBJECT_T75");
    case PROCI_MOTION_GESTURES_T76:
      return("PROCI_MOTION_GESTURES_T76");
    default:
      return("UNKNOWN");
  }
}

void write_to_object(int obj_num)
{
  uint8_t obj_tbl_num, i;
  uint8_t *temp;
  int yn;
  int value;

  obj_tbl_num = get_object_table_num(obj_num);

  temp = (uint8_t *)malloc(sizeof(char)*(info_block.objects[obj_tbl_num].size+1));
  if (temp == NULL)
  {
    fputs ("Memory error\n",stderr);
    exit (2);
  }

  if(obj_tbl_num == 255)
  {
    printf("Object not found\n");
  }
  else
  {
    printf("%s:\n", objname(info_block.objects[obj_tbl_num].object_type));
    mxt_read_register(temp, get_start_position(info_block.objects[obj_tbl_num]), info_block.objects[obj_tbl_num].size+1);


    for(i = 0; i < (info_block.objects[obj_tbl_num].size+1); i++)
    {
      printf("Object element %d =\t %d\n",i, *(temp+i));
      printf("Do you want to change this value? (1 for yes/2 for no)");
      if (scanf("%d", &yn) != 1)
      {
        printf("Input error\n");
        return;
      }
      if(yn == 1)
      {
        printf("Enter the value to be written to object element %d\t :", i);
        if (scanf("%d", &value) != 1)
        {
          printf("Input error\n");
          return;
        }
        *(temp+i) = (uint8_t) value;
        printf("wrote %d\n", (uint8_t) value);
      }
    }

    mxt_write_register(temp, get_start_position(info_block.objects[obj_tbl_num]), info_block.objects[obj_tbl_num].size+1);
  }
}


void read_object(int obj_num)
{
  uint8_t obj_tbl_num;
  uint8_t *temp;
  int i;

  obj_tbl_num = get_object_table_num(obj_num);

  temp = (uint8_t *)malloc(sizeof(char)*(info_block.objects[obj_tbl_num].size+1));
  if (temp == NULL)
  {
    fputs ("Memory error\n",stderr);
    exit (2);
  }

  if(obj_tbl_num == 255)
  {
    printf("Object not found\n");
  }
  else
  {
    printf("%s:\n", objname(info_block.objects[obj_tbl_num].object_type));
    mxt_read_register(temp, get_start_position(info_block.objects[obj_tbl_num]), info_block.objects[obj_tbl_num].size+1);

    for(i=0; i<(info_block.objects[obj_tbl_num].size+1); i++)
    {
      printf("Object element %d =\t %d\n",i, *(temp+i));
    }
  }
}

void print_objs()
{
  int i;
  for (i = 0; i < info_block.id->num_declared_objects; i++)
  {
    printf("\t %s\n", objname(info_block.objects[i].object_type));
  }
}

static char to_digit(char hex)
{
    char decimal;

    if (hex >= '0' && hex <= '9')
        decimal = hex - '0';
    else if (hex >= 'A' && hex <= 'F')
        decimal = hex - 'A' + 10;
    else if (hex >= 'a' && hex <= 'f')
        decimal = hex - 'a' + 10;
    else
        decimal = 0;

    return decimal;
}

int mxt_convert_hex(char *hex, unsigned char *databuf, uint8_t *count, unsigned int buf_size)
{
  unsigned int pos = 0;
  uint8_t datapos = 0;
  char highnibble;
  char lownibble;

  while (1) {
    highnibble = *(hex + pos);
    lownibble = *(hex + pos + 1);

    if (lownibble == '\0' || lownibble == '\n'
        || highnibble == '\0' || highnibble == '\n')
      break;

    *(databuf + datapos) = (to_digit(highnibble) << 4)
      | to_digit(lownibble);
    datapos++;

    pos += 2;
    if (pos > buf_size)
      return -1;
  }

  *count = datapos;
  return 0;
}
