//------------------------------------------------------------------------------
/// \file   diagnostic_data.c
/// \brief  Diagnostic Data functions
/// \author Atul Tiwari/Nick Dyer
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
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <math.h>
#include <inttypes.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"

#include "mxt_app.h"

#define MAX_FILENAME_LENGTH     255
#define file_template "data_%s.csv"

//******************************************************************************
/// \brief T37 Diagnostic Data object
struct t37_diagnostic_data {
  uint8_t mode;
  uint8_t page;
  uint8_t data[];
};

//******************************************************************************
/// \brief Retrieve and store object information for debug data operation
/// \return #mxt_rc
static int get_objects_addr(struct t37_ctx *ctx)
{
  struct mxt_touchscreen_info *ts_info = NULL;
  uint8_t buf;
  uint8_t siglimits[2];
  int t6_addr;
  int t100_addr;
  int ret;

  ret = mxt_read_touchscreen_info (ctx->mxt, &ts_info);
  if (ret != MXT_SUCCESS) {
    mxt_err(ctx->lc, "Read touchscreen info failed\n");
    free(ts_info);
    return MXT_INTERNAL_ERROR;
  }

  /* Check if P2P is enabled */
  t100_addr = mxt_get_object_address(ctx->mxt, TOUCH_MULTITOUCHSCREEN_T100, 0);  
  if (t100_addr == OBJECT_NOT_FOUND) {
    return MXT_ERROR_OBJECT_NOT_FOUND;
  } else {

    ret = mxt_read_register(ctx->mxt, &buf, t100_addr + MXT_T100_CALCFG, 1);
    if (ret )
      mxt_warn(ctx->lc, "Failed to determine P2P mode in T100");

    if (buf & 0x01) 
      ctx->p2p_enabled = true;
    else 
      ctx->p2p_enabled = false;
  }

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(ctx->mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain T8 object address */
  ctx->t8_addr = mxt_get_object_address(ctx->mxt, GEN_ACQUISITIONCONFIG_T8, 0);
  if (ctx->t8_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain T113 object address */
  ctx->t113_addr = mxt_get_object_address(ctx->mxt, SPT_PROXMEASURECONFIG_T113, 0);
  if (ctx->t113_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain T25 or T12 object address */
  ctx->t25_addr = mxt_get_object_address(ctx->mxt, SPT_SELFTEST_T25, 0);
  if (ctx->t25_addr == OBJECT_NOT_FOUND) {
     ctx->t12_addr = mxt_get_object_address(ctx->mxt, SPT_SELFTESTSIGLIMIT_T12, 0);
     
     if (ctx->t12_addr == OBJECT_NOT_FOUND) {
        return MXT_ERROR_OBJECT_NOT_FOUND;
      } else {
          ret = mxt_read_register(ctx->mxt, &siglimits[0], 
              ctx->t12_addr + MXT_T12_SIGLIMLO, 2);

          ctx->siglimlo = siglimits[0] | (siglimits[1] << 8);

          ret = mxt_read_register(ctx->mxt, &siglimits[0], 
              ctx->t12_addr + MXT_T12_SIGLIMUP, 2);
          ctx->siglimup = siglimits[0] | (siglimits[1] << 8);

          ret = mxt_read_register(ctx->mxt, &siglimits[0], 
              ctx->t12_addr + MXT_T12_SIGLIMRANGE, 2);

          ctx->siglimrange = siglimits[0] | (siglimits[1] << 8);

      }
      
    if (ret)
      mxt_warn(ctx->lc, "Failed to read signal limits in T12");

  } else {

     /* Pending for T25 signal limits */
     /* need to determine offsets based on chip family and variant id */

  }

  /* T37 command address */
  ctx->diag_cmd_addr = t6_addr + MXT_T6_DIAGNOSTIC_OFFSET;

  /* Obtain Debug Diagnostic object's address */
  ctx->t37_addr = mxt_get_object_address(ctx->mxt, DEBUG_DIAGNOSTIC_T37, 0);
  if (ctx->t37_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain Debug Diagnostic object's size */
  ctx->t37_size = mxt_get_object_size(ctx->mxt, DEBUG_DIAGNOSTIC_T37);
  if (ctx->t37_size == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* TBD - Pending removal - used to track self cap modes */
  /* GDV captures all most regardless is present */
  
  ctx->t111_instances = mxt_get_object_instances(ctx->mxt,
                        SPT_SELFCAPCONFIG_T111);

  ctx->t107_instances = mxt_get_object_instances(ctx->mxt,
                        PROCI_ACTIVESTYLUS_T107);
  
  ctx->t100_instances = mxt_get_object_instances(ctx->mxt,
                        TOUCH_MULTITOUCHSCREEN_T100);
  
  ctx->t9_instances = mxt_get_object_instances(ctx->mxt,
                        TOUCH_MULTITOUCHSCREEN_T9);

  ret = mxt_read_register(ctx->mxt, &buf, 
      ctx->t8_addr + MXT_T8_MEASALLOW_OFFSET, 1);
  if (ret)
    mxt_warn(ctx->lc, "Failed to determine scan modes in T8");

  ctx->t8_scan_allow = buf;

  ret = mxt_read_register(ctx->mxt, &buf, 
      ctx->t8_addr + MXT_T8_MEASIDLEDEF_OFFSET, 1);
  if (ret)
    mxt_warn(ctx->lc, "Failed to determine scan modes in T8");

  ctx->t8_scan_idle = buf;

  ret = mxt_read_register(ctx->mxt, &buf, 
      ctx->t8_addr + MXT_T8_MEASACTVDEF_OFFSET, 1);
  if (ret)
    mxt_warn(ctx->lc, "Failed to determine scan modes in T8");
  ctx->t8_scan_actv = buf;

  /* Get the ALTAXISEN bit */

  ret = mxt_read_register(ctx->mxt, &buf,
      ctx->t113_addr + MXT_T113_CTRL, 1);
  if (ret)
    mxt_warn(ctx->lc, "Failed to get the singled-ended axis info");
  
  ctx->scp_axis_en = buf;

  if (ctx->scp_axis_en & 0x04) {
    /* Use the longer axis btw XSIZE and YSIZE */
    if (ts_info[0].xsize <= ts_info[0].ysize) {
      ctx->scp_axis = 0;  /* 0 - indicates X axis */
    } else {
      ctx->scp_axis = 1;  /* 1 - indicates Y axis */
    }
  } else {  /* ALTAXISEN = 0 */
    /* Use the shorter axis btw XSIZE and YSIZE */
    if (ts_info[0].ysize >= ts_info[0].xsize) {
      ctx->scp_axis = 1;  /* 0 - indicates Y axis */
    } else {
      ctx->scp_axis = 0;  /* 1 - indicates X axis */
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read a single page of diagnostic data
/// \stored in byte buffer, 2 bytes per node, little-endian
/// \return #mxt_rc
static int mxt_get_t37_page(struct t37_ctx *ctx)
{
  int failures;
  int ret;
  uint8_t read_command = 1;
  uint8_t page_up_cmd = PAGE_UP;

  if (ctx->pass == 0 && ctx->page == 0) {
    mxt_dbg(ctx->lc, "Writing mode command %02X", ctx->mode);
    ret = mxt_write_register(ctx->mxt, &ctx->mode, ctx->diag_cmd_addr, 1);
    if (ret)
      return ret;
  } else {
    ret = mxt_write_register(ctx->mxt, &page_up_cmd, ctx->diag_cmd_addr, 1);
    if (ret)
      return ret;
  }

  /* Read back diagnostic register in T6 command processor until it has been
   * cleared. This means that the chip has actioned the command */
  failures = 0;

  while (read_command) {
    usleep(500);
    ret = mxt_read_register(ctx->mxt, &read_command, ctx->diag_cmd_addr, 1);
    if (ret) {
      mxt_err(ctx->lc, "Failed to read the status of diagnostic mode command");
      return ret;
    }

    if (read_command) {
      failures++;

      if (failures > 500) {
        mxt_err(ctx->lc, "Timeout waiting for command to be actioned");
        return MXT_ERROR_TIMEOUT;
      }
    }
  }

  ret = mxt_read_register(ctx->mxt, (uint8_t *)ctx->t37_buf,
                          ctx->t37_addr, ctx->t37_size);
  if (ret) {
    mxt_err(ctx->lc, "Failed to read page");
    return ret;
  }

  if (ctx->t37_buf->mode != ctx->mode) {
    mxt_err(ctx->lc, "Bad mode in diagnostic data read");
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }

  if (ctx->t37_buf->page != (ctx->pages_per_pass * ctx->pass + ctx->page)) {
    mxt_err(ctx->lc, "Bad page in diagnostic data read");
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Output header to CSV file
/// \return #mxt_rc
static int mxt_generate_hawkeye_header(struct t37_ctx *ctx)
{
  struct mxt_id_info *id = ctx->mxt->info.id;
  int ret;
  int x;
  int y;
  int i, pass, num_keys;
  int num_frames;
  int data_values = 0;

  /* Time header adjustments for MTS - Graphical Viewer */

  if (ctx->fformat == false) {
    if (ctx->t15_keyarray || ctx->self_cap) {
      ret = fprintf(ctx->hawkeye, "TIME(0-0),");  /* MTS modifed labels */
    } else {
      ret = fprintf(ctx->hawkeye, "time,TIN,");
    }
  }

  if (ret < 0)
    return MXT_ERROR_IO;

  /* Tested with U series, pending others */
  /* Expand for other U series, pending testing other series */
  if (ctx->self_cap) {

      const char * mode;
      switch (ctx->mode) {
        default:
        break;
        
        case SELF_CAP_DELTAS:
          mode = "delta";
          break;

        case SELF_CAP_REFS:
          mode = "ref";
          break;

        case SELF_CAP_SIGNALS:
          mode = "sig";
          break;
      }

      /* Global labels for all self cap SCT, SCH and SCP */
      /* Label for self cap SCT, SCH, SCP */
      for (y = 0; y < ctx->y_size; y++) {
        ret = fprintf(ctx->hawkeye, "Y%d_SCT_%s(%d-%d),", y, mode, 1, data_values);
        if (ret < 0)
          return MXT_ERROR_IO;
          
        data_values++;
      }

      for (x = 0; x < ctx->x_size; x++) {
        ret = fprintf(ctx->hawkeye, "X%d_SCT_%s(%d-%d),", x, mode, 1, data_values);
        if (ret < 0)
          return MXT_ERROR_IO;
          
        data_values++;
      }

      data_values = 0;

      for (y = 0; y < ctx->y_size; y++) {
        ret = fprintf(ctx->hawkeye, "Y%d_SCH_%s(%d-%d),", y, mode, 2 ,data_values);
        if (ret < 0)
          return MXT_ERROR_IO;
          
        data_values++;
      }

      data_values = 0;

      for (x = 0; x < ctx->x_size; x++) {
        ret = fprintf(ctx->hawkeye, "X%d_SCH_%s(%d-%d),", x, mode, 2, data_values);
        if (ret < 0)
          return MXT_ERROR_IO;
          
        data_values++;
      }

      data_values = 0;

      for (y = 0; y < ctx->y_size; y++) {
        ret = fprintf(ctx->hawkeye, "Y%d_SCP_%s(%d-%d),", y, mode, 3 ,data_values);
        if (ret < 0)
          return MXT_ERROR_IO;
          
        data_values++;
      }

      for (x = 0; x < ctx->x_size; x++) {
        ret = fprintf(ctx->hawkeye, "X%d_SCP_%s(%d-%d)", x, mode, 3, data_values);

        /* Don't place comma on last data in row */
        if (x != ctx->x_size - 1) {
          ret = fprintf(ctx->hawkeye, ",");
          if (ret < 0)
            return MXT_ERROR_IO;
        }
          
        data_values++;
      }
      
      ret = fprintf(ctx->hawkeye, "\n");

  } else if (ctx->active_stylus) {
    for (pass = 0; pass < ctx->passes; pass++) {
      const char *mode;
      switch (ctx->mode) {
      default:
      case AST_DELTAS:
        mode = "delta";
        break;
      case AST_REFS:
        mode = "ref";
        break;
      }

      for (y = 0; y < ctx->y_size; y++) {
        int y_real;
        const char* set;

        if ( y < id->matrix_y_size) {
          y_real = y;
          set = "0";
        } else {
          y_real = y - id->matrix_y_size;
          set = "1";
        }
        ret = fprintf(ctx->hawkeye, "AST_%s_Y%s_%d,", mode, set, y_real);
        if (ret < 0)
          return MXT_ERROR_IO;
      }

      for (x = 0; x < ctx->x_size; x++) {
        int x_real;
        const char* set;

        if ( x < ctx->x_size/2) {
          x_real = x;
          set = "0";
        } else {
          x_real = x - ctx->x_size/2;
          set = "1";
        }

        ret = fprintf(ctx->hawkeye, "AST_%s_X%s_%d,", mode, set, x_real);
        if (ret < 0)
          return MXT_ERROR_IO;
      }
    }

  } else if (ctx->t15_keyarray) {
    for (pass = 0; pass < ctx->passes; pass++) {
      const char *mode;
      switch (ctx->mode) {
      default:
      case KEY_DELTAS_MODE:
        mode = "_deltas";
        break;
      case KEY_REFS_MODE:
        mode = "_refs";
        break;
      case KEY_SIGS_MODE:
        mode = "_sigs";
        break;
      }
      
      num_keys = ctx->key_buf[pass];
      
      for (i = 0; i < num_keys; i++) {      
        ret = fprintf(ctx->hawkeye, "Key_%d%s_%d,", i, mode, pass);
        
        if (ret < 0)
          return MXT_ERROR_IO;
      }       
    }
  } else {  /* For Mutual Cap */

    if (ctx->fformat == false) {    /* Check for format 0 */
      for (x = 0; x < ctx->x_size; x++) {
        for (y = 0; y < ctx->y_size; y++) {
          ret = fprintf(ctx->hawkeye, "X%dY%d_%s16,", x, y,
                        (ctx->mode == DELTAS_MODE) ? "Delta" : "Reference");
          if (ret < 0)
            return MXT_ERROR_IO;
        }
      }

      ret = fprintf(ctx->hawkeye, "\n");

      if (ret < 0)
        return MXT_ERROR_IO;
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Sort interleaved debug data
/// \return #mxt_rc
static int sort_debug_data(struct mxt_device *mxt, struct t37_ctx *ctx)
{
  uint16_t offset1 = 0;
  uint16_t offset2 = 0;
  uint16_t count = 0;
  int i, j;

  ctx->temp_buf = (uint16_t *)calloc(ctx->data_values, sizeof(uint16_t));
  if (!ctx->temp_buf)
    mxt_err(ctx->lc, "Buffer calloc failure");
  
  offset2 = ctx->y_size/2;    //For interleaved

  while (count < ctx->data_values)
  {
    for (i = 0; i < 2; i++) {	//Odd and Even sort
      for (j = 0; j < ctx->y_size/2 ; j++) {

        ctx->temp_buf[count] = ctx->data_buf[j + offset1]; //First data element
        ctx->temp_buf[count + 1] = ctx->data_buf[j + offset2];  //Second data element
        count = count + 2;
      
      }
      //zero the counts and continue to next two rows until all data_values are done
      offset1 = offset1 + (ctx->y_size);
      offset2 = offset2 + (ctx->y_size);
    }
  }

  count = 0;

  while (count < ctx->data_values)
  {

    //Reorder data elements into data buf
    ctx->data_buf[count] = ctx->temp_buf[count];
    count++;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Sort M1 selfcap debug data
/// \Sort following SCT, SCH and SCP format
/// \Sorted by Y, then X lines with SCP and SCH, if present
/// \TBD - Sort by page size next update
/// \return #mxt_rc
static int sort_debug_m1_selfcap_data(struct mxt_device *mxt, struct t37_ctx *ctx)
{

  struct mxt_id_info *id = mxt->info.id;
  uint16_t count = 0;
  uint16_t tmpcnt = 0;
  int data_ofs = 0;
  int page_ofs = 0;
  int pass = 0;
  int i;
  int ret = 0;

  /* Create buffer by data values */
  ctx->temp_buf = (uint16_t *)calloc(ctx->data_values, sizeof(uint16_t));
  if (!ctx->temp_buf)
    mxt_err(ctx->lc, "Buffer calloc failure");

   /* Combine into sequential SCT, SCH, and SCP format */
  if (id->variant == 0x00) {
  
    for (pass = 0; pass < ctx->passes; pass++) {
      for (page_ofs = 0; page_ofs < ctx->pages_per_pass; page_ofs++) {
         /* Skip empty space in buffer, goto SCP data */

        data_ofs = (ctx->x_size + ctx->y_size) * page_ofs;
      
        /* Load SCT values */
        if (page_ofs == 0) {
          for (i = 0; i < ctx->y_size; i++) {     
            ctx->temp_buf[count] = ctx->data_buf[data_ofs + i];
            count++; 
          }

          for (i = 0; i < ctx->x_size; i++) {
            ctx->temp_buf[count] = ctx->data_buf[data_ofs + ctx->y_size + i];
            count++;
          }
        }

        if (page_ofs == 2) {
          /* Load SCH values, even if just zeros */
          for (i = 0; i < ctx->y_size; i++) {     
            ctx->temp_buf[count] = ctx->data_buf[data_ofs + i];
            count++; 
          }

          for (i = 0; i < ctx->x_size; i++) {
            ctx->temp_buf[count] = ctx->data_buf[data_ofs + ctx->y_size + i];
            count++;
          }
        }

        if (page_ofs == 4) {
          /* Determine axis of SCP data and load */

          if (ctx->scp_axis) {
            data_ofs = (ctx->page_size/2 * page_ofs);
            for (i = 0; i < ctx->y_size; i++) {     
              ctx->temp_buf[count] = ctx->data_buf[data_ofs + i];
              count++; 
            }

            for (i = 0; i < ctx->x_size; i++) {
              ctx->temp_buf[count] = 0;
              count++;
            }
          } else {
            data_ofs = (ctx->page_size/2 * page_ofs);
            for (i = 0; i < ctx->y_size; i++) {
              ctx->temp_buf[count] = 0;
              count++;
            }

            for (i = 0; i < ctx->x_size; i++) {
              ctx->temp_buf[count] = ctx->data_buf[data_ofs + i];
              count++;
            }
          }
        }
      }
    }
  }

  while (tmpcnt < count) {
    ctx->data_buf[tmpcnt] = ctx->temp_buf[tmpcnt];
    tmpcnt++;

  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Sort interleaved selfcap debug data
/// \Sort to final sequential order, no gaps
/// \Sorted by xsize an ysize
/// \Future improvement - remove unused x and y lines
/// \return #mxt_rc
static int sort_debug_selfcap_data(struct mxt_device *mxt, struct t37_ctx *ctx)
{
  
  struct mxt_id_info *id = mxt->info.id;
  struct mxt_touchscreen_info *ts_info = NULL;
  uint16_t offset1 = 0;
  uint16_t offset2 = 0;
  uint16_t count = 0;
  uint16_t x_offset = 0;
  int i, j;
  int ret = 0;

  ret = mxt_read_touchscreen_info (ctx->mxt, &ts_info);
  if (ret != MXT_SUCCESS) {
    mxt_err(ctx->lc, "Read touchscreen info failed\n");
    free(ts_info);
    return MXT_INTERNAL_ERROR;
  }

  /* Create buffer by data values */
  ctx->temp_buf = (uint16_t *)calloc(ctx->data_values, sizeof(uint16_t));
  if (!ctx->temp_buf)
    mxt_err(ctx->lc, "Buffer calloc failure");

  /* Sort even and odd references, not for deltas */
  for (ctx->page = 0; ctx->page < ctx->pages_per_pass; ctx->page++) {

    count = (ctx->page * ctx->page_size/2);

    /* Y then X even, X odd */
    if (id->variant == 0x06 || id->variant == 0x07) {
      for (i = 0; i < ctx->y_size/2; i++) {
        /* 0, 2, 4, 6 */
        ctx->temp_buf[count] = ctx->data_buf[i + (ctx->page_size/2 * ctx->page)];

        /* 1, 3, 5, 7 */
        ctx->temp_buf[count + 1] = ctx->data_buf[i + (ctx->y_size/2) + (ctx->page_size/2 * ctx->page)];

        count+=2;
      }
    } else {  /* Some parts Y is sequential */
      for (i = 0; i < ctx->y_size; i++) {
        /* Sequential for some U series */
        ctx->temp_buf[count] = ctx->data_buf[i + (ctx->page_size/2 * ctx->page)];
        count++;
      }
    }
    
    /* even X then odd X */
    for (i = 0; i < ctx->x_size/2; i++) {
      /* 0, 2, 4, 6 */
      ctx->temp_buf[count] = ctx->data_buf[i + ctx->y_size + (ctx->page_size/2 * ctx->page)];

      /* 1, 3, 5, 7 */
      ctx->temp_buf[count + 1] = ctx->data_buf[i + ctx->y_size + ctx->start_offsetx + (ctx->page_size/2 * ctx->page)];

      count+=2;
    }
  }

  count = 0;

  while (count < ctx->data_values)
  {
    /* Reorder data elements into data_buf in SCT, SCH and SCP format */
    /* Remove unused axis  data for SCP - pending */
    ctx->data_buf[count] = ctx->temp_buf[count];
    count++;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Convert T37 byte buffer to 16bit node buffer, stored big-endian
/// \Copy all bytes from T37 to 16 bit value to be sorted later
/// \return #mxt_rc
static int mxt_debug_insert_data_self_cap(struct mxt_device *mxt, struct t37_ctx *ctx)
{
  struct mxt_id_info *id = ctx->mxt->info.id;
  int i;
  int ofs = 0;
  uint16_t val;

  switch (ctx->mode) {
    case SELF_CAP_DELTAS:
    case SELF_CAP_SIGNALS:
    case SELF_CAP_REFS:

      /* For self cap, combine into 16 bit words to be sorted later */
      for (i = 0; i < ctx->page_size; i += 2) {
    
        val = (ctx->t37_buf->data[i+1] << 8) | ctx->t37_buf->data[i];

        ofs = ctx->y_ptr;

        /* Fix only for 640UD for compatibility with GDV */
        if ((id->family == 0xa6) && (id->variant == 0x17)) {
          if (val == 32768) {
            ctx->data_buf[ofs] = 0;
          } else {
            ctx->data_buf[ofs] = val;
          }
        } else {
            ctx->data_buf[ofs] = val;
        }
        
        ctx->y_ptr++;
      }

      break;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Insert t15 key array data into buffer at appropriate co-ordinates
/// \return #mxt_rc
static int mxt_debug_insert_data_key_array(struct t37_ctx *ctx)
{
  int offset = 0;
  int pass_ofs = 0;
  int data_pos = 0;
  uint16_t val;

  /* Manage key array and multiple instance */

  for (offset = 0; offset < (ctx->key_buf[ctx->pass]); offset++) {

    val = (ctx->t37_buf->data[data_pos+1] << 8) | ctx->t37_buf->data[data_pos];

    data_pos +=2;
    
    if (ctx->passes != 0) {
      pass_ofs = ctx->key_buf[(ctx->pass - 1)];
    }
      
    ctx->data_buf[offset + pass_ofs] = val;
  }
  
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Insert page of data into buffer at appropriate co-ordinates
/// \return #mxt_rc
static int mxt_debug_insert_data(struct t37_ctx *ctx)
{
  struct mxt_id_info *id = ctx->mxt->info.id;
  uint16_t value;
  int ofs;
  int i;

  for (i = 0; i < ctx->page_size; i += 2) {
    
    value = (ctx->t37_buf->data[i+1] << 8) | ctx->t37_buf->data[i];

    ofs = ctx->y_ptr;
    
    /* The last page may overlap the end of the matrix */
    if (ofs >= ctx->data_values)
      return MXT_SUCCESS;

    /* Set the saturated values to 0 */
    /* Fix for 640UD only, may not be applicable to other devices */
    /* Only for GDV compatibilty */
    if ((id->family == 0xa6) && (id->variant == 0x17)) {
      if (value == 32768) {
        ctx->data_buf[ofs] = 0;
      } else {
        ctx->data_buf[ofs] = value;
      }
    } else {
        ctx->data_buf[ofs] = value;
    }

    ctx->y_ptr++;

  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Write data to file
/// \return #mxt_rc
static int mxt_hawkeye_output(struct t37_ctx *ctx)
{
  int x, y, i;
  int pass;
  int page_ofs;
  int ret;
  int num_frames;
  int ofs = 0;
  int data_ofs = 0;
  int32_t value = 0;
  uint8_t totalkeys = 0;
  uint8_t ts_xsize = 0;
  uint8_t ts_ysize = 0;
  uint8_t ts_xorigin = 0;
  uint8_t ts_yorigin = 0;  
  struct mxt_t15_info *mxt_key = NULL;
  struct mxt_id_info *id = ctx->mxt->info.id;
  struct mxt_touchscreen_info *ts_info = NULL;

  ret = mxt_read_touchscreen_info(ctx->mxt, &ts_info);
  if (ret != MXT_SUCCESS) {
    mxt_err(ctx->lc, "Read touchscreen info failed\n");
    free(ts_info);
    ts_info = NULL;
    return MXT_INTERNAL_ERROR;
  }

  switch(ctx->mode) {

    case KEY_DELTAS_MODE:
    case KEY_REFS_MODE:
    case KEY_SIGS_MODE:
    case KEY_RAW_SIGS_MODE:

      break;

    case SELF_CAP_DELTAS:
    case SELF_CAP_REFS:
    case SELF_CAP_SIGNALS:

      ret = mxt_print_timestamp(ctx->hawkeye, false);
      if (ret)
        return ret;

      break;

    default:

      if ((ctx->mode == DELTAS_MODE || ctx->mode == REFS_MODE) &&
          ctx->fformat == 0) {

        ret = mxt_print_timestamp(ctx->hawkeye, false);
        if (ret)
          return ret;
  
        /* print frame number */
        ret = fprintf(ctx->hawkeye, "%u,", ctx->frame);
        if (ret < 0)
        return MXT_ERROR_IO;
      }

      break;
  } /* End of 1st switch */

  if ((ctx->self_cap)) {

    switch(ctx->mode) {
      case SELF_CAP_REFS:
      case SELF_CAP_DELTAS:
      case SELF_CAP_SIGNALS:

        /* Ref data is sequential after sorting */
        for (pass = 0; pass < ctx->passes; pass++) {
          for (page_ofs = 0; page_ofs < ctx->pages_per_pass; page_ofs++) {

            /* Tested 336U, 640U, 1296M1 */

            if (id->family == 0xa6) {
              data_ofs = ctx->page_size/2 * page_ofs;   /* full page */

              if (id->variant == 0x14) {
                /* Unify input data to be by page and not SCT,SCH,SCP format */
                /* Combine into one input format to make simple, TBD */
                /* For 336UD-HA series */
                data_ofs = ctx->page_size/2 * page_ofs;
              }

            } else if (id->family == 167) {
              
              if (id->variant == 0x00) {
                if (page_ofs == 0x03) {
                  goto data_end;
                }
              }

              data_ofs = (ctx->x_size + ctx->y_size) * page_ofs;
            }
            
            for (y = 0; y < ctx->y_size; y++) {

              value = (int16_t)ctx->data_buf[data_ofs + y];

              ret = fprintf(ctx->hawkeye, "%d,",
                      (ctx->mode == SELF_CAP_DELTAS) ? (int16_t)value : value);
              if (ret < 0)
              return MXT_ERROR_IO;
            }

            for (x = 0; x < ctx->x_size; x++) {

              value = (int16_t)ctx->data_buf[data_ofs + ctx->y_size + x];

              if ((page_ofs == ctx->pages_per_pass-1) && (x == ctx->x_size -1)) {
                ret = fprintf(ctx->hawkeye, "%d",
                      (ctx->mode == SELF_CAP_DELTAS) ? (int16_t)value : value);
              } else {
                ret = fprintf(ctx->hawkeye, "%d,", 
                      (ctx->mode == SELF_CAP_DELTAS) ? (int16_t)value : value);
              }

              if (ret < 0)
                return MXT_ERROR_IO;

            }

          }
        }

data_end:

        ret = fprintf(ctx->hawkeye, "\n");
        if (ret < 0)
          return MXT_ERROR_IO;     
         
        default:
          break;
    
      } /* End of switch ctx->mode */

  } else if (ctx->active_stylus) {
    for (pass = 0; pass < ctx->passes; pass++) {
      int pass_ofs = (ctx->y_size + ctx->x_size) * pass;

      for (y = 0; y < ctx->y_size; y++) {
        value = (int16_t)ctx->data_buf[pass_ofs + y];
        ret = fprintf(ctx->hawkeye, "%d,", (int16_t) value);
        if (ret < 0)
          return MXT_ERROR_IO;
      }

      for (x = 0; x < ctx->x_size; x++) {
        value = (int16_t)ctx->data_buf[pass_ofs + ctx->y_size + x];
        ret = fprintf(ctx->hawkeye, "%d,", (int16_t) value);
        if (ret < 0)
          return MXT_ERROR_IO;
      }
    }

    ret = fprintf(ctx->hawkeye, "\n");

  } else if (ctx->t15_keyarray) {
      for (i = 0; i < ctx->passes; i++) {
        totalkeys = totalkeys + ctx->key_buf[i];
      }

      for (i = 0; i < totalkeys; i++) {
        value = (int16_t)ctx->data_buf[i];
        ret = fprintf(ctx->hawkeye, "%d,",
                      (ctx->mode == KEY_DELTAS_MODE) ? (int16_t)value : value);
        if (ret < 0)
          return MXT_ERROR_IO;
    }
  } else { /* Mutual data */

      if (ctx->fformat == 0 ) {

        /* Need fix for mxT640UD full X and Y matrix size  */
        /* Use for matrix aleady limited in initialize function */
        /* iterate through columns */
        for (x = 0; x < ctx->x_size; x++) {
          for (y = 0; y < ctx->y_size; y++) {
            ofs = y + x * ctx->y_size;
        //for (x = 0; x < ts_info[pass].xsize; x++) {
         // for (y = 0; y < ts_info[pass].ysize; y++) {
         //   ofs = y + x * ts_info[pass].ysize;

              value = ctx->data_buf[ofs];

            if (!ctx->p2p_enabled) {
              ret = fprintf(ctx->hawkeye, "%d,", 
                          (ctx->mode == DELTAS_MODE) ? (int16_t)value : value);
            } else {
              ret = fprintf(ctx->hawkeye, "%d,", 
                          ((ctx->mode == DELTAS_MODE) || (ctx->mode == REFS_MODE)) ? (int16_t)value : (int16_t) value);
            }
            
            if (ret < 0)
              return MXT_ERROR_IO;
          }
        }
         
        ret = fprintf(ctx->hawkeye, "\n");

        if (ret < 0)
          return MXT_ERROR_IO;
      } else {  /* Start of format 1 or format 2*/

        /* Pass used for slider or 2nd touchscreen */
        pass = ctx->instance;
         
        /* Setup X Axis columns for touchscreen matrix */
    
        ret = fprintf(ctx->hawkeye, "Frame %d", ctx->frame);
                   
        for (x = 0; x < ts_info[pass].xsize; x++) {
          ret = fprintf(ctx->hawkeye, ",X%d_%s16", (x + ts_info[pass].xorigin), 
                       (ctx->mode == DELTAS_MODE) ? "Deltas" : "Refs");      
        }
    
        /* Insert newline for first row of data */
        ret = fprintf(ctx->hawkeye, "\n");
         
        /* Insert y data per rows */
        for (y = 0; y < ts_info[pass].ysize; y++) {

          ret = fprintf(ctx->hawkeye, "Y%d_%s16", (y + ts_info[pass].yorigin), 
                         (ctx->mode == DELTAS_MODE) ? "Deltas" : "Refs"); 
           
          ret = fprintf(ctx->hawkeye, ",");

          data_ofs = ((ts_info[pass].xorigin * ctx->y_size) + (ts_info[pass].yorigin));

          for (x = 0; x < ts_info[pass].xsize; x++) {
            value = (int16_t)ctx->data_buf[y + data_ofs];
   
            if (ctx->fformat == 1) {
              ret = fprintf(ctx->hawkeye, "%d",
                       (ctx->mode == DELTAS_MODE) ? (int16_t) value: value);

              if (ret < 0)
                return MXT_ERROR_IO;
              
            } else if ((ctx->fformat == 2) && (ctx->mode == REFS_MODE)) {

              if ((value > ctx->siglimlo) && (value < ctx->siglimup)) {
                ret = fprintf(ctx->hawkeye, "PASS/%d", value);
              } else {
                ret = fprintf(ctx->hawkeye, "FAIL/%d", value);
              }

              if (ret < 0)
                  return MXT_ERROR_IO;
            }
            
            if (x == ((ts_info[pass].xsize) - 1)) {
              break;
            } else {
              ret = fprintf(ctx->hawkeye, ",");
              if (ret < 0)
                return MXT_ERROR_IO;
            }
            
            data_ofs = data_ofs + ctx->y_size;
          }

          ret = fprintf(ctx->hawkeye, "\n");
          if (ret < 0)
            return MXT_ERROR_IO;
               
          if (y == ((ts_info[pass].ysize) - 1)) {
            ret = fprintf(ctx->hawkeye, "\n");
            if (ret < 0)
              return MXT_ERROR_IO;
        
            break;
          }  
        }  
      }
  }
  
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Select multitouch instance
/// \return #mxt_rc
static int get_instance_num(uint16_t *instance)
{
  printf("Enter instance number: ");
  
  if (scanf("%hu", instance) == EOF) {
    fprintf(stderr, "Could not handle the input, exiting");
    return MXT_ERROR_BAD_INPUT;
  }
  
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Get file format
/// \return #mxt_rc
static int get_file_format(uint16_t *fformat)
{
  printf("Enter file format 0/1/2: ");

  if (scanf("%hu", fformat) == EOF) {
    fprintf(stderr, "Could not handle the input, exiting");
    return MXT_ERROR_BAD_INPUT;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Input number of frames
/// \return #mxt_rc
static int get_num_frames(uint16_t *frames)
{
  printf("Enter number of frames: ");

  if (scanf("%hu", frames) == EOF) {
    fprintf(stderr, "Could not handle the input, exiting");
    return MXT_ERROR_BAD_INPUT;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Get file attr, 0 - write, 1 - append
/// \return #mxt_rc
static int get_file_attr(uint16_t *file_attr)
{
  printf("Enter file attribute 0 - Write to file, 1 - Append to file: ");

  if (scanf("%hu", file_attr) == EOF) {
    fprintf(stderr, "Could not handle the input, exiting");
    return MXT_ERROR_BAD_INPUT;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Initialise parameters and allocate buffers
/// \return #mxt_rc
int mxt_debug_dump_initialise(struct mxt_device *mxt, struct t37_ctx *ctx)
{
  int ret, pass;
  struct mxt_id_info *id = ctx->mxt->info.id;
  struct mxt_touchscreen_info *ts_info = NULL;
  struct mxt_t15_info *mxt_key_info;
  uint8_t instance = 0;
  uint8_t data;
  uint8_t buf_ofs;

  /* TBD - Make global */
  /* Need info for 0xA7 chips that are configurable */
  ret = mxt_read_touchscreen_info (ctx->mxt, &ts_info);
  if (ret != MXT_SUCCESS) {
    mxt_err(ctx->lc, "Read touchscreen info failed\n");
    free(ts_info);
    ts_info = NULL;
    return MXT_INTERNAL_ERROR;
  } 

  ctx->active_stylus = false;
  ctx->self_cap = false;
  ctx->t15_keyarray = false;

  ret = get_objects_addr(ctx);
  if (ret) {
    mxt_err(ctx->lc, "Failed to get object information");
    return ret;
  }

  mxt_dbg(ctx->lc, "t37_size: %d", ctx->t37_size);

  /* Minus header */
  ctx->page_size = ctx->t37_size - 2;

  mxt_dbg(ctx->lc, "T37 object size = %d,page_size = %d",
      ctx->t37_size, ctx->page_size);

  switch (ctx->mode) {
  case DELTAS_MODE:
  case REFS_MODE:

    if (id->family == 0xA0 && id->variant == 0x00) {
      /* mXT1386 data is formatted into stripes */
      ctx->x_size = 27;
      ctx->y_size = id->matrix_y_size;
      ctx->data_values = 27 * ctx->y_size;
      ctx->passes = 3;
      ctx->pages_per_pass = 8;
    } else if (id->family == 0xA7 && id->variant == 0x00) {
      ctx->x_size = ts_info[0].xsize;
      ctx->y_size = ts_info[0].ysize;
      ctx->data_values = ctx->x_size * ctx->y_size;
      ctx->passes = 1;
      ctx->pages_per_pass = (ctx->data_values * 2 + (ctx->page_size - 1)) /
                            ctx->page_size;
    } else {
      /* mxt640UD is full matrix */
      ctx->x_size = id->matrix_x_size;
      ctx->y_size = id->matrix_y_size;
      ctx->data_values = ctx->x_size * ctx->y_size;
      ctx->passes = 1;
      ctx->pages_per_pass = (ctx->data_values * 2 + (ctx->page_size - 1)) /
                            ctx->page_size;
    }

    ctx->stripe_width = ctx->y_size;
      
    mxt_dbg(ctx->lc, "ctx->data_values: %d", ctx->data_values);
    mxt_dbg(ctx->lc, "ctx->passes: %d", ctx->passes);
    mxt_dbg(ctx->lc, "ctx->y_size %d", ctx->y_size);
    mxt_dbg(ctx->lc, "ctx->x_size: %d", ctx->x_size);
    mxt_dbg(ctx->lc, "stripe_width: %d", ctx->stripe_width);
    mxt_dbg(ctx->lc, "pages_per_pass: %d", ctx->pages_per_pass);

    break;

  case SELF_CAP_DELTAS:
  case SELF_CAP_SIGNALS:
  case SELF_CAP_REFS:
  
    if (((ctx->t8_scan_allow & 0x0a) == 0x00) &&
        ((ctx->t8_scan_idle & 0x0a) == 0x00) &&
        ((ctx->t8_scan_actv & 0x02) == 0x00)) {
      mxt_err(ctx->lc, "Self cap data not available or not enabled");
      return MXT_ERROR_NOT_SUPPORTED;
    } else {
      ctx->self_cap = true;
    }

    if ((ctx->t8_scan_allow & SCP_BIT_MASK) ||
       (ctx->t8_scan_idle & SCP_BIT_MASK) ||
       (ctx->t8_scan_actv & SCP_BIT_MASK)) {

        ctx->scp_enabled = true;
    } else {
        ctx->scp_enabled = false;
    }

    if (ctx->t111_instances == 0) {
      mxt_err(ctx->lc, "T111 not found");
      return MXT_ERROR_OBJECT_NOT_FOUND;
    }

    if (id->family == 0xa6) {

      ctx->passes = 1;  /* U series SCT, SCH, SCP all read in one pass */
      /* 640UD - Y0 to Y19, X0 to X31, full matrix */
      ctx->y_size = id->matrix_y_size;  /* Most legacy parts are based on matrix size */
      ctx->x_size = id->matrix_x_size; 
      ctx->pages_per_pass = 3;          /* Fixed output format, SCT, SCH and SCP */
      /* Increase data_values to full page captures */
      ctx->data_values = (ctx->page_size/2 * ctx->pages_per_pass);
     
      /* This is the offset value from the beginning end of the even X lines for sc refs */
      /* Ymax + even Xmax + offset of data to first odd Xmin */
      switch (id->variant) {

      case 0x01: /* 640U offset for odd X */
        ctx->start_offsetx = 0x14;
        break;

      case 0x06: /* 336U offset for odd X */
        ctx->start_offsetx = 0x0C;
        break;

      case 0x08: /* 144U offset for odd X */
        ctx->start_offsetx = 0x18;
        break;

      case 0x07: /* 308U offset pending more updates */
        ctx->start_offsetx = 0x30;
        break;

      case 0x14: /* 336UD offset and scp_start_page */
        ctx->start_offsetx = 0x0C;
        ctx->scp_start_page = 2;

      case 0x17: /* 640UD offset and scp_start_page */
        ctx->start_offsetx = 0x14;
        ctx->scp_start_page = 2;

      default:
        break;

      }

    } else if (id->family == 0xa7) {
      /* 0xA7 chip using X and Y actual size, not matrix max */
      ctx->x_size = ts_info[0].xsize;
      ctx->y_size = ts_info[0].ysize; 

      /* Read all T37 data, SCP data page is offset */
      if (id->variant == 0x00) {
        ctx->xscp_page_start = 4;
        ctx->pages_per_pass = 5;
      } else {  /* Adjust later for other A7 devices */
        ctx->yscp_page_start = 3;
        ctx->pages_per_pass = 3;
      }

      /* Self cap captures Y_SCT, X_SCT and SCP all at once */
      ctx->passes = 1;
      /* Read full T37 page, sort later */
      ctx->data_values = (ctx->page_size/2 * ctx->pages_per_pass);

    } else { /* TBD - Check compatibility with legacy devices */

    // Read Ymax Y values, plus Ymax or 2Ymax X values
      ctx->passes = ctx->t111_instances;
      ctx->y_size = id->matrix_y_size;
      ctx->x_size = ctx->y_size * ((id->matrix_x_size > ctx->y_size) ? 2 : 1);
      ctx->data_values = (ctx->y_size + ctx->x_size) * ctx->passes;
      
      ctx->pages_per_pass = ((ctx->y_size + ctx->x_size)*sizeof(uint16_t) + (ctx->page_size - 1)) /
                          ctx->page_size;
    }

      mxt_dbg(ctx->lc, "ctx->y_size %d", ctx->y_size);
      mxt_dbg(ctx->lc, "ctx->x_size: %d", ctx->x_size);
      mxt_dbg(ctx->lc, "ctx->data_values: %d", ctx->data_values);

    break;
      
  case KEY_DELTAS_MODE:
  case KEY_REFS_MODE:
  case KEY_SIGS_MODE:
  case KEY_RAW_SIGS_MODE:    
      
    ctx->t15_keyarray = true;
    ctx->data_values = 0;

    ret = mxt_read_t15_key_info (mxt, &mxt_key_info);
    if (ret != MXT_SUCCESS) {
        mxt_err(ctx->lc, "Read t15 key array failed\n");
        free(mxt_key_info);
        return MXT_INTERNAL_ERROR;
    }
      
    /* Get t15 instances */
    ctx->t15_instances = mxt_get_object_instances(mxt, TOUCH_KEYARRAY_T15);
      
    // Setup of passes and set Y max and X max to 32 total buttons
    ctx->passes = ctx->t15_instances;   // Passes equals # available instances 
      
    /* Allocate storage for t15 x and y size */
    ctx->key_buf = (uint8_t *)calloc((ctx->passes), sizeof(uint8_t));
    if (!ctx->key_buf) {
      mxt_err(ctx->lc, "calloc key_buf failed");      
    }
     
    buf_ofs = 0;
      
    /* Calculate # of keys with multiple instances */
    for (pass = 0; pass < ctx->passes; pass++) {
      data = (mxt_key_info[pass].t15_ysize) * (mxt_key_info[pass].t15_xsize);
      ctx->data_values = ctx->data_values + data;
      ctx->key_buf[pass] = data;
      
      mxt_dbg(ctx->lc, "T15_ysize: %d", mxt_key_info[pass].t15_ysize);
      mxt_dbg(ctx->lc, "T15_xsize: %d", mxt_key_info[pass].t15_xsize);
      mxt_dbg(ctx->lc, "Keys per pass: %d", ctx->data_values);
    }

    /* Calculate # of keys for array buffer */
    ctx->pages_per_pass = 1;      
      
    break;

  case AST_DELTAS:
  case AST_REFS:
      
    ctx->active_stylus = true;

    if (id->family != 164) {
      mxt_err(ctx->lc, "active stylus data not available");
      return MXT_ERROR_NOT_SUPPORTED;
    }

    if (ctx->t107_instances == 0) {
      mxt_err(ctx->lc, "T107 not found");
      return MXT_ERROR_OBJECT_NOT_FOUND;
    }

    // Read Ymax Y values, plus Ymax or 2Ymax X values
    ctx->passes = ctx->t107_instances;
    ctx->y_size = 2 * id->matrix_y_size;    // Two scans per axis
    ctx->x_size = ctx->y_size * ((id->matrix_x_size > ctx->y_size) ? 2 : 1);
    ctx->data_values = (ctx->y_size + ctx->x_size) * ctx->passes;
    ctx->pages_per_pass = ((ctx->y_size + ctx->x_size)*sizeof(uint16_t) +(ctx->page_size - 1)) /
                          ctx->page_size;
    break;

  default:
    mxt_err(ctx->lc, "Unsupported mode %02X", ctx->mode);
    return MXT_INTERNAL_ERROR;
  } /* End of ctx->mode switch statement */

  mxt_dbg(ctx->lc, "passes: %d", ctx->passes);
  mxt_dbg(ctx->lc, "pages_per_pass: %d", ctx->pages_per_pass);
  
  if (!(ctx->t15_keyarray)) {
    mxt_dbg(ctx->lc, "x_size: %d", ctx->x_size);
    mxt_dbg(ctx->lc, "y_size: %d", ctx->y_size);
    mxt_dbg(ctx->lc, "data_values: %d", ctx->data_values);
  }

  /* allocate t37 buffers */
  ctx->t37_buf = (struct t37_diagnostic_data *)calloc(1, ctx->t37_size);
  if (!ctx->t37_buf) {
    mxt_err(ctx->lc, "calloc failure");
    free(ctx->t37_buf);
    ctx->t37_buf = NULL;
    return MXT_ERROR_NO_MEM;
  }

  /* allocate data buffer */
  ctx->data_buf = (uint16_t *)calloc(ctx->data_values, sizeof(uint16_t));
  if (!ctx->data_buf) {
    mxt_err(ctx->lc, "calloc failure");

    /* free other buffer in error path */
    free(ctx->data_buf);
    ctx->data_buf = NULL;
    return MXT_ERROR_NO_MEM;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read one frame of diagnostic data
/// \return #mxt_rc
int mxt_read_diagnostic_data_frame(struct mxt_device *mxt, struct t37_ctx* ctx)
{
  struct mxt_id_info *id = mxt->info.id;
  int ret;

    /* iterate through stripes */
    /* Calculate stripe parameters */
    ctx->stripe_starty = 0;
    ctx->stripe_endy = ctx->stripe_starty + ctx->stripe_width - 1;
    ctx->x_ptr = 0;
    ctx->y_ptr = ctx->stripe_starty;
    ctx->pass = 0;

    for (ctx->page = 0; ctx->page < ctx->pages_per_pass; ctx->page++) {
      mxt_dbg(ctx->lc, "Frame %d Pass %d Page %d Stripe Start %d Stripe Width %d\n", ctx->frame, ctx->pass,
              ctx->page, ctx->stripe_starty, ctx->stripe_width);

      ret = mxt_get_t37_page(ctx);
      if (ret)
        return ret;

      mxt_debug_insert_data(ctx);
    }

    if (id->family == 0xA6 && ctx->mode == REFS_MODE) {

      switch (id->variant) {
      case 0x06 ... 0x08:
      case 0x0A:
      case 0x0C ... 0x14:

        sort_debug_data(mxt, ctx);
        break;

      default:
        break;
      }
    }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read one frame of diagnostic data
/// \return #mxt_rc
static int mxt_read_diagnostic_data_self_cap(struct mxt_device *mxt, struct t37_ctx* ctx)
{
  struct mxt_id_info *id = mxt->info.id;
  int ret;

  ctx->y_ptr = 0;

  for (ctx->pass = 0; ctx->pass < ctx->passes; ctx->pass++) {
    for (ctx->page = 0; ctx->page < ctx->pages_per_pass; ctx->page++) {
      mxt_dbg(ctx->lc, "Frame %d Pass %d Page %d", ctx->frame, ctx->pass,
              ctx->page);

      ret = mxt_get_t37_page(ctx);
      if (ret)
        return ret;


      mxt_debug_insert_data_self_cap(mxt, ctx);
    }
  }

  if (id->family == 0xA6 && ctx->mode == SELF_CAP_REFS) {

    switch (id->variant) {
      case 0x01 ... 0x02:
      case 0x06 ... 0x08:
      case 0x14:
      case 0x17:
      /* TBD */
      // case 0x0A ... 0x0F:

        sort_debug_selfcap_data(mxt, ctx);

        break;

      default:
        break;
    }
  } else if (id->family == 0xA7) {  /* Deltas and references */

    switch (id->variant) {
      case 0x00:
        /* Use different sorting routine, non-std */
        sort_debug_m1_selfcap_data(mxt, ctx);

        break;
    }
    /* do no sort */
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read one frame of diagnostic data
/// \return #mxt_rc
static int mxt_read_diagnostic_data_ast(struct mxt_device *mxt, struct t37_ctx* ctx)
{
  return mxt_read_diagnostic_data_self_cap(mxt, ctx);
}

//******************************************************************************
/// \brief Read one frame of diagnostic data t15 key array
/// \return #mxt_rc
static int mxt_read_diagnostic_data_t15key(struct mxt_device *mxt, struct t37_ctx* ctx)
{
  int ret;
  
  /* iterate through stripes */
  for (ctx->pass = 0; ctx->pass < ctx->passes; ctx->pass++) {
    for (ctx->page = 0; ctx->page < ctx->pages_per_pass; ctx->page++) {
      mxt_dbg(ctx->lc, "Frame %d Pass %d Page %d", ctx->frame, ctx->pass,
              ctx->page);

      ret = mxt_get_t37_page(ctx);
      if (ret)
        return ret;

      mxt_debug_insert_data_key_array(ctx);
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Retrieve data from the T37 Diagnostic Data object
/// \return #mxt_rc
int mxt_debug_dump(struct mxt_device *mxt, int mode, const char *csv_file,
                   uint16_t frames, uint16_t instance, uint16_t format, uint8_t attr)
{
  struct t37_ctx ctx;
  time_t t1;
  time_t t2;
  int ret;

  ctx.lc = mxt->ctx;
  ctx.mxt = mxt;
  ctx.mode = mode;
  ctx.fformat = format;
  ctx.file_attr = attr;

  if (frames == 0) {
    mxt_warn(ctx.lc, "Warning: Defaulting to 1 frame");
    frames = 1;
  }

  ret = mxt_debug_dump_initialise(mxt, &ctx);
  if (ret)
    return ret;
  
  if (ctx.t100_instances) {
    if (instance >= ctx.t100_instances) {
      mxt_warn(ctx.lc, "Warning: Instance %d does not exist. Defaulting to T100 instance 0", instance);
      instance = 0;
    }
    
  } else if (ctx.t9_instances) {
    if (instance >= ctx.t9_instances) {
      mxt_warn(ctx.lc, "Warning: Instance %d does not exist. Defaulting to T9 instance 0", instance);
      instance = 0;
    }
  }
  
  ctx.instance = instance;

  /* Check, if ASCII append lower case 'a' */
  if (ctx.file_attr == 1) {
    ctx.hawkeye = fopen(csv_file, "a");
  } else {
    ctx.hawkeye = fopen(csv_file, "w");
  }

  if (ctx.hawkeye == NULL) {
    mxt_err(ctx.lc, "Failed to open file!");
    ret = MXT_ERROR_IO;
    goto free;
  }

  ret = mxt_generate_hawkeye_header(&ctx);
  if (ret)
    goto close;

  mxt_info(ctx.lc, "Reading %u frames", frames);

  t1 = time(NULL);

  for (ctx.frame = 1; ctx.frame <= frames; ctx.frame++) {
    if (ctx.self_cap) {
      ret = mxt_read_diagnostic_data_self_cap(mxt, &ctx);
    } else if (ctx.active_stylus) {
      ret = mxt_read_diagnostic_data_ast(mxt, &ctx);
    } else if (ctx.t15_keyarray) {
      ret = mxt_read_diagnostic_data_t15key(mxt, &ctx);
    } else { /* Mutual */
      ret = mxt_read_diagnostic_data_frame(mxt, &ctx);
    }
    if (ret)
      goto close;

    ret = mxt_hawkeye_output(&ctx);
    if (ret)
      goto close;
  }

  t2 = time(NULL);
  mxt_info(ctx.lc, "%u frames in %d seconds", frames, (int)(t2-t1));

  ret = MXT_SUCCESS;

close:
  fclose(ctx.hawkeye);
free:
  free(ctx.data_buf);
  ctx.data_buf = NULL;
  free(ctx.t37_buf);
  ctx.t37_buf = NULL;

  return ret;
}

//******************************************************************************
/// \brief Handle menu input for diagnostic data functions
static void mxt_dd_cmd(struct mxt_device *mxt, char menu_1, char menu_2, const char *csv_file)
{
  uint16_t frames;
  int ret;
  uint16_t instance = 0;
  uint16_t format = 0;
  uint16_t file_attr = 0;
  
  ret = get_instance_num(&instance);
  
  ret = get_num_frames(&frames);
  
  ret = get_file_format(&format);

  ret = get_file_attr(&file_attr);
 
  switch (menu_1) {
  case 'm':
    switch (menu_2) {
    case 'd':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, DELTAS_MODE, csv_file, frames, instance, format, file_attr);
      break;
    case 'r':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, REFS_MODE, csv_file, frames, instance, format, file_attr);
      break;
        
    default:
      printf("Invalid menu option\n");
    }
      break;
  
  case 's':
    switch (menu_2) {
    case 'd':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, SELF_CAP_DELTAS, csv_file, frames, instance, format, file_attr);
      break;
    case 'r':
      if (ret == MXT_SUCCESS) 
        mxt_debug_dump(mxt, SELF_CAP_REFS, csv_file, frames, instance, format, file_attr);
      break;
        
      default:
        printf("Invalid menu option\n");
    }
    break;
    
  case 'k':
    switch (menu_2) {
    case 'd':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, KEY_DELTAS_MODE, csv_file, frames, instance, format, file_attr);
      break;
    case 'r':
      if (ret == MXT_SUCCESS) 
        mxt_debug_dump(mxt, KEY_REFS_MODE, csv_file, frames, instance, format, file_attr);
      break;
    case 's':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, KEY_SIGS_MODE, csv_file, frames, instance, format, file_attr); 
      break;

      default:
        printf("Invalid menu option\n");
    }
    break;
    
  case 'a':
    switch (menu_2) {
    case 'd':
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, AST_DELTAS, csv_file, frames, instance, format, file_attr);
      break;
    case 'r':
      if (ret == MXT_SUCCESS) 
        mxt_debug_dump(mxt, AST_REFS, csv_file, frames, instance, format, file_attr);
      break;
        
      default:
        printf("Invalid menu option\n");
    }
    break;
   
  default:
    printf("Invalid menu option\n");
    break;
  }
}

//******************************************************************************
/// \brief Menu #1 interface for diagonistic data functions
void mxt_dd_menu(struct mxt_device *mxt)
{
  char menu_input;

  while(true) {
    printf("\nSelect one of the options:\n\n"
           "Enter M:   Dump (M)utual Cap Diagnostic Data\n"
           "Enter S:   Dump (S)elf-Cap Diagnostic Data\n"
           "Enter K:   Dump (K)ey Array Diagnostic Data\n"
           "Enter A:   Dump (A)ctive Stylus Diagnostic Data\n"
           "Enter Q:   (Q)uit\n");

    if (scanf("%1s", &menu_input) == EOF) {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }
      /* force lower case */
      menu_input = tolower(menu_input);
    
    if (menu_input == 'q') {
      printf("Quit\n");
      return;
    }

    mxt_dd_menu2(mxt, menu_input);
  }
}

//******************************************************************************
/// \brief Menu #2 interface for diagonistic data functions
void mxt_dd_menu2(struct mxt_device *mxt, char selection)
{

  switch (selection) {
  case 'm':
    mxt_mutual_menu(mxt, selection);
    break;
  case 's':
    mxt_self_menu(mxt, selection);
  case 'k':
    mxt_key_array_menu(mxt, selection);
    break;
  case 'a':
    mxt_stylus_menu(mxt, selection);
    break;
      
  default:
    printf("Invalid menu option\n");
  }    
}

//******************************************************************************
/// \brief Menu for mutual diagonistic data functions
void mxt_mutual_menu(struct mxt_device *mxt, char selection)
{
  char menu_input;
  char csv_file_in[MAX_FILENAME_LENGTH + 1];

  while(true) {
    printf("\nSelect one of the options:\n\n"
           "Enter D:   (D)elta mutual dump\n"
           "Enter R:   (R)eference mutual dump\n"
           "Enter Q:   (Q)uit\n");

    if (scanf("%1s", &menu_input) == EOF) {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }
    
    /* force lower case */
    menu_input = tolower(menu_input);
    
    switch (menu_input) {
      case 'd':
      case 'r':
      case 'q':
        if (menu_input == 'q') {
          printf("Quit\n");
          return;
        }

        printf("\nFile name: ");
        if (scanf("%255s", csv_file_in) == EOF) {
          fprintf(stderr, "Could not handle the input, exiting");
          return;
        }

        mxt_dd_cmd(mxt, selection, menu_input, csv_file_in);
        
      break;
        
      default:
        printf("Invalid menu option\n");  
    }
  }
}

//******************************************************************************
/// \brief Menu interface for diagonistic data functions
void mxt_self_menu(struct mxt_device *mxt, char selection)
{
  char menu_input;
  char csv_file_in[MAX_FILENAME_LENGTH + 1];

  while(true) {
    printf("\nSelect one of the options:\n\n"
           "Enter D:   (D)elta self-cap dump\n"
           "Enter R:   (R)eference self-cap dump\n"
           "Enter Q:   (Q)uit\n");

    if (scanf("%1s", &menu_input) == EOF) {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }
    
    /* force lower case */
    menu_input = tolower(menu_input);
    
    switch (menu_input) {
      case 'd':
      case 'r':
      case 'q':
        if (menu_input == 'q') {
          printf("Quit\n");
          return;
        }

        printf("\nFile name: ");
        if (scanf("%255s", csv_file_in) == EOF) {
          fprintf(stderr, "Could not handle the input, exiting");
          return;
        }

        mxt_dd_cmd(mxt, selection, menu_input, csv_file_in);
        
        break;
        

         default:
        printf("Invalid menu option\n");    }
  }
}

//******************************************************************************
/// \brief Menu interface for diagonistic data functions
void mxt_stylus_menu(struct mxt_device *mxt, char selection)
{
  char menu_input;
  char csv_file_in[MAX_FILENAME_LENGTH + 1];

  while(true) {
    printf("\nSelect one of the options:\n\n"
           "Enter D:   (D)elta mutual dump\n"
           "Enter R:   (R)eference mutual dump\n"
           "Enter Q:   (Q)uit\n");

    if (scanf("%1s", &menu_input) == EOF) {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }
    
      /* force lower case */
    menu_input = tolower(menu_input);
    
    switch (menu_input) {
      case 'd':
      case 'r':
      case 'q':

        if (menu_input == 'q') {
          printf("Quit\n");
          return;
        }

      printf("\nFile name: ");
      if (scanf("%255s", csv_file_in) == EOF) {
        fprintf(stderr, "Could not handle the input, exiting");
        return;
      }

      mxt_dd_cmd(mxt, selection, menu_input, csv_file_in);
        
      break;
        
      default:
      printf("Invalid menu option\n");     
    }
  }
}

//******************************************************************************
/// \brief Menu interface for diagonistic data functions
void mxt_key_array_menu(struct mxt_device *mxt, char selection)
{
  char menu_input;
  char csv_file_in[MAX_FILENAME_LENGTH + 1];

  while(true) {
    printf("\nSelect one of the options:\n\n"
           "Enter D:   (D)elta mutual dump\n"
           "Enter R:   (R)eference mutual dump\n"
           "Enter S:   (S)ignal mutual dump\n"
           "Enter Q:   (Q)uit\n");

    if (scanf("%1s", &menu_input) == EOF) {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }
    
       /* force lower case */
    menu_input = tolower(menu_input);
    
      switch (menu_input) {
        case 'd':
        case 'r':
        case 's':
        case 'q':

      if (menu_input == 'q') {
        printf("Quit\n");
        return;
      }

      printf("\nFile name: ");
      if (scanf("%255s", csv_file_in) == EOF) {
        fprintf(stderr, "Could not handle the input, exiting");
        return;
      }

      mxt_dd_cmd(mxt, selection, menu_input, csv_file_in);
          
      break;
          
      default:
      printf("Invalid menu option\n"); 
    }
  }
}

//******************************************************************************
/// \brief Disable golden references objects
/// \return #mxt_rc
int disable_gr(struct mxt_device *mxt)
{
  uint16_t addr;
  uint8_t disable = 0;

  mxt_info(mxt->ctx, "Disabling SPT_GOLDENREFERENCES_T66");

  addr = mxt_get_object_address(mxt, SPT_GOLDENREFERENCES_T66, 0);
  if (addr == OBJECT_NOT_FOUND)
    return OBJECT_NOT_FOUND;

  mxt_write_register(mxt, &disable, addr, 1);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Get value from debug data buffer
int16_t get_value(struct t37_ctx *ctx, int x, int y)
{
  int ofs = x * ctx->y_size + y;
  return (int16_t)ctx->data_buf[ofs];
}

//******************************************************************************
/// \brief Calculate stats for the data frame
int debug_frame_calc_stats(struct t37_ctx *ctx)
{
  int x, y;
  ctx->mean = 0;
  ctx->variance = 0;

  /* mean */
  for (x = 0; x < ctx->x_size; x++) {
    for (y = 0; y < ctx->y_size; y++) {
      ctx->mean += get_value(ctx, x, y);
    }
  }
  ctx->mean /= (ctx->x_size * ctx->y_size);
  mxt_dbg(ctx->lc, "Mean of dataset: %0.2f", ctx->mean);

  /* variance */
  for (x = 0; x < ctx->x_size; x++) {
    for (y = 0; y < ctx->y_size; y++) {
      ctx->variance += fabs(get_value(ctx, x, y) - ctx->mean);
    }
  }
  ctx->variance /= (ctx->x_size * ctx->y_size);
  mxt_dbg(ctx->lc, "Variance of dataset: %0.2f", ctx->variance);

  /* standard deviation */
  ctx->std_dev = sqrt(ctx->variance);
  mxt_dbg(ctx->lc, "Standard deviation of dataset: %0.2f", ctx->std_dev);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Normalise a data frame
int debug_frame_normalise(struct t37_ctx *ctx)
{
  int x, y;
  for (x = 0; x < ctx->x_size; x++) {
    for (y = 0; y < ctx->y_size; y++) {
      ctx->data_buf[x * ctx->y_size + y] =
        (ctx->data_buf[x * ctx->y_size + y] - ctx->mean)/ctx->std_dev;
    }
  }
  return MXT_SUCCESS;

}

//******************************************************************************
/// \brief Read screen parameters for touchscreen objects
/// \return #mxt_rc
int mxt_read_touchscreen_info(struct mxt_device *mxt, struct mxt_touchscreen_info **mxt_ts_info)
{
  int i, addr;
  uint8_t T100_instances = mxt_get_object_instances(mxt, TOUCH_MULTITOUCHSCREEN_T100);
  uint8_t T9_instances = mxt_get_object_instances(mxt, TOUCH_MULTITOUCHSCREEN_T9);
  uint8_t instance_count = T100_instances + T9_instances;

  struct mxt_touchscreen_info *mxt_ts =
    calloc(instance_count, sizeof(struct mxt_touchscreen_info));
  if (!mxt_ts)
    return MXT_ERROR_NO_MEM;

  if (T100_instances > 0) {
    for (i = 0; i < T100_instances; i++) {
      addr = mxt_get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T100, i);
      mxt_ts[i].instance_addr = addr;
      /* get x-axis values */
      mxt_read_register(mxt, &mxt_ts[i].xorigin, addr + T100_XORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_ts[i].xsize, addr + T100_XSIZE_OFFSET, 1);
      /* get y-axis values */
      mxt_read_register(mxt, &mxt_ts[i].yorigin, addr + T100_YORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_ts[i].ysize, addr + T100_YSIZE_OFFSET, 1);
    }
  } else if (T9_instances > 0) {
    for (i = 0; i < T9_instances; i++) {
      addr = mxt_get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T9, i);
      mxt_ts[i].instance_addr = addr;
      /* get x-axis values */
      mxt_read_register(mxt, &mxt_ts[i].xorigin, addr + T9_XORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_ts[i].xsize, addr + T9_XSIZE_OFFSET, 1);
      /* get y-axis values */
      mxt_read_register(mxt, &mxt_ts[i].yorigin, addr + T9_YORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_ts[i].ysize, addr + T9_YSIZE_OFFSET, 1);
    }
  } else {
    mxt_err(mxt->ctx, "No multi-touch device found");
    return MXT_ERROR_NO_DEVICE;
  }
          
  *mxt_ts_info = mxt_ts;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read key array parameters from T15 objects
/// \return #mxt_rc
int mxt_read_t15_key_info(struct mxt_device *mxt, struct mxt_t15_info **mxt_key_info)
{
  int i, addr;
  
  uint8_t t15_count = mxt_get_object_instances(mxt, TOUCH_KEYARRAY_T15);
  
  /* Check if t15 key instance exists */
  if (t15_count == 0) {
	    mxt_err(mxt->ctx, "T15 key array not found\n");
      return MXT_ERROR_OBJECT_NOT_FOUND;
  }
  
  /* Allocate memory for mxt_key to hold key array info */
  struct mxt_t15_info *mxt_key = calloc(t15_count, sizeof(struct mxt_t15_info));
  if (!mxt_key)
      return MXT_ERROR_NO_MEM;
	
  if (t15_count > 0) {
    for (i = 0; i < t15_count; i++) {
      addr = mxt_get_object_address(mxt, TOUCH_KEYARRAY_T15, i);

      /* Check enable bit */
      mxt_read_register(mxt, &mxt_key[i].t15_enable, addr + T15_CTRL_OFFSET, 1);
      
      if (!(mxt_key[i].t15_enable & 0x01)) {
        mxt_warn(mxt->ctx, "T15 key array instance %d not enabled\n", i);
      }	

      mxt_key[i].t15_instance_addr = addr;
      
      /* get x-axis values */
      mxt_read_register(mxt, &mxt_key[i].t15_xorigin, addr + T15_XORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_key[i].t15_xsize, addr + T15_XSIZE_OFFSET, 1);
  
      /* get y-axis values */
      mxt_read_register(mxt, &mxt_key[i].t15_yorigin, addr + T15_YORIGIN_OFFSET, 1);
      mxt_read_register(mxt, &mxt_key[i].t15_ysize, addr + T15_YSIZE_OFFSET, 1);
    }
  }
  
  *mxt_key_info = mxt_key;
  
  return MXT_SUCCESS;
}


//*****************************************************************************
/// \brief Handle value offset
float reference_no_offset(float val)
{
  if (val > 16000)
    val -= 16384;

  if (val < 384)
    val = 0;

  return val;
}

//******************************************************************************
/// \brief Enter T7 Free-run power mode Run broken line detection algorithm
/// \return #mxt_rc
int mxt_free_run_mode(struct mxt_device *mxt)
{
  uint16_t t7_addr = mxt_get_object_address(mxt, GEN_POWERCONFIG_T7, 0);
  const uint8_t t7_freerun[2] = { 0xff, 0xff };

  mxt_info(mxt->ctx, "Going into T7 Free-run Mode");

  return mxt_write_register(mxt, t7_freerun, t7_addr, 2);
}


//******************************************************************************
/// \brief Disable touch objects T9, T100 and T43
/// \return #mxt_rc
int mxt_disable_touch(struct mxt_device *mxt)
{
  uint16_t addr;
  uint8_t disable = 0;

  mxt_info(mxt->ctx, "Disabling Touch Objects");

  addr = mxt_get_object_address(mxt, TOUCH_KEYARRAY_T15, 0);
  if (!(addr == OBJECT_NOT_FOUND)) {
    mxt_write_register(mxt, &disable, addr, sizeof(disable));
    mxt_dbg(mxt->ctx, "Disabling TOUCH_KEYARRAY_T15");
  }

  addr = mxt_get_object_address(mxt, SPT_DIGITIZER_T43, 0);
  if (!(addr == OBJECT_NOT_FOUND)) {
    mxt_write_register(mxt, &disable, addr, sizeof(disable));
    mxt_dbg(mxt->ctx, "Disabling SPT_DIGITIZER_T43");
  }

  addr = mxt_get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T9, 0);
  if (!(addr == OBJECT_NOT_FOUND)) {
    mxt_write_register(mxt, &disable, addr, sizeof(disable));
    mxt_dbg(mxt->ctx, "Disabling TOUCH_MULTITOUCHSCREEN_T9 instance 0");
  }

  addr = mxt_get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T9, 1);
  if (!(addr == OBJECT_NOT_FOUND)) {
    mxt_write_register(mxt, &disable, addr, sizeof(disable));
    mxt_dbg(mxt->ctx, "Disabling TOUCH_MULTITOUCHSCREEN_T9 instance 1");
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Output hawkeye data to terminal
/// \return #mxt_rc
int debug_frame(struct t37_ctx *ctx)
{
  int x;
  int y;
  int ofs;
  int16_t value;
  char *buf;
  const char *end;
  char *p;

  /* Allow for column widths */
  size_t bufsize = 6 + 7*ctx->y_size;
  buf = calloc(bufsize, sizeof(char));
  end = buf + bufsize;

  /* Header */
  p = buf;
  p += snprintf(p, end-p, "      ");
  for (y = 0; y < ctx->y_size; y++)
    p+= snprintf(p, end-p, "Y%-5d ", y);
  mxt_dbg(ctx->lc, "%s", buf);

  for (x = 0; x < ctx->x_size; x++) {
    p = buf;
    p += snprintf(p, end-p, "X%-4d ", x);
    for (y = 0; y < ctx->y_size; y++) {
      ofs = y + x * ctx->y_size;

      value = (int16_t)ctx->data_buf[ofs];

      p+= snprintf(p, end-p, "%-6d ", value);
    }
    mxt_dbg(ctx->lc, "%s", buf);
  }

  free(buf);
  return MXT_SUCCESS;
}
