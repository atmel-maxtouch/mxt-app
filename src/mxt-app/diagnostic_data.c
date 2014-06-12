//------------------------------------------------------------------------------
/// \file   diagnostic_data.c
/// \brief  T37 Diagnostic Data functions
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

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"

#include "mxt_app.h"

#define MAX_FILENAME_LENGTH     255

//******************************************************************************
/// \brief Retrieve and store object information for debug data operation
/// \return #mxt_rc
static int get_objects_addr(struct t37_ctx *ctx)
{
  int t6_addr;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(ctx->mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* T37 commands address */
  ctx->diag_cmd_addr = t6_addr + MXT_T6_DIAGNOSTIC_OFFSET;

  /* Obtain Debug Diagnostic object's address */
  ctx->t37_addr = mxt_get_object_address(ctx->mxt, DEBUG_DIAGNOSTIC_T37, 0);
  if (ctx->t37_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain Debug Diagnostic object's size */
  ctx->t37_size = mxt_get_object_size(ctx->mxt, DEBUG_DIAGNOSTIC_T37);
  if (ctx->t37_size == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  ctx->t111_instances = mxt_get_object_instances(ctx->mxt,
                                                 SPT_SELFCAPCONFIG_T111);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Retrieve a single page of diagnostic data
/// \return #mxt_rc
static int mxt_debug_dump_page(struct t37_ctx *ctx)
{
  int failures;
  int ret;
  uint8_t read_command = 1;
  uint8_t read_mode;
  uint8_t read_page;
  uint8_t page_up_cmd = PAGE_UP;

  if (ctx->page == 0)
  {
    mxt_verb(ctx->lc, "Writing mode command");
    ret = mxt_write_register(ctx->mxt, &ctx->mode, ctx->diag_cmd_addr, 1);
    if (ret)
      return ret;
  }
  else
  {
    ret = mxt_write_register(ctx->mxt, &page_up_cmd, ctx->diag_cmd_addr, 1);
    if (ret)
      return ret;
  }

  /* Read back diagnostic register in T6 command processor until it has been
   * cleared. This means that the chip has actioned the command */
  failures = 0;

  while (read_command)
  {
    ret = mxt_read_register(ctx->mxt, &read_command, ctx->diag_cmd_addr, 1);
    if (ret)
    {
      mxt_err(ctx->lc, "Failed to read the status of diagnostic mode command");
      return ret;
    }

    if (read_command)
    {
      failures++;

      if (failures > 500)
      {
        mxt_err(ctx->lc, "Timeout waiting for command to be actioned");
        return MXT_ERROR_TIMEOUT;
      }
    }
  }

  ret = mxt_read_register(ctx->mxt, &read_mode, ctx->t37_addr, 1);
  if (ret)
  {
    mxt_err(ctx->lc, "Failed to read current mode");
    return ret;
  }

  ret = mxt_read_register(ctx->mxt, &read_page, ctx->t37_addr + 1, 1);
  if (ret)
  {
    mxt_err(ctx->lc, "Failed to read page number");
    return ret;
  }

  if ((read_mode != ctx->mode) || (read_page != ctx->page))
  {
    mxt_err(ctx->lc, "Bad page/mode in diagnostic data read");
    return MXT_ERROR_UNEXPECTED_DEVICE_STATE;
  }

  ret = mxt_read_register(ctx->mxt, ctx->page_buf,
                          ctx->t37_addr + 2, ctx->page_size);
  if (ret)
  {
    mxt_err(ctx->lc, "Failed to read page");
    return ret;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Output header to CSV file
/// \return #mxt_rc
static int mxt_generate_hawkeye_header(struct t37_ctx *ctx)
{
  int ret;
  int x;
  int y;
  int pass;

  ret = fprintf(ctx->hawkeye, "time,TIN,");
  if (ret < 0)
    return MXT_ERROR_IO;

  if (ctx->self_cap)
  {
    for (pass = 0; pass < ctx->num_passes; pass++)
    {
      const char *set;
      switch (pass)
      {
        default:
        case 0: set = "touch"; break;
        case 1: set = (ctx->num_passes == 3) ? "hover" : "prox"; break;
        case 2: set = "prox"; break;
      }

      const char * mode;
      switch (ctx->mode)
      {
        default:
        case SELF_CAP_DELTAS:    mode = "delta"; break;
        case SELF_CAP_REFS:      mode = "ref"; break;
        case SELF_CAP_SIGNALS:   mode = "sig"; break;
      }

      for (y = 0; y < ctx->y_size; y++)
      {
        ret = fprintf(ctx->hawkeye, "Y%d_SC_%s_%s,", y, set, mode);
        if (ret < 0)
          return MXT_ERROR_IO;
      }

      for (x = 0; x < ctx->x_size; x++)
      {
        int x_real;

        if (ctx->x_size < ctx->y_size)
          x_real = x;
        else
          x_real = (x * 2) % ctx->x_size;

        ret = fprintf(ctx->hawkeye, "X%d_SC_%s_%s,", x_real, set, mode);
        if (ret < 0)
          return MXT_ERROR_IO;
      }
    }
  }
  else
  {
    for (x = 0; x < ctx->x_size; x++)
    {
      for (y = 0; y < ctx->y_size; y++)
      {
        ret = fprintf(ctx->hawkeye, "X%dY%d_%s16,", x, y,
            (ctx->mode == DELTAS_MODE) ? "Delta" : "Reference");
        if (ret < 0)
          return MXT_ERROR_IO;
      }
    }
  }

  ret = fprintf(ctx->hawkeye, "\n");
  if (ret < 0)
    return MXT_ERROR_IO;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Insert page of data into buffer at appropriate co-ordinates
/// \return #mxt_rc
static int mxt_debug_insert_data_self_cap(struct t37_ctx *ctx)
{
  int i;
  int ofs;

  for (i = 0; i < ctx->page_size; i += 2)
  {
    int data_pos = ctx->page * ctx->page_size/2 + i/2;

    if (data_pos > ctx->num_data_values)
      return MXT_SUCCESS;

    ofs = data_pos;

    ctx->data_buf[ofs] = (ctx->page_buf[i+1] << 8) | ctx->page_buf[i];
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Insert page of data into buffer at appropriate co-ordinates
/// \return #mxt_rc
static int mxt_debug_insert_data(struct t37_ctx *ctx)
{
  int i;
  uint16_t value;
  int ofs;

  for (i = 0; i < ctx->page_size; i += 2)
  {
    if (ctx->x_ptr > ctx->x_size)
    {
      mxt_err(ctx->lc, "x pointer overrun");
      return MXT_INTERNAL_ERROR;
    }

    value = (ctx->page_buf[i+1] << 8) | ctx->page_buf[i];

    ofs = ctx->y_ptr + ctx->x_ptr * ctx->y_size;

    /* The last page may overlap the end of the matrix */
    if (ofs >= ctx->num_data_values)
      return MXT_SUCCESS;

    ctx->data_buf[ofs] = value;

    ctx->y_ptr++;

    if (ctx->y_ptr > ctx->stripe_endy)
    {
      ctx->y_ptr = ctx->stripe_starty;
      ctx->x_ptr++;
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Write data to file
/// \return #mxt_rc
static int mxt_hawkeye_output(struct t37_ctx *ctx)
{
  int x;
  int y;
  int ofs;
  int16_t value;
  int ret;

  ret = mxt_print_timestamp(ctx->hawkeye, false);
  if (ret)
    return ret;

  /* print frame number */
  ret = fprintf(ctx->hawkeye, ",%u,", ctx->frame);
  if (ret < 0)
    return MXT_ERROR_IO;

  if (ctx->self_cap)
  {
    for (y = 0; y < ctx->y_size; y++)
    {
      value = (int16_t)ctx->data_buf[y];
      ret = fprintf(ctx->hawkeye, "%d,", value);
      if (ret < 0)
        return MXT_ERROR_IO;
    }

    for (x = 0; x < ctx->x_size; x++)
    {
      value = (int16_t)ctx->data_buf[ctx->y_size + x];
      ret = fprintf(ctx->hawkeye, "%d,", value);
      if (ret < 0)
        return MXT_ERROR_IO;
    }
  }
  else
  {
    /* iterate through columns */
    for (x = 0; x < ctx->x_size; x++)
    {
      for (y = 0; y < ctx->y_size; y++)
      {
        ofs = y + x * ctx->y_size;

        value = (int16_t)ctx->data_buf[ofs];

        ret = fprintf(ctx->hawkeye, "%d,", value);
        if (ret < 0)
          return MXT_ERROR_IO;
      }
    }
  }

  ret = fprintf(ctx->hawkeye, "\n");
  if (ret < 0)
    return MXT_ERROR_IO;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Input number of frames
/// \return #mxt_rc
static int get_num_frames(uint16_t *frames)
{
  printf("Number of frames: ");

  if (scanf("%hu", frames) == EOF)
  {
    fprintf(stderr, "Could not handle the input, exiting");
    return MXT_ERROR_BAD_INPUT;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Initialise parameters and allocate buffers
/// \return #mxt_rc
int mxt_debug_dump_initialise(struct t37_ctx *ctx)
{
  struct mxt_id_info *id = ctx->mxt->info.id;
  int ret;

  ret = get_objects_addr(ctx);
  if (ret)
  {
    mxt_err(ctx->lc, "Failed to get object information");
    return ret;
  }

  mxt_dbg(ctx->lc, "t37_size: %d", ctx->t37_size);
  ctx->page_size = ctx->t37_size - 2;
  mxt_dbg(ctx->lc, "page_size: %d", ctx->page_size);

  switch (ctx->mode)
  {
    case DELTAS_MODE:
    case REFS_MODE:
      ctx->self_cap = false;

      if (id->family == 0xA0 && id->variant == 0x00)
      {
        /* mXT1386 data is formatted into stripes */
        ctx->x_size = 27;
        ctx->y_size = id->matrix_y_size;
        ctx->num_data_values = 27 * ctx->y_size;
        ctx->num_passes = 3;
        ctx->pages = 8;
      }
      else
      {
        ctx->x_size = id->matrix_x_size;
        ctx->y_size = id->matrix_y_size;
        ctx->num_data_values = ctx->x_size * ctx->y_size;
        ctx->num_passes = 1;
        ctx->pages = (ctx->num_data_values*2 + (ctx->page_size - 1)) /
                     ctx->page_size;
      }

      ctx->stripe_width = ctx->y_size / ctx->num_passes;
      mxt_dbg(ctx->lc, "stripe_width: %d", ctx->stripe_width);
      break;

    case SELF_CAP_SIGNALS:
    case SELF_CAP_DELTAS:
    case SELF_CAP_REFS:
      ctx->self_cap = true;

      if (id->family == 164)
      {
        switch (id->variant)
        {
          case 5:
          case 9:
            // mXT336T
            ctx->y_size = 14;
            ctx->x_size = 24;
            break;

          case 2:
          case 7:
            // mXT640T
            ctx->y_size = 20;
            ctx->x_size = 32;
            break;

          case 11:
          case 12:
            // mXT1066T
            ctx->y_size = 26;
            ctx->x_size = 41;
            break;

          case 3:
          case 4:
          case 13:
          case 14:
          case 15:
          case 16:
            // mXT2952T
            ctx->y_size = 72;
            ctx->x_size = 41;
            break;

          default:
            goto self_cap_unsupported;
        }
      }
      else
      {
        goto self_cap_unsupported;
      }

      if (ctx->t111_instances == 0)
      {
        goto self_cap_unsupported;
      }
      else
      {
        ctx->num_passes = ctx->t111_instances;
      }

      ctx->num_data_values = ctx->y_size * ((ctx->x_size > ctx->y_size) ? 3 : 2);
      ctx->pages = (ctx->num_data_values*2 +(ctx->page_size - 1)) /
                         ctx->page_size;
      break;

    default:
      mxt_err(ctx->lc, "Unsupported mode %02X", ctx->mode);
      return MXT_INTERNAL_ERROR;
  }

  mxt_dbg(ctx->lc, "Number of passes: %d", ctx->num_passes);
  mxt_dbg(ctx->lc, "Pages per pass: %d", ctx->pages);
  mxt_dbg(ctx->lc, "X size: %d", ctx->x_size);
  mxt_dbg(ctx->lc, "Y size: %d", ctx->y_size);
  mxt_dbg(ctx->lc, "Number of values: %d", ctx->num_data_values);

  /* allocate page/data buffers */
  ctx->page_buf = (uint8_t *)calloc(ctx->page_size, sizeof(uint8_t));
  if (!ctx->page_buf) {
    mxt_err(ctx->lc, "calloc failure");
    return MXT_ERROR_NO_MEM;
  }

  ctx->data_buf = (uint16_t *)calloc(ctx->num_data_values, sizeof(uint16_t));
  if (!ctx->data_buf) {
    mxt_err(ctx->lc, "calloc failure");
    free(ctx->page_buf);
    ctx->page_buf = NULL;
    return MXT_ERROR_NO_MEM;
  }

  memset(ctx->data_buf, 0, ctx->num_data_values * sizeof(uint16_t));

  return MXT_SUCCESS;

self_cap_unsupported:
  mxt_err(ctx->lc, "Self cap data not available");
  return MXT_ERROR_NOT_SUPPORTED;
}

//******************************************************************************
/// \brief Read one frame of diagnostic data
/// \return #mxt_rc
int mxt_debug_dump_frame(struct t37_ctx* ctx)
{
  int page;
  int ret;

  /* iterate through stripes */
  for (ctx->pass = 0; ctx->pass < ctx->num_passes; ctx->pass++)
  {
    if (!ctx->self_cap)
    {
      /* Calculate stripe parameters */
      ctx->stripe_starty = ctx->stripe_width * ctx->pass;
      ctx->stripe_endy = ctx->stripe_starty + ctx->stripe_width - 1;
      ctx->x_ptr = 0;
      ctx->y_ptr = ctx->stripe_starty;
    }
    else
    {
      ctx->x_ptr = 0;
      ctx->y_ptr = 0;
    }

    for (page = 0; page < ctx->pages; page++)
    {
      ctx->page = ctx->pages * ctx->pass + page;

      mxt_dbg(ctx->lc, "Frame %d Pass %d Page %d", ctx->frame, ctx->pass,
          ctx->page);

      ret = mxt_debug_dump_page(ctx);
      if (ret)
        return ret;

      if (ctx->self_cap)
        mxt_debug_insert_data_self_cap(ctx);
      else
        mxt_debug_insert_data(ctx);
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Retrieve data from the T37 Diagnostic Data object
/// \return #mxt_rc
int mxt_debug_dump(struct mxt_device *mxt, int mode, const char *csv_file,
                   uint16_t frames)
{
  struct t37_ctx ctx;
  time_t t1;
  time_t t2;
  int ret;

  ctx.lc = mxt->ctx;
  ctx.mxt = mxt;
  ctx.mode = mode;

  if (frames == 0)
  {
     mxt_warn(ctx.lc, "Defaulting to 1 frame");
     frames = 1;
  }

  ret = mxt_debug_dump_initialise(&ctx);
  if (ret)
    return ret;

  /* Open Hawkeye output file */
  ctx.hawkeye = fopen(csv_file,"w");
  if (!ctx.hawkeye) {
    mxt_err(ctx.lc, "Failed to open file!");
    ret = -1;
    goto free;
  }

  ret = mxt_generate_hawkeye_header(&ctx);
  if (ret)
    return ret;

  mxt_info(ctx.lc, "Reading %u frames", frames);

  t1 = time(NULL);

  for (ctx.frame = 1; ctx.frame <= frames; ctx.frame++)
  {
    ret = mxt_debug_dump_frame(&ctx);

    mxt_hawkeye_output(&ctx);
  }

  fclose(ctx.hawkeye);

  t2 = time(NULL);
  mxt_info(ctx.lc, "%u frames in %d seconds", frames, (int)(t2-t1));

  ret = MXT_SUCCESS;

free:
  free(ctx.data_buf);
  ctx.data_buf = NULL;
  free(ctx.page_buf);
  ctx.page_buf = NULL;

  return ret;
}

//******************************************************************************
/// \brief Handle menu input for diagnostic data functions
static void mxt_dd_cmd(struct mxt_device *mxt, char selection, const char *csv_file)
{
  uint16_t frames;
  int ret;

  switch (selection)
  {
    case 'd':
    case 'D':
      ret = get_num_frames(&frames);
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, DELTAS_MODE, csv_file, frames);
      break;
    case 'r':
    case 'R':
      ret = get_num_frames(&frames);
      if (ret == MXT_SUCCESS)
        mxt_debug_dump(mxt, REFS_MODE, csv_file, frames);
      break;
    case 'q':
    case 'Q':
      printf("Quitting the debug dump utility\n");
      break;
    default:
      printf("Invalid menu option\n");
      break;
  }
}

//******************************************************************************
/// \brief Menu interface for diagonistic data functions
void mxt_dd_menu(struct mxt_device *mxt)
{
  char menu_input;
  char csv_file_in[MAX_FILENAME_LENGTH + 1];

  while(true)
  {
    printf("\nSelect one of the options:\n\n"
        "Enter D:   (D)elta dump\n"
        "Enter R:   (R)eference dump\n"
        "Enter Q:   (Q)uit the application\n");

    if (scanf("%1s", &menu_input) == EOF)
    {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }

    if ((menu_input == 'q') || (menu_input == 'Q'))
    {
      printf("Quitting the debug dump utility\n");
      return;
    }

    printf("\nFile name: ");
    if (scanf("%255s", csv_file_in) == EOF)
    {
      fprintf(stderr, "Could not handle the input, exiting");
      return;
    }

    mxt_dd_cmd(mxt, menu_input, csv_file_in);
  }
}
