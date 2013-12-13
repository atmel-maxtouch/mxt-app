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

/* GEN_COMMANDPROCESSOR_T6 Register offsets from T6 base address */
#define MXT_CP_T6_RESET_OFFSET      0x00
#define MXT_CP_T6_BACKUPNV_OFFSET   0x01
#define MXT_CP_T6_CALIBRATE_OFFSET  0x02
#define MXT_CP_T6_REPORTALL_OFFSET  0x03
#define MXT_CP_T6_RESERVED_OFFSET   0x04
#define MXT_CP_T6_DIAGNOSTIC_OFFSET 0x05

#define MAX_FILENAME_LENGTH     255

//******************************************************************************
/// \brief T37 Diagnostic Data context object
struct t37_ctx {
  struct mxt_device *mxt;
  struct libmaxtouch_ctx *lc;

  int x_size;
  int y_size;

  int num_stripes;
  int stripe_width;
  int stripe_starty;
  int stripe_endy;
  uint8_t page_size;
  uint8_t mode;

  int diag_cmd_addr;
  int t37_addr;
  int t37_size;

  uint16_t frame;
  int stripe;
  int page;
  int x_ptr;
  int y_ptr;

  uint8_t *page_buf;
  uint16_t *data_buf;

  FILE *hawkeye;
};

//******************************************************************************
/// \brief Retrieve and store object addresses for debug data operation
/// \return #mxt_rc
static int get_objects_addr(struct t37_ctx *ctx)
{
  int t6_addr;

  /* Obtain command processor's address */
  t6_addr = mxt_get_object_address(ctx->mxt, GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* T37 commands address */
  ctx->diag_cmd_addr = t6_addr + MXT_CP_T6_DIAGNOSTIC_OFFSET;

  /* Obtain Debug Diagnostic object's address */
  ctx->t37_addr = mxt_get_object_address(ctx->mxt, DEBUG_DIAGNOSTIC_T37, 0);
  if (ctx->t37_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Obtain Debug Diagnostic object's size */
  ctx->t37_size = mxt_get_object_size(ctx->mxt, DEBUG_DIAGNOSTIC_T37);
  if (ctx->t37_size == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

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

  /* Read back diagnostic register in T6 command processor until is has been
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
static void mxt_generate_hawkeye_header(struct t37_ctx *ctx)
{
  int x;
  int y;

  fprintf(ctx->hawkeye, "Frame,");

  for (x = 0; x < ctx->x_size; x++)
  {
    for (y = 0; y < ctx->y_size; y++)
    {
      fprintf(ctx->hawkeye, "X%dY%d_%s16,", x, y,
        (ctx->mode == DELTAS_MODE) ? "Delta" : "Reference");
    }
  }

  fprintf(ctx->hawkeye, "\n");
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
    if (ofs >= (ctx->x_size * ctx->y_size))
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

#if 0
//******************************************************************************
/// \brief Output hawkeye data to terminal
/// \return #mxt_rc
static int mxt_debug_print(struct t37_ctx *ctx)
{
  int x;
  int y;
  int ofs;
  int16_t value;

  /* clear screen */
  printf("\e[1;1H\e[2J");

  for (x = 0; x < ctx->x_size; x++)
  {
    for (y = 0; y < ctx->y_size; y++)
    {
      ofs = y + x * ctx->y_size;

      value = (int16_t)ctx->data_buf[ofs];

      printf("%6d ", value);

    }
    printf("\n");
  }

  return MXT_SUCCESS;
}
#endif

//******************************************************************************
/// \brief Generate hawkeye control file
static void mxt_hawkeye_generate_control_file(struct t37_ctx *ctx)
{
  int x;
  int y;
  FILE *fp;

  fp = fopen("control.txt","w");
  if (!fp) {
    mxt_warn(ctx->lc, "Failed to save control.txt!");
    return;
  }

  fprintf(fp, "uint16_lsb_msb,1,1,FRAME\n");

  for (x = 0; x < ctx->x_size; x++)
  {
    for (y = 0; y < ctx->y_size; y++)
    {
      fprintf(fp, "int16_lsb_msb,%d,%d,X%dY%d_%s16\n", y+1, x+3, x, y,
        (ctx->mode == DELTAS_MODE) ? "Delta" : "Reference");
    }
  }

  fclose(fp);
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

  ret = mxt_print_timestamp(ctx->hawkeye);
  if (ret)
    return ret;

  /* print frame number */
  ret = fprintf(ctx->hawkeye, ",%u,", ctx->frame);
  if (ret < 0)
    return MXT_ERROR_IO;

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
/// \brief Retrieve data from the T37 Diagnostic Data object
/// \return #mxt_rc
int mxt_debug_dump(struct mxt_device *mxt, int mode, const char *csv_file,
                   uint16_t frames)
{
  struct t37_ctx ctx;
  int x_size, y_size;
  int pages_per_stripe;
  int num_stripes;
  int num_debug_bytes;
  int ret;
  int page;
  time_t t1;
  time_t t2;

  ctx.lc = mxt->ctx;
  ctx.mxt = mxt;

  if (frames == 0)
  {
     mxt_warn(ctx.lc, "Defaulting to 1");
     frames = 1;
  }

  x_size = mxt->info.id->matrix_x_size;
  y_size = mxt->info.id->matrix_y_size;

  ctx.mode = mode;

  ret = get_objects_addr(&ctx);
  if (ret)
  {
    mxt_err(ctx.lc, "Failed to get object information");
    return ret;
  }

  mxt_dbg(ctx.lc, "t37_size: %d", ctx.t37_size);
  ctx.page_size = ctx.t37_size - 2;
  mxt_dbg(ctx.lc, "page_size: %d", ctx.page_size);

  if (mxt->info.id->family == 0xA0 && mxt->info.id->variant == 0x00)
  {
    /* mXT1386 */
    num_stripes = 3;
    pages_per_stripe = 8;
    ctx.x_size = 27;
  }
  else
  {
    num_debug_bytes = x_size * y_size * 2;
    mxt_dbg(ctx.lc, "num_debug_bytes: %d", num_debug_bytes);

    num_stripes = 1;
    pages_per_stripe = (num_debug_bytes + (ctx.page_size - 1)) / ctx.page_size;
    ctx.x_size = x_size;
  }

  ctx.num_stripes = num_stripes;
  ctx.stripe_width = y_size / num_stripes;
  ctx.y_size = y_size;

  mxt_dbg(ctx.lc, "Number of stripes: %d", num_stripes);
  mxt_dbg(ctx.lc, "Pages per stripe: %d", pages_per_stripe);
  mxt_dbg(ctx.lc, "Stripe width: %d", ctx.stripe_width);
  mxt_dbg(ctx.lc, "X size: %d", ctx.x_size);
  mxt_dbg(ctx.lc, "Y size: %d", ctx.y_size);

  /* allocate page/data buffers */
  ctx.page_buf = (uint8_t *)calloc(ctx.page_size, sizeof(uint8_t));
  if (!ctx.page_buf) {
    mxt_err(ctx.lc, "calloc failure");
    return MXT_ERROR_NO_MEM;
  }

  ctx.data_buf = (uint16_t *)calloc(ctx.x_size * ctx.y_size, sizeof(uint16_t));
  if (!ctx.data_buf) {
    mxt_err(ctx.lc, "calloc failure");
    ret = -1;
    goto free_page_buf;
  }

  /* Open Hawkeye output file */
  ctx.hawkeye = fopen(csv_file,"w");
  if (!ctx.hawkeye) {
    mxt_err(ctx.lc, "Failed to open file!");
    ret = -1;
    goto free;
  }

  mxt_generate_hawkeye_header(&ctx);

  mxt_info(ctx.lc, "Reading %u frames", frames);

  t1 = time(NULL);

  for (ctx.frame = 1; ctx.frame <= frames; ctx.frame++)
  {
    /* iterate through stripes */
    for (ctx.stripe = 0; ctx.stripe < num_stripes; ctx.stripe++)
    {
      /* Select stripe */
      ctx.stripe_starty = ctx.stripe_width * ctx.stripe;
      ctx.stripe_endy = ctx.stripe_starty + ctx.stripe_width - 1;
      ctx.x_ptr = 0;
      ctx.y_ptr = ctx.stripe_starty;

      for (page = 0; page < pages_per_stripe; page++)
      {
        ctx.page = pages_per_stripe * ctx.stripe + page;

        mxt_dbg(ctx.lc, "Frame %d Stripe %d Page %d",
            ctx.frame, ctx.stripe, ctx.page);

        ret = mxt_debug_dump_page(&ctx);
        if (ret < 0)
          goto free;

        mxt_debug_insert_data(&ctx);
      }
    }

    mxt_hawkeye_output(&ctx);
  }

  fclose(ctx.hawkeye);

  mxt_hawkeye_generate_control_file(&ctx);

  t2 = time(NULL);
  mxt_info(ctx.lc, "%u frames in %d seconds", frames, (int)(t2-t1));

  ret = MXT_SUCCESS;

free:
  free(ctx.data_buf);
  ctx.data_buf = NULL;
free_page_buf:
  free(ctx.page_buf);
  ctx.page_buf = NULL;

  return ret;
}

//******************************************************************************
/// \brief Handle menu input for diagonistic data functions
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
