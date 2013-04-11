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
#include <ctype.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/log.h"

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

static int get_objects_addr(struct t37_ctx *mxt_dd)
{
  int t6_addr;

  /* Obtain command processor's address */
  t6_addr = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
  if (t6_addr == OBJECT_NOT_FOUND) return -1;

  /* T37 commands address */
  mxt_dd->diag_cmd_addr = t6_addr + MXT_CP_T6_DIAGNOSTIC_OFFSET;

  /* Obtain Debug Diagnostic object's address */
  mxt_dd->t37_addr = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
  if (mxt_dd->t37_addr == OBJECT_NOT_FOUND) return -1;

  /* Obtain Debug Diagnostic object's size */
  mxt_dd->t37_size = get_object_size(DEBUG_DIAGNOSTIC_T37);
  if (mxt_dd->t37_size == OBJECT_NOT_FOUND) return -1;

  return 0;
}

static int mxt_debug_dump_page(struct t37_ctx *mxt_dd)
{
  int failures;
  int ret;
  uint8_t read_command = 1;
  uint8_t read_mode;
  uint8_t read_page;
  uint8_t page_up_cmd = PAGE_UP;

  if (mxt_dd->page == 0)
  {
    LOG(LOG_VERBOSE, "Writing mode command");
    mxt_write_register(&mxt_dd->mode, mxt_dd->diag_cmd_addr, 1);
  }
  else
  {
    mxt_write_register(&page_up_cmd, mxt_dd->diag_cmd_addr, 1);
  }

  /* Read back diagnostic register in T6 command processor until is has been
   * cleared. This means that the chip has actioned the command */
  failures = 0;

  while (read_command)
  {
    ret = mxt_read_register(&read_command, mxt_dd->diag_cmd_addr, 1);
    if (ret < 0)
    {
      LOG(LOG_ERROR, "Failed to read the status of diagnostic mode command");
      return -1;
    }

    if (read_command)
    {
      failures++;

      if (failures > 500)
      {
        LOG(LOG_ERROR, "Timeout waiting for command to be actioned");
        printf("Timeout\n");
        return -1;
      }
    }
  }

  ret = mxt_read_register(&read_mode, mxt_dd->t37_addr, 1);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read current mode");
    return -1;
  }

  ret = mxt_read_register(&read_page, mxt_dd->t37_addr + 1, 1);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read page number");
    return -1;
  }

  if ((read_mode != mxt_dd->mode) || (read_page != mxt_dd->page))
  {
    LOG(LOG_ERROR, "Bad page/mode in diagnostic data read");
    return -1;
  }

  ret = mxt_read_register(mxt_dd->page_buf,
                          mxt_dd->t37_addr + 2, mxt_dd->page_size);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read page");
    return -1;
  }

  return 0;
}

static void mxt_generate_hawkeye_header(struct t37_ctx *mxt_dd)
{
  int x;
  int y;

  fprintf(mxt_dd->hawkeye, "Frame,");

  for (x = 0; x < mxt_dd->x_size; x++)
  {
    for (y = 0; y < mxt_dd->y_size; y++)
    {
      fprintf(mxt_dd->hawkeye, "X%dY%d_%s16,", x, y,
        (mxt_dd->mode == DELTAS_MODE) ? "Delta" : "Reference");
    }
  }

  fprintf(mxt_dd->hawkeye, "\n");
}

static int mxt_debug_insert_data(struct t37_ctx *mxt_dd)
{
  int i;
  uint16_t value;
  int ofs;

  for (i = 0; i < mxt_dd->page_size; i += 2)
  {
    if (mxt_dd->x_ptr > mxt_dd->x_size)
    {
      LOG(LOG_ERROR, "x pointer overrun");
      return -1;
    }

    value = (mxt_dd->page_buf[i+1] << 8) | mxt_dd->page_buf[i];

    ofs = mxt_dd->y_ptr + mxt_dd->x_ptr * mxt_dd->y_size;

    /* The last page may overlap the end of the matrix */
    if (ofs >= (mxt_dd->x_size * mxt_dd->y_size))
      return 0;

    mxt_dd->data_buf[ofs] = value;

    mxt_dd->y_ptr++;

    if (mxt_dd->y_ptr > mxt_dd->stripe_endy)
    {
      mxt_dd->y_ptr = mxt_dd->stripe_starty;
      mxt_dd->x_ptr++;
    }
  }

  return 0;
}

#if 0
static int mxt_debug_print(struct t37_ctx *mxt_dd)
{
  int x;
  int y;
  int ofs;
  int16_t value;

  /* clear screen */
  printf("\e[1;1H\e[2J");

  for (x = 0; x < mxt_dd->x_size; x++)
  {
    for (y = 0; y < mxt_dd->y_size; y++)
    {
      ofs = y + x * mxt_dd->y_size;

      value = (int16_t)mxt_dd->data_buf[ofs];

      printf("%6d ", value);

    }
    printf("\n");
  }

  return 0;
}
#endif

static void mxt_print_timestamp(struct t37_ctx *mxt_dd)
{
  struct timeval tv;
  time_t nowtime;
  struct tm *nowtm;
  char tmbuf[64];

  gettimeofday(&tv, NULL);
  nowtime = tv.tv_sec;
  nowtm = localtime(&nowtime);
  strftime(tmbuf, sizeof tmbuf, "%H:%M:%S", nowtm);

  fprintf(mxt_dd->hawkeye, "%s.%06ld,", tmbuf, tv.tv_usec);
}

static void mxt_hawkeye_generate_control_file(struct t37_ctx *mxt_dd)
{
  int x;
  int y;
  FILE *fp;

  fp = fopen("control.txt","w");
  if (!fp) {
    LOG(LOG_ERROR, "Failed to open file!");
    return;
  }

  fprintf(fp, "uint16_lsb_msb,1,1,FRAME\n");

  for (x = 0; x < mxt_dd->x_size; x++)
  {
    for (y = 0; y < mxt_dd->y_size; y++)
    {
      fprintf(fp, "int16_lsb_msb,%d,%d,X%dY%d_%s16\n", y+1, x+3, x, y,
        (mxt_dd->mode == DELTAS_MODE) ? "Delta" : "Reference");
    }
  }

  fclose(fp);
}

static int mxt_hawkeye_output(struct t37_ctx *mxt_dd)
{
  int x;
  int y;
  int ofs;
  int16_t value;

  mxt_print_timestamp(mxt_dd);

  /* print frame number */
  fprintf(mxt_dd->hawkeye, "%u,", mxt_dd->frame);

  /* iterate through columns */
  for (x = 0; x < mxt_dd->x_size; x++)
  {
    for (y = 0; y < mxt_dd->y_size; y++)
    {
      ofs = y + x * mxt_dd->y_size;

      value = (int16_t)mxt_dd->data_buf[ofs];

      fprintf(mxt_dd->hawkeye, "%d,", value);
    }
  }
  fprintf(mxt_dd->hawkeye, "\n");

  return 0;
}

static uint16_t get_num_frames(void)
{
  uint16_t frames;

  printf("Number of frames: ");

  if (scanf("%hu", &frames) == EOF)
  {
    LOG(LOG_ERROR, "Could not handle the input, exiting");
    return -1;
  }

  return frames;
}

int mxt_debug_dump(int mode, const char *csv_file,
                          uint16_t frames)
{
  struct t37_ctx mxt_dd;
  int x_size, y_size;
  int pages_per_stripe;
  int num_stripes;
  int num_debug_bytes;
  int ret;
  int page;
  time_t t1;
  time_t t2;

  if (frames == 0)
  {
     LOG(LOG_WARN, "Defaulting to 1");
     frames = 1;
  }

  x_size = info_block.id->matrix_x_size;
  y_size = info_block.id->matrix_y_size;

  mxt_dd.mode = mode;

  ret = get_objects_addr(&mxt_dd);
  if (ret)
  {
    LOG(LOG_ERROR, "Failed to get object information");
    return -1;
  }

  LOG(LOG_DEBUG, "t37_size: %d", mxt_dd.t37_size);
  mxt_dd.page_size = mxt_dd.t37_size - 2;
  LOG(LOG_DEBUG, "page_size: %d", mxt_dd.page_size);

  if (info_block.id->family_id == 0xA0 && info_block.id->variant_id == 0x00)
  {
    /* mXT1386 */
    num_stripes = 3;
    pages_per_stripe = 8;
    mxt_dd.x_size = 27;
  }
  else
  {
    num_debug_bytes = x_size * y_size * 2;
    LOG(LOG_DEBUG, "num_debug_bytes: %d", num_debug_bytes);

    num_stripes = 1;
    pages_per_stripe = (num_debug_bytes + (mxt_dd.page_size - 1)) / mxt_dd.page_size;
    mxt_dd.x_size = x_size;
  }

  mxt_dd.num_stripes = num_stripes;
  mxt_dd.stripe_width = y_size / num_stripes;
  mxt_dd.y_size = y_size;

  LOG(LOG_DEBUG, "Number of stripes: %d", num_stripes);
  LOG(LOG_DEBUG, "Pages per stripe: %d", pages_per_stripe);
  LOG(LOG_DEBUG, "Stripe width: %d", mxt_dd.stripe_width);
  LOG(LOG_DEBUG, "X size: %d", mxt_dd.x_size);
  LOG(LOG_DEBUG, "Y size: %d", mxt_dd.y_size);

  /* allocate page/data buffers */
  mxt_dd.page_buf = (uint8_t *)malloc(sizeof(uint8_t) * mxt_dd.page_size);
  if (!mxt_dd.page_buf) {
    LOG(LOG_ERROR, "malloc failure");
    return -1;
  }

  mxt_dd.data_buf = (uint16_t *)malloc(sizeof(uint16_t)
                                     * mxt_dd.x_size * mxt_dd.y_size);
  if (!mxt_dd.data_buf) {
    LOG(LOG_ERROR, "malloc failure");
    ret = -1;
    goto free_page_buf;
  }

  /* Open Hawkeye output file */
  mxt_dd.hawkeye = fopen(csv_file,"w");
  if (!mxt_dd.hawkeye) {
    printf("Failed to open file!\n");
    ret = -1;
    goto free;
  }

  mxt_generate_hawkeye_header(&mxt_dd);

  printf("Reading %u frames\n", frames);

  t1 = time(NULL);

  for (mxt_dd.frame = 1; mxt_dd.frame <= frames; mxt_dd.frame++)
  {
    /* iterate through stripes */
    for (mxt_dd.stripe = 0; mxt_dd.stripe < num_stripes; mxt_dd.stripe++)
    {
      /* Select stripe */
      mxt_dd.stripe_starty = mxt_dd.stripe_width * mxt_dd.stripe;
      mxt_dd.stripe_endy = mxt_dd.stripe_starty + mxt_dd.stripe_width - 1;
      mxt_dd.x_ptr = 0;
      mxt_dd.y_ptr = mxt_dd.stripe_starty;

      for (page = 0; page < pages_per_stripe; page++)
      {
        mxt_dd.page = pages_per_stripe * mxt_dd.stripe + page;

        LOG(LOG_DEBUG, "Stripe %d Page %d", mxt_dd.stripe, mxt_dd.page);

        ret = mxt_debug_dump_page(&mxt_dd);
        if (ret < 0)
        {
          LOG(LOG_ERROR, "Quitting...");
          goto free;
        }

        mxt_debug_insert_data(&mxt_dd);
      }
    }

    mxt_hawkeye_output(&mxt_dd);
  }

  fclose(mxt_dd.hawkeye);

  mxt_hawkeye_generate_control_file(&mxt_dd);

  t2 = time(NULL);
  printf("%u frames in %d seconds\n", frames, (int)(t2-t1));

  ret = 0;

free:
  free(mxt_dd.data_buf);
  mxt_dd.data_buf = NULL;
free_page_buf:
  free(mxt_dd.page_buf);
  mxt_dd.page_buf = NULL;

  return ret;
}

/*!
 * @brief
 * @return Zero.
 */
static void mxt_dd_cmd(char selection, const char *csv_file)
{
  uint16_t frames;

  switch (selection)
  {
    case 'd':
    case 'D':
      frames = get_num_frames();
      mxt_debug_dump(DELTAS_MODE, csv_file, frames);
      break;
    case 'r':
    case 'R':
      frames = get_num_frames();
      mxt_debug_dump(REFS_MODE, csv_file, frames);
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

/*!
 * @brief  Menu function for the debug dump utility.
 * @return Zero.
 */
void mxt_dd_menu(void)
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
      LOG(LOG_ERROR, "Could not handle the input, exiting");
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
      LOG(LOG_ERROR, "Could not handle the input, exiting");
      return;
    }

    mxt_dd_cmd(menu_input, csv_file_in);
  }
}
