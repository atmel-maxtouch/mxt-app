//------------------------------------------------------------------------------
/// \file   broken_line.c
/// \brief  Broken line detection
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2014 Atmel Corporation. All rights reserved.
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
#include <math.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "broken_line.h"
#include "mxt_app.h"

//******************************************************************************
/// \brief Set T8.CHRGTIME
/// \return #mxt_rc
static int set_chrgtime(struct mxt_device *mxt, struct broken_line_options *bl_opts)
{
  int ret;
  uint8_t val;
  uint16_t t8_addr;

  mxt_info(mxt->ctx, "Changing acquisition parameters");

  t8_addr = mxt_get_object_address(mxt, GEN_ACQUISITIONCONFIG_T8, 0);
  if (!t8_addr)
    return OBJECT_NOT_FOUND;

  ret = mxt_read_register(mxt, &val, t8_addr, 1);
  if (ret)
    return ret;

  if (bl_opts->pattern == BROKEN_LINE_PATTERN_ITO) {
    val = (uint8_t)((float)val * 1.5F);
  } else {
    val /= 2;
  }

  return mxt_write_register(mxt, &val, t8_addr, 1);
}

//*****************************************************************************
/// \brief Perform defect scan for one axis
/// \return Number of failing lines.
static int micro_defect_scan(struct libmaxtouch_ctx *ctx,
                             float *data,
                             float thr_border,
                             float thr_center,
                             int origin,
                             int last,
                             char axis)
{
  int num_failed = 0;

  mxt_dbg(ctx, "Scanning %c axis thr_border:%f thr_center:%f",
          axis, thr_border, thr_center);

  // Check border
  if (fabs(data[origin]) > thr_border) {
    mxt_err(ctx, "FAIL: Border broken line detection test failed. Line: %c%d, Value: %f",
            axis, origin, fabs(data[0]));
    num_failed++;
  }

  if (fabs(data[last]) > thr_border) {
    mxt_err(ctx, "FAIL: Border broken line detection test failed. Line: %c%d, Value: %f",
            axis, last, fabs(data[last]));
    num_failed++;
  }

  // Check center area
  int i;
  for (i = origin + 1; i < last; i++) {
    if (fabs(data[i]) > thr_center) {
      mxt_err(ctx, "FAIL: Broken line detection test failed. Line: %c%d Val: %f",
              axis, i, fabs(data[i]));
      num_failed++;
    }
  }

  return num_failed;
}

//******************************************************************************
/// \brief Perform broken line calculation
/// \return #mxt_rc
static int broken_line_calc(struct t37_ctx *frame,
                            struct mxt_touchscreen_info *ts,
                            struct broken_line_options *bl_opts)
{
  uint16_t last_x = ts->xorigin + (ts->xsize - (bl_opts->dualx ? 2 : 1));
  uint16_t last_y = ts->yorigin + (ts->ysize - 1);

  mxt_dbg(frame->lc, "debug frame: x_size: %d, y_size: %d",frame->x_size,frame->y_size);
  mxt_dbg(frame->lc, "TS xorigin: %d, yorigin: %d", ts->xorigin, ts->yorigin);
  mxt_dbg(frame->lc, "TS size: xsize: %d, ysize: %d",ts->xsize,ts->ysize);
  mxt_dbg(frame->lc, "last_x: %d, last_y %d", last_x, last_y);

  float Average_X[frame->x_size];
  float Average_Y[frame->y_size];
  float MicroDiff_X[frame->x_size - 1];
  float MicroDiff_Y[frame->y_size - 1];
  int num_failed = 0;
  uint16_t x, y, num_points;
  float average, diff;

  // Calculate X averages
  for (x = ts->xorigin; x <= last_x; x++) {
    average = 0;
    num_points = 0;

    // Skip using border points
    for (y = ts->yorigin + 1; y <= last_y - 1; y++) {
      average += abs(get_value(frame, x, y));
      num_points++;
    }

    average /= num_points;
    Average_X[x] = reference_no_offset(average);
    mxt_dbg(frame->lc, "Average_X[%d]: %f", x, Average_X[x]);
  }

  for (x = ts->xorigin; x <= last_x - 1; x++) {
    average = (Average_X[x] + Average_X[x + 1]) / 2.0;

    if (average <= 0)
      diff = 0;
    else
      diff = (Average_X[x] - Average_X[x + 1]) / average;

    MicroDiff_X[x] = diff * 100;
    mxt_dbg(frame->lc, "MicroDiff_X[%d]: %f", x, MicroDiff_X[x]);
  }

  num_failed += micro_defect_scan(frame->lc, MicroDiff_X,
                                  bl_opts->x_border_threshold,
                                  bl_opts->x_center_threshold,
                                  ts->xorigin, last_x - 1, 'X');

  // Calculate Y averages
  for (y = ts->yorigin; y <= last_y; y++) {
    num_points = 0;
    average = 0;

    // Skip using border points
    for (x = ts->xorigin + 1; x <= last_x - 1; x++) {
      average += abs(get_value(frame, x, y));
      num_points++;
    }

    average /= num_points;
    Average_Y[y] = reference_no_offset(average);
    mxt_dbg(frame->lc, "Average_Y[%d]: %f", y, Average_Y[y]);
  }

  for (y = ts->yorigin; y <= last_y - 1; y++) {
    average = (Average_Y[y] + Average_Y[y + 1]) / 2.0;

    if (average <= 0)
      diff = 0;
    else
      diff = (Average_Y[y] - Average_Y[y + 1]) / average;

    MicroDiff_Y[y] = diff * 100;
    mxt_dbg(frame->lc, "MicroDiff_Y[%d]: %f", y, MicroDiff_Y[y]);
  }

  num_failed += micro_defect_scan(frame->lc, MicroDiff_Y,
                                  bl_opts->y_border_threshold,
                                  bl_opts->y_center_threshold,
                                  ts->yorigin, last_y - 1, 'Y');

  if (num_failed > 0) {
    mxt_err(frame->lc, "FAIL: Broken line(s) detected");
    return MXT_BROKEN_LINE_DETECTED;
  } else {
    mxt_info(frame->lc, "PASS: Broken lines not detected");
    return MXT_SUCCESS;
  }
}

//******************************************************************************
/// \brief Run broken line detection algorithm
/// \return #mxt_rc
int mxt_broken_line(struct mxt_device *mxt, struct broken_line_options *bl_opts)
{
  int ret;
  struct mxt_touchscreen_info *mxt_ts_info = NULL;

  ret = mxt_disable_touch(mxt);
  if (ret)
    return ret;

  ret = set_chrgtime(mxt, bl_opts);
  if (ret)
    return ret;

  ret = disable_gr(mxt);
  if (ret || ret != OBJECT_NOT_FOUND)
    return ret;

  ret =  mxt_free_run_mode(mxt);
  if (ret)
    return ret;

  ret = mxt_calibrate_chip(mxt);
  if (ret)
    return ret;

  mxt_info(mxt->ctx, "Acquiring one frame of reference data");
  struct t37_ctx frame = {0};
  frame.mxt = mxt;
  frame.lc = mxt->ctx;
  frame.mode = REFS_MODE;

  ret = mxt_debug_dump_initialise(mxt, &frame);
  if (ret)
    return ret;

  ret = mxt_read_diagnostic_data_frame(mxt, &frame);
  if (ret)
    goto free;

  debug_frame(&frame);

  ret = mxt_read_touchscreen_info(mxt, &mxt_ts_info);
  if (ret)
    goto free;

  if (mxt_ts_info)
    ret = broken_line_calc(&frame, mxt_ts_info, bl_opts);

  mxt_info(frame.lc, "Resetting device");
  if (mxt_reset_chip(mxt, false, 0))
    mxt_err(frame.lc, "Unable to reset device");

  ret = MXT_SUCCESS;

free:
  free(mxt_ts_info);
  mxt_ts_info = NULL;
  free(frame.data_buf);
  frame.data_buf = NULL;
  free(frame.t37_buf);
  frame.t37_buf = NULL;

  return ret;
}
