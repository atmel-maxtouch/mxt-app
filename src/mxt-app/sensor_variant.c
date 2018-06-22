//------------------------------------------------------------------------------
/// \file   sensor_variant.c
/// \brief  Sensor variant algorithm
/// \author Steven Swann
//------------------------------------------------------------------------------
// Copyright 2016 Atmel Corporation. All rights reserved.
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
#include <stdbool.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"
#include "sensor_variant.h"

//******************************************************************************
/// \brief Calculate polynomial from coefficients
/// \return #mxt_rc
int calculate_poly(double *data, double *coeff, int len, double *result)
{
  int i;

  for (i = 0; i < len; i++)
    result[i] = ft_peval(data[i], coeff);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Detect failures
/// \return number of failures
static int failure_scan(struct libmaxtouch_ctx *ctx, double *result,
                        double *data, int len, bool *status,
                        struct sensor_variant_options *sv_opts)
{
  int i;
  uint32_t failures = 0;

  /* test dataset */
  for (i = 0; i < len; i++) {
    double diff = result[i] - data[i];
    double percent = (diff / result[i]) * 100;

    /* test result */
    if (percent > sv_opts->upper_limit) {
      status[i] = true;
      failures++;
      mxt_dbg(ctx,
          "  [%d] actual:%0.2f calc:%0.2f diff:%0.2f, %0.1f%% greater than upper limit.",
          i, data[i], result[i], diff, percent);
    } else if (percent < (-1 * sv_opts->lower_limit)) {
      status[i] = true;
      failures++;
      mxt_dbg(ctx,
          "  [%d] actual:%0.2f calc:%0.2f diff:%0.2f, %0.1f%% less than lower limit.",
          i, data[i], result[i], diff, percent);
    } else {
      status[i] = false;
      mxt_verb(ctx, "  [%d] actual:%0.2f calc:%0.2f diff:%0.2f, %0.1f%%",
               i, data[i], result[i], diff, percent);
    }
  }

  return failures;
}

//******************************************************************************
/// \brief Fit polynominal and test single line
/// \return #mxt_rc
int check_line(struct libmaxtouch_ctx *ctx, struct sensor_variant_options *sv_opts,
                      double *xval, double *yval, int len, uint32_t *total_failed,
                      bool *status, double *coeff)
{
  bool pass;

  /* Find polynomial fit function */
  pass = ft_polyfit(ctx, xval, yval, coeff, len);
  if (!pass) {
    mxt_err(ctx, "  Error: Failed to fit polynomial");
    return MXT_INTERNAL_ERROR;
  }

  double *result = calloc(len, sizeof(double));
  if (!result)
    return MXT_ERROR_NO_MEM;

  calculate_poly(xval, coeff, len, result);

  *total_failed = failure_scan(ctx, result, yval, len, status, sv_opts);

  free(result);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Check for sub-matrix failures
/// \return number of failures detected
int check_sub_matrix(struct t37_ctx *ctx, bool *status, int x_size,
                     int y_size, struct sensor_variant_options *sv_opts)
{
  int x, y, tx, ty;
  uint32_t failures = 0;
  uint8_t sub_mtx = sv_opts->matrix_size;

  bool (*stat)[y_size] = (bool (*)[y_size])status;

  /* create y dataset */
  for (x = 0; x < x_size; x++) {
    for (y = 0; y < y_size; y++) {
      if (stat[x][y]) {

        uint32_t defects = 0;
        /* we have a failed node */
        mxt_dbg(ctx->lc,"  Defect detected at X%dY%d", x, y);
        for (tx = (x - sub_mtx >= 0 ? x - sub_mtx : 0);
             tx <= x + sub_mtx && tx < x_size; tx++) {
          for (ty = (y - sub_mtx >= 0 ? y - sub_mtx : 0);
               ty <= y + sub_mtx && ty < y_size; ty++) {

            if (stat[tx][ty] && tx != x && ty != y) {
              /* only a fail if more than max_defects defects within sub-matrix */
              if (++defects > sv_opts->max_defects) {
                mxt_info(ctx->lc, "  Sub-matrix (%d, %d) failed", x, y);
                failures++;

                /* next */
                tx = x_size;
                ty = y_size;
              }
            }
          }
        }
      }
    }
  }

  mxt_info(ctx->lc, "  %d sub-matrix defects detected", failures);
  return failures;
}

//******************************************************************************
/// \brief Get single value from frame as double
static double get_frame_val(struct t37_ctx *frame, uint16_t x, uint16_t y)
{
  int ofs = (x * frame->y_size) + y;
  return (double)frame->data_buf[ofs];
}

//******************************************************************************
/// \brief Fetch diagnostic data as array of doubles
/// \return #mxt_rc
int get_xline_data(struct t37_ctx *frame, uint16_t x, uint16_t ysize,
                    double *yval)
{
  int y;

  /* create y dataset */
  for (y = 0; y < ysize; y++)
    yval[y] = get_frame_val(frame, x, y);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Fetch diagnostic data as array of doubles
/// \return #mxt_rc
int get_yline_data(struct t37_ctx *frame, uint16_t y, uint16_t xsize,
                    double *xval)
{
  int x;

  /* create y dataset */
  for (x = 0; x < xsize; x++)
    xval[x] = get_frame_val(frame, x, y);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Perform Sensor variant algorithm
/// \return #mxt_rc
int sensor_variant_algorithm(struct t37_ctx *frame,
                             struct mxt_touchscreen_info *ts,
                             struct sensor_variant_options *sv_opts)
{
  int ret;
  int x,y;
  uint32_t total_failed = 0;
  uint16_t num_x = ts->xsize;

  mxt_dbg(frame->lc, "debug frame: x_size: %d, y_size: %d",
          frame->x_size, frame->y_size);
  mxt_dbg(frame->lc, "TS xorigin: %d, yorigin: %d", ts->xorigin, ts->yorigin);
  mxt_dbg(frame->lc, "TS size: xsize: %d, ysize: %d", ts->xsize, ts->ysize);
  mxt_dbg(frame->lc, "Dual X lines: %s", (sv_opts->dualx ? "ON" : "OFF"));

  if (sv_opts->dualx)
    num_x--;

  /* Init matrix memory */
  double *xval = calloc(num_x, sizeof(double));
  if (!xval) {
    return MXT_ERROR_NO_MEM;
  }

  double *yval = calloc(ts->ysize, sizeof(double));
  if (!yval) {
    free(xval);
    return MXT_ERROR_NO_MEM;
  }

  bool *status = calloc(num_x * ts->ysize, sizeof(bool));
  if (!status) {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  /* Test X lines */
  for (x = 0; x < num_x; x++)
    xval[x] = (double)x;

  for (x = 0; x < num_x; x++) {
    uint32_t failures;
    double coeff[POLY_DEGREE + 1];
    get_xline_data(frame, x, ts->ysize, yval);

    ret = check_line(frame->lc, sv_opts, xval, yval, ts->ysize,
                     &failures, status + (x * ts->ysize), coeff);
    if (ret)
      goto free;

    mxt_dbg(frame->lc, "X%d coefficients (%0.2f,%0.2f,%0.2f) failures %d",
            x, coeff[0], coeff[1], coeff[2], failures);

    total_failed += failures;
  }

  /* Test Y lines */
  for (y = 0; y < ts->ysize; y++)
    yval[y] = (double)y;

  for (y = 0; y < ts->ysize; y++) {
    uint32_t failures;
    double coeff[POLY_DEGREE + 1];
    bool yline_status[num_x];
    get_yline_data(frame, y, num_x, xval);

    ret = check_line(frame->lc, sv_opts, yval, xval, num_x,
                     &failures, yline_status, coeff);
    if (ret)
      goto free;

    /* status array is not contiguous for y */
    for (x = 0; x < num_x; x++)
      if (yline_status[x])
        status[(x * ts->ysize) + y] = true;

    mxt_dbg(frame->lc, "Y%d coefficients (%0.2f,%0.2f,%0.2f) failures %d",
                y, coeff[0], coeff[1], coeff[2], failures);

    total_failed += failures;
  }

  /* check results */
  if (total_failed) {
    printf("Sensor Variant defects detected:\n");

    printf("    ");
    for (x = 0; x < num_x; x++) {
      printf("X%-3d", x);
    }
    printf("\n");

    for (y = 0; y < ts->ysize; y++) {
      printf("Y%-3d", y);
      for (x = 0; x < num_x; x++) {
        printf("%c   ", (status[(x * ts->ysize) +  y] ? 'O' : '-'));
      }
      printf("\n");
    }
  }

  if (sv_opts->matrix_size)
    total_failed = check_sub_matrix(frame, status, num_x, ts->ysize, sv_opts);

  if (total_failed > sv_opts->max_defects) {
    mxt_err(frame->lc, "FAIL: %d Sensor Variant issues detected", total_failed);
    ret = MXT_SENSOR_VARIANT_DETECTED;
  } else {
    mxt_info(frame->lc, "PASS: Sensor Variant issues not detected");
    ret = MXT_SUCCESS;
  }

free:
  free(xval);
  free(yval);
  free(status);

  return ret;
}

//******************************************************************************
/// \brief Validate options for sensor variant
/// \return #mxt_rc
int validate_sensor_variant_options(struct mxt_device *mxt,
                                    struct sensor_variant_options *sv_opts)
{
  /* check upper and lower limits */
  if (sv_opts->upper_limit > 100) {
    mxt_err(mxt->ctx, "Sensor variant upper limit greater than 100%%");
    return MXT_ERROR_BAD_INPUT;
  }

  if (sv_opts->lower_limit > 100) {
    mxt_err(mxt->ctx, "Sensor variant lower limit greater than 100%%");
    return MXT_ERROR_BAD_INPUT;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Run sensor variant algorithm
/// \return #mxt_rc
int mxt_sensor_variant(struct mxt_device *mxt, struct sensor_variant_options *sv_opts)
{
  int ret;
  struct mxt_touchscreen_info *mxt_ts_info = NULL;

  ret = validate_sensor_variant_options(mxt, sv_opts);
  if (ret)
    return ret;

  ret = mxt_disable_touch(mxt);
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

  struct t37_ctx *frame = calloc(1, sizeof(struct t37_ctx));
  if (!frame)
    return MXT_ERROR_NO_MEM;

  frame->mxt = mxt;
  frame->lc = mxt->ctx;
  frame->mode = REFS_MODE;

  ret = mxt_debug_dump_initialise(frame);
  if (ret)
    return ret;

  ret = mxt_read_diagnostic_data_frame(frame);
  if (ret)
    goto free;

  debug_frame(frame);

  ret = mxt_read_touchscreen_info(mxt, &mxt_ts_info);
  if (ret)
    goto free;

  if (mxt_ts_info)
    ret = sensor_variant_algorithm(frame, mxt_ts_info, sv_opts);
  if (ret)
    goto free;

  mxt_info(frame->lc, "Resetting device");
  if (mxt_reset_chip(mxt, false))
    mxt_err(frame->lc, "Unable to reset device");

  ret = MXT_SUCCESS;

free:
  free(mxt_ts_info);
  mxt_ts_info = NULL;
  free(frame->data_buf);
  frame->data_buf = NULL;
  free(frame->t37_buf);
  frame->t37_buf = NULL;
  free(frame);

  return ret;
}
