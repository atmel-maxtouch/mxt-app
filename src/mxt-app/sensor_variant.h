#pragma once
//------------------------------------------------------------------------------
/// \file   sensor_variant.h
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
#define UPPER_LIMIT   15
#define LOWER_LIMIT   15
#define POLY_DEGREE   2

struct mxt_device;
struct libmaxtouch_ctx;
struct mxt_touchscreen_info;
struct t37_ctx;

//******************************************************************************
/// \brief Sensor Variant algorithm context options
//     sensor_variant_options
struct sensor_variant_options
{
  bool dualx;
  uint32_t max_defects;
  uint8_t matrix_size;
  uint8_t upper_limit;
  uint8_t lower_limit;
};

int check_sub_matrix(struct t37_ctx *ctx, bool *status, int x_size, int y_size, struct sensor_variant_options *sv_opts);
int sensor_variant_algorithm(struct t37_ctx *frame, struct mxt_touchscreen_info *ts, struct sensor_variant_options *sv_opts);
int validate_sensor_variant_options(struct mxt_device *mxt, struct sensor_variant_options *sv_opts);
int mxt_sensor_variant(struct mxt_device *mxt, struct sensor_variant_options *sv_opts);
int calculate_poly(double *data, double *coeff, int len, double *result);
int check_line(struct libmaxtouch_ctx *ctx, struct sensor_variant_options *sv_opts, double *xval, double *yval, int len, uint32_t *num_failed, bool *status, double *coeff);
double ft_peval(double x, double *coeffs);
bool ft_polyfit(struct libmaxtouch_ctx *ctx, double *xdata, double *ydata, double *result, int len);
int get_xline_data(struct t37_ctx *frame, uint16_t x, uint16_t ysize, double *yval);
int get_yline_data(struct t37_ctx *frame, uint16_t y, uint16_t xsize, double *xval);
