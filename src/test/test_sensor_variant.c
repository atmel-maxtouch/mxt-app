//------------------------------------------------------------------------------
/// \file   test_sensor_variant.c
/// \brief  Tests against sensor_variant.c
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
#include <stdlib.h>
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/info_block.h"

#include "mxt-app/mxt_app.h"

#include "mxt-app/sensor_variant.h"
#include "run_unit_tests.h"

/* Data captured from 874U */
uint16_t data_buf_pass[] = {
  8473, 8571, 8591, 8596, 8529, 8473, 8510, 8476, 8363, 8329, 8230, 8209, 8167, 8181, 8211, 8207, 8152, 8107, 8013, 7992, 8010, 8053, -32768,
  8256, 8462, 8497, 8494, 8440, 8414, 8467, 8456, 8376, 8372, 8317, 8274, 8186, 8177, 8186, 8166, 8095, 8059, 7970, 7931, 7922, 7826, -32768,
  8238, 8435, 8474, 8473, 8418, 8386, 8435, 8430, 8347, 8337, 8276, 8245, 8169, 8166, 8167, 8152, 8087, 8061, 7976, 7942, 7944, 7905, -32768,
  8200, 8416, 8469, 8484, 8418, 8376, 8424, 8418, 8334, 8322, 8271, 8236, 8168, 8163, 8179, 8173, 8109, 8089, 8007, 7982, 7969, 7903, -32768,
  8174, 8386, 8439, 8462, 8394, 8349, 8396, 8396, 8314, 8308, 8258, 8231, 8156, 8149, 8179, 8181, 8118, 8098, 8015, 8010, 8011, 7966, -32768,
  8165, 8355, 8406, 8432, 8365, 8308, 8366, 8376, 8295, 8295, 8241, 8211, 8139, 8141, 8169, 8173, 8116, 8114, 8028, 8019, 8033, 8053, -32768,
  8130, 8340, 8394, 8413, 8349, 8300, 8357, 8370, 8288, 8290, 8244, 8219, 8153, 8161, 8187, 8185, 8128, 8138, 8058, 8032, 8048, 8030, -32768,
  8104, 8305, 8356, 8375, 8317, 8277, 8324, 8331, 8253, 8265, 8220, 8199, 8132, 8143, 8172, 8171, 8118, 8136, 8062, 8043, 8060, 8066, -32768,
  8081, 8270, 8316, 8339, 8281, 8242, 8286, 8291, 8222, 8236, 8191, 8186, 8115, 8127, 8145, 8154, 8111, 8128, 8051, 8042, 8062, 8106, -32768,
  8029, 8255, 8301, 8325, 8262, 8223, 8264, 8279, 8209, 8221, 8185, 8180, 8108, 8127, 8152, 8156, 8108, 8130, 8051, 8040, 8049, 8034, -32768,
  8015, 8226, 8282, 8306, 8244, 8200, 8241, 8251, 8194, 8203, 8167, 8170, 8094, 8111, 8145, 8155, 8098, 8124, 8051, 8044, 8059, 8066, -32768,
  8004, 8203, 8262, 8292, 8223, 8176, 8212, 8237, 8183, 8191, 8149, 8152, 8081, 8106, 8141, 8151, 8100, 8122, 8051, 8050, 8070, 8095, -32768,
  7980, 8202, 8260, 8286, 8222, 8183, 8228, 8250, 8185, 8192, 8149, 8142, 8087, 8126, 8156, 8154, 8113, 8148, 8073, 8069, 8073, 8058, -32768,
  7970, 8183, 8241, 8272, 8212, 8185, 8238, 8251, 8169, 8183, 8140, 8142, 8077, 8110, 8147, 8145, 8097, 8141, 8074, 8071, 8081, 8112, -32768,
  7942, 8162, 8221, 8252, 8191, 8168, 8219, 8237, 8157, 8165, 8130, 8127, 8051, 8084, 8126, 8130, 8078, 8115, 8047, 8029, 8034, 8088, -32768,
  7908, 8144, 8208, 8238, 8179, 8155, 8203, 8224, 8153, 8155, 8113, 8105, 8028, 8064, 8107, 8127, 8047, 8081, 8013, 7973, 7938, 7959, -32768,
  7905, 8116, 8185, 8220, 8165, 8129, 8173, 8203, 8134, 8137, 8088, 8084, 8010, 8039, 8085, 8112, 8054, 8094, 8034, 7998, 7987, 8073, -32768,
  7883, 8099, 8165, 8201, 8145, 8109, 8161, 8188, 8115, 8114, 8070, 8076, 8000, 8025, 8063, 8097, 8062, 8108, 8038, 8035, 8053, 8124, -32768,
  7851, 8108, 8173, 8203, 8154, 8131, 8179, 8203, 8126, 8141, 8105, 8101, 8024, 8057, 8100, 8131, 8099, 8140, 8070, 8075, 8088, 8060, -32768,
  7878, 8104, 8164, 8200, 8153, 8137, 8185, 8203, 8111, 8135, 8103, 8103, 8036, 8076, 8128, 8158, 8118, 8168, 8092, 8101, 8121, 8127, -32768,
  7859, 8080, 8152, 8187, 8141, 8126, 8174, 8193, 8116, 8120, 8089, 8103, 8038, 8075, 8128, 8160, 8130, 8166, 8093, 8100, 8124, 8138, -32768,
  8261, 8386, 8445, 8473, 8417, 8416, 8459, 8488, 8429, 8430, 8400, 8404, 8337, 8389, 8431, 8454, 8436, 8483, 8417, 8428, 8467, 8622, -32768,
  8198, 8363, 8419, 8456, 8409, 8401, 8455, 8483, 8420, 8427, 8397, 8402, 8338, 8385, 8427, 8444, 8424, 8478, 8408, 8410, 8442, 8537, -32768,
  8200, 8383, 8436, 8470, 8417, 8409, 8465, 8496, 8431, 8445, 8425, 8429, 8357, 8406, 8444, 8458, 8439, 8492, 8426, 8434, 8468, 8532, -32768,
  8217, 8355, 8401, 8438, 8397, 8374, 8426, 8465, 8407, 8421, 8388, 8395, 8320, 8386, 8426, 8436, 8412, 8458, 8389, 8406, 8459, 8652, -32768,
  8216, 8373, 8439, 8452, 8422, 8403, 8460, 8490, 8431, 8447, 8410, 8417, 8347, 8412, 8467, 8472, 8440, 8477, 8411, 8425, 8465, 8624, -32768,
  8211, 8389, 8460, 8486, 8442, 8431, 8492, 8521, 8462, 8467, 8438, 8450, 8386, 8445, 8502, 8511, 8468, 8506, 8436, 8451, 8479, 8594, -32768,
  8217, 8381, 8444, 8481, 8438, 8423, 8475, 8516, 8456, 8465, 8441, 8442, 8373, 8435, 8484, 8502, 8464, 8501, 8431, 8445, 8479, 8635, -32768,
  8222, 8383, 8445, 8486, 8439, 8425, 8486, 8517, 8468, 8478, 8453, 8449, 8373, 8431, 8482, 8501, 8473, 8516, 8435, 8452, 8481, 8629, -32768,
  8223, 8400, 8461, 8495, 8444, 8431, 8499, 8538, 8479, 8487, 8463, 8475, 8395, 8440, 8496, 8516, 8486, 8520, 8457, 8468, 8492, 8602, -32768,
  8241, 8402, 8469, 8501, 8448, 8433, 8493, 8526, 8469, 8484, 8458, 8470, 8394, 8439, 8494, 8511, 8480, 8508, 8439, 8464, 8497, 8653, -32768,
  8258, 8419, 8489, 8517, 8464, 8447, 8511, 8542, 8482, 8497, 8473, 8483, 8415, 8464, 8507, 8531, 8500, 8527, 8458, 8477, 8502, 8656, -32768,
  8275, 8441, 8509, 8539, 8484, 8470, 8529, 8564, 8495, 8512, 8493, 8504, 8434, 8477, 8521, 8539, 8513, 8548, 8477, 8486, 8516, 8669, -32768,
  8302, 8448, 8504, 8533, 8492, 8473, 8530, 8555, 8495, 8503, 8485, 8501, 8432, 8470, 8514, 8538, 8511, 8537, 8477, 8491, 8528, 8737, -32768,
  8327, 8478, 8533, 8559, 8514, 8495, 8554, 8571, 8513, 8528, 8507, 8523, 8455, 8492, 8539, 8566, 8543, 8571, 8510, 8524, 8557, 8751, -32768,
  8337, 8483, 8531, 8561, 8525, 8510, 8559, 8578, 8519, 8534, 8513, 8529, 8459, 8501, 8547, 8568, 8550, 8579, 8518, 8535, 8577, 8774, -32768,
  8355, 8478, 8530, 8555, 8525, 8507, 8552, 8570, 8525, 8542, 8512, 8524, 8458, 8505, 8549, 8562, 8550, 8577, 8521, 8541, 8588, 8841, -32768,
  8345, 8398, 8483, 8533, 8518, 8512, 8576, 8619, 8579, 8611, 8564, 8580, 8523, 8583, 8645, 8658, 8644, 8689, 8635, 8678, 8762, 9284, -32768,
};

void validate_sensor_variant_options_test(void **state)
{
  /* setup */
  struct libmaxtouch_ctx ctx;
  ctx.log_level = LOG_SILENT;
  ctx.log_fn = mxt_log_stdout;

  struct mxt_device mxt;
  mxt.ctx = &ctx;

  struct sensor_variant_options sv_opts;
  int ret = MXT_SUCCESS;

  /* perform tests */
  sv_opts.upper_limit = 101;
  sv_opts.lower_limit = 50;

  ret = validate_sensor_variant_options(&mxt, &sv_opts);
  assert_int_equal(ret, MXT_ERROR_BAD_INPUT);

  sv_opts.upper_limit = 50;
  sv_opts.lower_limit = 101;

  ret = validate_sensor_variant_options(&mxt, &sv_opts);
  assert_int_equal(ret, MXT_ERROR_BAD_INPUT);

  sv_opts.upper_limit = 50;
  sv_opts.lower_limit = 50;

  ret = validate_sensor_variant_options(&mxt, &sv_opts);
  assert_int_equal(ret, MXT_SUCCESS);
}

void get_xyline_data_test(void **state)
{
  int ret;

  struct libmaxtouch_ctx ctx;
  ctx.log_level = LOG_VERBOSE;
  ctx.log_fn = mxt_log_stderr;

  struct t37_ctx frame;
  frame.lc = &ctx;
  frame.y_size = 4;

  uint16_t data[] = {
    0,  1,   2, 0,    /* X0 */
    10, 11, 12, 0,    /* X1 */
  };

  frame.data_buf = data;

  double val[3];

  ret = get_xline_data(&frame, 0, 3, val);
  assert_int_equal(ret, MXT_SUCCESS);
  assert_float_equal(val[0], 0.0);
  assert_float_equal(val[1], 1.0);
  assert_float_equal(val[2], 2.0);

  ret = get_xline_data(&frame, 1, 3, val);
  assert_int_equal(ret, MXT_SUCCESS);
  assert_float_equal(val[0], 10.0);
  assert_float_equal(val[1], 11.0);
  assert_float_equal(val[2], 12.0);

  ret = get_yline_data(&frame, 0, 2, val);
  assert_int_equal(ret, MXT_SUCCESS);
  assert_float_equal(val[0], 0);
  assert_float_equal(val[1], 10);

  ret = get_yline_data(&frame, 1, 2, val);
  assert_int_equal(ret, MXT_SUCCESS);
  assert_float_equal(val[0], 1);
  assert_float_equal(val[1], 11);

  ret = get_yline_data(&frame, 2, 2, val);
  assert_int_equal(ret, MXT_SUCCESS);
  assert_float_equal(val[0], 2);
  assert_float_equal(val[1], 12);
}

void polyfit_test(void **state)
{
  struct libmaxtouch_ctx ctx;
  ctx.log_level = LOG_SILENT;
  ctx.log_fn = mxt_log_stderr;

  double coeff[3];

  /* Test polynomial data */
  double xval[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
  double yval[] = { 7991, 7984, 7979, 7976, 7975, 7976, 7979, 7984, 7991, 8000 };

  assert_true(ft_polyfit(&ctx, xval, yval, coeff, 10));

  assert_float_equal(coeff[0], 8000);
  assert_float_equal(coeff[1], -10);
  assert_float_equal(coeff[2], 1);
}

void calculate_poly_test(void **state)
{
  double coeff[] = { 1.0, 2.0, 3.0 };
  double data[] = { 1, 2, 0.1 };
  double result[3];
  double expected[] = { 6, 17, 1.203 };

  int ret = calculate_poly(data, coeff, 3, result);
  assert_int_equal(MXT_SUCCESS, ret);

  for (int i=0; i<3; i++)
    assert_float_equal(result[i], expected[i]);
}

void check_line_test(void **state)
{
  struct libmaxtouch_ctx ctx;
  ctx.log_level = LOG_SILENT;
  ctx.log_fn = mxt_log_stdout;

  struct sensor_variant_options sv_opts;
  sv_opts.upper_limit = 15;
  sv_opts.lower_limit = 15;
  sv_opts.dualx = 0;
  sv_opts.max_defects = 0;
  sv_opts.matrix_size = 0;

  /* Recorded data for one yline, with two nodes 9 & 17 adjusted to be incorrect */
  double yval[] = {
    8473, 8571, 8591, 8596, 8529, 8473, 8510, 8476, 8363, 4329, 8230, 8209,
    8167, 8181, 8211, 8207, 8152, 14107, 8013, 7992, 8010, 8053
  };

  double xval[] = {
    0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21
  };

  bool status[22];
  double coeff[3];

  uint32_t num_failed = 0;

  int ret = check_line(&ctx, &sv_opts, xval, yval, 22, &num_failed, status, coeff);
  assert_int_equal(ret, MXT_SUCCESS);

  assert_int_equal(2, num_failed);
  assert_float_equal(coeff[0], 8574.057312);
  assert_float_equal(coeff[1], -101.489977);
  assert_float_equal(coeff[2], 5.816347);
}

void sensor_variant_algorithm_test(void **state)
{
  int ret;

  struct libmaxtouch_ctx ctx;
  ctx.log_level = LOG_DEBUG;
  ctx.log_fn = mxt_log_stdout;

  struct mxt_device mxt;
  mxt.ctx = &ctx;

  struct t37_ctx frame;
  frame.mxt = &mxt;
  frame.lc = &ctx;
  frame.mode = REFS_MODE;
  frame.x_size = 38;
  frame.y_size = 23;
  frame.data_buf = data_buf_pass;

  struct mxt_touchscreen_info ts_info;
  ts_info.xorigin = 0;
  ts_info.yorigin = 0;
  ts_info.xsize = 38;
  ts_info.ysize = 22;

  struct sensor_variant_options sv_opts;
  sv_opts.upper_limit = 15;
  sv_opts.lower_limit = 15;
  sv_opts.dualx = 0;
  sv_opts.max_defects = 0;
  sv_opts.matrix_size = 0;

  /* check all good with normal data */
  ret = sensor_variant_algorithm(&frame, &ts_info, &sv_opts);
  assert_int_equal(ret, MXT_SUCCESS);

  /* Set a couple of points out of limits */
  data_buf_pass[100] = 2000;
  data_buf_pass[310] = 12000;
  ret = sensor_variant_algorithm(&frame, &ts_info, &sv_opts);
  assert_int_equal(ret, MXT_SENSOR_VARIANT_DETECTED);

  /* Try max defect setting, should now pass again */
  sv_opts.max_defects = 5;
  ret = sensor_variant_algorithm(&frame, &ts_info, &sv_opts);
  assert_int_equal(ret, MXT_SUCCESS);

  /* Introduce more defects and try sub matrix scan */
  sv_opts.max_defects = 0;
  sv_opts.matrix_size = 3;
  data_buf_pass[101] = 2000;
  data_buf_pass[122] = 2000;
  data_buf_pass[137] = 2000;
  ret = sensor_variant_algorithm(&frame, &ts_info, &sv_opts);
  assert_int_equal(ret, MXT_SENSOR_VARIANT_DETECTED);
}
