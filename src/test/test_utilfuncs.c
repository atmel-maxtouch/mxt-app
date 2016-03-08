//------------------------------------------------------------------------------
/// \file   test_utilfuncs.c
/// \brief  Tests against libmaxtouch/utilfuncs.h
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
#include <stdarg.h>
#include <stddef.h>
#include <setjmp.h>
#include <cmocka.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/utilfuncs.h"

#include "run_unit_tests.h"

void mxt_convert_hex_test(void **state)
{
  /* test setup */
  uint8_t databuf[5] = {0};
  char hex[4];
  uint16_t count;
  int ret;

  /* perform tests */
  strcpy(hex, "09");
  mxt_convert_hex(hex, databuf, &count, sizeof(databuf)),
  assert_int_equal(ret, MXT_SUCCESS);
  assert_int_equal(databuf[0], 9);
  assert_int_equal(count, 1);

  strcpy(hex, "0F");
  ret = mxt_convert_hex(hex, databuf, &count, sizeof(databuf));
  assert_int_equal(ret, MXT_SUCCESS);
  assert_int_equal(databuf[0], 0x0F);
  assert_int_equal(count, 1);

  strcpy(hex, "0FAB");
  ret = mxt_convert_hex(hex, databuf, &count, sizeof(databuf));
  assert_int_equal(ret, MXT_SUCCESS);
  assert_true(databuf[0] == 0x0F);
  assert_true(databuf[1] == 0xAB);
  assert_int_equal(count, 2);

  /* test error conditions */
  strcpy(hex, "F");
  ret = mxt_convert_hex(hex, databuf, &count, sizeof(databuf));
  assert_int_equal(ret, MXT_ERROR_BAD_INPUT);

  strcpy(hex, "0FAB");
  ret = mxt_convert_hex(hex, databuf, &count, 1);
  assert_int_equal(ret, MXT_ERROR_NO_MEM);

  strcpy(hex, "0xA5");
  ret = mxt_convert_hex(hex, databuf, &count, sizeof(databuf));
  assert_int_equal(ret, MXT_ERROR_BAD_INPUT);
}
