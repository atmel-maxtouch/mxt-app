//------------------------------------------------------------------------------
/// \file   run_tests.c
/// \brief  Test suite for mxt-app.
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
#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
 
+include "mxt-app/mxt_app.h"
#include "run_unit_tests.h"

	
//******************************************************************************
/// \brief Run all unit tests
int main(int argc, char *argv[])
{
  /* Test suite */
  const struct CMUnitTest tests[] = {
    unit_test(mxt_convert_hex_test),
    unit_test(validate_sensor_variant_options_test),
    unit_test(get_xyline_data_test),
    unit_test(polyfit_test),
    unit_test(calculate_poly_test),
    unit_test(check_line_test),
    unit_test(sensor_variant_algorithm_test),
  };

  return cmocka_run_group_tests(tests, NULL, NULL);
}
