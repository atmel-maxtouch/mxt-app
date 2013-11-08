//------------------------------------------------------------------------------
/// \file   log.c
/// \brief  Provides a macro for logging messages.
/// \author Tim Culmer
//------------------------------------------------------------------------------
// Copyright 2011 Atmel Corporation. All rights reserved.
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

#include "stdio.h"
#include "stdint.h"

#include "libmaxtouch.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

/* Default log level */
mxt_log_level log_level = LOG_INFO;

//******************************************************************************
/// \brief  Returns the input log level as a human-readable string.
/// \return Log level string
static const char get_log_level_string(mxt_log_level level)
{
  switch (level)
  {
    case LOG_SILENT:
      return 'S';
    case LOG_FATAL:
      return 'F';
    case LOG_ERROR:
      return 'E';
    case LOG_WARN:
      return 'W';
    case LOG_INFO:
      return 'I';
    case LOG_DEBUG:
      return 'D';
    case LOG_VERBOSE:
    case LOG_DEFAULT:
    case LOG_UNKNOWN:
    default:
      return 'V';
  }
}


//******************************************************************************
/// \brief  Sets verbosity level
void mxt_set_verbose(uint8_t verbose)
{
  switch (verbose)
  {
    case 0:
      log_level = LOG_SILENT;
      break;
    case 1:
      log_level = LOG_ERROR;
      break;
    case 2:
      log_level = LOG_INFO;
      break;
    case 3:
      log_level = LOG_DEBUG;
      break;
    default:
      log_level = LOG_VERBOSE;
      break;
  }
}

//******************************************************************************
/// \brief Output log message to stdout, with optional timestamp
void mxt_log_message(mxt_log_level level, const char *fmt, ...)
{
  va_list args;

  va_start(args, fmt);

  if (log_level < LOG_INFO)
  {
    mxt_print_timestamp(stdout);
    printf(" %c: ", get_log_level_string(level));
  }

  vprintf(fmt, args);
  printf("\n");
}
