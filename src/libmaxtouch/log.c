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
#include "malloc.h"

#include "libmaxtouch.h"
#include "libmaxtouch/utilfuncs.h"

#if ANDROID
#include <android/log.h>

#ifndef LOG_TAG
#define LOG_TAG "libmaxtouch"
#endif
#endif

//******************************************************************************
/// \brief  Returns the input log level as a human-readable string.
/// \return Log level string
static const char get_log_level_string(enum mxt_log_level level)
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
/// \brief  Get log verbosity level
enum mxt_log_level mxt_get_log_level(struct libmaxtouch_ctx *ctx)
{
  return ctx->log_level;
}

//******************************************************************************
/// \brief  Set log verbosity level
void mxt_set_log_level(struct libmaxtouch_ctx *ctx, uint8_t verbose)
{
  switch (verbose)
  {
    case 0:
      ctx->log_level = LOG_SILENT;
      break;
    case 1:
      ctx->log_level = LOG_ERROR;
      break;
    case 2:
      ctx->log_level = LOG_INFO;
      break;
    case 3:
      ctx->log_level = LOG_DEBUG;
      break;
    default:
      ctx->log_level = LOG_VERBOSE;
      break;
  }
}

//*****************************************************************************
/// \brief Output buffer to debug as hex
void mxt_log_buffer(struct libmaxtouch_ctx *ctx, enum mxt_log_level level,
                           const char *prefix,
                           const unsigned char *data, size_t count)
{
#if ENABLE_DEBUG
  unsigned int i;
  char *hexbuf;
  size_t strsize = count*3 + 1;

  if (mxt_get_log_level(ctx) > level)
    return;

  hexbuf = (char *)calloc(strsize, sizeof(char));
  if (hexbuf == NULL)
  {
    mxt_err(ctx, "%s: calloc failure", __func__);
    return;
  }

  for (i = 0; i < count; i++)
    sprintf(&hexbuf[3 * i], "%02X ", data[i]);

  mxt_log(ctx, LOG_VERBOSE, "%s: %s", prefix, hexbuf);

  free(hexbuf);
#endif
}

//******************************************************************************
/// \brief Log function
void mxt_log(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, ...)
{
        va_list args;

        va_start(args, format);
        ctx->log_fn(ctx, level, format, args);
        va_end(args);
}

//******************************************************************************
/// \brief Output log message to stderr, with optional timestamp
void mxt_log_stderr(struct libmaxtouch_ctx *ctx, enum mxt_log_level level,
                    const char *format, va_list va_args)
{
  if (mxt_get_log_level(ctx) < LOG_INFO)
  {
    mxt_print_timestamp(stderr);
    fprintf(stderr, " %c: ", get_log_level_string(level));
  }

  vfprintf(stderr, format, va_args);
  printf("\n");
}

#if ANDROID
//******************************************************************************
/// \brief Log using the Android API
void mxt_log_android(struct libmaxtouch_ctx *ctx, enum mxt_log_level level,
                     const char *format, va_list args)
{
  __android_log_vprint(level, LOG_TAG, format, args);
}
#endif
