#pragma once
//------------------------------------------------------------------------------
/// \file   log.h
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

#include <stdint.h>
#include <stdarg.h>

#define ENABLE_LOGGING   1

#ifndef NDEBUG
#define ENABLE_DEBUG     1
#else
#define ENABLE_DEBUG     0
#endif


/* Log levels - designed to match Android's log levels */
enum mxt_log_level {
  LOG_UNKNOWN = 0,
  LOG_DEFAULT = 1,
  LOG_VERBOSE = 2,
  LOG_DEBUG   = 3,
  LOG_INFO    = 4,
  LOG_WARN    = 5,
  LOG_ERROR   = 6,
  LOG_FATAL   = 7,
  LOG_SILENT  = 8
};

struct libmaxtouch_ctx;

enum mxt_log_level mxt_get_log_level(struct libmaxtouch_ctx *ctx);
void mxt_set_log_level(struct libmaxtouch_ctx *ctx, uint8_t verbose);
void mxt_log_stdout(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, va_list va_args);
void mxt_log_stderr(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, va_list args);
void mxt_log_android(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, va_list args);
void mxt_log_buffer(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *prefix, const unsigned char *data, size_t count);

static inline void __attribute__((always_inline, format(printf, 2, 3)))
mxt_log_null(struct libmaxtouch_ctx *ctx, const char *format, ...) {}

void mxt_log(struct libmaxtouch_ctx *ctx, enum mxt_log_level level, const char *format, ...);

#define mxt_log_cond(ctx, level, arg...) \
  do { \
  if (level >= mxt_get_log_level(ctx)) \
    mxt_log(ctx, level, ## arg); \
  } while (0)

#if ENABLE_LOGGING

#if ENABLE_DEBUG
#define mxt_verb(ctx, arg...) mxt_log_cond(ctx, LOG_VERBOSE, ## arg)
#define mxt_dbg(ctx, arg...) mxt_log_cond(ctx, LOG_DEBUG, ## arg)
#else
#define mxt_verb(ctx, arg...) mxt_log_null(ctx, ## arg)
#define mxt_dbg(ctx, arg...) mxt_log_null(ctx, ## arg)
#endif

#define mxt_info(ctx, arg...) mxt_log_cond(ctx, LOG_INFO, ## arg)
#define mxt_warn(ctx, arg...) mxt_log_cond(ctx, LOG_WARN, ## arg)
#define mxt_err(ctx, arg...) mxt_log_cond(ctx, LOG_ERROR, ## arg)

#else
/* Disable logging */
#define mxt_verb(ctx, arg...) mxt_log_null(ctx, ## arg)
#define mxt_dbg(ctx, arg...) mxt_log_null(ctx, ## arg)
#define mxt_info(ctx, arg...) mxt_log_null(ctx, ## arg)
#define mxt_warn(ctx, arg...) mxt_log_null(ctx, ## arg)
#define mxt_err(ctx, arg...) mxt_log_null(ctx, ## arg)
#endif
