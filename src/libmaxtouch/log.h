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

#define DEBUG 1

#ifndef LOG_TAG
#define LOG_TAG "libmaxtouch"
#endif

/* Log levels - designed to match Android's log levels */
typedef enum mxt_log_level {
  LOG_UNKNOWN = 0,
  LOG_DEFAULT = 1,
  LOG_VERBOSE = 2,
  LOG_DEBUG   = 3,
  LOG_INFO    = 4,
  LOG_WARN    = 5,
  LOG_ERROR   = 6,
  LOG_FATAL   = 7,
  LOG_SILENT  = 8
} mxt_log_level;

extern mxt_log_level log_level;

void mxt_set_verbose(uint8_t verbose);


#if DEBUG

#include <stdio.h>

const char* get_log_level_string(mxt_log_level level);

#if ANDROID
/* Log using Android API */
#include <android/log.h>

#define LOG(level, format, ...) \
  if (level >= log_level) { \
    printf("%s: " format "\n", get_log_level_string(level), ##__VA_ARGS__); \
  } else { \
    __android_log_print(ANDROID_ ## level, LOG_TAG, format, ##__VA_ARGS__); \
  }
#else
/* Log to STDOUT */
#define LOG(level, format, ...) \
  if (level >= log_level) \
  { \
    printf("%s: " format "\n", get_log_level_string(level), ##__VA_ARGS__); \
  }

#endif

#else
/* Disable logging */
#define LOG(...) /* Do nothing */
#endif
