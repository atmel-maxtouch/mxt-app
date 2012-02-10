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

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

/* Log levels - designed to match Android's log levels */
#define LOG_UNKNOWN 0
#define LOG_DEFAULT 1
#define LOG_VERBOSE 2
#define LOG_DEBUG   3
#define LOG_INFO    4
#define LOG_WARN    5
#define LOG_ERROR   6
#define LOG_FATAL   7
#define LOG_SILENT  8

/* Disable logging */
#if LOG_LEVEL == LOG_SILENT
#define LOG(...) /* Do nothing */

/* Log using Android API */
#elif ANDROID
#include <android/log.h>
#define LOG(level, ...) \
if (level >= LOG_LEVEL) \
{ \
__android_log_print(ANDROID_ ## level, "libmaxtouch", __VA_ARGS__); \
}

/* Log to STDOUT */
#else
#include <stdio.h>
char * get_log_level_string(int level);
#define LOG(level, format, ...) \
  if (level >= LOG_LEVEL) \
  { \
    printf("%s: " format "\n", get_log_level_string(level), ##__VA_ARGS__); \
  }

#endif

