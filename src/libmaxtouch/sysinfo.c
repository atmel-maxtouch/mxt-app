//------------------------------------------------------------------------------
/// \file   sysinfo.c
/// \brief  Get system uptime
/// \author Nick Dyer
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

#ifndef ANDROID
#include <sys/sysinfo.h>
//******************************************************************************
/// \brief  Get system uptime in seconds since boot
int get_uptime(unsigned long *uptime)
{
  int ret;
  struct sysinfo sys_info;

  ret = sysinfo(&sys_info);
  if (ret != 0)
    return ret;

  *uptime = sys_info.uptime;
  return ret;
}
#else
#include <stdio.h>
#include <locale.h>
//******************************************************************************
/// \brief  Get system uptime in seconds since boot
int get_uptime(unsigned long *uptime)
{
  FILE *fp;
  int ret = -1;
  unsigned long upsecs;
  char buf[64];
  char *b;

  fp = fopen("/proc/uptime", "r");
  if (fp != NULL)
  {
    b = fgets(buf, BUFSIZ, fp);
    if (b == buf)
    {
      /* The following sscanf must use the C locale.  */
      setlocale(LC_NUMERIC, "C");
      ret = sscanf(buf, "%lf", &upsecs);
      setlocale(LC_NUMERIC, "");
      if (ret == 1) {
        *uptime = upsecs;
        ret = 0;
      }
    }

    fclose(fp);
  }

  return ret;
}
#endif
