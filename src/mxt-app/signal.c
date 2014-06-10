//------------------------------------------------------------------------------
/// \file   signal.c
/// \brief  Signal handling functionality for mxt-app
/// \author Steven Swann
//------------------------------------------------------------------------------
// Copyright 2014 Atmel Corporation. All rights reserved.
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
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <signal.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

//******************************************************************************
/// \brief Signal handler semaphore
volatile sig_atomic_t mxt_sigint_rx = 0;

//******************************************************************************
/// \brief Signal handler to catch SIGINT (Ctrl-C) when viewing continuous msgs
static void mxt_signal_handler(int signal_num)
{
  if (signal_num == SIGINT) {
    mxt_sigint_rx = 1;
  }
}

//******************************************************************************
/// \brief Handles SIGINT signal
void mxt_init_sigint_handler(struct mxt_device *mxt, struct sigaction sa)
{
  sa.sa_handler = mxt_signal_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if (sigaction(SIGINT, &sa, NULL) == -1)
    mxt_err(mxt->ctx, "Can't catch SIGINT");
}

//******************************************************************************
/// \brief Sets default function for SIGINT signal
void mxt_release_sigint_handler(struct mxt_device *mxt, struct sigaction sa)
{
  sa.sa_handler = SIG_DFL;
  if (sigaction(SIGINT, &sa, NULL) == -1)
    mxt_err(mxt->ctx, "Can't return SIGINT to default handler");

  mxt_sigint_rx = 0;
}

//******************************************************************************
/// \brief Get internal sigint flag
sig_atomic_t mxt_get_sigint_flag(void)
{
  return mxt_sigint_rx;
}
