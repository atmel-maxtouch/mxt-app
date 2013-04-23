//------------------------------------------------------------------------------
/// \file   buffer.c
/// \brief  Buffer allocation functions
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2012 Atmel Corporation. All rights reserved.
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

#include <stdlib.h>

#include "libmaxtouch/log.h"

#include "buffer.h"

#define BUFFER_BLOCKSIZE       16

//******************************************************************************
/// \brief Allocate memory associated with buffer
int mxt_buf_init(struct mxt_buffer *ctx)
{
  int *ptr;

  ctx->capacity = BUFFER_BLOCKSIZE;
  ctx->size = 0;
  ptr = malloc(ctx->capacity*sizeof(uint8_t));

  if (ptr)
  {
    ctx->data = (uint8_t *)ptr;
    return 0;
  }
  else
  {
    LOG(LOG_ERROR, "malloc failed");
    return -1;
  }
}

//******************************************************************************
/// \brief Reallocate buffer memory if necessary
static int mxt_buf_realloc(struct mxt_buffer *ctx)
{
  int *ptr = 0;

  /* Check whether we are still within bounds of buffer */
  if (ctx->size <= ctx->capacity)
    return 0;

  ctx->capacity += BUFFER_BLOCKSIZE;
  ptr = realloc(ctx->data, ctx->capacity*sizeof(uint8_t));

  if (ptr)
  {
    ctx->data = (uint8_t *)ptr;
    return 0;
  }
  else
  {
    LOG(LOG_ERROR, "realloc failed");
    return -1;
  }
}

//******************************************************************************
/// \brief Add value to buffer
int mxt_buf_add(struct mxt_buffer *ctx, uint8_t value)
{
  size_t offset;
  int ret;

  offset = ctx->size;

  ret = mxt_buf_realloc(ctx);
  if (ret < 0)
    return ret;

  *(ctx->data + offset) = value;

  /* update new size */
  ctx->size = offset + 1;

  return 0;
}

//******************************************************************************
/// \brief Free memory associated with buffer
void mxt_buf_free(struct mxt_buffer *ctx)
{
  if (ctx->data)
  {
    free(ctx->data);
    ctx->data = 0;
  }
}
