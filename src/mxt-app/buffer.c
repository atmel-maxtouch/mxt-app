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
  ptr = calloc(ctx->capacity, sizeof(uint8_t));

  if (ptr)
  {
    ctx->data = (uint8_t *)ptr;
    return 0;
  }
  else
  {
    return -1;
  }
}

//******************************************************************************
/// \brief Reallocate buffer memory if necessary
static int mxt_buf_realloc(struct mxt_buffer *ctx, size_t new_size)
{
  int *ptr = 0;
  size_t new_capacity;

  /* Check whether we are still within bounds of buffer */
  if (new_size <= ctx->capacity)
    return 0;

  new_capacity = ctx->capacity + BUFFER_BLOCKSIZE;
  ptr = realloc(ctx->data, new_capacity * sizeof(uint8_t));

  if (ptr)
  {
    ctx->data = (uint8_t *)ptr;
    ctx->capacity = new_capacity;
    return 0;
  }
  else
  {
    return -1;
  }
}

//******************************************************************************
/// \brief Add value to buffer
int mxt_buf_add(struct mxt_buffer *ctx, uint8_t value)
{
  int ret;
  size_t new_size = ctx->size + 1;

  ret = mxt_buf_realloc(ctx, new_size);
  if (ret < 0)
    return ret;

  *(ctx->data + ctx->size) = value;

  /* update new size */
  ctx->size = new_size;

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

//******************************************************************************
/// \brief Reset buffer
void mxt_buf_reset(struct mxt_buffer *ctx)
{
  ctx->size = 0;
}
