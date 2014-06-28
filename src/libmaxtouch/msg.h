#pragma once
//------------------------------------------------------------------------------
/// \file   msg.h
/// \brief  Message handling function headers
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

int t44_get_msg_count(struct mxt_device *mxt, int *count);
char *t44_get_msg_string(struct mxt_device *mxt);
int t44_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf, size_t buflen, int *count);
int t44_msg_reset(struct mxt_device *mxt);
int mxt_read_messages(struct mxt_device *mxt, int timeout_seconds, void *context, int (*msg_func)(struct mxt_device *mxt, uint8_t *msg, void *context, uint8_t size), int *flag);
int mxt_get_calibrate_msgs(struct mxt_device *mxt, int timeout, int *state);
int mxt_flush_msgs(struct mxt_device *mxt);
uint32_t mxt_get_config_crc(struct mxt_device *mxt);
