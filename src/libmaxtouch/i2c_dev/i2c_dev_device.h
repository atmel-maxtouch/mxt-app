#pragma once
//------------------------------------------------------------------------------
/// \file   i2c_dev_device.h
/// \brief  headers for MXT device low level access via i2c-dev interface
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

//******************************************************************************
/// \brief Device information for i2c-dev backend
struct i2c_dev_conn_info
{
  int adapter;
  int address;
};

//******************************************************************************
/// \brief Device information for i2c-dev backend
struct i2c_dev_device
{
};

int i2c_dev_open(struct mxt_device *mxt);
void i2c_dev_release(struct mxt_device *mxt);
int i2c_dev_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, int count);
int i2c_dev_write_register(struct mxt_device *mxt, unsigned char const *buf, int start_register, int count);
int i2c_dev_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count);
int i2c_dev_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count);
