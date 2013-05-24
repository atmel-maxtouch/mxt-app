#pragma once
//------------------------------------------------------------------------------
/// \file   sysfs_device.h
/// \brief  headers for MXT device low level access via I2C
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

int sysfs_scan(void);
void sysfs_release(void);
int sysfs_read_register(unsigned char *buf, int start_register, int count);
int sysfs_write_register(unsigned char const *buf, int start_register, int count);
int sysfs_set_debug(bool debug_state);
bool sysfs_get_debug(void);
int sysfs_get_i2c_adapter(void);
int sysfs_get_i2c_address(void);
char *sysfs_get_directory(void);
int sysfs_get_msg_count(void);
char *sysfs_get_msg_string(void);
int sysfs_get_msg_bytes(unsigned char *buf, size_t buflen);
int sysfs_msg_reset(void);
bool sysfs_has_debug_ng(void);
char *sysfs_get_msg_string_ng(void);
int sysfs_get_msg_bytes_ng(unsigned char *buf, size_t buflen);
int sysfs_get_msg_count_ng(void);
int sysfs_msg_reset_ng(void);
int sysfs_get_debug_ng_fd(void);
