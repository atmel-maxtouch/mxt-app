#pragma once
//------------------------------------------------------------------------------
/// \file   libmaxtouch.h
/// \brief  headers for MXT device low level access
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

#include <stdbool.h>

/* Address offsets for the command processor fields */
#define RESET_OFFSET        0
#define BACKUPNV_OFFSET     1
#define CALIBRATE_OFFSET    2

/* Values to write to the command processor fields */
#define RESET_COMMAND       0x01
#define BOOTLOADER_COMMAND  0xA5
#define BACKUPNV_COMMAND    0x55
#define CALIBRATE_COMMAND   0x01

//******************************************************************************
/// \brief Device connection type
typedef enum mxt_device_type_tag {
  E_UNCONNECTED,
  E_SYSFS,
  E_USB,
  E_I2C_DEV,
} mxt_device_type;

int mxt_scan(void);
void mxt_set_device_type(mxt_device_type type);
int mxt_get_info(void);
void mxt_release(void);
mxt_device_type mxt_get_device_type(void);
char * mxt_get_input_event_file(void);
int mxt_read_register(unsigned char *buf, int start_register, int count);
int mxt_write_register(unsigned char const *buf, int start_register, int count);
int mxt_set_debug(bool debug_state);
bool mxt_get_debug(void);
int mxt_set_pause(bool pause_state);
bool mxt_get_pause(void);
int mxt_reset_chip(bool bootloader_mode);
int mxt_calibrate_chip(void);
int mxt_backup_config(void);
int mxt_load_config_file(const char *cfg_file, bool override_checking);
int mxt_save_config_file(const char *cfg_file);
