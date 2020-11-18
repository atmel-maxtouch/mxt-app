#pragma once

//------------------------------------------------------------------------------
/// \file   debugfs_device.h
/// \brief  headers for MXT device low level debug access via I2C
/// \author Michael Gong
//------------------------------------------------------------------------------
// Copyright 2020 Microchip Corporation. All rights reserved.
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
/// \brief debugfs device
struct debugfs_device {
  char *tmp_path;
  size_t dir_max;
  char *file_path;
  bool enabled;
};

int debugfs_scan(struct mxt_device *mxt);
int debugfs_open(struct mxt_device *mxt);
int debugfs_set_irq(struct mxt_device *mxt, bool enable);
int debugfs_get_debug_irq(struct mxt_device *mxt, bool *value);
int debugfs_set_debug_irq(struct mxt_device *mxt, bool debug_state);
int debugfs_get_tx_seq_num(struct mxt_device *mxt, uint16_t *value);
int debugfs_set_tx_seq_num(struct mxt_device *mxt, uint8_t value);
int debugfs_get_crc_enabled(struct mxt_device *mxt, bool *value);
static int read_debugfs_byte(struct mxt_device *mxt, char *filename, uint16_t *value);
static int write_debugfs_byte(struct mxt_device *mxt, const char *filename, uint8_t value);
int debugfs_update_seq_num(struct mxt_device *mxt, uint8_t value);



