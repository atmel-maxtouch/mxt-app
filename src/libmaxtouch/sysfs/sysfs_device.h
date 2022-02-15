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

struct dmesg_item;

//******************************************************************************
/// \brief sysfs device connection information
struct sysfs_conn_info {
  char *path;
  bool acpi;
  int i2c_bus;
  int i2c_addr;
  int spi_cs;
  int spi_bus;
  bool spi_found;
  bool i2c_found;
};

//******************************************************************************
/// \brief sysfs device
struct sysfs_device {
  struct sysfs_conn_info conn;
  char *mem_access_path;
  char *temp_path;
  size_t path_max;
  bool debug_v2;
  uint16_t debug_v2_msg_count;
  uint16_t debug_v2_msg_ptr;
  uint8_t *debug_v2_msg_buf;
  char *debug_msg_buf;
  int debug_msg_buf_size;
  int debug_notify_fd;
  size_t debug_v2_size;
  bool b_i2c_device;
  bool b_spi_device;
  char *diag_msg_path;

  int dmesg_count;
  struct dmesg_item *dmesg_head;
  struct dmesg_item *dmesg_ptr;

  unsigned long timestamp;
  unsigned long mtimestamp;
};

int sysfs_reset_chip(struct mxt_device *mxt, bool reset_chip);
int sysfs_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn);
int sysfs_spi_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn);
int sysfs_open_i2c(struct mxt_device *mxt);
int sysfs_open_spi(struct mxt_device *mxt);
void sysfs_release(struct mxt_device *mxt);
int sysfs_new_device(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn, const char *dirname);
int sysfs_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, size_t count, size_t *bytes_transferred);
int sysfs_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count);
int sysfs_bootloader_write(struct mxt_device *mxt, unsigned const char *buf, int count);
int sysfs_write_register(struct mxt_device *mxt, unsigned char const *buf, int start_register, size_t count);
int sysfs_set_debug(struct mxt_device *mxt, bool debug_state);
int sysfs_get_debug(struct mxt_device *mxt, bool *value);
int sysfs_set_debug_irq(struct mxt_device *mxt, bool debug_state);
int sysfs_get_crc_enabled(struct mxt_device *mxt, bool *value);
char *sysfs_get_directory(struct mxt_device *mxt);
bool sysfs_has_debug_v2(struct mxt_device *mxt);
char *sysfs_get_msg_string_v2(struct mxt_device *mxt);
int sysfs_get_msg_bytes_v2(struct mxt_device *mxt, unsigned char *buf, size_t buflen, int *count);
int sysfs_get_msgs_v2(struct mxt_device *mxt, int *count);
int sysfs_msg_reset_v2(struct mxt_device *mxt);
int sysfs_get_debug_v2_fd(struct mxt_device *mxt);
int sysfs_get_i2c_address(struct libmaxtouch_ctx *ctx, struct mxt_conn_info *conn, int *adapter, int *address);
