//------------------------------------------------------------------------------
/// \file   i2c_dev_device.c
/// \brief  MXT device low level access via i2c-dev interface
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

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

struct mxt_device;
struct mxt_conn_info;

#include "i2c_dev_device.h"
#include "libmaxtouch/libmaxtouch.h"

#define I2C_SLAVE_FORCE 0x0706

/* Deep sleep retry delay 25 ms */
#define I2C_RETRY_DELAY 25000

//******************************************************************************
/// \brief  Register i2c-dev device
/// \return #mxt_rc
int i2c_dev_open(struct mxt_device *mxt)
{
  mxt_info
  (
    mxt->ctx, "Registered i2c-dev adapter:%d address:0x%x",
    mxt->conn->i2c_dev.adapter, mxt->conn->i2c_dev.address
  );

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Release device
void i2c_dev_release(struct mxt_device *mxt)
{
}

//******************************************************************************
/// \brief  Open the i2c dev interface and set the slave address
/// \return #mxt_rc
static int open_and_set_slave_address(struct mxt_device *mxt, int *fd_out)
{
  int fd;
  int ret;
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", mxt->conn->i2c_dev.adapter);
  fd = open(filename, O_RDWR);
  if (fd < 0) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  ret = ioctl(fd, I2C_SLAVE_FORCE, mxt->conn->i2c_dev.address);
  if (ret < 0) {
    mxt_err(mxt->ctx, "Error setting slave address, error %s (%d)", strerror(errno), errno);
    close(fd);
    return mxt_errno_to_rc(errno);
  }

  *fd_out = fd;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int i2c_dev_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, int count)
{
  int fd = -ENODEV;
  int ret;
  char register_buf[2];

  mxt_dbg(mxt->ctx, "start_register:%d count:%d", start_register, count);

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  register_buf[0] = start_register & 0xff;
  register_buf[1] = (start_register >> 8) & 0xff;

  if (write(fd, &register_buf, 2) != 2) {
    mxt_verb(mxt->ctx, "I2C retry");
    usleep(I2C_RETRY_DELAY);
    if (write(fd, &register_buf, 2) != 2) {
      mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    }
  }

  if (read(fd, buf, count) != count) {
    mxt_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
  } else {
    ret = MXT_SUCCESS;
  }

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int i2c_dev_write_register(struct mxt_device *mxt, unsigned char const *val, int start_register, int datalength)
{
  int fd = -ENODEV;
  int count;
  int ret;
  unsigned char *buf;

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  count = datalength + 2;
  buf = (unsigned char *)calloc(count, sizeof(unsigned char));

  buf[0] = start_register & 0xff;
  buf[1] = (start_register >> 8) & 0xff;
  memcpy(buf + 2, val, datalength);

  if (write(fd, buf, count) != count) {
    mxt_verb(mxt->ctx, "I2C retry");
    usleep(I2C_RETRY_DELAY);
    if (write(fd, buf, count) != count) {
      mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
    } else {
      ret = MXT_SUCCESS;
    }
  } else {
    ret = MXT_SUCCESS;
  }

  free(buf);
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Bootloader read
/// \return #mxt_rc
int i2c_dev_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count)
{
  int fd = -ENODEV;
  int ret;

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  mxt_dbg(mxt->ctx, "Reading %d bytes", count);

  if (read(fd, buf, count) != count) {
    mxt_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
  } else {
    ret = MXT_SUCCESS;
  }

  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Bootloader write
/// \return #mxt_rc
int i2c_dev_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count)
{
  int fd = -ENODEV;
  int ret;

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  mxt_dbg(mxt->ctx, "Writing %d bytes", count);

  if (write(fd, buf, count) != count) {
    mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
  } else {
    ret = MXT_SUCCESS;
  }

  close(fd);
  return ret;
}
