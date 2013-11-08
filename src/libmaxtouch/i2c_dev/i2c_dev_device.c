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
#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"

#define I2C_SLAVE_FORCE 0x0706

/* Deep sleep retry delay 25 ms */
#define I2C_RETRY_DELAY 25000

//******************************************************************************
/// \brief  Register i2c-dev device
/// \return 0 = success, negative for error
int i2c_dev_open(struct mxt_device *mxt)
{
  LOG
  (
    LOG_INFO, "Registered i2c-dev adapter:%d address:0x%x",
    mxt->conn.i2c_dev.adapter, mxt->conn.i2c_dev.address
  );

  return 0;
}

//******************************************************************************
/// \brief  Release device
void i2c_dev_release(struct mxt_device *mxt)
{
}

//******************************************************************************
/// \brief  Open the i2c dev interface and set the slave address
static int open_and_set_slave_address(struct mxt_device *mxt)
{
  int fd;
  int ret;
  char filename[20];

  snprintf(filename, 19, "/dev/i2c-%d", mxt->conn.i2c_dev.adapter);
  fd = open(filename, O_RDWR);
  if (fd < 0) {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return fd;
  }

  ret = ioctl(fd, I2C_SLAVE_FORCE, mxt->conn.i2c_dev.address);
  if (ret < 0) {
    LOG(LOG_ERROR, "Error setting slave address, error %s (%d)", strerror(errno), errno);
    close(fd);
    return ret;
  }

  return fd;
}

//******************************************************************************
/// \brief  Read register from MXT chip
int i2c_dev_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, int count)
{
  int fd;
  int ret;
  char register_buf[2];

  LOG(LOG_DEBUG, "start_register:%d count:%d", start_register, count);

  fd = open_and_set_slave_address(mxt);

  if (fd < 0)
    return fd;

  register_buf[0] = start_register & 0xff;
  register_buf[1] = (start_register >> 8) & 0xff;

  if (write(fd, &register_buf, 2) != 2) {
    LOG(LOG_VERBOSE, "I2C retry");
    usleep(I2C_RETRY_DELAY);
    if (write(fd, &register_buf, 2) != 2) {
      LOG(LOG_ERROR, "Error %s (%d) writing to i2c", strerror(errno), errno);
      ret = -1;
      goto close;
    }
  }

  if (read(fd, buf, count) != count) {
    LOG(LOG_ERROR, "Error %s (%d) reading from i2c", strerror(errno), errno);
    ret = -1;
  }
  else
  {
    ret = 0;
  }

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
int i2c_dev_write_register(struct mxt_device *mxt, unsigned char const *val, int start_register, int datalength)
{
  int fd;
  int count;
  int ret;
  unsigned char *buf;

  fd = open_and_set_slave_address(mxt);

  if (fd < 0)
    return fd;

  count = datalength + 2;
  buf = (unsigned char *)calloc(count, sizeof(unsigned char));

  buf[0] = start_register & 0xff;
  buf[1] = (start_register >> 8) & 0xff;
  memcpy(buf + 2, val, datalength);

  if (write(fd, buf, count) != count) {
    LOG(LOG_VERBOSE, "I2C retry");
    usleep(I2C_RETRY_DELAY);
    if (write(fd, buf, count) != count) {
      LOG(LOG_ERROR, "Error %s (%d) writing to i2c", strerror(errno), errno);
      ret = -1;
    }
    else
    {
      ret = 0;
    }
  }
  else
  {
    ret = 0;
  }

  free(buf);
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Raw read from chip
int i2c_dev_bootloader_read(struct mxt_device *mxt, unsigned char *buf, int count)
{
  int fd;
  int ret;

  fd = open_and_set_slave_address(mxt);

  if (fd < 0)
    return fd;

  LOG(LOG_DEBUG, "Reading %d bytes", count);

  if (read(fd, buf, count) != count) {
    LOG(LOG_ERROR, "Error %s (%d) reading from i2c", strerror(errno), errno);
    ret = -1;
  }
  else
  {
    ret = 0;
  }

  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Bootloader write
int i2c_dev_bootloader_write(struct mxt_device *mxt, unsigned char const *buf, int count)
{
  int fd;
  int ret;

  fd = open_and_set_slave_address(mxt);

  LOG(LOG_DEBUG, "Writing %d bytes", count);

  if (fd < 0)
    return fd;

  if (write(fd, buf, count) != count) {
    LOG(LOG_ERROR, "Error %s (%d) writing to i2c", strerror(errno), errno);
    ret = -1;
  }
  else
  {
    ret = 0;
  }

  close(fd);
  return ret;
}
