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
  mxt_info(
    mxt->ctx, "\nDevice registered on i2c-dev adapter:%d address:0x%x",
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

/* TBD - Keep for now */
//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int i2c_dev_read_register_enc(struct mxt_device *mxt, unsigned char *buf,
                          int start_register, int count, size_t *bytes_read)
{
  int fd = -ENODEV;
  char register_buf[4];
  uint8_t tx_header = 0x04;
  uint8_t crc_data = 0;
  uint8_t tx_value = 0; 
  uint16_t tx_seq_num;
  int ret, err;
  int i, j;

  if (count > mxt->ctx->i2c_block_size)
    count = mxt->ctx->i2c_block_size;

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  if (mxt->mxt_crc.crc_enabled == false) {

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
  } else {
      if (mxt->debug_fs.enabled == true) {
        err = debugfs_set_irq(mxt, false);

        if (err)
          mxt_dbg(mxt->ctx, "Could not disable IRQ");

        err = debugfs_get_tx_seq_num(mxt, &tx_seq_num);

        if (err) {
          mxt_dbg(mxt->ctx, "Failed to get the tx seq num");
        } 
      }

      register_buf[0] = start_register & 0xff;
      register_buf[1] = (start_register >> 8) & 0xff;
      register_buf[2] = tx_seq_num;  //Must be tx seq no

      //Calculate CRC byte
      for (i = 0; i < (tx_header - 1); i++) {
        crc_data = mxt_calc_crc8(crc_data, register_buf[i]);
      }

      register_buf[3] = crc_data;   //CRC8 bit checksum

      if (write(fd, &register_buf, 4) != 4) {
        mxt_verb(mxt->ctx, "I2C retry");
        usleep(I2C_RETRY_DELAY);

        if (write(fd, &register_buf, 4) != 4) {
          mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
          ret = mxt_errno_to_rc(errno);
          goto close;
        }
      }

      err = debugfs_update_seq_num(mxt, (uint8_t) tx_seq_num);

      if (err) {
        mxt_dbg(mxt->ctx, "Failed to get the tx seq num");
        goto close;
      }
    }

  ssize_t read_rc;
  read_rc = read(fd, buf, count);
  if (read_rc < 0) {
    mxt_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  } else if (read_rc == 0) {
    /* end of file */
    ret = MXT_ERROR_IO;
    goto close;
  } else {
    *bytes_read = (size_t)read_rc;
    ret = MXT_SUCCESS;
  }

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Read register from MXT chip with encryption
/// \return #mxt_rc
int i2c_dev_read_register(struct mxt_device *mxt, unsigned char *buf,
                          int start_register, int count, size_t *bytes_read)
{
  int fd = -ENODEV;
  unsigned char *readbuf;
  char register_buf[4];
  unsigned char *databuf;
  uint8_t tx_header = 0x04;
  uint8_t crc_data = 0;
  uint8_t tx_value = 0; 
  uint16_t bytesToRead = 0;
  uint16_t bytesRead = 0;
  uint16_t tx_seq_num;
  uint16_t msg_length = 0;
  size_t w_hdr_size = 0;
  int buf_count = 0;
  int ret, err;
  int i, j;
  uint16_t test_reg = 0;

  /* Make copy of bytes to read */
  bytesToRead = count;
  msg_length = count;
  buf_count = count + 4;

  readbuf = calloc(count, sizeof(unsigned char));
  if (!readbuf)
    return MXT_ERROR_NO_MEM;

  databuf = calloc(count, sizeof(unsigned char));
  if (!databuf)
    return MXT_ERROR_NO_MEM;

  /* Modify the block size to write */

  if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED) &&
    (start_register != 0x0000) && (start_register > mxt->mxt_dev.t38_addr + mxt->mxt_dev.t38_size)) {

    if (count > mxt->mxt_enc.enc_blocksize) {
      msg_length = mxt->mxt_enc.enc_blocksize;
    } else if (count < 16) {
      msg_length = 16;  /* Must to be multiple of 16 bytes between 16 < 48 max blocksize */
    } else if ((count % 16 != 0) && (count != 0)) {
            /* Adjust size, multiple of 16 if size < 16 bytes*/
      msg_length = (16 - (count % 16) + count);
    }
  } else {
    if (count > mxt->ctx->i2c_block_size) {
      msg_length = mxt->ctx->i2c_block_size;
    }
  }

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;
  
  /* Normal and encrypted reads, added datasize*/
  if (mxt->mxt_crc.crc_enabled != true) {

    register_buf[0] = start_register & 0xff;
    register_buf[1] = (start_register >> 8) & 0xff;
    
    w_hdr_size = 2;

    if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED)) {
      register_buf[2] = 0x00;
      register_buf[3] = 0x00;
      w_hdr_size = 4;
    }
    

    if (write(fd, &register_buf, w_hdr_size) != w_hdr_size) {
      mxt_verb(mxt->ctx, "I2C retry");
      usleep(I2C_RETRY_DELAY);

      if (write(fd, &register_buf, w_hdr_size) != w_hdr_size) {
        mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
        ret = mxt_errno_to_rc(errno);
        goto close;
      }
    }
  } else { /* HA reads */
      if (mxt->debug_fs.enabled == true) {
        err = debugfs_set_irq(mxt, false);

        if (err)
          mxt_dbg(mxt->ctx, "Could not disable IRQ");

        err = debugfs_get_tx_seq_num(mxt, &tx_seq_num);

        if (err) {
          mxt_dbg(mxt->ctx, "Failed to get the tx seq num");
        } 
      }

      register_buf[0] = start_register & 0xff;
      register_buf[1] = (start_register >> 8) & 0xff;
      register_buf[2] = tx_seq_num;  //Must be tx seq no

      //Calculate CRC byte
      for (i = 0; i < (tx_header - 1); i++) {
        crc_data = mxt_calc_crc8(crc_data, register_buf[i]);
      }

      register_buf[3] = crc_data;   //CRC8 bit checksum

      if (write(fd, &register_buf, 4) != 4) {
        mxt_verb(mxt->ctx, "I2C retry");
        usleep(I2C_RETRY_DELAY);

        if (write(fd, &register_buf, 4) != 4) {
          mxt_err(mxt->ctx, "Error %s (%d) writing to i2c", strerror(errno), errno);
          ret = mxt_errno_to_rc(errno);
          goto close;
        }
      }

      err = debugfs_update_seq_num(mxt, (uint8_t) tx_seq_num);

      if (err) {
        mxt_dbg(mxt->ctx, "Failed to get the tx seq num");
        goto close;
      }
    } /* Finish HA reads */

  do {
    ssize_t read_rc;
    read_rc = read(fd, readbuf, msg_length);
    if (read_rc < 0) {
      mxt_err(mxt->ctx, "Error %s (%d) reading from i2c", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    } else if (read_rc == 0) {
      /* end of file */
      ret = MXT_ERROR_IO;
      goto close;
    } else {

      memcpy(&databuf[bytesRead], &readbuf[0], msg_length);

      *bytes_read = (size_t)read_rc;
      bytesRead += msg_length;
      bytesToRead -= msg_length;

      if (bytesToRead < msg_length) {
        if (bytesToRead < 16 && bytesToRead !=0) {
          msg_length = 16;
        } else {
          msg_length = bytesToRead;
        }
      }

    }
  } while (bytesToRead > 0);

  memcpy(buf, databuf, count);

ret = MXT_SUCCESS;

close:
  databuf = NULL;
  readbuf = NULL;
  free(databuf);
  free(readbuf);
  close(fd);
  return ret;
}
//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int i2c_dev_write_crc(struct mxt_device *mxt, unsigned char const *val,
                           int start_register, size_t datalength)
{
  int fd = -ENODEV;
  int count;
  int ret, err, i, j;
  uint16_t msg_count;
  unsigned char *buf;
  uint8_t msgbuf[15];
  uint16_t tx_seq_num;
  uint8_t txbuf[4];
  uint8_t tx_header = 4;
  uint8_t crc_data = 0;
  uint16_t bytesToWrite = 0;
  uint16_t message_length = 0;
  uint8_t max_data_length = 11;
  uint16_t write_addr = 0;
  uint16_t bytesWritten = 0;
  uint16_t retry_counter = 0;

  err = debugfs_set_irq(mxt, false);

  if (err)
    mxt_dbg(mxt->ctx, "Could not disable IRQ");

  err = debugfs_get_tx_seq_num(mxt, &tx_seq_num);

  if (err) {
    mxt_dbg(mxt->ctx, "i2c-dev: Failed to get the tx seq num");
  } 
  
  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  count = datalength + 2;     //Total bytes + header
  bytesToWrite = datalength;  //Total bytes

  //Limit the size for HA parts

  if (datalength > max_data_length) {
    message_length = max_data_length;
  } else {
    message_length = datalength;
  }

  msg_count = message_length + 4;

  buf = (unsigned char *)calloc(count, sizeof(unsigned char));

  if (!(datalength == 0x00))  //Prevent memory crash
      memcpy(buf, val, datalength);

  do {

    write_addr = start_register + bytesWritten;

    mxt_dbg(mxt->ctx, "i2c_dev write: Reg Addr: %x\n", write_addr);

    msgbuf[0] = write_addr & 0xff;
    msgbuf[1] = (write_addr >> 8) & 0xff;
    msgbuf[msg_count-2] = tx_seq_num;

    j = 0;

    while (j < message_length) {
      msgbuf[2 + j] = buf[bytesWritten + j];
      j++;
    }

    crc_data = 0;

    //Calculate CRC byte
    for (i = 0; i < (msg_count - 1); i++) {
        crc_data = mxt_calc_crc8(crc_data, msgbuf[i]);
        mxt_dbg(mxt->ctx, "Write CRC: Data[%d] = %x, crc = 0x%x\n",
        i, msgbuf[i], crc_data);
    }

    msgbuf[msg_count - 1] = crc_data;   //Insert CRC, end of message

    ret = write(fd, msgbuf, msg_count);

    if (ret == msg_count) {

      if(tx_seq_num == 255)
        tx_seq_num = 0;
      else
        tx_seq_num++;
    
      ret = 0;
      bytesWritten = bytesWritten + message_length;
      bytesToWrite = bytesToWrite - message_length;

      retry_counter = 0;

      if (bytesToWrite < message_length){
        message_length = bytesToWrite;
        msg_count = message_length + 4;
      }
    }

    retry_counter++;

    if (retry_counter == 10)
      break;

} while (bytesToWrite > 0);

  tx_seq_num--; //Minus 1 before updating

  ret = debugfs_update_seq_num(mxt, (uint8_t) tx_seq_num);

  if ((mxt->mxt_crc.reset_triggered == false) && (mxt->mxt_crc.config_triggered == false) &&
    mxt->mxt_crc.processing_msg == false){

    err = debugfs_set_irq(mxt, true);

      if (err)
      mxt_dbg(mxt->ctx, "Could not disable IRQ");

}
  buf = NULL;
  free(buf);
  close(fd);
  return ret;
}

/* TBD - Keep code */
//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int i2c_dev_write_reg(struct mxt_device *mxt, unsigned char const *val,
                           int start_register, size_t datalength)
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

  buf = NULL;
  free(buf);
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int i2c_dev_write_register(struct mxt_device *mxt, unsigned char const *val,
                           int start_register, size_t datalength)
{
  int fd = -ENODEV;
  int count;
  int ret;
  unsigned char *abuf;
  unsigned char *tbuf;
  uint16_t enc_datasize = 0;
  uint16_t data_size = 0;
  uint16_t bytesToWrite = 0;
  uint16_t msg_length = 0;
  uint16_t bytesWritten = 0;
  uint16_t write_addr = 0;
  uint8_t retry_counter = 0;
  uint16_t t38_addr = 0;
  uint16_t t38_size = 0;
  size_t msg_count;
  uint16_t write_offset = 0;

  ret = open_and_set_slave_address(mxt, &fd);
  if (ret)
    return ret;

  if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED)) {
    if (mxt->mxt_enc.enc_cfg_write == true) {
      msg_length = datalength - 2;
      bytesToWrite = datalength - 2;
    } else {
      msg_length = datalength;
      bytesToWrite = datalength;
    }
  } else {
    msg_length = datalength;
    bytesToWrite = datalength;
  }

  /* Allocate enough space for reg_addr, datasize, and message */
  count = datalength + 4;

  abuf = (unsigned char *)calloc(count, sizeof(unsigned char));
  if (!abuf)
    return MXT_ERROR_NO_MEM;

  tbuf = (unsigned char *)calloc(count, sizeof(unsigned char));
  if (!tbuf)
    return MXT_ERROR_NO_MEM;

  /* Limit the number of bytes to block size if encrypted */  
  if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED) &&
    (mxt->mxt_enc.enc_cfg_write == true)) {
      enc_datasize = (((val[1] >> 8) & 0xff00) | (val[0] & 0xff));
      memcpy(&tbuf[0], &val[2], bytesToWrite);

    /* Limit msg_length if larger than blocksize */
    if (msg_length > mxt->mxt_enc.enc_blocksize) {
      msg_length = mxt->mxt_enc.enc_blocksize;
    }
  } else {
    memcpy(&tbuf[0], &val[0], bytesToWrite);
  }

  do {

    write_addr = start_register + bytesWritten;

    abuf[0] = write_addr & 0xff;
    abuf[1] = (write_addr >> 8) & 0xff;

    if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED)) {
      if (write_addr >= ((mxt->mxt_dev.t38_addr + mxt->mxt_dev.t38_size))) {
        if (enc_datasize < mxt->mxt_enc.enc_blocksize) {
          data_size = enc_datasize;
        } else {
          data_size = mxt->mxt_enc.enc_blocksize;
        }
      } else  {
          data_size = 0x0000;
      }

      abuf[2] = data_size & 0xff;
      abuf[3] = (data_size >> 8) & 0xff;

      msg_count = msg_length + 4;

      /* Copy all data to buf after addr and datasize bytes */
      /* write offset use to skip embedded datasize */
      memcpy(&abuf[4], (tbuf + bytesWritten), msg_count);

    } else {
      msg_count = msg_length + 2;
      memcpy(&abuf[2], (tbuf + bytesWritten), msg_count);
    }

    ret = write(fd, abuf, msg_count);

    if (ret == msg_count) {
      ret = MXT_SUCCESS;
      bytesWritten += msg_length;
      bytesToWrite -= msg_length;
      enc_datasize -= msg_length;

      if (bytesToWrite < msg_length) {
        msg_length = bytesToWrite;
      }

      retry_counter = 0;

    } else {
        mxt_warn(mxt->ctx, "Warning error %s, (%d) occurred while writing to i2c. Retrying", 
          strerror(errno), errno);
        retry_counter++;
        usleep(I2C_RETRY_DELAY);    
    }

    if (retry_counter == 3) {
      mxt_err(mxt->ctx, "Error %s, (%d) writing to i2c", strerror(errno), errno);
      break;
    }

  } while (bytesToWrite > 0);

  
  abuf = NULL;
  tbuf = NULL;
  free(abuf);
  free(tbuf);
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
int i2c_dev_bootloader_write(struct mxt_device *mxt, unsigned char const *buf,
                             int count, size_t *bytes_read)
{
  int fd = -ENODEV;
  int ret;

  if (count > mxt->ctx->i2c_block_size)
    count = mxt->ctx->i2c_block_size;

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

  *bytes_read = count;
  close(fd);
  return ret;
}
