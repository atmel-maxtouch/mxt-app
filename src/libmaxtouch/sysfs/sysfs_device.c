//------------------------------------------------------------------------------
/// \file   sysfs_device.c
/// \brief  MXT device low level access via I2C
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

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <malloc.h>
#include <stdio.h>
#include <libgen.h>
#include <stdlib.h>

#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"
#include "sysfs_device.h"
#include "dmesg.h"

#define SYSFS_I2C_ROOT "/sys/bus/i2c/drivers/"
#define SYSFS_SPI_ROOT "/sys/bus/spi/drivers/"
#define SYSFS_I2C_DRIVER_DIR /sys/bus/i2c/drivers/atmel_mxt_ts/
#define CONVERTTOSTRING(x) #x
#define TOSTRING(x) CONVERTTOSTRING(x)


//******************************************************************************
/// \brief Construct filename of path
static char *make_path(struct mxt_device *mxt, const char *filename)
{
  snprintf(mxt->sysfs.temp_path, mxt->sysfs.path_max,
           "%s/%s", mxt->conn->sysfs.path, filename);

  return mxt->sysfs.temp_path;
}

//******************************************************************************
/// \brief Open sysfs MSG notify attribute
/// \return #mxt_rc
static int sysfs_open_notify_fd(struct mxt_device *mxt)
{
  char *filename = make_path(mxt, "debug_notify");

  mxt->sysfs.debug_notify_fd = open(filename, O_RDONLY);
  if (mxt->sysfs.debug_notify_fd < 0) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Reopen sysfs MSG notify attribute
static void sysfs_reopen_notify_fd(struct mxt_device *mxt)
{
  uint16_t val;

  close(mxt->sysfs.debug_notify_fd);
  sysfs_open_notify_fd(mxt);
  read(mxt->sysfs.debug_notify_fd, &val, 2);
}

//******************************************************************************
/// \brief Create sysfs connection info
/// \return #mxt_rc
static int sysfs_new_connection(struct libmaxtouch_ctx *ctx,
                                struct mxt_conn_info **conn,
                                const char *dir, bool acpi)
{
  int ret = 0;
  struct mxt_conn_info *c;

 if (strncmp(dir, TOSTRING(SYSFS_I2C_DRIVER_DIR), 33) == 0)
     ret = mxt_new_conn(&c, E_SYSFS_I2C);
  else 
    ret = mxt_new_conn(&c, E_SYSFS_SPI);
 
  if (ret)
    return ret;

  c->sysfs.path = (char *)calloc(strlen(dir) + 1, sizeof(char));
  memcpy(c->sysfs.path, dir, strlen(dir) + 1);

  c->sysfs.acpi = acpi;

  *conn = c;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Get fd for select
int sysfs_get_debug_v2_fd(struct mxt_device *mxt)
{
  return mxt->sysfs.debug_notify_fd;
}

//******************************************************************************
/// \brief Check sysfs device directory for correct attributes
/// \return #mxt_rc
static int scan_sysfs_directory(struct libmaxtouch_ctx *ctx,
                                struct mxt_conn_info **conn,
                                struct dirent *dev_dir,
                                const char *dir,
                                bool acpi)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  bool mem_access_found = false;
  bool debug_found = false;
  bool debug_v2_found = false;
  int ret = 0;

  length = strlen(dir) + strlen(dev_dir->d_name) + 2;

  if ((pszDirname = (char *)calloc(length, sizeof(char))) == NULL) {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  snprintf(pszDirname, length, "%s/%s", dir, dev_dir->d_name);

  pDirectory = opendir(pszDirname);
  if (!pDirectory) {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  while ((pEntry = readdir(pDirectory)) != NULL) {
    if (!strcmp(pEntry->d_name, "mem_access")) {
      mxt_dbg(ctx, "Found mem_access interface at %s/mem_access", pszDirname);
      mem_access_found = true;
    } else if (!strcmp(pEntry->d_name, "debug_enable")) {
      mxt_dbg(ctx, "Found debug_enable interface at %s/debug_enable", pszDirname);
      debug_found = true;
    } else if (!strcmp(pEntry->d_name, "debug_msg")) {
      mxt_dbg(ctx, "Found Debug V2 at %s/debug_msg", pszDirname);
      debug_v2_found = true;
    }
  }

  /* If device found, store it and return success */
  if (mem_access_found && (debug_found || debug_v2_found)) {
    ctx->scan_count++;

    if (ctx->query) {
      printf("sysfs:%s Atmel %s interface\n", pszDirname,
             debug_v2_found ? "Debug V2" : "Debug");
    } else {
      ret = sysfs_new_connection(ctx, conn, pszDirname, acpi);
      mxt_dbg(ctx, "Found new device connection at: %s", pszDirname);
      goto close;
    }
  } else {
    mxt_verb(ctx, "Ignoring %s", pszDirname);
  }

  ret = MXT_ERROR_NO_DEVICE;

close:
  (void)closedir(pDirectory);
free:
  free(pszDirname);

  return ret;
}

//******************************************************************************
/// \brief  Process a driver directory in sysfs looking for MXT devices
/// \return #mxt_rc
static int scan_driver_directory(struct libmaxtouch_ctx *ctx,
                                 struct mxt_conn_info **conn,
                                 const char *path, struct dirent *dir)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  unsigned char acpi[PATH_MAX];
  int adapter;
  unsigned int address;
  unsigned int bus_num;
  unsigned int cs_num;
  int ret = 0;

  length = strlen(path) + strlen(dir->d_name) + 1;

  if ((pszDirname = (char *)calloc(length, sizeof(char))) == NULL) {
    mxt_err(ctx, "calloc failure");
    return MXT_ERROR_NO_MEM;
  }

  snprintf(pszDirname, length, "%s%s", path, dir->d_name);

  pDirectory = opendir(pszDirname);
  if (pDirectory == NULL) {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  while ((pEntry = readdir(pDirectory)) != NULL) {
    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue;

    if (sscanf(pEntry->d_name, "%d-%x", &adapter, &address) == 2) {
      ret = scan_sysfs_directory(ctx, conn, pEntry, pszDirname, false);
      if (ret != MXT_ERROR_NO_DEVICE) goto close;

    } else if (sscanf(pEntry->d_name, "i2c-%s", acpi) == 1) {
      ret = scan_sysfs_directory(ctx, conn, pEntry, pszDirname, true);
      if (ret != MXT_ERROR_NO_DEVICE) goto close;

    } else if (sscanf(pEntry->d_name, "spi%d.%d", &bus_num, &cs_num) == 2) {
      ret = scan_sysfs_directory(ctx, conn, pEntry, pszDirname, false);
      if (ret != MXT_ERROR_NO_DEVICE) goto close;

    }
  }

  ret = MXT_ERROR_NO_DEVICE;

close:
  (void)closedir(pDirectory);
free:
  free(pszDirname);

  return ret;
}

//******************************************************************************
/// \brief  Scan for devices
/// \return #mxt_rc
int sysfs_scan(struct libmaxtouch_ctx *ctx, struct mxt_conn_info **conn)
{
  struct dirent *pEntry;
  struct mxt_conn_info *cn;
  DIR *pDirectory;
  int ret = 0;

  cn = *conn;

  if (cn->type == E_SYSFS_SPI) {

    pDirectory = opendir(SYSFS_SPI_ROOT);

    while ((pEntry = readdir(pDirectory)) != NULL) {
      if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
        continue;

      ret = scan_driver_directory(ctx, conn, SYSFS_SPI_ROOT, pEntry);

      /* If found or memory error close, otherwise no device check I2C */ 
      if (ret != MXT_ERROR_NO_DEVICE) 
        goto close;
    }
  }

    /* Check for I2C */
    pDirectory = opendir(SYSFS_I2C_ROOT);
  
    if (!pDirectory) {
        goto close;
    }

    while ((pEntry = readdir(pDirectory)) != NULL) {
      if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
        continue;

      ret = scan_driver_directory(ctx, conn, SYSFS_I2C_ROOT, pEntry);

      if (ret != MXT_ERROR_NO_DEVICE)
        goto close;
    }
  
close:
  (void)closedir(pDirectory);

  return ret;
}

//******************************************************************************
/// \brief  Open I2C device
/// \return #mxt_rc
int sysfs_open_i2c(struct mxt_device *mxt)
{
  struct sysfs_conn_info *conn = &mxt->conn->sysfs;
  char *filename;
  struct stat filestat;
  int ret;

  mxt->sysfs.path_max = strlen(conn->path) + 20;

  // Allocate temporary path space
  mxt->sysfs.temp_path = calloc(mxt->sysfs.path_max + 1, sizeof(char));
  if (!mxt->sysfs.temp_path)
    return MXT_ERROR_NO_MEM;

  // Cache memory access path for fast access
  mxt->sysfs.mem_access_path = calloc(mxt->sysfs.path_max + 1, sizeof(char));
  if (!mxt->sysfs.mem_access_path)
    return MXT_ERROR_NO_MEM;

  snprintf(mxt->sysfs.mem_access_path, mxt->sysfs.path_max,
           "%s/mem_access", conn->path);

  // Check whether debug v2 or not
  filename = make_path(mxt, "debug_msg");

  ret = stat(filename, &filestat);
  if (ret < 0) {
    if (errno == ENOENT) {
      mxt->sysfs.debug_v2 = false;
    } else {
      mxt_err(mxt->ctx, "Could not stat %s, error %s (%d)",
              filename, strerror(errno), errno);
      return mxt_errno_to_rc(errno);
    }
  } else {
    mxt->sysfs.debug_v2 = true;
  }

  mxt_info(mxt->ctx, "\nDevice registered on sysfs path:%s", conn->path);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Open SPI device
/// \return #mxt_rc
int sysfs_open_spi(struct mxt_device *mxt)
{
  struct sysfs_conn_info *conn = &mxt->conn->sysfs;
  char *filename;
  struct stat filestat;
  int ret;

  mxt->sysfs.path_max = strlen(conn->path) + 20;

  // Allocate temporary path space
  mxt->sysfs.temp_path = calloc(mxt->sysfs.path_max + 1, sizeof(char));
  if (!mxt->sysfs.temp_path)
    return MXT_ERROR_NO_MEM;

  // Cache memory access path for fast access
  mxt->sysfs.mem_access_path = calloc(mxt->sysfs.path_max + 1, sizeof(char));
  if (!mxt->sysfs.mem_access_path)
    return MXT_ERROR_NO_MEM;

  snprintf(mxt->sysfs.mem_access_path, mxt->sysfs.path_max,
           "%s/mem_access", conn->path);

  // Check whether debug v2 or not
  filename = make_path(mxt, "debug_msg");

  ret = stat(filename, &filestat);
  if (ret < 0) {
    if (errno == ENOENT) {
      mxt->sysfs.debug_v2 = false;
    } else {
      mxt_err(mxt->ctx, "Could not stat %s, error %s (%d)",
              filename, strerror(errno), errno);
      return mxt_errno_to_rc(errno);
    }
  } else {
    mxt->sysfs.debug_v2 = true;
  }

  mxt_info(mxt->ctx, "\nDevice registered on sysfs path:%s", conn->path);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Release device
void sysfs_release(struct mxt_device *mxt)
{
  if (mxt) {
    free(mxt->sysfs.temp_path);
    mxt->sysfs.temp_path = NULL;

    free(mxt->sysfs.debug_v2_msg_buf);
    mxt->sysfs.debug_v2_msg_buf = NULL;
  }
}

//******************************************************************************
/// \brief Open memory access file
/// \return #mxt_rc
static int open_device_file(struct mxt_device *mxt, int *fd_out)
{
  int fd;

  // Check device is initialised
  if (!mxt || !mxt->sysfs.mem_access_path) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

  fd = open(mxt->sysfs.mem_access_path, O_RDWR);

  if (fd < 0) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)",
            mxt->sysfs.mem_access_path, strerror(errno), errno);

    return mxt_errno_to_rc(errno);
  }

  *fd_out = fd;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  sysfs bootloader read from MXT chip
/// \return #mxt_rc
int sysfs_bootloader_read(struct mxt_device *mxt, unsigned char *buf,
                        int count)
{
  uint16_t start_register = 0x0000;
  size_t bytes_read;
  int ret;

  ret = sysfs_read_register(mxt, buf, start_register, count, &bytes_read);

  return ret;
}

//******************************************************************************
/// \brief  sysfs bootloader write from MXT chip
/// \return #mxt_rc
int sysfs_bootloader_write(struct mxt_device *mxt, unsigned const char *buf, 
                        int count)
{
  int ret;
  size_t *bytes;

  ret =  sysfs_write_register(mxt, buf, 0x0000, count);

  return ret;

}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int sysfs_read_register(struct mxt_device *mxt, unsigned char *buf,
                        int start_register, size_t count, size_t *bytes_read)
{
  int fd = -ENODEV;
  struct mxt_id_info *id = mxt->info.id;
  uint8_t register_buf[4]; 
  int crc_bytes;
  uint8_t crc_data;
  uint8_t tx_header = 0x03;
  uint32_t addr_offset = 0;
  ssize_t read_rc;
  int ret;
  int i;

  ret = open_device_file(mxt, &fd);
  if (ret)
    return ret;

if ((start_register & 0xffff) != 0x0000) {
  if (lseek(fd, start_register, 0) < 0) {
    mxt_err(mxt->ctx, "lseek error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }
}

  *bytes_read = 0;
  while (*bytes_read < count) {
    ret = read(fd, buf + *bytes_read, count - *bytes_read);
    if (ret == 0) {
      ret = MXT_ERROR_IO;
      goto close;
    } else if (ret < 0) {
      mxt_err(mxt->ctx, "read error %s (%d)", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    }

    *bytes_read += ret;
  }

  ret = MXT_SUCCESS;

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int sysfs_write_register(struct mxt_device *mxt, unsigned char const *buf,
                         int start_register, size_t count)
{
  int fd = -ENODEV;
  int ret;
  size_t bytes_written;

  ret = open_device_file(mxt, &fd);
  if (ret)
    return ret;

  if (lseek(fd, start_register, 0) < 0) {
    mxt_err(mxt->ctx, "lseek error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  bytes_written = 0;
  while (bytes_written < count) {
    ret = write(fd, buf+bytes_written, count - bytes_written);
    if (ret == 0) {
      ret = MXT_ERROR_IO;
      goto close;
    } else if (ret < 0) {
      mxt_err(mxt->ctx, "Error %s (%d) writing to register", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    }

    bytes_written += ret;
  }

  ret = MXT_SUCCESS;

close:
  close(fd);

  return ret;
}


//******************************************************************************
/// \brief  Write boolean to file as ASCII 0/1
/// \param  mxt Device context
/// \param  filename Name of file to write
/// \param  value Value to write
/// \return #mxt_rc
static int write_boolean_file(struct mxt_device *mxt, const char *filename,
                              bool value)
{
  FILE *file;

  file = fopen(filename, "w+");
  if (!file) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  if (value == true) {
    fputs("1", file);
  } else {
    fputs("0", file);
  }

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read boolean from file as ASCII 0/1
/// \param  mxt Device context
/// \param  filename Name of file to read
/// \param  value Value read from file
/// \return #mxt_rc
static int read_boolean_file(struct mxt_device *mxt, char *filename,
                             bool *value)
{
  FILE *file;
  char val;
  int ret;

  file = fopen(filename, "r");
  if (!file) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  ret = fread(&val, sizeof(char), 1, file);
  if (ret < 0) {
    mxt_err(mxt->ctx, "Error reading files");
    return MXT_ERROR_IO;
  }

  if (val == 49) { // ASCII '1'
    *value = true;
  } else {
    *value = false;
  }

  fclose(file);

  return MXT_SUCCESS;
}


//******************************************************************************
/// \brief  Write sysfs byte to file
/// \param  mxt Device context
/// \param  filename Name of file to write
/// \param  value Value to write
/// \return #mxt_rc
static int write_sysfs_byte(struct mxt_device *mxt, const char *filename,
                              uint8_t value)
{
  
  FILE *file;
  int ret;
  char in_str[3];

  file = fopen(filename, "r+");
  if (!file) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  memset(in_str, '\0', 3);    //Set array to NULL

  sprintf(in_str,"%x", value);   //String copy unsigned hex value to array

  ret = fwrite(in_str, sizeof(char), strlen(in_str), file);
    if (ret == 0) {
      ret = MXT_ERROR_IO;
      goto close;
    } else if (ret < 0) {
      mxt_err(mxt->ctx, "Error %s (%d) writing to register", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    }

close:

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read sysfs byte to file
/// \param  mxt Device context
/// \param  filename Name of file to read
/// \param  value Value read from file
/// \return #mxt_rc
static int read_sysfs_byte(struct mxt_device *mxt, char *filename,
                             uint16_t *value)
{
  FILE *file;
  char val[3];   //Minimum 3 to hold NULL/EOF value
  uint16_t dec_val;
  int ret;
  int i;

  file = fopen(filename, "r");
  if (!file) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)\n", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  ret = fread(val, sizeof(char), 6, file); 

  if (ret < 0) {
   mxt_err(mxt->ctx, "Error reading files");
    return MXT_ERROR_IO;
  }

  dec_val = (int)strtol(val, NULL, 16); //Convert to a integer

  *value = dec_val;

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Set debug irq state
/// \param  mxt Device context
/// \param  debug_state true = irq enabled, false = irq disabled
/// \return #mxt_rc
int sysfs_reset_chip(struct mxt_device *mxt, bool reset_chip)
{
  int ret;

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

    ret = write_boolean_file(mxt, make_path(mxt, "mxt_reset"), reset_chip);

  return ret;
}

//******************************************************************************
/// \brief  Set debug state
/// \param  mxt Device context
/// \param  debug_state true = debug enabled, false = debug disabled
/// \return #mxt_rc
int sysfs_set_debug(struct mxt_device *mxt, bool debug_state)
{
  int ret;

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

  if (mxt->sysfs.debug_v2 == true) {
    ret = write_boolean_file(mxt, make_path(mxt, "debug_v2_enable"), debug_state);
    if (ret)
      ret = write_boolean_file(mxt, make_path(mxt, "debug_enable"), debug_state);

    if (debug_state) {
      ret = sysfs_open_notify_fd(mxt);
      if (ret)
        return ret;
    } else {
      close(mxt->sysfs.debug_notify_fd);
    }
  } else {
    if (debug_state) {
      ret = dmesg_alloc_buffer(mxt);
      if (ret)
        return ret;
    } else {
      dmesg_free_buffer(mxt);
    }

    ret = write_boolean_file(mxt, make_path(mxt, "debug_enable"), debug_state);
  }

  return ret;
}

//******************************************************************************
/// \brief  Set debug irq state
/// \param  mxt Device context
/// \param  debug_state true = irq enabled, false = irq disabled
/// \return #mxt_rc
int sysfs_set_debug_irq(struct mxt_device *mxt, bool debug_state)
{
  int ret;

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

    ret = write_boolean_file(mxt, make_path(mxt, "debug_irq"), debug_state);

  return ret;
}

//******************************************************************************
/// \brief  Get debug message string
/// \param  mxt Device context
/// \return C string or NULL
char *sysfs_get_msg_string_v2(struct mxt_device *mxt)
{
  int ret, i;
  int size;
  size_t length;
  unsigned char databuf[20];
  static char msg_string[255];

  ret = sysfs_get_msg_bytes_v2(mxt, &databuf[0], sizeof(databuf), &size);
  if (ret)
    return NULL;

  length = snprintf(msg_string, sizeof(msg_string), "MXT MSG:");
  for (i = 0; i < size; i++) {
    length += snprintf(msg_string + length, sizeof(msg_string) - length,
                       "%02X ", databuf[i]);
  }

  return &msg_string[0];
}

//******************************************************************************
/// \brief Get debug message bytes
/// \param  mxt Device context
/// \param  buf  Pointer to buffer
/// \param  buflen  Length of buffer
/// \param  count length of message in bytes
/// \return #mxt_rc
int sysfs_get_msg_bytes_v2(struct mxt_device *mxt, unsigned char *buf,
                           size_t buflen, int *count)
{
  uint16_t t5_size;

  if (!mxt->sysfs.debug_v2_msg_buf) //If nothing in buffer, return error
    return MXT_INTERNAL_ERROR;

  if (mxt->mxt_crc.crc_enabled == true)
    t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5);  //Must match Linux driver
  else 
    t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;

  if (buflen < t5_size)
    return MXT_ERROR_NO_MEM;

  if (mxt->sysfs.debug_v2_msg_ptr > mxt->sysfs.debug_v2_msg_count)
    return MXT_INTERNAL_ERROR;

  memcpy(buf,
         mxt->sysfs.debug_v2_msg_buf + mxt->sysfs.debug_v2_msg_ptr * t5_size,
         t5_size);

  mxt->sysfs.debug_v2_msg_ptr++;

  *count = t5_size;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Reset debug messages
/// \param  mxt Device context
int sysfs_msg_reset_v2(struct mxt_device *mxt)
{
  free(mxt->sysfs.debug_v2_msg_buf);
  mxt->sysfs.debug_v2_msg_buf = NULL;
  
  usleep(100000);   /* delay to get rid of messages */
  
  if (mxt->sysfs.debug_v2_msg_buf != NULL);
    free(mxt->sysfs.debug_v2_msg_buf);

  return 0;
}

//******************************************************************************
/// \brief Return whether device has debug V2 support
/// \param  mxt Device context
bool sysfs_has_debug_v2(struct mxt_device *mxt)
{
  return mxt->sysfs.debug_v2;
}

//******************************************************************************
/// \brief  Get messages (V2 interface)
/// \param  mxt Device context
/// \param  count Number of messages retrieved
/// \return #mxt_rc
int sysfs_get_msgs_v2(struct mxt_device *mxt, int *count)
{
  int num_bytes;
  uint16_t t5_size;
  char *filename;
  struct stat filestat;
  int ret;
  int fd;

  sysfs_reopen_notify_fd(mxt);

  filename = make_path(mxt, "debug_msg");

  ret = stat(filename, &filestat);
  if (ret < 0) {
    mxt_err(mxt->ctx, "Could not stat %s, error %s (%d)",
            filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  mxt->sysfs.debug_v2_size = filestat.st_size;

  if (mxt->sysfs.debug_v2_msg_buf) {
    free(mxt->sysfs.debug_v2_msg_buf);
    mxt->sysfs.debug_v2_msg_buf = NULL;
  }

  mxt->sysfs.debug_v2_msg_buf = calloc(mxt->sysfs.debug_v2_size, sizeof(uint8_t));

  fd = open(filename, O_RDWR);
  if (fd < 0) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  if (mxt->mxt_crc.crc_enabled == true)
    t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5);  // Must match Linux driver being used
   else 
    t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;  //Size based on chip (non-CRC)

  num_bytes = read(fd, mxt->sysfs.debug_v2_msg_buf, mxt->sysfs.debug_v2_size);
  if (num_bytes < 0) {
    mxt_err(mxt->ctx, "read error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  mxt->sysfs.debug_v2_msg_count = num_bytes / t5_size;
  mxt->sysfs.debug_v2_msg_ptr = 0;

  ret = MXT_SUCCESS;
  *count = mxt->sysfs.debug_v2_msg_count;
  mxt_verb(mxt->ctx, "msg_count = %d", mxt->sysfs.debug_v2_msg_count);

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Get debug state
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_get_debug(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_boolean_file(mxt, make_path(mxt, "debug_enable"), value);
}

//******************************************************************************
/// \brief  Set in_bootloader state
/// \param  mxt Device context
/// \param  debug_state true = irq enabled, false = irq disabled
/// \return #mxt_rc
int sysfs_set_bootloader(struct mxt_device *mxt, bool value)
{
  int ret;

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

    ret = write_boolean_file(mxt, make_path(mxt, "in_bootloader"), value);

  return ret;
}

//******************************************************************************
/// \brief  Get in_bootloader state
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_get_bootloader(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_boolean_file(mxt, make_path(mxt, "in_bootloader"), value);
}

//******************************************************************************
/// \brief  Get crc status
/// \param  mxt Device context
/// \param  value true (debug HA found) or false (HA not found)
/// \return #mxt_rc
int sysfs_get_crc_enabled(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_boolean_file(mxt, make_path(mxt, "crc_enabled"), value);
}

//******************************************************************************
/// \brief  Get tx seq number value
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_get_tx_seq_num(struct mxt_device *mxt, uint16_t *value)
{

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_sysfs_byte(mxt, make_path(mxt, "tx_seq_num"), value);
}

//******************************************************************************
/// \brief  Set tx seq number value
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_set_tx_seq_num(struct mxt_device *mxt, uint8_t value)
{

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return write_sysfs_byte(mxt, make_path(mxt, "tx_seq_num"), value);
}

//******************************************************************************
/// \brief  Get debug IRQ state
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_get_debug_irq(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_boolean_file(mxt, make_path(mxt, "debug_irq"), value);
}

//******************************************************************************
/// \brief  Get sysfs directory
/// \param  mxt Device context
/// \return location of the sysfs interface files
char *sysfs_get_directory(struct mxt_device *mxt)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return 0;
  }

  return mxt->conn->sysfs.path;
}

//******************************************************************************
/// \brief  Get I2C adapter and address
/// \param  conn mxt connection info
/// \param  adapter OUT i2c adapter number
/// \param  address OUT i2c address
/// \return #mxt_rc
int sysfs_get_i2c_address(struct libmaxtouch_ctx *ctx,
                          struct mxt_conn_info *conn,
                          int *adapter, int *address)
{
  int ret;

  if (conn->sysfs.acpi)
    return MXT_ERROR_NOT_SUPPORTED;

  ret = sscanf(basename(conn->sysfs.path), "%d-%x", adapter, address);
  if (ret != 2) {
    mxt_err(ctx, "Couldn't parse sysfs path for adapter/address");
    return MXT_INTERNAL_ERROR;
  }

  return MXT_SUCCESS;
}
