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

#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"
#include "dmesg.h"
#include "sysfs_device.h"

#define SYSFS_I2C_ROOT "/sys/bus/i2c/drivers/"

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
                                const char *dirname)
{
  int ret;
  struct mxt_conn_info *c;

  ret = mxt_new_conn(&c, E_SYSFS);
  if (ret)
    return ret;

  c->sysfs.path = (char *)calloc(strlen(dirname) + 1, sizeof(char));
  memcpy(c->sysfs.path, dirname, strlen(dirname) + 1);

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
                                struct dirent *i2c_dir,
                                const char *dirname)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  bool mem_access_found = false;
  bool debug_found = false;
  bool debug_v2_found = false;
  int ret;

  length = strlen(dirname) + strlen(i2c_dir->d_name) + 2;

  if ((pszDirname = (char *)calloc(length, sizeof(char))) == NULL)
  {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  snprintf(pszDirname, length, "%s/%s", dirname, i2c_dir->d_name);

  pDirectory = opendir(pszDirname);
  if (!pDirectory)
  {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, "mem_access"))
    {
      mxt_dbg(ctx, "Found mem_access interface at %s/mem_access", pszDirname);
      mem_access_found = true;
    }
    else if (!strcmp(pEntry->d_name, "debug_enable"))
    {
      mxt_dbg(ctx, "Found debug_enable interface at %s/debug_enable", pszDirname);
      debug_found = true;
    }
    else if (!strcmp(pEntry->d_name, "debug_msg"))
    {
      mxt_dbg(ctx, "Found Debug V2 at %s/debug_msg", pszDirname);
      debug_v2_found = true;
    }
  }

  /* If device found, store it and return success */
  if (mem_access_found && (debug_found || debug_v2_found))
  {
    if (ctx->query)
    {
      printf("sysfs:%s Atmel %s interface\n", pszDirname,
             debug_v2_found ? "Debug V2" : "Debug");
      ret = MXT_SUCCESS;
    }
    else
    {
      ret = sysfs_new_connection(ctx, conn, pszDirname);
      mxt_dbg(ctx, "Found %s", pszDirname);
      goto close;
    }
  }
  else
  {
    mxt_verb(ctx, "Ignoring %s", pszDirname);
    ret = MXT_ERROR_NO_DEVICE;
  }

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
  int adapter;
  unsigned int address;
  int ret;

  length = strlen(path) + strlen(dir->d_name) + 1;

  if ((pszDirname = (char *)calloc(length, sizeof(char))) == NULL)
  {
    mxt_err(ctx, "calloc failure");
    return MXT_ERROR_NO_MEM;
  }

  snprintf(pszDirname, length, "%s%s", path, dir->d_name);

  pDirectory = opendir(pszDirname);
  if (pDirectory == NULL)
  {
    ret = MXT_ERROR_NO_MEM;
    goto free;
  }

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue;

    if (sscanf(pEntry->d_name, "%d-%x", &adapter, &address) == 2)
    {
      ret = scan_sysfs_directory(ctx, conn, pEntry, pszDirname);

      // If found or error finish
      if (ret != MXT_ERROR_NO_DEVICE) goto close;
    }
  }

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
  DIR *pDirectory;
  int ret;

  // Look in sysfs for driver entries
  pDirectory = opendir(SYSFS_I2C_ROOT);
  if (!pDirectory)
    return MXT_ERROR_NO_DEVICE;

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue;

    ret = scan_driver_directory(ctx, conn, SYSFS_I2C_ROOT, pEntry);

    // If found or error finish
    if (ret != MXT_ERROR_NO_DEVICE) goto close;
  }

close:
  (void)closedir(pDirectory);

  return ret;
}

//******************************************************************************
/// \brief  Open device
/// \return #mxt_rc
int sysfs_open(struct mxt_device *mxt)
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
    if (errno == ENOENT)
    {
      mxt->sysfs.debug_v2 = false;
    }
    else
    {
      mxt_err(mxt->ctx, "Could not stat %s, error %s (%d)",
              filename, strerror(errno), errno);
      return mxt_errno_to_rc(errno);
    }
  }
  else
  {
    mxt->sysfs.debug_v2 = true;
  }

  mxt_info(mxt->ctx, "Registered sysfs path:%s", conn->path);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Release device
void sysfs_release(struct mxt_device *mxt)
{
  if (mxt)
  {
    free(mxt->sysfs.temp_path);
    free(mxt->sysfs.debug_v2_msg_buf);
  }
}

//******************************************************************************
/// \brief Open memory access file
/// \return #mxt_rc
static int open_device_file(struct mxt_device *mxt, int *fd_out)
{
  int fd;

  // Check device is initialised
  if (!mxt || !mxt->sysfs.mem_access_path)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

  mxt_dbg(mxt->ctx, "%s", mxt->sysfs.mem_access_path);

  fd = open(mxt->sysfs.mem_access_path, O_RDWR);

  if (fd < 0)
  {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)",
        mxt->sysfs.mem_access_path, strerror(errno), errno);

    return mxt_errno_to_rc(errno);
  }

  *fd_out = fd;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read register from MXT chip
/// \return #mxt_rc
int sysfs_read_register(struct mxt_device *mxt, unsigned char *buf, int start_register, int count)
{
  int fd;
  int ret;
  int bytes_read;

  ret = open_device_file(mxt, &fd);
  if (ret)
    return ret;

  if (lseek(fd, start_register, 0) < 0)
  {
    mxt_err(mxt->ctx, "lseek error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  bytes_read = 0;
  while (bytes_read < count)
  {
    ret = read(fd, buf + bytes_read, count - bytes_read);
    if (ret < 0)
    {
      mxt_err(mxt->ctx, "read error %s (%d)", strerror(errno), errno);
      ret = mxt_errno_to_rc(errno);
      goto close;
    }

    bytes_read += ret;
  }

  ret = MXT_SUCCESS;

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
/// \return #mxt_rc
int sysfs_write_register(struct mxt_device *mxt, unsigned char const *buf, int start_register, int count)
{
  int fd;
  int ret;
  int bytes_written;

  ret = open_device_file(mxt, &fd);
  if (ret)
    return ret;

  if (lseek(fd, start_register, 0) < 0)
  {
    mxt_err(mxt->ctx, "lseek error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  bytes_written = 0;
  while (bytes_written < count)
  {
    ret = write(fd, buf+bytes_written, count - bytes_written);
    if (ret < 0)
    {
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
/// \return #mxt_rc
static int write_boolean_file(struct mxt_device *mxt, const char *filename,
                              bool value)
{
  FILE *file;

  file = fopen(filename, "w+");
  if (!file)
  {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  if (value == true)
  {
    fputs("1", file);
  }
  else
  {
    fputs("0", file);
  }

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Read boolean from file as ASCII 0/1
/// \return #mxt_rc
static int read_boolean_file(struct mxt_device *mxt, char *filename,
                             bool *value)
{
  FILE *file;
  char val;
  bool ret;

  file = fopen(filename, "r");
  if (!file)
  {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return mxt_errno_to_rc(errno);
  }

  ret = fread(&val, sizeof(char), 1, file);
  if (ret < 0)
  {
    mxt_err(mxt->ctx, "Error reading files");
    return MXT_ERROR_IO;
  }

  if (val == 49) // ASCII '0'
  {
    *value = true;
  }
  else
  {
    *value = false;
  }

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Set debug state
/// \return #mxt_rc
int sysfs_set_debug(struct mxt_device *mxt, bool debug_state)
{
  int ret;

  // Check device is initialised
  if (!mxt)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

  if (mxt->sysfs.debug_v2 == true)
  {
    ret = write_boolean_file(mxt, make_path(mxt, "debug_v2_enable"), debug_state);
    if (ret)
      ret = write_boolean_file(mxt, make_path(mxt, "debug_enable"), debug_state);

    if (debug_state)
    {
      ret = sysfs_open_notify_fd(mxt);
      if (ret)
        return ret;
    }
    else
    {
      close(mxt->sysfs.debug_notify_fd);
    }
  }
  else
  {
    ret = write_boolean_file(mxt, make_path(mxt, "debug_enable"), debug_state);
  }

  return ret;
}

//******************************************************************************
/// \brief  Get debug message string
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
  for (i = 0; i < size; i++)
  {
    length += snprintf(msg_string + length, sizeof(msg_string) - length,
        "%02X ", databuf[i]);
  }

  return &msg_string[0];
}

//******************************************************************************
/// \brief Get debug message bytes
/// \return #mxt_rc
int sysfs_get_msg_bytes_v2(struct mxt_device *mxt, unsigned char *buf,
                           size_t buflen, int *count)
{
  uint16_t t5_size;

  if (!mxt->sysfs.debug_v2_msg_buf)
    return MXT_INTERNAL_ERROR;

  t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;

  if (buflen < t5_size)
    return MXT_ERROR_NO_MEM;

  if (mxt->sysfs.debug_v2_msg_ptr > mxt->sysfs.debug_v2_msg_count)
    return MXT_INTERNAL_ERROR;

  memcpy(buf,
         mxt->sysfs.debug_v2_msg_buf + mxt->sysfs.debug_v2_msg_ptr * t5_size,
         t5_size);

  mxt->sysfs.debug_v2_msg_ptr++;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Reset debug messages
int sysfs_msg_reset_v2(struct mxt_device *mxt)
{
  free(mxt->sysfs.debug_v2_msg_buf);
  return 0;
}

//******************************************************************************
/// \brief Return whether device has debug V2 support
bool sysfs_has_debug_v2(struct mxt_device *mxt)
{
  return mxt->sysfs.debug_v2;
}

//******************************************************************************
/// \brief  Get messages (V2 interface)
/// \return #mxt_rc
int sysfs_get_msg_count_v2(struct mxt_device *mxt, int *count)
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

  free(mxt->sysfs.debug_v2_msg_buf);

  mxt->sysfs.debug_v2_msg_buf = calloc(mxt->sysfs.debug_v2_size, sizeof(uint8_t));

  fd = open(filename, O_RDWR);
  if (fd < 0) {
    mxt_err(mxt->ctx, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  t5_size = mxt_get_object_size(mxt, GEN_MESSAGEPROCESSOR_T5) - 1;

  num_bytes = read(fd, mxt->sysfs.debug_v2_msg_buf, mxt->sysfs.debug_v2_size);
  if (num_bytes < 0)
  {
    mxt_err(mxt->ctx, "read error %s (%d)", strerror(errno), errno);
    ret = mxt_errno_to_rc(errno);
    goto close;
  }

  mxt->sysfs.debug_v2_msg_count = num_bytes / t5_size;
  mxt->sysfs.debug_v2_msg_ptr = 0;

  ret = MXT_SUCCESS;
  *count = mxt->sysfs.debug_v2_msg_count;
  mxt_verb(mxt->ctx, "count = %d", mxt->sysfs.debug_v2_msg_count);

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Get debug state
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int sysfs_get_debug(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_boolean_file(mxt, make_path(mxt, "debug_enable"), value);
}

//******************************************************************************
/// \brief  Get sysfs directory
/// \return location of the sysfs interface files
char *sysfs_get_directory(struct mxt_device *mxt)
{
  // Check device is initialised
  if (!mxt)
  {
    mxt_err(mxt->ctx, "Device uninitialised");
    return 0;
  }

  return mxt->conn->sysfs.path;
}
