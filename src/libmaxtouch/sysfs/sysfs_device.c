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
#include "libmaxtouch/info_block.h"
#include "dmesg.h"
#include "sysfs_device.h"

#define SYSFS_I2C_ROOT "/sys/bus/i2c/drivers/"
#define SYSFS_SPI_ROOT "/sys/bus/spi/drivers/"

//******************************************************************************
/// \brief Device information
typedef struct sysfs_device_t {
  int adapter;
  int address;
  char *path;
  char *mem_access_path;
  bool debug_ng;
  uint16_t debug_ng_msg_count;
  uint16_t debug_ng_msg_ptr;
  uint8_t *debug_ng_msg_buf;
  int debug_notify_fd;
  size_t debug_ng_size;
} sysfs_device;

/* detected devices */
sysfs_device *gpDevice = NULL;

/* char buffer for constructing paths */
#define PATH_LENGTH 256
char tempPath[PATH_LENGTH + 1];

//******************************************************************************
/// \brief Construct filename of path
static char *make_path(const char *filename)
{
  snprintf(tempPath, PATH_LENGTH, "%s/%s", gpDevice->path, filename);

  return &tempPath[0];
}

static void sysfs_open_notify_fd(void)
{
  char *filename = make_path("debug_notify");

  gpDevice->debug_notify_fd = open(filename, O_RDONLY);
  if (gpDevice->debug_notify_fd < 0) {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return;
  }
}

static void sysfs_reopen_notify_fd(void)
{
  uint16_t val;

  close(gpDevice->debug_notify_fd);
  sysfs_open_notify_fd();
  read(gpDevice->debug_notify_fd, &val, 2);
}

//******************************************************************************
/// \brief Register sysfs device
static void sysfs_register_device(const char *dirname, int adapter,
                                  int address, bool debug_ng)
{
  gpDevice = (sysfs_device *)malloc(sizeof(sysfs_device));
  gpDevice->path = (char *)malloc(strlen(dirname) + 1);
  gpDevice->mem_access_path = (char *)malloc(strlen(dirname) + 20);

  gpDevice->adapter = adapter;
  gpDevice->address = address;

  gpDevice->path[0] = '\0';
  memcpy(gpDevice->path, dirname, strlen(dirname) + 1);

  // Cache memory access path for fast access
  snprintf(gpDevice->mem_access_path, strlen(dirname) + 20,
           "%s/mem_access", dirname);

  if (debug_ng)
  {
    sysfs_open_notify_fd();
    gpDevice->debug_ng = true;
  }

  LOG(LOG_INFO, "Registered sysfs adapter:%d address:%x path:%s",
        gpDevice->adapter, gpDevice->address, gpDevice->path);
}

//******************************************************************************
/// \brief Check whether device has NG debug
bool sysfs_has_debug_ng()
{
  return gpDevice->debug_ng;
}

//******************************************************************************
/// \brief Get fd for select
int sysfs_get_debug_ng_fd()
{
  return gpDevice->debug_notify_fd;
}

//******************************************************************************
/// \brief Check sysfs device directory for correct files
/// \return 1 = device found, 0 = not found, negative for error
static int scan_sysfs_directory(struct dirent *i2c_dir,
                                const char *dirname, int adapter, int address)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  bool mem_access_found = false;
  bool debug_found = false;
  bool debug_ng_found = false;
  int ret = 0;

  length = strlen(dirname) + strlen(i2c_dir->d_name) + 2;

  if ((pszDirname = (char *)malloc(length)) == NULL)
  {
    ret = -1;
    goto free;
  }

  snprintf(pszDirname, length, "%s/%s", dirname, i2c_dir->d_name);

  LOG(LOG_DEBUG, "Checking %s", pszDirname);

  pDirectory = opendir(pszDirname);

  if (!pDirectory)
    return 0;

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, "mem_access"))
    {
      LOG(LOG_DEBUG, "Found mem_access interface at %s/mem_access", pszDirname);
      mem_access_found = true;
    }

    if (!strcmp(pEntry->d_name, "debug_enable"))
    {
      LOG(LOG_DEBUG, "Found debug_enable interface at %s/debug_enable", pszDirname);
      debug_found = true;
    }

    if (!strcmp(pEntry->d_name, "debug_msg"))
    {
      LOG(LOG_DEBUG, "Found Debug NG at %s/debug_msg", pszDirname);
      debug_ng_found = true;
    }
  }

  /* If device found, store it and return success */
  if (mem_access_found && (debug_found || debug_ng_found))
  {
    sysfs_register_device(pszDirname, adapter, address, debug_ng_found);
    ret = 1;
    goto close;
  }

close:
  (void)closedir(pDirectory);
free:
  free(pszDirname);

  return ret;
}

//******************************************************************************
/// \brief  Process a driver directory in sysfs looking for MXT devices
/// \return 1 = device found, 0 = not found, negative for error
static int scan_driver_directory(const char *path, struct dirent *dir)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  int adapter;
  unsigned int address;
  int ret = 0;

  /* Driver name must match otherwise return zero devices found */
  if (strcmp(dir->d_name, "qt602240_ts")
      && strncasecmp(dir->d_name, "atmel mxt", 9)
      && strncasecmp(dir->d_name, "atmel_mxt", 9)
      && strncasecmp(dir->d_name, "MXT", 3)
      && strcmp(dir->d_name, "sec_touch")
      && strcmp(dir->d_name, "maXTouch"))
  {
    LOG(LOG_DEBUG, "Ignoring device %s", dir->d_name);
    return 0;
  }

  length = strlen(path) + strlen(dir->d_name) + 1;

  if ((pszDirname = (char *)malloc(length)) == NULL)
  {
    LOG(LOG_DEBUG, "malloc failure");
    return -1;
  }

  snprintf(pszDirname, length, "%s%s", path, dir->d_name);

  pDirectory = opendir(pszDirname);

  if (pDirectory == NULL) goto free;

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue;

    if (sscanf(pEntry->d_name, "%d-%x", &adapter, &address) == 2)
    {
      ret = scan_sysfs_directory(pEntry, pszDirname, adapter, address);

      // If found or error finish
      if (ret != 0) goto close;
    }

    if (sscanf(pEntry->d_name, "spi%d.%d", &adapter, &address) == 2)
    {
      ret = scan_sysfs_directory(pEntry, pszDirname, adapter, address);

      // If found or error finish
      if (ret != 0) goto close;
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
/// \return 1 = device found, 0 = not found, negative for error
static int sysfs_scan_tree(const char *root)
{
  struct dirent *pEntry;
  DIR *pDirectory;
  int ret = 0;

  // Look in sysfs for driver entries
  pDirectory = opendir(root);

  if (!pDirectory)
    return 0;

  LOG(LOG_DEBUG, "Scanning %s", root);

  while (ret == 0)
  {
    pEntry = readdir(pDirectory);
    if (!pEntry)
      break;

    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue;

    ret = scan_driver_directory(root, pEntry);
  }

  (void)closedir(pDirectory);

  return ret;
}

//******************************************************************************
/// \brief  Scan for devices
/// \return 1 = device found, 0 = not found, negative for error
int sysfs_scan()
{
  int ret;

  ret = sysfs_scan_tree(SYSFS_I2C_ROOT);

  if (ret == 0)
  {
    ret = sysfs_scan_tree(SYSFS_SPI_ROOT);
  }

  if (ret == 0)
  {
    LOG(LOG_WARN, "Unable to find any sysfs devices");
  }

  return ret;
}

//******************************************************************************
/// \brief  Release device
void sysfs_release()
{
  if (gpDevice)
  {
    if (gpDevice->path)
    {
      free(gpDevice->path);
    }

    if (gpDevice->debug_ng_msg_buf)
    {
      free(gpDevice->debug_ng_msg_buf);
      close(gpDevice->debug_notify_fd);
    }

    free(gpDevice);
  }
}

//******************************************************************************
/// \brief  Get i2c adapter number
int sysfs_get_i2c_adapter()
{
  return gpDevice->adapter;
}


//******************************************************************************
/// \brief  Get i2c address
int sysfs_get_i2c_address()
{
  return gpDevice->address;
}

//******************************************************************************
/// \brief Open memory access file
static int open_device_file(void)
{
  int file;

  // Check device is initialised
  if (!gpDevice || !gpDevice->mem_access_path)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  file = open(gpDevice->mem_access_path, O_RDWR);

  if (file < 0)
  {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)",
        gpDevice->mem_access_path, strerror(errno), errno);

    return -1;
  }

  return file;
}

//******************************************************************************
/// \brief  Read register from MXT chip
int sysfs_read_register(unsigned char *buf, int start_register, int count)
{
  int fd;
  int ret;
  int bytes_read;

  fd = open_device_file();

  if (fd < 0)
    return fd;

  if (lseek(fd, start_register, 0) < 0)
  {
    LOG(LOG_ERROR, "lseek error %s (%d)", strerror(errno), errno);
    ret = -1;
    goto close;
  }

  bytes_read = 0;
  while (bytes_read < count)
  {
    ret = read(fd, buf + bytes_read, count - bytes_read);
    if (ret < 0)
    {
      LOG(LOG_ERROR, "read error %s (%d)", strerror(errno), errno);
      ret = -1;
      goto close;
    }

    bytes_read += ret;
  }

  ret = 0;

close:
  close(fd);
  return ret;
}

//******************************************************************************
/// \brief  Write register to MXT chip
int sysfs_write_register(unsigned char const *buf, int start_register, int count)
{
  int fd;
  int ret;
  int bytes_written;

  fd = open_device_file();

  if (fd < 0)
    return fd;

  if (lseek(fd, start_register, 0) < 0)
  {
    LOG(LOG_ERROR, "lseek error %s (%d)", strerror(errno), errno);
    ret = -1;
    goto close;
  }

  bytes_written = 0;
  while (bytes_written < count)
  {
    ret = write(fd, buf+bytes_written, count - bytes_written);
    if (ret < 0)
    {
      LOG(LOG_ERROR, "Error %s (%d) writing to register", strerror(errno), errno);
      ret = -1;
      goto close;
    }

    bytes_written += ret;
  }

  ret = 0;

close:
  close(fd);

  return ret;
}


//******************************************************************************
/// \brief  Write boolean to file as ASCII 0/1
static int write_boolean_file(const char *filename, bool value)
{
  FILE *file;

  if (!filename)
  {
    LOG(LOG_ERROR, "write_boolean_file: No filename");
    return -1;
  }

  file = fopen(filename, "w+");

  if (!file)
  {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return -1;
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

  return 0;
}

//******************************************************************************
/// \brief  Read boolean from file as ASCII 0/1
static bool read_boolean_file(char *filename)
{
  FILE *file;
  char val;
  bool ret;

  if (!filename)
  {
    LOG(LOG_ERROR, "read_boolean_file: No filename");
    return false;
  }

  file = fopen(filename, "r");

  if (!file)
  {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    return false;
  }

  ret = fread(&val, sizeof(char), 1, file);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Error reading files");
    return false;
  }

  if (val == 49) // ASCII '0'
  {
    ret = true;
  }
  else
  {
    ret = false;
  }

  fclose(file);

  return ret;
}

//******************************************************************************
/// \brief  Set debug state
/// \return 0 on success or negative error
int sysfs_set_debug(bool debug_state)
{
  int ret;

  // Check device is initialised
  if (!gpDevice)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  if (gpDevice->debug_ng == true)
  {
    ret = write_boolean_file(make_path("debug_v2_enable"), debug_state);
    if (ret == -1)
      ret = write_boolean_file(make_path("debug_enable"), debug_state);
  }
  else
  {
    ret = write_boolean_file(make_path("debug_enable"), debug_state);
  }

  return ret;
}

//******************************************************************************
/// \brief  Get debug message string
char *sysfs_get_msg_string_ng(void)
{
  int ret, i;
  uint16_t t5_size;
  size_t length;
  unsigned char databuf[20];
  static char msg_string[255];

  t5_size = get_object_size(GEN_MESSAGEPROCESSOR_T5) - 1;

  ret = sysfs_get_msg_bytes_ng(&databuf[0], sizeof(databuf));
  if (ret < 0)
    return NULL;

  length = snprintf(msg_string, sizeof(msg_string), "MXT MSG:");
  for (i = 0; i < t5_size; i++)
  {
    length += snprintf(msg_string + length, sizeof(msg_string) - length,
        "%02X ", databuf[i]);
  }

  return &msg_string[0];
}

//******************************************************************************
/// \brief Get debug message bytes
int sysfs_get_msg_bytes_ng(unsigned char *buf, size_t buflen)
{
  uint16_t t5_size;

  if (!gpDevice->debug_ng_msg_buf)
    return -1;

  t5_size = get_object_size(GEN_MESSAGEPROCESSOR_T5) - 1;

  if (buflen < t5_size)
    return -1;

  if (gpDevice->debug_ng_msg_ptr > gpDevice->debug_ng_msg_count)
    return -1;

  memcpy(buf,
         gpDevice->debug_ng_msg_buf + gpDevice->debug_ng_msg_ptr * t5_size,
         t5_size);

  gpDevice->debug_ng_msg_ptr++;

  return 0;
}

//******************************************************************************
/// \brief Reset debug messages
int sysfs_msg_reset_ng(void)
{
  // Free buffer of previous messages (TODO realloc)
  if (gpDevice->debug_ng_msg_buf)
  {
    free(gpDevice->debug_ng_msg_buf);
  }

  return 0;
}

//******************************************************************************
/// \brief  Get messages (new)
int sysfs_get_msg_count_ng()
{
  ssize_t count;
  uint16_t t5_size;
  char *filename;
  struct stat filestat;
  int ret;
  int fd;

  sysfs_reopen_notify_fd();

  filename = make_path("debug_msg");

  ret = stat(filename, &filestat);
  if (ret < 0) {
    LOG(LOG_ERROR, "Could not stat %s, error %s (%d)", filename, strerror(errno), errno);
    return 0;
  }

  gpDevice->debug_ng_size = filestat.st_size;

  if (gpDevice->debug_ng_msg_buf)
  {
    free(gpDevice->debug_ng_msg_buf);
  }

  gpDevice->debug_ng_msg_buf = calloc(gpDevice->debug_ng_size, sizeof(uint8_t));

  fd = open(filename, O_RDWR);
  if (fd < 0) {
    LOG(LOG_ERROR, "Could not open %s, error %s (%d)", filename, strerror(errno), errno);
    goto close;
  }

  t5_size = get_object_size(GEN_MESSAGEPROCESSOR_T5) - 1;

  count = read(fd, gpDevice->debug_ng_msg_buf, gpDevice->debug_ng_size);
  if (count < 0)
  {
    LOG(LOG_ERROR, "read error %s (%d)", strerror(errno), errno);
    goto close;
  }

  gpDevice->debug_ng_msg_count = count / t5_size;
  gpDevice->debug_ng_msg_ptr = 0;

close:
  close(fd);

  LOG(LOG_DEBUG, "count = %d", gpDevice->debug_ng_msg_count);

  return gpDevice->debug_ng_msg_count;
}

//******************************************************************************
/// \brief  Get debug state
/// \return true (debug enabled) or false (debug disabled)
bool sysfs_get_debug()
{
  // Check device is initialised
  if (!gpDevice)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return false;
  }

  return read_boolean_file(make_path("debug_enable"));
}

//******************************************************************************
/// \brief  Get sysfs directory
/// \return location of the sysfs interface files
char *sysfs_get_directory(void)
{
  // Check device is initialised
  if (!gpDevice)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return 0;
  }

  return gpDevice->path;
}
