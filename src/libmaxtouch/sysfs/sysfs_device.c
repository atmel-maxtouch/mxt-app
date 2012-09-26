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
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <malloc.h>
#include <stdio.h>

#include "libmaxtouch/log.h"
#include "dmesg.h"
#include "sysfs_device.h"

#define SYSFS_I2C_ROOT "/sys/bus/i2c/drivers/"
#define SYSFS_SPI_ROOT "/sys/bus/spi/drivers/"

//******************************************************************************
/// \brief Device information
typedef struct sysfs_device_tag {
  int adapter;
  int address;
  char *path;
  char *mem_access_path;
} sysfs_device;

/* detected devices */
sysfs_device *gpDevice = NULL;

/* char buffer for constructing paths */
#define PATH_LENGTH 256
char tempPath[PATH_LENGTH + 1];

//******************************************************************************
/// \brief Check sysfs device directory for correct files
/// \return 1 = device found, 0 = not found, negative for error
static int scan_sysfs_directory(struct dirent *i2c_dir, char *dirname, int adapter, int address)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  bool mem_access_found = false;
  bool pause_found = false;
  bool debug_found = false;
  int ret = 0;

  length = strlen(dirname) + strlen(i2c_dir->d_name) + 1;

  if ((pszDirname = (char *)malloc(length+1)) == NULL)
  {
    ret = -1;
    goto free;
  }

  strncpy(pszDirname, dirname, length);
  strncat(pszDirname, "/", length);
  strncat(pszDirname, i2c_dir->d_name, length);

  LOG(LOG_DEBUG, "Checking %s", pszDirname);

  pDirectory = opendir(pszDirname);

  if (pDirectory == NULL) return 0;

  while ((pEntry = readdir(pDirectory)) != NULL)
  {
    if (!strcmp(pEntry->d_name, "mem_access"))
    {
      LOG(LOG_DEBUG, "Found mem_access interface at %s/mem_access", pszDirname);
      mem_access_found = true;
    }

    if (!strcmp(pEntry->d_name, "pause_driver"))
    {
      LOG(LOG_DEBUG, "Found pause_driver interface at %s/pause_driver", pszDirname);
      pause_found = true;
    }

    if (!strcmp(pEntry->d_name, "debug_enable"))
    {
      LOG(LOG_DEBUG, "Found debug_enable interface at %s/debug_enable", pszDirname);
      debug_found = true;
    }
  }

  /* If device found, store it and return success */
  if (mem_access_found && pause_found && debug_found)
  {
    gpDevice = (sysfs_device *)malloc(sizeof(sysfs_device));
    gpDevice->path = (char *)malloc(strlen(pszDirname) + 1);
    gpDevice->mem_access_path = (char *)malloc(strlen(pszDirname) + 20);

    gpDevice->adapter = adapter;
    gpDevice->address = address;

    gpDevice->path[0] = '\0';
    strcpy(gpDevice->path, pszDirname);

    // Cache memory access path for fast access
    strcpy(gpDevice->mem_access_path, pszDirname);
    strncat(gpDevice->mem_access_path, "/mem_access", strlen(pszDirname) + 20);

    LOG(LOG_INFO, "Registered sysfs adapter:%d address:%x path:%s",
          gpDevice->adapter, gpDevice->address, gpDevice->path);

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
static int scan_driver_directory(char *path, struct dirent *dir)
{
  char *pszDirname;
  size_t length;
  DIR *pDirectory;
  struct dirent *pEntry;
  int adapter;
  unsigned int address;
  int ret = 0;

  /* Driver name must match otherwise return zero devices found */
  if ((strcmp(dir->d_name, "qt602240_ts") != 0)
      && (strcmp(dir->d_name, "Atmel MXT224") != 0)
      && (strcmp(dir->d_name, "Atmel MXT540E") != 0)
      && (strcmp(dir->d_name, "Atmel MXT768E") != 0)
      && (strcmp(dir->d_name, "atmel_mxt_ts") != 0)
      && (strcmp(dir->d_name, "sec_touch") != 0)
      && (strcmp(dir->d_name, "maXTouch") != 0))
    return 0;

  length = strlen(path) + strlen(dir->d_name);

  if ((pszDirname = (char *)malloc(length+1)) == NULL)
  {
    LOG(LOG_DEBUG, "malloc failure");
    return -1;
  }

  strncpy(pszDirname, path, length);
  strncat(pszDirname, dir->d_name, length);

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
static int sysfs_scan_tree(char *root)
{
  struct dirent *pEntry;
  DIR *pDirectory;
  int ret = 0;

  // Look in sysfs for driver entries
  pDirectory = opendir(root);

  if (pDirectory == NULL) return 0;

  LOG(LOG_DEBUG, "Scanning %s", root);

  while (ret == 0)
  {
    pEntry = readdir(pDirectory);
    if (pEntry == NULL)
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
  if (gpDevice != NULL)
  {
    if (gpDevice->path != NULL)
    {
      free(gpDevice->path);
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
  if (gpDevice == NULL || gpDevice->mem_access_path == NULL)
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
/// \brief Construct filename of path
static char *make_path(char *filename)
{
  strncpy(tempPath, gpDevice->path, PATH_LENGTH);
  strncat(tempPath, "/", PATH_LENGTH);
  strncat(tempPath, filename, PATH_LENGTH);

  return &tempPath[0];
}

//******************************************************************************
/// \brief  Write boolean to file as ASCII 0/1
static int write_boolean_file(char *filename, bool value)
{
  FILE *file;

  if (filename == NULL)
  {
    LOG(LOG_ERROR, "write_boolean_file: No filename");
    return -1;
  }

  file = fopen(filename, "w+");

  if (file == NULL)
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

  if (filename == NULL)
  {
    LOG(LOG_ERROR, "read_boolean_file: No filename");
    return false;
  }

  file = fopen(filename, "r");

  if (file == NULL)
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
  // Check device is initialised
  if (gpDevice == NULL)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  return write_boolean_file(make_path("debug_enable"), debug_state);
}


//******************************************************************************
/// \brief  Get debug state
/// \return true (debug enabled) or false (debug disabled)
bool sysfs_get_debug()
{
  // Check device is initialised
  if (gpDevice == NULL)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return false;
  }

  return read_boolean_file(make_path("debug_enable"));
}

//******************************************************************************
/// \brief  Set pause state
/// \return 0 on success or negative error
int sysfs_set_pause(bool pause_state)
{
  // Check device is initialised
  if (gpDevice == NULL)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return -1;
  }

  return write_boolean_file(make_path("pause_driver"), pause_state);
}

//******************************************************************************
/// \brief  Get pause state
/// \return true (driver paused) or false (normal operation)
bool sysfs_get_pause()
{
  // Check device is initialised
  if (gpDevice == NULL)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return false;
  }

  return read_boolean_file(make_path("pause_driver"));
}

//******************************************************************************
/// \brief  Get sysfs directory
/// \return location of the sysfs interface files
char *sysfs_get_directory(void)
{
  // Check device is initialised
  if (gpDevice == NULL)
  {
    LOG(LOG_ERROR, "Device uninitialised");
    return 0;
  }

  return gpDevice->path;
}
