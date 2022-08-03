//------------------------------------------------------------------------------
/// \file   debugfs_device.c
/// \brief  mXT device low level access via debugfs interface
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
// THIS SOFTWARE IS PROVIDED BY MICROCHIP ''AS IS'' AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
// EVENT SHALL MICROCHIP OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
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
#include "debugfs_device.h"

#define DEBUGFS_I2C_ROOT "/sys/kernel/debug/atmel_mxt_ts/"

//******************************************************************************
/// \brief Construct filename of path
static char *debugfs_path(struct mxt_device *mxt, const char *filename)
{
  snprintf(mxt->debug_fs.tmp_path, mxt->debug_fs.dir_max,
           "%s%s", mxt->debug_fs.file_path, filename);

  mxt_dbg(mxt->ctx, "Path: tmp_path %s", mxt->debug_fs.tmp_path);

  return mxt->debug_fs.tmp_path;
}

//******************************************************************************
/// \brief  Open device
/// \return #mxt_rc
int debugfs_open(struct mxt_device *mxt)
{
  
  mxt->debug_fs.dir_max = strlen(mxt->debug_fs.file_path) + 20;

   //Allocate temporary path space
  mxt->debug_fs.tmp_path = calloc(mxt->debug_fs.dir_max + 1, sizeof(char));
  if (!mxt->debug_fs.tmp_path)
    return MXT_ERROR_NO_MEM;

  mxt_info(mxt->ctx, "\nDevice registered on debug_fs path:%s", mxt->debug_fs.file_path);

  return MXT_SUCCESS;

}

//******************************************************************************
/// \brief  Read boolean from file as ASCII Y/N
/// \param  mxt Device context
/// \param  filename Name of file to read
/// \param  value Value read from file
/// \return #mxt_rc
static int debugfs_rd_file(struct mxt_device *mxt, char *filename,
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

  if (val == 89 || val == 121 ) { // ASCII 'Y', 'y'
    *value = true;
  } else if (val == 78 || val == 110) { //ASCII 'N', 'n'
    *value = false;
  } else {
    mxt_err(mxt->ctx, "Error reading value");
    *value = false;
  }

  fclose(file);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Write boolean to file as ASCII 0/1
/// \param  mxt Device context
/// \param  filename Name of file to write
/// \param  value Value to write
/// \return #mxt_rc
static int debugfs_wr_file(struct mxt_device *mxt, const char *filename,
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
/// \brief  Set debug irq state
/// \param  mxt Device context
/// \param  debug_state true = debug enabled, false = debug disabled
/// \return #mxt_rc
int debugfs_set_irq(struct mxt_device *mxt, bool enable)
{
  int ret;
  bool value;

  //Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

  ret = debugfs_set_debug_irq(mxt, enable);

  if (ret){
    mxt_err(mxt->ctx, "Could not write to debug_irq file");
    return MXT_ERROR_OBJECT_NOT_FOUND;
  }

  ret = debugfs_get_debug_irq(mxt, &value);

  if (ret){
    mxt_err(mxt->ctx, "Could not read debug_irq file");
    return MXT_ERROR_OBJECT_NOT_FOUND;
  }

  if (value == enable)
    return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Set debug irq state
/// \param  mxt Device context
/// \param  debug_state true = debug enabled, false = debug disabled
/// \return #mxt_rc
int debugfs_set_debug_irq(struct mxt_device *mxt, bool debug_state)
{
  int ret;

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return MXT_ERROR_NO_DEVICE;
  }

    ret = debugfs_wr_file(mxt, debugfs_path(mxt, "debug_irq"), debug_state);

  return ret;
}

//******************************************************************************
/// \brief  Get debug IRQ state
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int debugfs_get_debug_irq(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Cannot find debug_irq file");
    return false;
  }

  return debugfs_rd_file(mxt, debugfs_path(mxt, "debug_irq"), value);
}

//******************************************************************************
/// \brief  Get crc status flag
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int debugfs_get_crc_enabled(struct mxt_device *mxt, bool *value)
{
  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Cannot find crc_enabled file");
    return false;
  }

  return debugfs_rd_file(mxt, debugfs_path(mxt, "crc_enabled"), value);
}

//******************************************************************************
/// \brief  Write sysfs byte to file
/// \param  mxt Device context
/// \param  filename Name of file to write
/// \param  value Value to write
/// \return #mxt_rc
static int write_debugfs_byte(struct mxt_device *mxt, const char *filename,
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

  sprintf(in_str,"%d", value);   //String copy as decimal

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
static int read_debugfs_byte(struct mxt_device *mxt, char *filename,
                             uint16_t *value)
{
  FILE *file;
  char val[6];   //Minimum 3 to hold NULL/EOF value
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
/// \brief  Get tx seq number value
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int debugfs_get_tx_seq_num(struct mxt_device *mxt, uint16_t *value)
{

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return read_debugfs_byte(mxt, debugfs_path(mxt, "tx_seq_num"), value);

}

//******************************************************************************
/// \brief  Get tx seq number value
/// \param  mxt Device context
/// \param  value true (debug enabled) or false (debug disabled)
/// \return #mxt_rc
int debugfs_set_tx_seq_num(struct mxt_device *mxt, uint8_t value)
{

  // Check device is initialised
  if (!mxt) {
    mxt_err(mxt->ctx, "Device uninitialised");
    return false;
  }

  return write_debugfs_byte(mxt, debugfs_path(mxt, "tx_seq_num"), value);

}

int debugfs_update_seq_num(struct mxt_device *mxt, uint8_t value)
{
  int ret;

  if (value == 255) {
    value = 0x00;
  } else {
    value++;
  }
  
  ret = debugfs_set_tx_seq_num(mxt, value);

  return ret;
}

//******************************************************************************
/// \brief  Scan for devices
/// \return #mxt_rc
int debugfs_scan(struct mxt_device *mxt)
{
  struct dirent *pEntry;
  DIR *pDirectory;
  bool debug_irq_found = false;
  bool tx_seq_num_found = false;
  bool crc_enabled_found = false;
  int ret;

  // Look in debugfs for files
  pDirectory = opendir(DEBUGFS_I2C_ROOT);
  if (!pDirectory)
    return MXT_ERROR_NO_DEVICE;

  while ((pEntry = readdir(pDirectory)) != NULL) {
    if (!strcmp(pEntry->d_name, ".") || !strcmp(pEntry->d_name, ".."))
      continue; 

    if (!strcmp(pEntry->d_name, "crc_enabled")){
      mxt_dbg(mxt->ctx, "Found crc_enabled at %scrc_enabled", DEBUGFS_I2C_ROOT);
      crc_enabled_found = true;
    }

    if (!strcmp(pEntry->d_name, "debug_irq")) {
      mxt_dbg(mxt->ctx, "Found debug_irq at %sdebug_irq", DEBUGFS_I2C_ROOT);
      debug_irq_found = true;
    } 

    if (!strcmp(pEntry->d_name, "tx_seq_num")){
      mxt_dbg(mxt->ctx, "Found tx_seq_number at %stx_seq_num", DEBUGFS_I2C_ROOT);
      tx_seq_num_found = true;
    }
  }

  if (crc_enabled_found && debug_irq_found && tx_seq_num_found) {

    mxt->debug_fs.file_path = DEBUGFS_I2C_ROOT;
    ret = MXT_SUCCESS;

  } else {
    ret = MXT_ERROR_NO_DEVICE;
  }

  (void)closedir(pDirectory);

  return ret;
}