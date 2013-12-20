//------------------------------------------------------------------------------
/// \file   config.c
/// \brief  Configuration file handling
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2012 Atmel Corporation. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>

#include "libmaxtouch.h"
#include "info_block.h"

#define OBP_RAW_MAGIC      "OBP_RAW V1"

//******************************************************************************
/// \brief  Save configuration to file
/// \return #mxt_rc
int mxt_save_raw_file(struct mxt_device *mxt, const char *filename)
{
  int obj_idx, i, instance, num_bytes;
  uint8_t *temp;
  struct mxt_object object;
  struct mxt_id_info *id = mxt->info.id;
  FILE *fp;
  int ret;

  mxt_info(mxt->ctx, "Opening config file %s...", filename);

  fp = fopen(filename, "w");

  fprintf(fp, "OBP_RAW V1\n");

  fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X\n",
          id->family, id->variant,
          id->version, id->build,
          id->matrix_x_size, id->matrix_y_size,
          id->num_objects);

  fprintf(fp, "%06X\n", mxt->info.crc);

  /* can't read object table CRC at present */
  fprintf(fp, "000000\n");

  for (obj_idx = 0; obj_idx < id->num_objects; obj_idx++)
  {
    object = mxt->info.objects[obj_idx];
    num_bytes = MXT_SIZE(object);

    temp = (uint8_t *)calloc(num_bytes, sizeof(char));
    if (temp == NULL)
    {
      mxt_err(mxt->ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    for (instance = 0; instance < MXT_INSTANCES(object); instance++)
    {
      fprintf(fp, "%04X %04X %04X", object.type, instance, num_bytes);

      mxt_read_register(mxt, temp,
                       mxt_get_start_position(object, instance),
                       num_bytes);

      for (i=0; i< num_bytes; i++)
      {
        fprintf(fp, " %02X", *(temp + i));
      }

      fprintf(fp, "\n");
    }

    free(temp);
  }

  ret = MXT_SUCCESS;

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Load configuration file
/// \return #mxt_rc
static int mxt_load_xcfg_file(struct mxt_device *mxt, const char *filename)
{
  FILE *fp;
  uint8_t *mem;
  int c;
  char object[255];
  char tmp[255];
  char *substr;
  int object_id;
  int instance;
  int object_address;
  uint16_t expected_address;
  int object_size;
  uint8_t expected_size;
  int data;
  int file_read = 0;
  bool ignore_line = false;
  int offset;
  int width;
  int ret;

  mem = calloc(255, sizeof(uint8_t));
  if (!mem) {
    mxt_err(mxt->ctx, "Error allocating memory");
    return MXT_ERROR_NO_MEM;
  }

  fp = fopen(filename, "r");
  if (fp == NULL)
  {
    mxt_err(mxt->ctx, "Error opening %s!", filename);
    ret = mxt_errno_to_rc(errno);
    goto free;
  }

  while (!file_read)
  {
    /* First character is expected to be '[' - skip empty lines and spaces  */
    c = getc(fp);
    while ((c == '\n') || (c == '\r') || (c == 0x20))
    {
      c = getc(fp);
    }

    if (c != '[')
    {
      if (c == EOF)
        break;

      /* If we are ignoring the current section then look for the next section */
      if (ignore_line)
      {
        continue;
      }

      mxt_err(mxt->ctx, "Parse error: expected '[', read ascii char %c!", c);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (fscanf(fp, "%[^] ]", object) != 1)
    {
      printf("Object parse error\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    /* Ignore the comments and file header sections */
    if (!strcmp(object, "COMMENTS")
        || !strcmp(object, "VERSION_INFO_HEADER")
        || !strcmp(object, "APPLICATION_INFO_HEADER"))
    {
      ignore_line = true;
      mxt_dbg(mxt->ctx, "Skipping %s", object);
      continue;
    }

    ignore_line = false;

    if (fscanf(fp, "%s", tmp) != 1)
    {
      printf("Instance parse error\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (strcmp(tmp, "INSTANCE"))
    {
      mxt_err(mxt->ctx, "Parse error, expected INSTANCE");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (fscanf(fp, "%d", &instance) != 1)
    {
      printf("Instance number parse error\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    /* Read rest of header section */
    while(c != ']')
    {
      c = getc(fp);
      if (c == '\n')
      {
        mxt_err(mxt->ctx, "Parse error, expected ] before end of line");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }
    }

    while(c != '\n')
    {
      c = getc(fp);
    }

    while ((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    if (fscanf(fp, "%d", &object_address) != 1)
    {
      printf("Object address parse error\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    c = getc(fp);
    while((c != '=') && (c != EOF))
    {
      c = getc(fp);
    }

    if (fscanf(fp, "%d", &object_size) != 1)
    {
      mxt_err(mxt->ctx, "Object size parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    c = getc(fp);

    /* Find object type ID number at end of object string */
    substr = strrchr(object, '_');
    if (substr == NULL || (*(substr + 1) != 'T'))
    {
      mxt_err(mxt->ctx, "Parse error, could not find T number in %s", object);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (sscanf(substr + 2, "%d", &object_id) != 1)
    {
      mxt_err(mxt->ctx, "Unable to get object type ID for %s", object);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    mxt_dbg(mxt->ctx, "%s T%u OBJECT_ADDRESS=%d OBJECT_SIZE=%d",
        object, object_id, object_address, object_size);

    /* Check the address of the object */
    expected_address = mxt_get_object_address(mxt, (uint8_t)object_id,
                                              (uint8_t)instance);
    if (expected_address == OBJECT_NOT_FOUND)
    {
      mxt_err(mxt->ctx, "T%u not present on chip", object_id);
    }
    else if (object_address != (int)expected_address)
    {
      mxt_warn
      (
        mxt->ctx,
        "Address of %s in config file (0x%04X) does not match chip (0x%04X)",
        object, object_address, expected_address
      );

      object_address = expected_address;
    }

    /* Check the size of the object */
    expected_size = mxt_get_object_size(mxt, (uint8_t)object_id);
    if (object_size != (int)expected_size)
    {
      mxt_warn
      (
        mxt->ctx,
        "Size of %s in config file (%d bytes) does not match chip (%d bytes)",
        object, object_size, expected_size
      );
    }

    mxt_dbg(mxt->ctx, "Writing object of size %d at address %d...",
            object_size, object_address);

    ret = mxt_read_register(mxt, mem, object_address, object_size);
    if (ret)
      return ret;

    while (true)
    {
      /* Find next line, check first character valid and rewind */
      c = getc(fp);
      while((c == '\n') || (c == '\r') || (c == 0x20))
        c = getc(fp);

      fseek(fp, -1, SEEK_CUR);

      /* End of object */
      if (c == '[')
        break;

      /* Read address */
      if (fscanf(fp, "%d", &offset) != 1)
      {
        mxt_err(mxt->ctx, "Address parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      /* Read byte count of this register (max 2) */
      if (fscanf(fp, "%d", &width) != 1)
      {
        mxt_err(mxt->ctx, "Byte count parse error\n");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      while((c != '=') && (c != EOF))
      {
        c = getc(fp);
      }

      if (fscanf(fp, "%d", &data) != 1)
      {
        mxt_err(mxt->ctx, "Data parse error\n");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      c = getc(fp);

      switch (width)
      {
        case 1:
          *(mem + offset) = (char) data;
          break;
        case 2:
          *(mem + offset) = (char) data & 0xFF;
          *(mem + offset + 1) = (char) ((data >> 8) & 0xFF);
          break;
        default:
          mxt_err(mxt->ctx, "Only 16-bit / 8-bit config values supported!");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
      }
    }

    ret = mxt_write_register(mxt, mem, object_address, object_size);
    if (ret)
      return ret;
  }

  ret = MXT_SUCCESS;
  mxt_info(mxt->ctx, "Wrote config from %s", filename);

close:
  fclose(fp);
free:
  free(mem);
  return ret;
}

//******************************************************************************
/// \brief  Structure containing configuration data for a particular object
struct mxt_object_config
{
  uint32_t type;
  uint8_t instance;
  uint32_t size;
  uint8_t *data;
};

//******************************************************************************
/// \brief  Load configuration from .xcfg or RAW file, automatically detect
//          format
//  \return #mxt_rc
static int mxt_load_raw_file(struct mxt_device *mxt, const char *filename)
{
  struct mxt_id_info cfg_info;
  FILE *fp;
  int ret;
  size_t i;
  uint32_t info_crc, config_crc;
  size_t reg;
  char line[2048];

  fp = fopen(filename, "r");

  if (fgets(line, sizeof(line), fp) == NULL)
  {
    mxt_err(mxt->ctx, "Unexpected EOF");
  }

  if (strncmp(line, OBP_RAW_MAGIC, strlen(OBP_RAW_MAGIC)))
  {
    mxt_warn(mxt->ctx, "Not in OBP_RAW format");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  } else {
    mxt_dbg(mxt->ctx, "Loading OBP_RAW file");
  }

  /* Load information block and check */
  for (i = 0; i < sizeof(struct mxt_id_info); i++) {
    ret = fscanf(fp, "%hhx", (unsigned char *)&cfg_info + i);
    if (ret != 1) {
      mxt_err(mxt->ctx, "Bad format");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }
  }

  /* Read CRCs */
  ret = fscanf(fp, "%x", &info_crc);
  if (ret != 1) {
    mxt_err(mxt->ctx, "Bad format: failed to parse Info CRC");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  }

  ret = fscanf(fp, "%x", &config_crc);
  if (ret != 1) {
    mxt_err(mxt->ctx, "Bad format: failed to parse Config CRC");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  }

  /* The Info Block CRC is calculated over mxt_info and the object table
   * If it does not match then we are trying to load the configuration
   * from a different chip or firmware version, so the configuration CRC
   * is invalid anyway. */
  if (info_crc != mxt->info.crc) {
    mxt_warn(mxt->ctx, "Info Block CRC mismatch - device=0x%06X "
        "file=0x%06X - attempting to apply config",
        mxt->info.crc, info_crc);
  }

  while (true)
  {
    struct mxt_object_config cfg;

    /* Read type, instance, length */
    ret = fscanf(fp, "%x %" SCNx8 " %x", &cfg.type, &cfg.instance, &cfg.size);
    if (ret == EOF) {
      break;
    } else if (ret != 3) {
      mxt_err(mxt->ctx, "Bad format: failed to parse object");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    reg = mxt_get_object_address(mxt, cfg.type, cfg.instance);
    if (reg == OBJECT_NOT_FOUND) {
      ret = MXT_ERROR_OBJECT_NOT_FOUND;
      goto close;
    }

    mxt_dbg(mxt->ctx, "Found cfg for T%u instance %u", cfg.type, cfg.instance);

    if (cfg.size > mxt_get_object_size(mxt, cfg.type)) {
      /* Either we are in fallback mode due to wrong
       * config or config from a later fw version,
       * or the file is corrupt or hand-edited */
      mxt_warn(mxt->ctx, "Discarding %u bytes in T%u!",
          cfg.size - mxt_get_object_size(mxt, cfg.type), cfg.type);

      cfg.size = mxt_get_object_size(mxt, cfg.type);
    }

    /* Malloc memory to store configuration */
    cfg.data = calloc(mxt_get_object_size(mxt, cfg.type), sizeof(uint8_t));
    if (!cfg.data) {
      mxt_err(mxt->ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    /* Read object config */
    ret = mxt_read_register(mxt, cfg.data, reg,
                            mxt_get_object_size(mxt, cfg.type));
    if (ret)
      goto close;

    /* Update bytes from file */
    for (i = 0; i < cfg.size; i++) {
      uint8_t val;
      ret = fscanf(fp, "%hhx", &val);
      if (ret != 1) {
        mxt_err(mxt->ctx, "Parse error in T%d", cfg.type);
        ret = MXT_ERROR_FILE_FORMAT;
        free(cfg.data);
        goto close;
      }

      *(cfg.data + i) = val;
    }

    mxt_log_buffer(mxt->ctx, LOG_DEBUG, "CFG", cfg.data, cfg.size);

    /* Write object */
    ret = mxt_write_register(mxt, cfg.data, reg,
                             mxt_get_object_size(mxt, cfg.type));
    if (ret) {
      mxt_err(mxt->ctx, "Config write error, ret=%d", ret);
      goto close;
    }
  }

  ret = MXT_SUCCESS;
  mxt_info(mxt->ctx, "Wrote config from %s", filename);

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Load configuration from .xcfg or RAW file, automatically detect
//          format
/// \return 0 = success, negative = fail
int mxt_load_config_file(struct mxt_device *mxt, const char *filename)
{
  int ret;

  if (!strcmp(strrchr(filename, '.'), ".xcfg"))
  {
    mxt_info(mxt->ctx, "Loading .xcfg file");
    ret = mxt_load_xcfg_file(mxt, filename);
  }
  else
  {
    ret = mxt_load_raw_file(mxt, filename);
  }

  return ret;
}
