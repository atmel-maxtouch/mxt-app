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
#include <limits.h>

#include "libmaxtouch.h"
#include "info_block.h"
#include "utilfuncs.h"
#include "msg.h"

#define OBP_RAW_MAGIC      "OBP_RAW V1"

//******************************************************************************
/// \brief Config file types
enum mxt_config_type {
  CONFIG_RAW,
  CONFIG_XCFG
};

//******************************************************************************
/// \brief Configuration data for a single object
struct mxt_object_config {
  uint32_t type;
  uint8_t instance;
  uint32_t size;
  uint16_t start_position;
  uint8_t *data;
  struct mxt_object_config *next;
};

//******************************************************************************
/// \brief Device configuration
struct mxt_config {
  struct mxt_id_info id;
  struct mxt_object_config *head;
  uint32_t info_crc;
  uint32_t config_crc;
  enum mxt_config_type config_type;
};

//******************************************************************************
/// \brief  Determines whether the object type is used for CRC checksum calculation
/// \return True if used, false if not
static bool is_type_used_for_crc(const uint32_t type)
{
  switch (type) {
  case GEN_MESSAGEPROCESSOR_T5:
  case GEN_COMMANDPROCESSOR_T6:
  case DEBUG_DIAGNOSTIC_T37:
  case SPT_USERDATA_T38:
  case SPT_MESSAGECOUNT_T44:
  case SERIAL_DATA_COMMAND_T68:
    return false;

  default:
    return true;
  }
}

//******************************************************************************
/// \brief Some objects are volatile or read-only and should not be saved to config file
static bool mxt_object_is_volatile(uint16_t object_type)
{
  switch (object_type) {
  case DEBUG_DELTAS_T2:
  case DEBUG_REFERENCES_T3:
  case DEBUG_SIGNALS_T4:
  case GEN_MESSAGEPROCESSOR_T5:
  case GEN_COMMANDPROCESSOR_T6:
  case SPT_MESSAGECOUNT_T44:
  case GEN_DATASOURCE_T53:
    return true;

  default:
    return false;
  }
}

//******************************************************************************
/// \brief Free memory associated with configuration
static void mxt_free_config(struct mxt_config *cfg)
{
  struct mxt_object_config **curr = &cfg->head;
  struct mxt_object_config *next = cfg->head;

  while (*curr) {
    next = (*curr)->next;

    free((*curr)->data);
    (*curr)->data = NULL;
    free(*curr);
    *curr = NULL;

    *curr = next;
  }
}

//******************************************************************************
/// \brief Write configuration for a single object
static int mxt_write_object_config(struct mxt_device *mxt,
                                   const struct mxt_object_config *objcfg)
{
  uint8_t obj_buf[MXT_OBJECT_SIZE_MAX];
  uint16_t obj_addr;
  uint16_t device_size;
  uint16_t num_bytes;
  int ret;

  if (mxt_object_is_volatile(objcfg->type))
    return MXT_ERROR_OBJECT_IS_VOLATILE;

  obj_addr = mxt_get_object_address(mxt, objcfg->type, objcfg->instance);
  if (obj_addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  /* Read object config. This is done to retain any device configuration
   * remaining in trailing bytes not specified in the file. */
  memset(obj_buf, 0, sizeof(obj_buf));
  ret = mxt_read_register(mxt, obj_buf, obj_addr,
                          mxt_get_object_size(mxt, objcfg->type));
  if (ret)
    return ret;

  device_size = mxt_get_object_size(mxt, objcfg->type);

  if (device_size > objcfg->size) {
    mxt_warn(mxt->ctx, "Extending config by %d bytes in T%u",
             device_size - objcfg->size, objcfg->type);
    num_bytes = objcfg->size;
  } else if (objcfg->size > device_size) {
    /* Either we are in fallback mode due to wrong
     * config or config from a later fw version,
     * or the file is corrupt or hand-edited */
    mxt_warn(mxt->ctx, "Discarding %u bytes in T%u",
             objcfg->size - device_size, objcfg->type);
    num_bytes = device_size;
  } else {
    num_bytes = device_size;
  }

  /* Update bytes from config */
  memcpy(obj_buf, objcfg->data, num_bytes);

  /* Write object */
  ret = mxt_write_register(mxt, obj_buf, obj_addr, num_bytes);
  if (ret) {
    mxt_err(mxt->ctx, "Config write error, ret=%d", ret);
    return ret;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Write configuration to chip
static int mxt_write_device_config(struct mxt_device *mxt,
                                   struct mxt_config *cfg)
{
  struct mxt_object_config *objcfg = cfg->head;
  int ret;

  /* The Info Block CRC is calculated over mxt_id_info and the object table
   * If it does not match then we are trying to load the configuration
   * from a different chip or firmware version, so the configuration CRC
   * is invalid anyway. */
  if (cfg->info_crc && cfg->info_crc != mxt->info.crc)
    mxt_warn(mxt->ctx, "Info Block CRC mismatch - device=0x%06X "
             "file=0x%06X - attempting to apply config",
             mxt->info.crc, cfg->info_crc);

  mxt_info(mxt->ctx, "Writing config to chip");

  while (objcfg) {
    mxt_verb(mxt->ctx, "T%d instance %d size %d",
             objcfg->type, objcfg->instance, objcfg->size);

    ret = mxt_write_object_config(mxt, objcfg);
    if (ret == MXT_ERROR_OBJECT_NOT_FOUND)
      mxt_warn(mxt->ctx, "T%d not present", objcfg->type);
    else if (ret == MXT_ERROR_OBJECT_IS_VOLATILE)
      mxt_warn(mxt->ctx, "Skipping volatile T%d", objcfg->type);
    else if (ret)
      return ret;

    objcfg = objcfg->next;
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read configuration from chip
static int mxt_read_device_config(struct mxt_device *mxt,
                                  struct mxt_config *cfg)
{
  int obj_idx, instance;
  int ret;

  /* Copy ID information */
  memcpy(&cfg->id, mxt->info.id, sizeof(struct mxt_id_info));

  cfg->info_crc = mxt->info.crc;

  cfg->config_crc = mxt_get_config_crc(mxt);

  struct mxt_object_config **curr = &cfg->head;

  for (obj_idx = 0; obj_idx < mxt->info.id->num_objects; obj_idx++) {
    struct mxt_object object = mxt->info.objects[obj_idx];

    if (mxt_object_is_volatile(object.type))
      continue;

    for (instance = 0; instance < MXT_INSTANCES(object); instance++) {

      struct mxt_object_config *objcfg = calloc(1, sizeof(struct mxt_object_config));
      if (!objcfg) {
        ret = MXT_ERROR_NO_MEM;
        goto free;
      }

      objcfg->type = object.type;
      objcfg->size = MXT_SIZE(object);
      objcfg->instance = instance;
      objcfg->start_position = mxt_get_start_position(object, instance);

      /* Malloc memory to store configuration */
      objcfg->data = calloc(objcfg->size, sizeof(uint8_t));
      if (!objcfg->data) {
        free(objcfg);
        mxt_err(mxt->ctx, "Failed to allocate memory");
        ret = MXT_ERROR_NO_MEM;
        goto free;
      }

      ret = mxt_read_register(mxt, objcfg->data,
                              objcfg->start_position, objcfg->size);
      if (ret) {
        free(objcfg->data);
        free(objcfg);
        goto free;
      }

      *curr = objcfg;
      curr = &objcfg->next;
    }
  }

  mxt_info(mxt->ctx, "Read config from device");

  return MXT_SUCCESS;

free:
  mxt_free_config(cfg);
  return ret;
}

//******************************************************************************
/// \brief  Save configuration to OBP_RAW file
/// \return #mxt_rc
static int mxt_save_raw_file(struct libmaxtouch_ctx *ctx,
                             const char *filename,
                             struct mxt_config *cfg)
{
  struct mxt_id_info *id = &cfg->id;
  struct mxt_object_config *objcfg = cfg->head;
  FILE *fp;
  int ret;
  unsigned int i;

  fp = fopen(filename, "w");
  if (fp == NULL) {
    mxt_err(ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  ret = fprintf(fp, "OBP_RAW V1\n");
  if (ret < 0)
    goto fprintf_error;

  ret = fprintf(fp, "%02X %02X %02X %02X %02X %02X %02X\n"
                "%06X\n"
                "%06X\n",
                id->family, id->variant,
                id->version, id->build,
                id->matrix_x_size, id->matrix_y_size,
                id->num_objects,
                cfg->info_crc, cfg->config_crc);
  if (ret < 0)
    goto fprintf_error;

  while (objcfg) {
    if (mxt_object_is_volatile(objcfg->type))
      continue;

    ret = fprintf(fp, "%04X %04X %04X", objcfg->type, objcfg->instance, objcfg->size);
    if (ret < 0)
      goto fprintf_error;

    for (i = 0; i < objcfg->size; i++) {
      ret = fprintf(fp, " %02X", objcfg->data[i]);
      if (ret < 0)
        goto fprintf_error;
    }

    ret = fprintf(fp, "\n");
    if (ret < 0)
      goto fprintf_error;

    objcfg = objcfg->next;
  }

  fclose(fp);

  mxt_info(ctx, "Saved config to %s in OBP_RAW format", filename);
  return MXT_SUCCESS;

fprintf_error:
  fclose(fp);
  return mxt_errno_to_rc(errno);
}

//******************************************************************************
/// \brief  Save configuration to .xcfg file
/// \return #mxt_rc
static int mxt_save_xcfg_file(struct libmaxtouch_ctx *ctx,
                              const char *filename,
                              struct mxt_config *cfg)
{
  struct mxt_id_info *id = &cfg->id;
  struct mxt_object_config *objcfg = cfg->head;
  FILE *fp;
  int ret;
  unsigned int i;

  fp = fopen(filename, "w");
  if (fp == NULL) {
    mxt_err(ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  ret = fprintf(fp, "[COMMENTS]\nDate and time: ");
  if (ret < 0)
    goto fprintf_error;

  ret = mxt_print_timestamp(fp, true);
  if (ret) {
    fclose(fp);
    return ret;
  }

  ret = fprintf(fp, "\n"
                "[VERSION_INFO_HEADER]\n"
                "FAMILY_ID=%d\n"
                "VARIANT=%d\n"
                "VERSION=%d\n"
                "BUILD=%d\n"
                "CHECKSUM=0x%06X\n"
                "INFO_BLOCK_CHECKSUM=0x%02X\n"
                "[APPLICATION_INFO_HEADER]\n"
                "NAME=libmaxtouch\n"
                "VERSION=%s\n",
                id->family, id->variant,
                id->version, id->build,
                cfg->config_crc, cfg->info_crc,
                MXT_VERSION);
  if (ret < 0)
    goto fprintf_error;

  while (objcfg) {
    if (mxt_object_is_volatile(objcfg->type))
      continue;

    const char *obj_name = mxt_get_object_name(objcfg->type);
    if (obj_name) {
      ret = fprintf(fp, "[%s INSTANCE %d]\n", obj_name, objcfg->instance);
      if (ret < 0)
        goto fprintf_error;
    } else {
      ret = fprintf(fp, "[UNKNOWN_T%d INSTANCE %d]\n", objcfg->type, objcfg->instance);
      if (ret < 0)
        goto fprintf_error;
    }

    ret = fprintf(fp, "OBJECT_ADDRESS=%d\n"
                  "OBJECT_SIZE=%d\n",
                  objcfg->start_position,
                  objcfg->size);
    if (ret < 0)
      goto fprintf_error;

    for (i = 0; i < objcfg->size; i++) {
      ret = fprintf(fp, "%d 1 UNKNOWN[%d]=%d\n", i, i, objcfg->data[i]);
      if (ret < 0)
        goto fprintf_error;
    }

    objcfg = objcfg->next;
  }

  fclose(fp);

  ret = MXT_SUCCESS;
  mxt_info(ctx, "Saved config to %s in .xcfg format", filename);
  return MXT_SUCCESS;

fprintf_error:
  fclose(fp);
  return mxt_errno_to_rc(errno);
}

//******************************************************************************
/// \brief  Load configuration from .xcfg file
/// \return #mxt_rc
static int mxt_load_xcfg_file(struct libmaxtouch_ctx *ctx, const char *filename,
                              struct mxt_config *cfg)
{
  FILE *fp;
  int c;
  char object[255];
  char tmp[255];
  char *substr;
  int object_id;
  int instance;
  int object_address;
  int object_size;
  int data;
  int file_read = 0;
  bool ignore_line = false;
  int ret;
  struct mxt_object_config **next = &cfg->head;
  struct mxt_object_config *objcfg;

  cfg->config_type = CONFIG_XCFG;

  fp = fopen(filename, "r");
  if (fp == NULL) {
    mxt_err(ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  while (!file_read) {
    /* First character is expected to be '[' - skip empty lines and spaces  */
    c = getc(fp);
    while ((c == '\n') || (c == '\r') || (c == 0x20))
      c = getc(fp);

    if (c != '[') {
      if (c == EOF)
        break;

      /* If we are ignoring the current section then look for the next section */
      if (ignore_line) {
        continue;
      }

      mxt_err(ctx, "Parse error: expected '[', read ascii char %c!", c);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (fscanf(fp, "%[^] ]", object) != 1) {
      mxt_err(ctx, "Object parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    /* Ignore the comments and file header sections */
    if (!strcmp(object, "COMMENTS")
        || !strcmp(object, "APPLICATION_INFO_HEADER")) {
      ignore_line = true;
      mxt_dbg(ctx, "Skipping %s", object);
      continue;
    }

    ignore_line = false;

    /* Extract the checksum */
    if (!strcmp(object, "VERSION_INFO_HEADER")) {
      while (false == ignore_line) {
        if (fscanf(fp, "%s", tmp) != 1) {
          mxt_err(ctx, "Version info header parse error");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
        }
        if (!strncmp(tmp, "CHECKSUM", 8)) {
          sscanf(tmp, "%[^'=']=%x", object, &cfg->config_crc);
          mxt_dbg(ctx, "Config CRC from file: %s", tmp);
          ignore_line = true;
        }
      }
      continue;
    }

    if (fscanf(fp, "%s", tmp) != 1) {
      mxt_err(ctx, "Instance parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (strcmp(tmp, "INSTANCE")) {
      mxt_err(ctx, "Parse error, expected INSTANCE, got %s", tmp);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (fscanf(fp, "%d", &instance) != 1) {
      mxt_err(ctx, "Instance number parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    /* Read rest of header section */
    while(c != ']') {
      c = getc(fp);
      if (c == '\n') {
        mxt_err(ctx, "Parse error, expected ] before end of line");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }
    }

    while(c != '\n')
      c = getc(fp);

    while ((c != '=') && (c != EOF))
      c = getc(fp);

    if (fscanf(fp, "%d", &object_address) != 1) {
      mxt_err(ctx, "Object address parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    c = getc(fp);
    while((c != '=') && (c != EOF))
      c = getc(fp);

    if (fscanf(fp, "%d", &object_size) != 1) {
      mxt_err(ctx, "Object size parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    c = getc(fp);

    /* Find object type ID number at end of object string */
    substr = strrchr(object, '_');
    if (substr == NULL || (*(substr + 1) != 'T')) {
      mxt_err(ctx, "Parse error, could not find T number in %s", object);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (sscanf(substr + 2, "%d", &object_id) != 1) {
      mxt_err(ctx, "Unable to get object type ID for %s", object);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    mxt_dbg(ctx, "%s T%u OBJECT_ADDRESS=%d OBJECT_SIZE=%d",
            object, object_id, object_address, object_size);

    objcfg = calloc(1, sizeof(struct mxt_object_config));
    if (!objcfg) {
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    objcfg->type = object_id;
    objcfg->size = object_size;
    objcfg->instance = instance;
    objcfg->start_position = object_address;

    *next = objcfg;
    next = &objcfg->next;

    /* Allocate memory to store configuration */
    objcfg->data = calloc(object_size, sizeof(uint8_t));
    if (!objcfg->data) {
      mxt_err(ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    while (true) {
      int offset;
      int width;

      /* Find next line, check first character valid and rewind */
      c = getc(fp);
      while((c == '\n') || (c == '\r') || (c == 0x20))
        c = getc(fp);

      fseek(fp, -1, SEEK_CUR);

      /* End of object */
      if (c == '[')
        break;

      /* End of file */
      if (c == EOF)
        break;

      /* Read address */
      if (fscanf(fp, "%d", &offset) != 1) {
        mxt_err(ctx, "Address parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      /* Read byte count of this register (max 2) */
      if (fscanf(fp, "%d", &width) != 1) {
        mxt_err(ctx, "Byte count parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      while((c != '=') && (c != EOF)) {
        c = getc(fp);
      }

      if (fscanf(fp, "%d", &data) != 1) {
        mxt_err(ctx, "Data parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      c = getc(fp);

      switch (width) {
      case 1:
        objcfg->data[offset] = (char) data;
        break;
      case 2:
        objcfg->data[offset] = (char) data & 0xFF;
        objcfg->data[offset + 1] = (char) ((data >> 8) & 0xFF);
        break;
      case 4:
        objcfg->data[offset] = (char) data & 0xFF;
        objcfg->data[offset + 1] = (char) ((data >> 8) & 0xFF);
        objcfg->data[offset + 2] = (char) ((data >> 16) & 0xFF);
        objcfg->data[offset + 3] = (char) ((data >> 24) & 0xFF);
        break;
      default:
        mxt_err(ctx, "Only 1, 2 and 4 byte config values are supported");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }
    }
  }

  ret = MXT_SUCCESS;
  mxt_info(ctx, "Configuration read from %s in XCFG format", filename);

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Load configuration from RAW file
//  \return #mxt_rc
static int mxt_load_raw_file(struct libmaxtouch_ctx *ctx, const char *filename,
                             struct mxt_config *cfg)
{
  FILE *fp;
  int ret;
  size_t i;
  char line[2048];
  struct mxt_object_config **next;
  struct mxt_object_config *objcfg;

  cfg->config_type = CONFIG_RAW;

  fp = fopen(filename, "r");
  if (fp == NULL) {
    mxt_err(ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  if (fgets(line, sizeof(line), fp) == NULL) {
    mxt_err(ctx, "Unexpected EOF");
  }

  if (strncmp(line, OBP_RAW_MAGIC, strlen(OBP_RAW_MAGIC))) {
    mxt_warn(ctx, "Not in OBP_RAW format");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  } else {
    mxt_dbg(ctx, "Loading OBP_RAW file");
  }

  /* Load information block and check */
  for (i = 0; i < sizeof(struct mxt_id_info); i++) {
    ret = fscanf(fp, "%hhx", (unsigned char *)&cfg->id + i);
    if (ret != 1) {
      mxt_err(ctx, "Bad format");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }
  }

  /* Read CRCs */
  ret = fscanf(fp, "%x", &cfg->info_crc);
  if (ret != 1) {
    mxt_err(ctx, "Bad format: failed to parse Info CRC");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  }

  ret = fscanf(fp, "%x", &cfg->config_crc);
  if (ret != 1) {
    mxt_err(ctx, "Bad format: failed to parse Config CRC");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  }

  next = &cfg->head;

  while (true) {
    objcfg = calloc(1, sizeof(struct mxt_object_config));
    if (!objcfg) {
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    /* Read type, instance, length */
    ret = fscanf(fp, "%x %" SCNx8 " %x",
                 &objcfg->type, &objcfg->instance, &objcfg->size);
    if (ret == EOF) {
      free(objcfg);
      break;
    } else if (ret != 3) {
      mxt_err(ctx, "Bad format: failed to parse object");
      free(objcfg);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    mxt_dbg(ctx, "OBP_RAW T%u instance %u", objcfg->type, objcfg->instance);

    *next = objcfg;
    next = &objcfg->next;

    /* Malloc memory to store configuration */
    objcfg->data = calloc(objcfg->size, sizeof(uint8_t));
    if (!objcfg->data) {
      mxt_err(ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    /* Read bytes from file */
    for (i = 0; i < objcfg->size; i++) {
      uint8_t val;
      ret = fscanf(fp, "%hhx", &val);
      if (ret != 1) {
        mxt_err(ctx, "Parse error in T%d", objcfg->type);
        ret = MXT_ERROR_FILE_FORMAT;
        free(objcfg->data);
        goto close;
      }

      *(objcfg->data + i) = val;
    }

    mxt_log_buffer(ctx, LOG_DEBUG, "CFG:", objcfg->data, objcfg->size);
  }

  ret = MXT_SUCCESS;
  mxt_info(ctx, "Configuration read from %s in OBP_RAW format", filename);

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Get the configuration from file
/// \return #mxt_rc
static int mxt_get_config_from_file(struct libmaxtouch_ctx *ctx,
                                    const char *filename, struct mxt_config *cfg)
{
  int ret;
  char *extension = strrchr(filename, '.');

  if (cfg) {
    if (extension && !strcmp(extension, ".xcfg")) {
      ret = mxt_load_xcfg_file(ctx, filename, cfg);
      if (ret)
        return ret;
    } else {
      ret = mxt_load_raw_file(ctx, filename, cfg);
      if (ret)
        return ret;
    }
  } else {
    ret = MXT_INTERNAL_ERROR;
    mxt_err(ctx, "Config is null");
  }
  return ret;
}

//******************************************************************************
/// \brief  Load configuration from .xcfg or RAW file, automatically detect
//          format and write to device
/// \return #mxt_rc
int mxt_load_config_file(struct mxt_device *mxt, const char *filename)
{
  struct mxt_config cfg = {{0}};
  int ret = mxt_get_config_from_file(mxt->ctx, filename, &cfg);
  if (ret == MXT_SUCCESS) {
    ret = mxt_write_device_config(mxt, &cfg);
    mxt_free_config(&cfg);
  }

  return ret;
}

//******************************************************************************
/// \brief  Save configuration to file
/// \return #mxt_rc
int mxt_save_config_file(struct mxt_device *mxt, const char *filename)
{
  int ret;

  char *extension = strrchr(filename, '.');
  struct mxt_config cfg = {{0}};

  ret = mxt_read_device_config(mxt, &cfg);
  if (ret)
    return ret;

  if (extension && !strcmp(extension, ".xcfg"))
    ret = mxt_save_xcfg_file(mxt->ctx, filename, &cfg);
  else
    ret = mxt_save_raw_file(mxt->ctx, filename, &cfg);

  mxt_free_config(&cfg);

  return ret;
}

//******************************************************************************
/// \brief  Zero all configuration settings
/// \return #mxt_rc
int mxt_zero_config(struct mxt_device *mxt)
{
  int obj_idx, instance, num_bytes;
  uint8_t *buf;
  struct mxt_object object;
  struct mxt_id_info *id = mxt->info.id;
  int ret;

  mxt_info(mxt->ctx, "Zeroing all configuration settings...");

  /* Single buffer to match MAX object size*/
  buf = (uint8_t *)calloc(1, MXT_OBJECT_SIZE_MAX);
  if (buf == NULL) {
    mxt_err(mxt->ctx, "Failed to allocate memory");
    return MXT_ERROR_NO_MEM;
  }

  for (obj_idx = 0; obj_idx < id->num_objects; obj_idx++) {
    object = mxt->info.objects[obj_idx];
    num_bytes = MXT_SIZE(object);

    for (instance = 0; instance < MXT_INSTANCES(object); instance++) {
      int address = mxt_get_start_position(object, instance);
      ret = mxt_write_register(mxt, buf, address, num_bytes);
      if (ret)
        goto free;
    }
  }
  ret = MXT_SUCCESS;

free:
  free(buf);
  return ret;
}

//******************************************************************************
/// \brief  Check the checksum for a given file
/// \return #mxt_rc
int mxt_checkcrc(struct libmaxtouch_ctx *ctx, struct mxt_device *mxt, char *filename, uint32_t *crc)
{
  int ret;
  struct mxt_config cfg = {{0}};
  uint16_t obj_idx = 0;

  int start_pos = INT_MAX;
  uint16_t end_pos = 0;

  /* Get the mxt_config and object configurations */
  ret = mxt_get_config_from_file(ctx, filename, &cfg);
  if (ret)
    goto free;

  /* Find limits of CRC region */
  struct mxt_object_config *objcfg = cfg.head;
  if (mxt == NULL) {
    if (cfg.config_type == CONFIG_RAW) {
      mxt_err(ctx, "RAW config format only supported with chip present");
      ret = MXT_ERROR_NO_DEVICE;
      goto free;
    } else {
      while (objcfg) {
        if (is_type_used_for_crc(objcfg->type)) {
          if (start_pos > objcfg->start_position)
            start_pos = objcfg->start_position;

          if (end_pos < (objcfg->start_position + objcfg->size))
            end_pos = objcfg->start_position + objcfg->size;
        }

        objcfg = objcfg->next;
      }
    }
  } else {
    for (obj_idx = 0; obj_idx < mxt->info.id->num_objects; obj_idx++) {
      struct mxt_object *object = &mxt->info.objects[obj_idx];
      if (is_type_used_for_crc(object->type)) {
        int sp = mxt_get_object_address(mxt, object->type, 0);
        if (start_pos > sp)
          start_pos = sp;

        int obj_size = mxt_get_object_size(mxt, object->type);
        if (end_pos < (sp + obj_size))
          end_pos = sp + obj_size;
      }
    }
  }

  mxt_verb(ctx, "CRC start_pos:%d end_pos:%d", start_pos, end_pos);

  /* Allocate buffer for CRC calculation */
  uint8_t *buffer = (uint8_t *)calloc(end_pos, sizeof(uint8_t));
  if (!buffer) {
    ret = MXT_ERROR_NO_MEM;
    mxt_err(ctx, "Could not allocate memory for buffer");
    goto free;
  }

  /* Loop through until all buffers have been read in order */
  objcfg = cfg.head;
  while (objcfg) {
    if (is_type_used_for_crc(objcfg->type)) {
      uint16_t off = objcfg->start_position;
      if (cfg.config_type != CONFIG_XCFG)
        off = mxt_get_object_address(mxt, objcfg->type, objcfg->instance);

      memcpy(buffer + off, objcfg->data, objcfg->size);
    }

    objcfg = objcfg->next;
  }

  uint32_t calc_crc;
  ret = mxt_calculate_crc(ctx, &calc_crc, buffer + start_pos, end_pos - start_pos);

  if (calc_crc == cfg.config_crc) {
    mxt_info(ctx, "File checksum verified: %d", cfg.config_crc);
    *crc = cfg.config_crc;
    ret = MXT_SUCCESS;
  } else {
    mxt_err(ctx, "Checksum error: calc=%06X file=%06X", calc_crc, cfg.config_crc);
    ret = MXT_ERROR_CHECKSUM_MISMATCH;
  }

  free(buffer);

free:
  mxt_free_config(&cfg);
  return ret;
}
