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
#include "mxt-app/serial_data.h"
#include "mxt-app/buffer.h"

#define OBP_RAW_MAGIC_V1      "OBP_RAW V1"
#define OBP_RAW_MAGIC_V2      "OBP_RAW V2"
#define OBP_RAW_MAGIC_V3      "OBP_RAW V3"
#define ENC_BLOCK_HDR         "MAX_ENCRYPTION_BLOCKS"
#define ENC_HDR               "ENCRYPTION"
#define ENC_BIT_MASK          0x7F


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
  uint16_t instance;
  uint32_t size;
  uint32_t enc_size;
  uint16_t start_position;
  uint8_t *data;
  struct mxt_object_config *next;
};

//******************************************************************************
/// \brief Device configuration
struct mxt_config {
  struct mxt_device mxt;
  struct mxt_id_info id;
  struct mxt_object_config *head;
  uint32_t info_crc;
  uint32_t config_crc;
  int cfg_enc;
  int cfg_blksize;
  int cfg_version;
  enum mxt_config_type config_type;
};

//******************************************************************************
/// \brief Check encryption status of device
/// \return Success, if T2 read; error if T2 not read
int mxt_check_encryption(struct mxt_device *mxt)
{
  uint8_t buf[3];
  uint32_t checksum;
  int ret;

  mxt->mxt_enc.addr = mxt_get_object_address(mxt,
    GEN_ENCRYPTIONSTATUS_T2, 0);
  if (mxt->mxt_enc.addr == OBJECT_NOT_FOUND)
    return MXT_ERROR_OBJECT_NOT_FOUND;

  ret = mxt_read_register(mxt, buf, mxt->mxt_enc.addr, 1);

  if ((buf[0] & MSGENCEN) || (buf[0] & CONFIGENCEN)) {
    /* TBD may be redundant as MSBit is checked with variant ID */
    SET_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED);

    mxt_info(mxt->ctx, "Device encryption enabled\n");

    if (buf[0] & MSGENCEN) {
      SET_BIT(mxt->mxt_enc.encryption_state, MSG_ENCRYPTED);
      mxt->mxt_enc.msg_enc_enabled = true;
    }
    else {
      CLEAR_BIT(mxt->mxt_enc.encryption_state, MSG_ENCRYPTED);
      mxt->mxt_enc.msg_enc_enabled = false;
    }

    if (buf[0] & CONFIGENCEN) {
      SET_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED);
      mxt->mxt_enc.enc_blocksize = 0x30;
    } else {
      CLEAR_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED);
    }
  } else {
    mxt->mxt_enc.encryption_state = 0x00;
  }

  ret = mxt_read_register(mxt, buf, mxt->mxt_enc.addr + T2_PAYLOADCRC_OFFSET, 3);

  checksum = (buf[0] | (buf[1] << 8) | (buf[2] << 16));

  mxt_dbg(mxt->ctx, "mxt-app: T2 Payload CRC = 0x%06X", checksum);

  ret = mxt_read_register(mxt, buf, mxt->mxt_enc.addr + T2_ENCKEYCRC_OFFSET, 3);

  checksum = (buf[0] | (buf[1] << 8) | (buf[2] << 16));

  mxt_dbg(mxt->ctx, "mxt-app: T2 Enc Customer Key CRC = 0x%06X", checksum);

  return ret;
}

//******************************************************************************
/// \brief  Determines whether the object type is used for CRC checksum calculation
/// \return True if used, false if not
static bool is_type_used_for_crc(const uint32_t type)
{
  switch (type) {
  case GEN_ENCRYPTIONSTATUS_T2:
  case GEN_MESSAGEPROCESSOR_T5:
  case GEN_COMMANDPROCESSOR_T6:
  case DEBUG_DIAGNOSTIC_T37:
  case SPT_USERDATA_T38:
  case SPT_MESSAGECOUNT_T44:
  case SERIAL_DATA_COMMAND_T68:
  case SPT_MESSAGECOUNT_T144:
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
  case DEBUG_REFERENCES_T3:
  case DEBUG_SIGNALS_T4:
  case GEN_MESSAGEPROCESSOR_T5:
  case GEN_COMMANDPROCESSOR_T6:
  case SPT_MESSAGECOUNT_T44:
  case GEN_DATASOURCE_T53:
  case DEBUG_DIAGNOSTIC_T37:
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
  struct t68_ctx ctx;
  uint8_t obj_buf[MXT_OBJECT_SIZE_MAX];
  uint16_t obj_addr;
  uint16_t device_size;
  uint16_t num_bytes;
  uint8_t value = 0;
  uint8_t diff = 0;
  bool t38_found = false;
  int ret = 0;
  int i;

  if (mxt_object_is_volatile(objcfg->type)) {
    ret = MXT_ERROR_OBJECT_IS_VOLATILE;
    goto close_object_write;
  }

  mxt_dbg(mxt->ctx, "write_object: T%d instance %d size %d",
             objcfg->type, objcfg->instance, objcfg->size);

  if ((objcfg->type == SERIAL_DATA_COMMAND_T68) && 
      (objcfg->instance & 0x8000)) {

    mxt_info(mxt->ctx, "Found T68 payload");

    ctx.t68_datatype = objcfg->instance & 0x000F;

    obj_addr = mxt_get_object_address(mxt, objcfg->type, 0);
    if (obj_addr == OBJECT_NOT_FOUND) {
       ret = MXT_ERROR_OBJECT_NOT_FOUND;
       goto close_object_write;
     }

    if (ctx.t68_datatype == 0x000C) {
      /* Check if device is encrypted or not */
      if (CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED)) {
        /* If either bit encrypted, do not allow programming of T68 payload */
        if (objcfg->data[32] & 0x06) {
          mxt_err(mxt->ctx, "Error: Device already encrypted, cannot program T68 payload\n");
          goto close_object_write;
        }
      }

      if (objcfg->data[32] & 0x04) {
        mxt_info(mxt->ctx, "Enabling T5 msg encryption");
        mxt->mxt_enc.msg_enc_enabled = true;
      } else if (objcfg->data[32] & 0x02) {
        mxt_info(mxt->ctx, "Enabling cfg encryption");
        mxt->mxt_enc.msg_enc_enabled = false;
      } else if ((objcfg->data[32] & 0x06) == 0x06) {
        mxt_info(mxt->ctx, "Enabling T5 msg and cfg encryption");
        mxt->mxt_enc.msg_enc_enabled = true;
      } else if ((objcfg->data[32] & 0x06) == 0x00) {
        /* not a scenario that should happen */
        mxt->mxt_enc.msg_enc_enabled = false;
        mxt_info(mxt->ctx, "Disabling device encryption");
      }
    }

    ret = mxt_buf_init(&ctx.buf);

    for (i=0; i < objcfg->size; i++) {
      value = objcfg->data[i];
      ret = mxt_buf_add(&ctx.buf, value);
    }

    /* Calculate position of CMD register */
    ctx.t68_size = mxt_get_object_size(mxt, SERIAL_DATA_COMMAND_T68);

    ctx.t68_data_size = ctx.t68_size - 9;

    /* Remove payload CRC, do not write to T68 */
    ctx.t68_length = objcfg->size - 4;

    /* Fill T68 payload with 0x00 padding and add to buffer */
    if (objcfg->size < ctx.t68_data_size) {
      diff = ctx.t68_data_size - objcfg->size;

      for (i=0; i < diff; i++) {
        ret = mxt_buf_add(&ctx.buf, 0x00);
      }
    }

    ret = mxt_load_t68_payload(mxt, &ctx);

  } else if (!(CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED))) {  

    obj_addr = mxt_get_object_address(mxt, objcfg->type, objcfg->instance);
    
    if (obj_addr == OBJECT_NOT_FOUND) {
      ret = MXT_ERROR_OBJECT_NOT_FOUND;
      goto close_object_write;
    }

    /* Read object config. This is done to retain any device configuration
    * remaining in trailing bytes not specified in the file. */
    memset(obj_buf, 0, sizeof(obj_buf));

    device_size = mxt_get_object_size(mxt, objcfg->type);

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

  } else { /* encrypted data */

    memset(obj_buf, 0, sizeof(obj_buf));

    obj_addr = mxt_get_object_address(mxt, objcfg->type, objcfg->instance);
    if (obj_addr == OBJECT_NOT_FOUND) {
      ret = MXT_ERROR_OBJECT_NOT_FOUND;
      goto close_object_write;
    }

    /* Add two bytes for embedded datasize */
    /* Send datasize as embedded data to driver */
    num_bytes = objcfg->size + 2;
  }

  if(objcfg->type == 38) {
    t38_found = true;
  }

  /* Update bytes from config */
  memcpy(obj_buf, objcfg->data, num_bytes);

  /* Write object */
  ret = mxt_write_register(mxt, obj_buf, obj_addr, num_bytes);
  if (ret) {
    mxt_err(mxt->ctx, "Config write error, ret=%d", ret);
    goto close_object_write;
  }
 
close_object_write:
  return ret;
}

//******************************************************************************
/// \brief Write configuration to chip
static int mxt_write_device_config(struct mxt_device *mxt,
                                   struct mxt_config *cfg)
{
  struct mxt_object_config *objcfg = cfg->head;
  int ret;
  int err;

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

    if (mxt->conn->type == E_I2C_DEV)
      mxt->mxt_crc.config_triggered = true;

    if (mxt->conn->type == E_SYSFS_SPI) {
      err = sysfs_set_debug_irq(mxt, false);
    }

    ret = mxt_write_object_config(mxt, objcfg);
    if (ret == MXT_ERROR_OBJECT_NOT_FOUND)
      mxt_warn(mxt->ctx, "T%d not present", objcfg->type);
    else if (ret == MXT_ERROR_OBJECT_IS_VOLATILE)
      mxt_warn(mxt->ctx, "Skipping volatile T%d", objcfg->type);
    else if (ret){
      mxt->mxt_crc.config_triggered = false;

    if (mxt->conn->type == E_SYSFS_SPI)
      err = sysfs_set_debug_irq(mxt, true);

      if (mxt->conn->type == E_I2C_DEV  && mxt->debug_fs.enabled == true) {
      	/* Allow messages to be read thru mxt-app */
        err = debugfs_set_irq(mxt, true);
      } else if (mxt->conn->type == E_SYSFS_I2C){
        if (mxt->mxt_crc.crc_enabled == true)
          err = sysfs_set_debug_irq(mxt, true);
      }

      return ret;
    }

    objcfg = objcfg->next;
  }
  
  mxt->mxt_crc.config_triggered = false;
  
  if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true){
    err = debugfs_set_irq(mxt, true);
  } else if (mxt->conn->type == E_SYSFS_I2C){
      if (mxt->mxt_crc.crc_enabled == true)
        err = sysfs_set_debug_irq(mxt, true);
  }

  if (mxt->conn->type == E_SYSFS_SPI)
      err = sysfs_set_debug_irq(mxt, true);

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Read configuration from chip
static int mxt_read_device_config(struct mxt_device *mxt,
                                  struct mxt_config *cfg)
{
  int obj_idx, instance;
  int ret;

  /* Restore config data from NVM and stop T70 events */
  ret = mxt_backup_config(mxt, RESTORENV_COMMAND);

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
                             struct mxt_config *cfg, uint8_t format)
{
  struct mxt_id_info *id = &cfg->id;
  struct mxt_object_config *objcfg = cfg->head;
  struct mxt_device mxt;
  uint8_t encryption = 0;
  uint8_t enc_blocksize = 0;
  FILE *fp;
  int ret;
  unsigned int i;

  mxt = cfg->mxt;

  fp = fopen(filename, "w");
  if (fp == NULL) {
    mxt_err(ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  switch (format) {
    case 0:
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

      break;

    case 3:

      ret = fprintf(fp, "OBP_RAW V3\n");
        if (ret < 0)
        goto fprintf_error;

    if (CHECK_BIT(mxt.mxt_enc.encryption_state, CFG_ENCRYPTED)) {
      encryption = true;
      enc_blocksize = 3;
    }
      ret = fprintf(fp, "ENCRYPTION %d\n", encryption);
        if (ret < 0)
        goto fprintf_error;

      ret = fprintf(fp, "MAX_ENCRYPTION_BLOCKS %d\n", enc_blocksize);
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

      break;

    default:
      break;
  }

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
                              struct mxt_config *cfg, uint8_t format)
{
  struct mxt_id_info *id = &cfg->id;
  struct mxt_object_config *objcfg = cfg->head;
  struct mxt_device mxt;
  FILE *fp;
  int ret;
  unsigned int i;
  bool encryption = false;
  uint8_t enc_blocksize = 0;

  mxt = cfg->mxt;

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

  switch (format) {
    case 0x00:
    case 0x01:

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

    break;

    case 0x02:

      mxt_err(ctx, "Format %x is not yet supported", format);

    case 0x03:

    /* Check encrption */
    if (CHECK_BIT(mxt.mxt_enc.encryption_state, CFG_ENCRYPTED)) {
      encryption = true;
      enc_blocksize = 3;
    }
    
      ret = fprintf(fp, "\n"
                "[VERSION_INFO_HEADER]\n"
                "FAMILY_ID=%d\n"
                "VARIANT=%d\n"
                "VERSION=%d\n"
                "BUILD=%d\n"
                "CHECKSUM=0x%06X\n"
                "INFO_BLOCK_CHECKSUM=0x%02X\n"
                "[FILE_INFO_HEADER]\n"
                "VERSION=3\n"
                "ENCRYPTION=%x\n"
                "MAX_ENCRYPTION_BLOCKS=%d\n"
                "[APPLICATION_INFO_HEADER]\n"
                "NAME=libmaxtouch\n"
                "VERSION=%s\n",
                id->family, id->variant,
                id->version, id->build,
                cfg->config_crc, cfg->info_crc,
                encryption, enc_blocksize, 
                MXT_VERSION);

      if (ret < 0)
        goto fprintf_error;

      break;

      case 0x04:
        mxt_err(ctx, "Format %x is not yet supported", format);

        break;

      default:
        break;

  }

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
static int mxt_load_xcfg_file(struct mxt_device *mxt, const char *filename,
                              struct mxt_config *cfg)
{
  FILE *fp;
  struct mxt_object_config **next = &cfg->head;
  struct mxt_object_config *objcfg;
  struct t68_ctx t68_ctx;
  char object[255];
  char tmp[255];
  char encryption_type[255];
  char *substr;
  int object_id;
  int instance;
  int object_address;
  int object_size;
  int data;
  int payload_crc;
  int payload_size;
  int file_read = 0;
  bool ignore_line = false;
  bool skip_addr_size = false;
  bool add_datasize = true;
  uint8_t family_id = 0;
  uint8_t variant_id = 0;
  uint16_t t38_addr = 0;
  bool t68_not = true;
  int ret;
  int c;

  cfg->config_type = CONFIG_XCFG;

  t38_addr = mxt_get_object_address(mxt, SPT_USERDATA_T38, 0);

  if(t38_addr == OBJECT_NOT_FOUND) {
    ret = OBJECT_NOT_FOUND;
    mxt_err(mxt->ctx, "Could not find T38 object or not supported");
    return MXT_ERROR_OBJECT_NOT_FOUND;
  }

  fp = fopen(filename, "r");
  if (fp == NULL) {
    mxt_err(mxt->ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  while (!file_read) {
    /* First character is expected to be '[' - skip empty lines and spaces  */
    c = getc(fp);

    /* Skip newline, carrage return,spaces, get next char*/
    while ((c == '\n') || (c == '\r') || (c == 0x20))
      c = getc(fp);

    /* Always expecting starting '[' for xcfg file */
    if (c != '[') {

      if (c == EOF)
        break;

      /* If we are ignoring the current section then look for the next section */
      if (ignore_line) {
        continue;
      }

      mxt_err(mxt->ctx, "Parse error: expected '[', read ascii char %c!", c);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    } /* End of '[' */

    if (fscanf(fp, "%[^] ]", object) != 1) {
      mxt_err(mxt->ctx, "Object parse error");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    /* Ignore the comments and file info header sections */
    if (!strcmp(object, "COMMENTS")
        || !strcmp(object, "APPLICATION_INFO_HEADER")) {
      ignore_line = true;
      mxt_dbg(mxt->ctx, "Skipping %s", object);
      continue;
    }

    ignore_line = false;

    /* Extract the checksum */
    if (!strcmp(object, "VERSION_INFO_HEADER")) {
      while (false == ignore_line) {
        if (fscanf(fp, "%s", tmp) != 1) {
          mxt_err(mxt->ctx, "Version info header parse error");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
        }

        if (!strncmp(tmp, "FAMILY_ID", 9)) {
          sscanf(tmp, "%[^'=']=%" SCNu8, object, &family_id);

          if (family_id != mxt->info.id->family) {
            mxt_err(mxt->ctx, "Family ID mismatch error");
            ret = MXT_ERROR_FILE_FORMAT;
            goto close;
          }
        }        

        if (!strncmp(tmp, "VARIANT", 7)) {
          sscanf(tmp, "%[^'=']=%" SCNu8, object, &variant_id);

          /* Check variant before flashing */
          if ((variant_id & ENC_BIT_MASK) != 
            (mxt->info.id->variant & ENC_BIT_MASK)) {
            mxt_err(mxt->ctx, "Variant ID mismatch error");
            ret = MXT_ERROR_FILE_FORMAT;
            goto close;
          }
        }

        if (!strncmp(tmp, "CHECKSUM", 8)) {
          sscanf(tmp, "%[^'=']=%x", object, &cfg->config_crc);
          mxt_info(mxt->ctx, "Config File Checksum: %s", tmp);
          ignore_line = true;
        }
      }
      continue;
    }

    /* Extract content of file info header */
    if (!strcmp(object, "FILE_INFO_HEADER")) {
      while (false == ignore_line) {
        if (fscanf(fp, "%s", tmp) != 1) {
          mxt_err(mxt->ctx, "Version info header parse error");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
        }

        if (!strncmp(tmp, "VERSION", 7)) {
          sscanf(tmp, "%[^'=']=%d", object, &cfg->cfg_version);
          mxt_info(mxt->ctx, "Config version = %x", cfg->cfg_version);
        }

        if (!strncmp(tmp, "ENCRYPTION", 10)) {
          sscanf(tmp, "%[^'=']=%s", object, encryption_type);

          if (!strcmp(encryption_type, "FALSE")) {
            cfg->cfg_enc = false;  
          } else {   
            cfg->cfg_enc = true;
            if (mxt->conn->type == E_SYSFS_I2C) {
              mxt_err(mxt->ctx, "Warning: Writing encrypted config not supported in sysfs mode\n"
                      "Upgrade mxt-app required or use i2c-dev mode\n");
               
               return MXT_ERROR_NOT_SUPPORTED;
            }
          }

          if (cfg->cfg_enc) {
            if (CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) {
              mxt_info(mxt->ctx, "Config and device are encrypted."
              "Okay to update chip configuration.");
            } else {
              mxt_info(mxt->ctx, "Cannot program encrypted cfg into unencrypted device");
              ret = MXT_ERROR_FILE_FORMAT;
              goto close;            
            }

          } else {
            mxt_info(mxt->ctx, "Config file is unencrypted. Checking device encryption");
            if (CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) {
              mxt_info(mxt->ctx, "Cannot program unencrypted cfg into encrypted device");
              ret = MXT_ERROR_FILE_FORMAT;
              goto close;
            } else {
              mxt_info(mxt->ctx, "Device config is unencrypted. Okay to write new config.");
            }
          }
        }
        
        if (!strncmp(tmp, "MAX_ENCRYPTION_BLOCKS", 21)) {
          sscanf(tmp, "%[^'=']=%d", object, &cfg->cfg_blksize);
          ignore_line = true;
        }
      }
      continue;
    }

    /* Found a T68 payload, load parameters into variables */
    if (!strncmp(object, "T68_SERIALDATACOMMAND_PAYLOAD", 29)) {

      if (strstr(object, "ENCRYPTION") != NULL) {
          mxt_dbg(mxt->ctx, "Found Encryption payload");
          instance = 0x800C;
        } else if (strstr(object, "PWMPATTERN") != NULL) {
          mxt_dbg(mxt->ctx, "Found PWM payload");
          instance = 0x800A;
        } else if (strstr(object, "SELFCAP_EDGE") != NULL) {
          mxt_dbg(mxt->ctx, "Found Edge Shaping payload");
          instance = 0x800B;
        } else {
          mxt_dbg(mxt->ctx, "Payload datatype unknown");
          ret = MXT_ERROR_BAD_INPUT;
          goto close;
        }

      while (false == ignore_line) {
        if (fscanf(fp, "%s", tmp) != 1) {
          mxt_err(mxt->ctx, "Version info header parse error");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
        }

        if (!strncmp(tmp, "PAYLOAD_CHECKSUM", 16)) {
          sscanf(tmp, "%[^'=']=%x", object, &payload_crc);
          mxt_dbg(mxt->ctx, "Payload_CRC = %x", payload_crc);
        }

        if (!strncmp(tmp, "PAYLOAD_SIZE", 12)) {
          sscanf(tmp, "%[^'=']=%d", object, &payload_size);
          mxt_dbg(mxt->ctx, "Payload_size = %d", payload_size);
          skip_addr_size = true;
          ignore_line = true;
        }
      }
    }

    ignore_line = false;

    if (skip_addr_size != true) {
      /* Got object name: Look for Instance num */
      if (fscanf(fp, "%s", tmp) != 1) {
        mxt_err(mxt->ctx, "Instance parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      if (strcmp(tmp, "INSTANCE")) {
        mxt_err(mxt->ctx, "Parse error, expected INSTANCE, got %s", tmp);
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      if (fscanf(fp, "%d", &instance) != 1) {
        mxt_err(mxt->ctx, "Instance number parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

    /* Read rest of header section */
      while(c != ']') {
        c = getc(fp);

        if (c == '\n') {
          mxt_err(mxt->ctx, "Parse error, expected ] before end of line");
          ret = MXT_ERROR_FILE_FORMAT;
          goto close;
        }
      }

      while(c != '\n') 
        c = getc(fp);

      while ((c != '=') && (c != EOF))
        c = getc(fp);

      if (fscanf(fp, "%d", &object_address) != 1) {
        mxt_err(mxt->ctx, "Object address parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      c = getc(fp);

      while((c != '=') && (c != EOF))
        c = getc(fp);

      if (fscanf(fp, "%d", &object_size) != 1) {
        mxt_err(mxt->ctx, "Object size parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      c = getc(fp);

      /* Find object type ID number at end of object string */
      substr = strrchr(object, '_');
      if (substr == NULL || (*(substr + 1) != 'T')) {
        mxt_err(mxt->ctx, "Parse error, could not find T number in %s", object);
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      if (sscanf(substr + 2, "%d", &object_id) != 1) {
        mxt_err(mxt->ctx, "Unable to get object type ID for %s", object);
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }
    }  /* End of skip address size */

    /* Save T68 data, found, don't do 2nd time */
    if (object_id == 68) {
      t68_ctx.t68_object_id = object_id;
      t68_ctx.t68_size = object_size;
      t68_ctx.t68_instance = instance;
      t68_ctx.t68_addr = object_address;
    }

    mxt_dbg(mxt->ctx, "%s T%u OBJECT_ADDRESS=%d OBJECT_SIZE=%d",
            object, object_id, object_address, object_size);

    objcfg = calloc(1, sizeof(struct mxt_object_config));
    if (!objcfg) {
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    /* Collect T68 addr, size, instance */
    if (skip_addr_size != true) {
      objcfg->type = object_id;
      objcfg->instance = instance;
      objcfg->start_position = object_address;
      objcfg->size = object_size;

      if (CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) {

        objcfg->enc_size = object_size;
        mxt->mxt_enc.enc_datasize = object_size;
        add_datasize = true;

        if (object_address > t38_addr) {
          /* Padding only needed for CFG enc enabled, not MSG */
          /* Required for xcfg, data is multiples of 16 */
          /* Num of objects requires padding */
          /* TBD - May not need, since .xcfg file content already aligned */
          if ((object_size % 16 == 0x00) && (object_size != 0x00)) {
            objcfg->size = object_size;
          } else {
            /* Adjust size, multiple of 16 if size < 16 bytes*/
            objcfg->size = (16 - (object_size % 16) + object_size);
          }
        }
        /* increase object size overall to 2 for datasize header */
        //objcfg->size += 2;
      } /* Add msg enc handling */

      t68_not = true;
    /* TBD need to figure out how to manage MSG enabled */
    /* adds datasize bytes but does not limit blocksize */
    /* Does not need padding */
    } else {
      objcfg->type = t68_ctx.t68_object_id;
      objcfg->size = payload_size + 4;  /* do I need this for xcfg */
      //objcfg->size = payload_size + 2;  /* add two for embedded datasize */
      objcfg->instance = instance;
      objcfg->start_position = t68_ctx.t68_addr;
      skip_addr_size = false;
      add_datasize = false;
      t68_not = false;
    }

    *next = objcfg;
    next = &objcfg->next;

    /* Allocate memory to store configuration */
    objcfg->data = calloc(objcfg->size + 2, sizeof(uint8_t));
    if (!objcfg->data) {
      mxt_err(mxt->ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    int index = 0;

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
      if (c == EOF) {
        file_read = true;
        break;
      }

      /* Read address */
      if (fscanf(fp, "%d", &offset) != 1) {
        mxt_err(mxt->ctx, "Address parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      /* Embed the datasize to the content of the configuration */
      if ((CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) && 
          (add_datasize == true)) {
          /* increase the offset to save two bytes of datasize */
          /* Store at the beginning of data buffer */
          objcfg->data[offset] = objcfg->enc_size & 0x00ff;
          objcfg->data[offset + 1] = ((objcfg->enc_size >> 8) & 0x00ff);

          add_datasize = false;

      } /* Add msg encryption handling later */

      /* Read byte count of this register (max 2) */
      if (fscanf(fp, "%d", &width) != 1) {
        mxt_err(mxt->ctx, "Byte count parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      while((c != '=') && (c != EOF)) {
        c = getc(fp);
      }

      if (fscanf(fp, "%d", &data) != 1) {
        mxt_err(mxt->ctx, "Data parse error");
        ret = MXT_ERROR_FILE_FORMAT;
        goto close;
      }

      c = getc(fp);

      if ((CHECK_BIT(mxt->mxt_enc.encryption_state, DEV_ENCRYPTED)) && (t68_not == true)) {
        switch (width) {
          case 1:
            objcfg->data[2 + index] = (char) data & 0xFF;
            index += width;
            break;
          case 2:
            objcfg->data[2 + index] = (char) data & 0xFF;
            objcfg->data[2 + index + 1] = (char) ((data >> 8) & 0xFF);
            index += width;
            break;
          case 3:
            objcfg->data[2 + index] = (char) data & 0xFF;
            objcfg->data[2 + index + 1] = (char) ((data >> 8) & 0xFF);
            objcfg->data[2 + index + 2] = (char) ((data >> 16) & 0xFF);
            index += width;
            break;
          case 4:
            objcfg->data[2 + index] = (char) data & 0xFF;
            objcfg->data[2 + index + 1] = (char) ((data >> 8) & 0xFF);
            objcfg->data[2 + index + 2] = (char) ((data >> 16) & 0xFF);
            objcfg->data[2 + index + 2] = (char) ((data >> 24) & 0xFF);
            index += width;
            break;
          default:
            mxt_err(mxt->ctx, "Only 1, 2, 3 and 4 byte config values are supported");
            ret = MXT_ERROR_FILE_FORMAT;
            goto close;
        }
      } else {
          switch (width) {
          case 1:
            objcfg->data[offset] = (char) data & 0xFF;
            break;
          case 2:
            objcfg->data[offset] = (char) data & 0xFF;
            objcfg->data[offset + 1] = (char) ((data >> 8) & 0xFF);
            break;

          case 3:
            objcfg->data[offset] = (char) data & 0xFF;
            objcfg->data[offset + 1] = (char) ((data >> 8) & 0xFF);
            objcfg->data[offset + 2]  = (char) ((data >> 16) & 0xFF);
            break;
          case 4:
            objcfg->data[offset] = (char) data & 0xFF;
            objcfg->data[offset + 1] = (char) ((data >> 8) & 0xFF);
            objcfg->data[offset + 2] = (char) ((data >> 16) & 0xFF);
            objcfg->data[offset + 3] = (char) ((data >> 24) & 0xFF);
            break;
          default:
            mxt_err(mxt->ctx, "Only 1, 2, 3 and 4 byte config values are supported");
            ret = MXT_ERROR_FILE_FORMAT;
            goto close;
          }
        }
    }
	}
  ret = MXT_SUCCESS;
  mxt_info(mxt->ctx, "Configuration read from %s in XCFG format", filename);

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Load configuration from RAW file
//  \return #mxt_rc
static int mxt_load_raw_file(struct mxt_device *mxt, const char *filename,
                             struct mxt_config *cfg)
{
  FILE *fp;
  int ret;
  size_t i;
  bool t38_found = false;
  bool t68_payload_found = false;
  char line[2048];
  struct mxt_object_config **next;
  struct mxt_object_config *objcfg;

  cfg->config_type = CONFIG_RAW;

  fp = fopen(filename, "r");
  if (fp == NULL) {
    mxt_err(mxt->ctx, "Error opening %s: %s", filename, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  if (fgets(line, sizeof(line), fp) == NULL) {
    mxt_err(mxt->ctx, "Unexpected EOF");
  }

  if (!strncmp(line, OBP_RAW_MAGIC_V1, strlen(OBP_RAW_MAGIC_V1))) {
    mxt_info(mxt->ctx, "Found Version 1 config file");
  } else if (!strncmp(line, OBP_RAW_MAGIC_V2, strlen(OBP_RAW_MAGIC_V2))) {
    mxt_info(mxt->ctx, "Found Version 2 config file");
  } else if (!strncmp(line, OBP_RAW_MAGIC_V3, strlen(OBP_RAW_MAGIC_V3))) {
    mxt_info(mxt->ctx, "Found Version 3 config file");
    cfg->cfg_version = 0x03;
  } else {
    ret = MXT_ERROR_FILE_FORMAT;
    mxt_dbg(mxt->ctx, "Not in OBP_RAW format");
    goto close;
  }

  mxt_dbg(mxt->ctx, "Loading OBP_RAW file");

  if (cfg->cfg_version == 0x03) {

    ret = (fscanf(fp, "%s%d", line, &cfg->cfg_enc));
    if (ret != 2) {
      mxt_err(mxt->ctx, "Bad format, ENCRYPTION header not found\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (cfg->cfg_enc) {
      if (CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) {
        mxt_info(mxt->ctx, "Config and device are encrypted."
        " Okay to update chip configuration.");
      } else {
        mxt_info(mxt->ctx, "Cannot program encrypted cfg into unencrypted device");
        ret = MXT_ERROR_NOT_SUPPORTED;
        goto close;
      }
    } else {
      mxt_info(mxt->ctx, "Config file is unencrypted. Checking device encryption");

      if (CHECK_BIT(mxt->mxt_enc.encryption_state, CFG_ENCRYPTED)) {
        mxt_err(mxt->ctx, "Cannot program unencrypted cfg into"
          "encrypted device");
        ret = MXT_ERROR_NOT_SUPPORTED;
        goto close;
      } else {
        mxt_info(mxt->ctx, "Device config is unencrypted. Okay to write new config");
      }
    }

    if (!strncmp(line, "ENC_HDR", strlen(ENC_HDR))) {
      mxt_err(mxt->ctx, "Unexpected header found %s", line);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    ret = (fscanf(fp, "%s%d", line, &cfg->cfg_blksize));
    if (ret != 2) {
      mxt_err(mxt->ctx, "Bad format, ENCRYPTION header not found\n");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if (!strncmp(line, "ENC_BLOCK_HDR",
      strlen(ENC_BLOCK_HDR))) {
      mxt_err(mxt->ctx, "Unexpected header found %s", line);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }
  }

  /* Load information block and check */
  for (i = 0; i < sizeof(struct mxt_id_info); i++) {
    ret = fscanf(fp, "%hhx", (unsigned char *)&cfg->id + i);
    if (ret != 1) {
      mxt_err(mxt->ctx, "Bad format");
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }
  }

  /* Read CRCs */
  ret = fscanf(fp, "%x", &cfg->info_crc);
  if (ret != 1) {
    mxt_err(mxt->ctx, "Bad format: failed to parse Info CRC");
    ret = MXT_ERROR_FILE_FORMAT;
    goto close;
  }

  ret = fscanf(fp, "%x", &cfg->config_crc);
  if (ret != 1) {
    mxt_err(mxt->ctx, "Bad format: failed to parse Config CRC");
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

    /* Read type,instance,length - Use SCNx16 for T68 payload */
    ret = fscanf(fp, "%x %" SCNx16 " %x",
                 &objcfg->type, &objcfg->instance, &objcfg->size);
    if (ret == EOF) {
      free(objcfg);
      break;
    } else if (ret != 3) {
      mxt_err(mxt->ctx, "Bad format: failed to parse object");
      free(objcfg);
      ret = MXT_ERROR_FILE_FORMAT;
      goto close;
    }

    if ((objcfg->type == 68) && (objcfg->instance & 0x8000)) {
      mxt_info(mxt->ctx, "T68 payload found\n");
      t68_payload_found = true;
    }

    if (cfg->cfg_version = 0x03) {
    /* Initialize objcfg->enc_size */
      objcfg->enc_size = objcfg->size;
      mxt->mxt_enc.enc_datasize = objcfg->size;

      if (objcfg->type == 38) {
        t38_found = true;
      }    

      /* Required for raw, data must be in multiples of 16 for enc */
      /* Object size requires padding */

      if (t38_found && (t68_payload_found != true)) {
        if (cfg->cfg_enc) {

          if (!((objcfg->enc_size % 16 == 0x00) && (objcfg->enc_size != 0x00))) {
            objcfg->size = (16 - (objcfg->enc_size % 16) + 
              objcfg->enc_size);
          }
        }
      }
    } /* End of cfg->version */

    mxt_dbg(mxt->ctx, "OBP_RAW T%u instance %u", objcfg->type, objcfg->instance);

    *next = objcfg;
    next = &objcfg->next;

    /*TBD need to allocate addition 2 bytes for encryption only */
    /* Malloc memory to store configuration and two byte datasize */
    objcfg->data = calloc(objcfg->size + 2, sizeof(uint8_t));
    if (!objcfg->data) {
      mxt_err(mxt->ctx, "Failed to allocate memory");
      ret = MXT_ERROR_NO_MEM;
      goto close;
    }

    if (cfg->cfg_enc && (t68_payload_found != true)) {
      objcfg->data[0] = objcfg->enc_size & 0x00ff;
      objcfg->data[1] = (objcfg->enc_size >> 8) & 0x00ff;
    }

    /* Read bytes from file, load into data buffer*/
    for (i = 0; i < objcfg->size; i++) {
      uint8_t val;
      ret = fscanf(fp, "%hhx", &val);
      if (ret != 1) {
        mxt_err(mxt->ctx, "Parse error in T%d", objcfg->type);
        ret = MXT_ERROR_FILE_FORMAT;
        free(objcfg->data);
        goto close;
      }

      if (cfg->cfg_enc && (t68_payload_found != true)) {
        objcfg->data[2 + i] = val;
      } else {
        objcfg->data[i] = val;
      }
     
    }

    t68_payload_found = false;

    mxt_log_buffer(mxt->ctx, LOG_DEBUG, "CFG:", objcfg->data, objcfg->size);
  } //End of while {true} loop

  ret = MXT_SUCCESS;
  mxt_info(mxt->ctx, "Config loaded from %s in raw format", filename);

close:
  fclose(fp);
  return ret;
}

//******************************************************************************
/// \brief  Get the configuration from file
/// \return #mxt_rc
static int mxt_get_config_from_file(struct mxt_device *mxt,
          const char *filename, struct mxt_config *cfg)
{
  int ret;
  char *extension = strrchr(filename, '.');

  if (cfg) {
    if (extension && !strcmp(extension, ".xcfg")) {
      ret = mxt_load_xcfg_file(mxt, filename, cfg);
      if (ret)
        return ret;
    } else {
      ret = mxt_load_raw_file(mxt, filename, cfg);
      if (ret)
        return ret;
    }
  } else {
    ret = MXT_INTERNAL_ERROR;
    mxt_err(mxt->ctx, "Config is null");
  }
  return ret;
}

//******************************************************************************
/// \brief  Load configuration from .xcfg or RAW file, automatically detect
//          format and write to device
/// \return #mxt_rc
int mxt_load_config_file(struct mxt_device *mxt, const char *filename)
{
  uint8_t backup_cmd = BACKUPNV_COMMAND;
  struct mxt_config cfg = {{0}};
  int ret;

  ret = mxt_get_config_from_file(mxt, filename, &cfg);

  if (ret == MXT_SUCCESS) {
    mxt->mxt_enc.enc_cfg_write = true;
    ret = mxt_write_device_config(mxt, &cfg);
    mxt_free_config(&cfg);
  } else {
    mxt_dbg(mxt->ctx, "Error loading file content\n");
    mxt->mxt_enc.enc_cfg_write = false;
    goto load_config;
  }

  mxt->mxt_enc.enc_cfg_write = false;

  ret = mxt_backup_config(mxt, backup_cmd);
  
  if (ret) {
    mxt_err(mxt->ctx, "Error backing up");
  } else {
      mxt_info(mxt->ctx, "Configuration backed up");

    if (mxt->mxt_enc.msg_enc_enabled) {
      ret = mxt_set_irq(mxt, false);
    }

    ret = mxt_reset_chip(mxt, false, 0);
      
    if (ret) {
      mxt_err(mxt->ctx, "Error resetting");
    } else {
        mxt_info(mxt->ctx, "Chip reset");
    }
  }

  ret = mxt_msg_reset(mxt);
  if (ret)
    mxt_err(mxt->ctx, "Messages did not clear. Interrupt line may be low\n" 
      "Device reset required\n");

  ret = mxt_read_info_block (mxt);
  if (ret)
    mxt_info(mxt->ctx, "Warning: Could not read infoblock");

  mxt_check_encryption(mxt);

  if (mxt->mxt_enc.msg_enc_enabled) {
    ret = mxt_set_irq(mxt, true);
    if (ret)
      mxt_dbg(mxt->ctx, "debug irq was not set\n");
  }

  ret = mxt_msg_reset(mxt);
  if (ret)
    mxt_err(mxt->ctx, "Messages did not clear. Interrupt line may be low\n" 
      "Device reset required\n");

load_config:
 //mxt_free_config(&cfg);
  return ret;
}

//******************************************************************************
/// \brief  Save configuration to file
/// \return #mxt_rc
int mxt_save_config_file(struct mxt_device *mxt, const char *filename, uint8_t format)
{
  int ret;

 if (mxt->conn->type == E_I2C_DEV)
      mxt->mxt_crc.config_triggered = true;

  char *extension = strrchr(filename, '.');
  struct mxt_config cfg = {{0}};

  ret = mxt_read_device_config(mxt, &cfg);
  if (ret)
    goto config_done;

  if (extension && !strcmp(extension, ".xcfg"))
    ret = mxt_save_xcfg_file(mxt->ctx, filename, &cfg, format);
  else
    ret = mxt_save_raw_file(mxt->ctx, filename, &cfg, format);

  mxt_free_config(&cfg);

config_done:

 if (mxt->conn->type == E_I2C_DEV)
      mxt->mxt_crc.config_triggered = false;

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
int mxt_checkcrc(struct mxt_device *mxt, char *filename)
{
  int ret;
  struct mxt_config cfg = {{0}};
  uint16_t obj_idx = 0;

  int start_pos = INT_MAX;
  uint16_t end_pos = 0;

  /* Get the mxt_config and object configurations */
  ret = mxt_get_config_from_file(mxt, filename, &cfg);
  if (ret)
    goto free;

  /* Find limits of CRC region */
  struct mxt_object_config *objcfg = cfg.head;
  if (mxt == NULL) {
    if (cfg.config_type == CONFIG_RAW) {
      mxt_err(mxt->ctx, "RAW config format only supported with chip present");
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

  mxt_info(mxt->ctx, "CRC start_pos:%d end_pos:%d", start_pos, end_pos);

  /* Allocate buffer for CRC calculation */
  uint8_t *buffer = (uint8_t *)calloc(end_pos, sizeof(uint8_t));
  if (!buffer) {
    ret = MXT_ERROR_NO_MEM;
    mxt_err(mxt->ctx, "Could not allocate memory for buffer");
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
  ret = mxt_calculate_crc(mxt->ctx, &calc_crc, buffer + start_pos, end_pos - start_pos);

  if (calc_crc == cfg.config_crc) {
    mxt_info(mxt->ctx, "File checksum verified: %06X", cfg.config_crc);
    ret = MXT_SUCCESS;
  } else {
    mxt_err(mxt->ctx, "Checksum error: calc=%06X file=%06X", calc_crc, cfg.config_crc);
    ret = MXT_ERROR_CHECKSUM_MISMATCH;
  }

  free(buffer);

free:
  mxt_free_config(&cfg);
  return ret;
}
