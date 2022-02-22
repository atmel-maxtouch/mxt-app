//------------------------------------------------------------------------------
/// \file   info_block.c
/// \brief  Functions for accessing the mXT chip's information block.
/// \author Iiro Valkonen
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

#include "libmaxtouch.h"

/*!
 * @brief Information block checksum return function.
 * @return 24-bit checksum value in 32-bit integer.
 */
static uint32_t convert_crc(struct mxt_raw_crc *crc)
{
  return ((crc->CRC_hi<<16u) | (crc->CRC));
}

/*!
 * @brief  Checksum algorithm.
 * @return Calculated checksum.
 */
static uint32_t crc24(uint32_t crc, uint8_t firstbyte, uint8_t secondbyte)
{
  static const uint32_t CRCPOLY = 0x0080001B;
  uint32_t result;
  uint16_t data_word;

  data_word = (uint16_t) ((uint16_t)(secondbyte << 8u) | firstbyte);
  result = ((crc << 1u) ^ (uint32_t)data_word);

  /* Check if 25th bit is set, and XOR the result to create 24-bit checksum */
  if (result & 0x1000000) {
    result ^= CRCPOLY;
  }
  return result;
}

/*!
 * @brief  Calculate and verify checksum over a region of memory
 * @return #mxt_rc
 */
int mxt_calculate_crc(struct libmaxtouch_ctx *ctx, uint32_t *crc_result,
                      uint8_t *base_addr, size_t size)
{
  static const uint32_t MASK_24_BITS = 0x00FFFFFF;
  uint32_t calc_crc = 0; /* Calculated checksum */
  uint16_t crc_byte_index = 0;

  mxt_dbg(ctx, "Calculating CRC over %zd bytes", size);

  /* Call the CRC function crc24() iteratively to calculate the CRC,
   * passing it two bytes at a time.  */
  while (crc_byte_index < ((size % 2) ? (size - 1) : size)) {
    calc_crc = crc24(calc_crc, *(base_addr + crc_byte_index),
                     *(base_addr + crc_byte_index + 1));
    crc_byte_index += 2;
  }

  /* Call crc24() for the final byte, plus an extra
   *  0 value byte to make the sequence even if it's odd */
  if (size % 2) {
    calc_crc = crc24(calc_crc, *(base_addr + crc_byte_index), 0);
  }

  /* Mask 32-bit calculated checksum to 24-bit */
  calc_crc &= calc_crc & MASK_24_BITS;

  *crc_result = calc_crc;

  return MXT_SUCCESS;
}

/*!
 * @brief  Reads the information block from the chip.
 * @return #mxt_rc
 */
int mxt_read_info_block(struct mxt_device *mxt)
{
  uint32_t calc_crc;
  bool crc_flag = false;
  int ret = 0;

  /* Read the ID Information from the chip */
  uint8_t *info_blk = (uint8_t *)calloc(1, sizeof(struct mxt_id_info));

  if (info_blk == NULL) {
    mxt_err(mxt->ctx, "Memory allocation failure");
    return MXT_ERROR_NO_MEM;
  }

  if (mxt->conn->type == E_I2C_DEV) {

    ret = debugfs_scan(mxt);

    if (ret == MXT_SUCCESS) {

      ret = debugfs_open(mxt);
      if (ret)
        mxt_err(mxt->ctx, "Could not register debugfs interface");
      else
        mxt->debug_fs.enabled = true;

      ret = debugfs_get_crc_enabled(mxt, &crc_flag);
      if (ret)
        mxt_err(mxt->ctx, "Could not get crc_enabled flag");

      if (crc_flag)
        mxt->mxt_crc.crc_enabled = true;

    } else {
      mxt_dbg(mxt->ctx, "Debugfs attributes not found");
      mxt->debug_fs.enabled = false;
    }
  }

  ret = mxt_read_register(mxt, info_blk, 0, sizeof(struct mxt_id_info));
  if (ret) {
    mxt_err(mxt->ctx, "Failed to read ID information");
    return ret;
  }

  /* Determine the number of bytes for checksum calculation */
  int num_objects = ((struct mxt_id_info*) info_blk)->num_objects;

  size_t crc_area_size = sizeof(struct mxt_id_info)
                         + num_objects * sizeof(struct mxt_object);

  /* Allocate space to read Information Block AND Checksum from the chip */
  size_t info_block_size = crc_area_size + sizeof(struct mxt_raw_crc);

  info_blk = (uint8_t *)realloc(info_blk, info_block_size);
  if (info_blk == NULL) {
    mxt_err(mxt->ctx, "Memory allocation failure");
    return MXT_ERROR_NO_MEM;
  }

  /* Read the entire Information Block from the chip */
  ret = mxt_read_register(mxt, info_blk, 0, info_block_size);
  if (ret) {
    mxt_err(mxt->ctx, "Failed to read Information Block");
    return ret;
  }

  /* Update pointers in device structure */
  mxt->info.raw_info = info_blk;
  mxt->info.id = (struct mxt_id_info*) info_blk;
  mxt->info.objects = (struct mxt_object*)(info_blk + sizeof(struct mxt_id_info));

  mxt->info.crc = convert_crc((struct mxt_raw_crc*) (info_blk + crc_area_size));

  /* Calculate and compare Information Block Checksum */
  
  ret = mxt_calculate_crc(mxt->ctx, &calc_crc, info_blk, crc_area_size);
  if (ret)
    return ret;

  /* A zero CRC indicates a communications error */
  if (calc_crc == 0) {
    mxt_err(mxt->ctx, "Info checksum zero - possible comms error or zero input");
    return MXT_ERROR_IO;
  }

  /* Compare the read checksum with calculated checksum */
  if (mxt->info.crc != calc_crc) {
    mxt_err(mxt->ctx, "Info checksum error calc=%06X read=%06X",
            calc_crc, mxt->info.crc);
    return MXT_ERROR_CHECKSUM_MISMATCH;
  }

  mxt_dbg(mxt->ctx, "Info checksum verified %06X", calc_crc);
  return MXT_SUCCESS;
}

/*!
 * @brief  Populates a look-up table for the report IDs.
 * @return #mxt_rc
 */
int mxt_calc_report_ids(struct mxt_device *mxt)
{
  /* Report ID zero is reserved - start from one */
  mxt->info.max_report_id = 1;
  int report_id_count = 1;

  int i;
  int instance;
  int report_index;

  struct mxt_object obj;

  /* Calculate the number of report IDs */
  for (i = 0; i < mxt->info.id->num_objects; i++) {
    obj = mxt->info.objects[i];
    mxt->info.max_report_id += MXT_INSTANCES(obj) * obj.num_report_ids;
  }

  /* Allocate memory for report ID look-up table */
  mxt->report_id_map = calloc(mxt->info.max_report_id,
                              sizeof(struct mxt_report_id_map));
  if (mxt->report_id_map == NULL) {
    mxt_err(mxt->ctx, "calloc failure");
    return MXT_ERROR_NO_MEM;
  }

  /* Store the object and instance for each report ID */
  for (i = 0; i < mxt->info.id->num_objects; i++) {
    obj = mxt->info.objects[i];

    for (instance = 0; instance < MXT_INSTANCES(obj); instance++) {
      for (report_index = 0; report_index < obj.num_report_ids; report_index++) {
        mxt->report_id_map[report_id_count].object_type = obj.type;
        mxt->report_id_map[report_id_count].instance = instance;
        report_id_count++;
      }
    }
  }

  mxt_verb(mxt->ctx, "Created a look-up table of %d Report IDs", report_id_count);

  return MXT_SUCCESS;
}

/*!
 * @brief  Outputs firmware version as formatted string
 * @return #mxt_rc
 */
int mxt_get_firmware_version(struct mxt_device *mxt, char *version_str)
{
  if (mxt->info.id == NULL)
    return MXT_ERROR_NO_DEVICE;

  snprintf(version_str, MXT_FW_VER_LEN, "%u.%u.%02X",
           (mxt->info.id->version & 0xF0) >> 4,
           (mxt->info.id->version & 0x0F),
           mxt->info.id->build);

  return MXT_SUCCESS;
}

/*!
 * @brief  Logs information about the chip.
 */
void mxt_display_chip_info(struct mxt_device *mxt)
{
  struct mxt_object obj;
  char firmware_version[MXT_FW_VER_LEN];
  struct mxt_id_info *id = mxt->info.id;
  uint16_t t144_addr = 0x0000;
  int i;

  mxt_get_firmware_version(mxt, (char *)&firmware_version);

  /* Display ID information */
  mxt_dbg(mxt->ctx, "Family ID = %u (0x%02X)",
          id->family, id->family);
  mxt_dbg(mxt->ctx, "Variant ID = %u (0x%02X)",
          id->variant, id->variant);
  mxt_dbg(mxt->ctx, "Firmware Version = %s", firmware_version);
  mxt_dbg(mxt->ctx, "Matrix X Size = %d", id->matrix_x_size);
  mxt_dbg(mxt->ctx, "Matrix Y Size = %d", id->matrix_y_size);
  mxt_dbg(mxt->ctx, "Number of elements in the Object Table = %d",
          id->num_objects);

  /* Display information about specific objects */
  for (i = 0; i < id->num_objects; i++) {
    obj = mxt->info.objects[i];

    mxt_dbg(mxt->ctx, "T%u size:%u instances:%u address:%u",
            obj.type, MXT_SIZE(obj),
            MXT_INSTANCES(obj), mxt_get_start_position(obj, 0));
  }

    t144_addr = mxt_get_object_address(mxt, SPT_MESSAGECOUNT_T144, 0);
    
    if (t144_addr == OBJECT_NOT_FOUND) {
      mxt->mxt_crc.crc_enabled = false;
      mxt_dbg(mxt->ctx, "T144 Object not Found\n");
    } else{
      mxt->mxt_crc.crc_enabled = true;
    }
}

/*!
 * @param  mxt Maxtouch Device
 * @param  object_type Object ID number.
 * @param  instance Instance number of the object.
 *
 * @brief  Returns the start address of the selected object and instance number
 *         in the chip's memory map.
 * @return Object address, or OBJECT_NOT_FOUND if object/instance not found.
 */
uint16_t mxt_get_object_address(struct mxt_device *mxt, uint16_t object_type, uint8_t instance)
{
  struct mxt_id_info *id = mxt->info.id;
  int i = 0;
  struct mxt_object obj;

  for (i = 0; i < id->num_objects; i++) {
    obj = mxt->info.objects[i];

    /* Does object type match? */
    if (obj.type == object_type) {
      /* Are there enough instances defined in the firmware? */
      if (obj.instances_minus_one >= instance) {
        return mxt_get_start_position(obj, instance);
      } else {
        mxt_warn(mxt->ctx, "T%u instance %u not present on device",
                 object_type, instance);
        return OBJECT_NOT_FOUND;
      }
    }
  }

  mxt_verb(mxt->ctx, "T%u not present on device", object_type);
  return OBJECT_NOT_FOUND;
}

/*!
 * @param mxt Maxtouch Device
 * @param object_type Object ID number.
 *
 * @brief  Returns the size of the specified type in the object table.
 * @return Object size, or OBJECT_NOT_FOUND if object not found.
 */
uint8_t mxt_get_object_size(struct mxt_device *mxt, uint16_t object_type)
{
  int i = mxt_get_object_table_num(mxt, object_type);
  if (i == 255) {
    return OBJECT_NOT_FOUND;
  }

  return MXT_SIZE(mxt->info.objects[i]);
}

/*!
 * @param mxt Maxtouch Device
 * @param object_type Object ID number.
 *
 * @brief  Returns the number of instances of the specific object type
 * @return number of instances, zero if not found
 */
uint8_t mxt_get_object_instances(struct mxt_device *mxt, uint16_t object_type)
{
  struct mxt_id_info *id = mxt->info.id;
  int i;

  for (i = 0; i < id->num_objects; i++) {
    if (mxt->info.objects[i].type == object_type) {
      return MXT_INSTANCES(mxt->info.objects[i]);
    }
  }

  return 0;
}

/*!
 * @param mxt Maxtouch Device
 * @param object_type Object ID number.
 *
 * @brief  Returns the index of the specified type in the object table.
 * @return Element index, or 255 if object type not found.
 */
uint8_t mxt_get_object_table_num(struct mxt_device *mxt, uint16_t object_type)
{
  struct mxt_id_info *id = mxt->info.id;
  int i;

  for (i = 0; i < id->num_objects; i++) {
    if (mxt->info.objects[i].type == object_type) {
      return i;
    }
  }

  mxt_warn(mxt->ctx, "Could not find object type T%u in object table", object_type);
  return 255;
}

/*!
 * @param obj Object table element.
 * @param instance Object instance index.
 *
 * @brief  Returns the start position for the specified object element by
 *         combining the least significant and most significant bytes.
 * @return Start position as a single value.
 */
uint16_t mxt_get_start_position(struct mxt_object obj, uint8_t instance)
{
  return (obj.start_pos_msb * 256) + obj.start_pos_lsb
         + (MXT_SIZE(obj) * instance);
}

/*!
 * @brief  Look up object type from report ID
 * @param  mxt Maxtouch Device
 * @param  report_id Report ID
 * @return Object type number, or OBJECT_NOT_FOUND
 */
uint16_t mxt_report_id_to_type(struct mxt_device *mxt, int report_id)
{
  if (report_id > mxt->info.max_report_id)
    return OBJECT_NOT_FOUND;

  return (mxt->report_id_map[report_id].object_type);
}
