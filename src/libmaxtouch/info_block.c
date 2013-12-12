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
uint32_t info_block_crc(struct raw_crc * crc)
{
  return ((crc->CRC_hi<<16u) | (crc->CRC));
}

/*!
 * @brief  Information Block Checksum algorithm.
 * @return Calculated Information Block Checksum.
 */
static uint32_t crc24(uint32_t crc, uint8_t firstbyte, uint8_t secondbyte)
{
  static const uint32_t CRCPOLY = 0x0080001B;
  uint32_t result;
  uint16_t data_word;

  data_word = (uint16_t) ((uint16_t)(secondbyte << 8u) | firstbyte);
  result = ((crc << 1u) ^ (uint32_t)data_word);

  /* Check if 25th bit is set, and XOR the result to create 24-bit checksum */
  if (result & 0x1000000)
  {
    result ^= CRCPOLY;
  }
  return result;
}

/*!
 * @brief  Calculates and reports the Information Block Checksum.
 * @return Zero on success, negative for error.
 */
static int info_block_checksum(struct mxt_device *mxt, uint32_t read_crc,
                               uint8_t *base_addr, size_t size)
{
  uint32_t calc_crc = 0; /* Checksum calculated by the driver code */
  uint16_t crc_byte_index = 0;

  /* Call the CRC function crc24() iteratively to calculate the CRC,
   * passing it two characters at a time.
   */

  while (crc_byte_index < ((size % 2) ? (size - 1) : size))
  {
    calc_crc = crc24(calc_crc, *(base_addr + crc_byte_index), *(base_addr + crc_byte_index + 1));
    crc_byte_index += 2;
  }

  /* Call crc24() for the final byte, plus an extra
   *  0 value byte to make the sequence even if it's odd
   */
  if (size % 2)
  {
    calc_crc = crc24(calc_crc, *(base_addr + crc_byte_index), 0);
  }

  calc_crc &= calc_crc & 0x00FFFFFF; /* Mask 32-bit calculated checksum to 24-bit */

  /* Compare the read checksum with calculated checksum */
  if (calc_crc == 0)
  {
    mxt_err(mxt->ctx, "Information Block Checksum zero");
    return -1;
  }

  if (read_crc != calc_crc)
  {
    mxt_err(mxt->ctx, "Information Block Checksum error calc=%06X read=%06X",
        calc_crc, read_crc);
    return -1;
  }

  mxt_dbg(mxt->ctx, "Information Block Checksum verified %06X", calc_crc);
  return 0;
}

/*!
 * @brief  Reads the information block from the chip.
 * @return Zero on success, negative for error.
 */
int read_information_block(struct mxt_device *mxt)
{
  int ret = -1;

  int memory_offset = 0;

  const uint8_t ID_OBJECTS_OFFSET = 6; /* Offset to number of objects */
  const uint8_t NUM_ID_BYTES = 7; /* Number of bytes of information in ID block */
  static const int CRC_LENGTH = 3; /* Number of CRC bytes */

  size_t info_block_size;
  size_t crc_area_size; /* Size of data for CRC calculation */
  uint8_t num_declared_objects;
  uint32_t read_crc = 0; /* Checksum value as read from the chip */
  uint8_t *info_block_buffer;

  /* Read "num_declared_objects" field of ID Information block */
  ret = mxt_read_register(mxt, (unsigned char *)&num_declared_objects, ID_OBJECTS_OFFSET, 1);
  if (ret < 0)
  {
    mxt_err(mxt->ctx, "Failed to read information about number of objects");
    return -1;
  }

  /* Determine the number of data bytes in Information Block for checksum calculation */
  crc_area_size = NUM_ID_BYTES + num_declared_objects * sizeof(struct object);

  /* Allocate space to read Information Block AND Checksum from the chip */
  info_block_size = crc_area_size + CRC_LENGTH;

  info_block_buffer = (unsigned char *)calloc(info_block_size, sizeof(unsigned char));
  if (info_block_buffer == NULL)
  {
    mxt_err(mxt->ctx, "Failed to allocate %zu bytes for the Information Block data",
        info_block_size);
    return -1;
  }
  mxt_verb(mxt->ctx, "Allocated %zu bytes to store Information Block data",
      info_block_size);

  /* Read the Information Block from the chip */
  ret = mxt_read_register(mxt, info_block_buffer, memory_offset, info_block_size);
  if (ret < 0)
  {
    mxt_err(mxt->ctx, "Failed to read the Information Block");
    return -1;
  }

  /*
   * id, objects, and crc pointers should be pointing
   * to the memory areas storing these values from the chip
   */
  mxt->info_block.id = (struct info_id*) info_block_buffer;
  mxt->info_block.objects = (struct object *) (info_block_buffer + NUM_ID_BYTES);
  mxt->info_block.crc = (struct raw_crc*) (info_block_buffer + crc_area_size);
  mxt->raw_info = info_block_buffer;

  /* Read CRC for checksum comparision */
  read_crc = info_block_crc(mxt->info_block.crc);

  /* Calculate and compare Information Block Checksum */
  ret = info_block_checksum(mxt, read_crc, info_block_buffer, crc_area_size);
  if (ret < 0)
    return ret;

  mxt_verb(mxt->ctx, "Information Block read successfully");

  return 0;
}

/*!
 * @brief  Populates a look-up table for the report IDs.
 * @return Zero on success, negative for error.
 */
int calc_report_ids(struct mxt_device *mxt)
{
  /* Report ID zero is reserved - start from one */
  int num_report_ids = 1;
  int report_id_count = 1;

  int element_index;
  int instance_index;
  int report_index;

  struct object element;

  /* Calculate the number of report IDs */
  for (element_index = 0; element_index < mxt->info_block.id->num_declared_objects; element_index++)
  {
    element = mxt->info_block.objects[element_index];
    num_report_ids += (element.instances + 1) * element.num_report_ids;
  }

  /* Allocate memory for report ID look-up table */
  mxt->report_id_map = calloc(num_report_ids, sizeof(struct report_id_map));
  if (mxt->report_id_map == NULL)
  {
    mxt_err(mxt->ctx, "calloc failure");
    return -1;
  }

  /* Store the object and instance for each report ID */
  for (element_index = 0; element_index < mxt->info_block.id->num_declared_objects; element_index++)
  {
    element = mxt->info_block.objects[element_index];

    for (instance_index = 0; instance_index < (element.instances + 1); instance_index++)
    {
      for (report_index = 0; report_index < element.num_report_ids; report_index++)
      {
        mxt->report_id_map[report_id_count].object_type = element.object_type;
        mxt->report_id_map[report_id_count].instance = instance_index;
        report_id_count++;
      }
    }
  }

  mxt_verb(mxt->ctx, "Created a look-up table of %d Report IDs", report_id_count);

  return 0;
}

int mxt_get_firmware_version(struct mxt_device *mxt, char *version_str)
{
  if (mxt->info_block.id == NULL)
    return -1;

  snprintf(version_str, MXT_FW_VER_LEN, "%u.%u.%02X",
           (mxt->info_block.id->version & 0xF0) >> 4,
           (mxt->info_block.id->version & 0x0F),
           mxt->info_block.id->build);

  return 0;
}

/*!
 * @brief  Logs information about the chip.
 */
void display_chip_info(struct mxt_device *mxt)
{
  struct object element;
  int element_index;
  char firmware_version[MXT_FW_VER_LEN];

  mxt_get_firmware_version(mxt, (char *)&firmware_version);

  /* Display ID information */
  mxt_info(mxt->ctx, "Family ID = %u (0x%02X)",
      mxt->info_block.id->family_id, mxt->info_block.id->family_id);
  mxt_info(mxt->ctx, "Variant ID = %u (0x%02X)",
      mxt->info_block.id->variant_id, mxt->info_block.id->variant_id);
  mxt_info(mxt->ctx, "Firmware Version = %s", firmware_version);
  mxt_info(mxt->ctx, "Matrix X Size = %d", mxt->info_block.id->matrix_x_size);
  mxt_info(mxt->ctx, "Matrix Y Size = %d", mxt->info_block.id->matrix_y_size);
  mxt_info(mxt->ctx, "Number of elements in the Object Table = %d",
      mxt->info_block.id->num_declared_objects);

  /* Display information about specific objects */
  for (element_index = 0; element_index < mxt->info_block.id->num_declared_objects; element_index++)
  {
    element = mxt->info_block.objects[element_index];

    mxt_info(mxt->ctx, "T%u size:%u instances:%u address:%u",
      element.object_type, element.size + 1,
      element.instances + 1, get_start_position(element, 0));
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
uint16_t get_object_address(struct mxt_device *mxt, uint16_t object_type, uint8_t instance)
{
  int element_index = 0;
  struct object element;

  for (element_index = 0; element_index < mxt->info_block.id->num_declared_objects; element_index++)
  {
    element = mxt->info_block.objects[element_index];

    /* Does object type match? */
    if (element.object_type == object_type)
    {
      /* Are there enough instances defined in the firmware? */
      if (element.instances >= instance)
      {
        return get_start_position(element, instance);
      }
      else
      {
        mxt_warn(mxt->ctx, "T%u instance %u does not exist",
                      object_type, instance);
        return OBJECT_NOT_FOUND;
      }
    }
  }

  mxt_warn(mxt->ctx, "T%u does not exist", object_type);
  return OBJECT_NOT_FOUND;
}

/*!
 * @param mxt Maxtouch Device
 * @param object_type Object ID number.
 *
 * @brief  Returns the size of the specified type in the object table.
 * @return Object size, or OBJECT_NOT_FOUND if object not found.
 */
uint8_t get_object_size(struct mxt_device *mxt, uint16_t object_type)
{
  int element_index = get_object_table_num(mxt, object_type);
  if (element_index == 255)
  {
    return OBJECT_NOT_FOUND;
  }

  return mxt->info_block.objects[element_index].size + 1;
}

/*!
 * @param mxt Maxtouch Device
 * @param object_type Object ID number.
 *
 * @brief  Returns the index of the specified type in the object table.
 * @return Element index, or 255 if object type not found.
 */
uint8_t get_object_table_num(struct mxt_device *mxt, uint16_t object_type)
{
  int element_index;

  for (element_index = 0; element_index < mxt->info_block.id->num_declared_objects; element_index++)
  {
    if (mxt->info_block.objects[element_index].object_type == object_type)
    {
      return element_index;
    }
  }

  mxt_warn(mxt->ctx, "Could not find object type T%u in object table", object_type);
  return 255;
}

/*!
 * @param element Object table element.
 * @param instance Object instance.
 *
 * @brief  Returns the start position for the specified object element by
 *         combining the least significant and most significant bytes.
 * @return Start position as a single value.
 */
uint16_t get_start_position(struct object obj, uint8_t instance)
{
  return (obj.start_pos_msbyte * 256) + obj.start_pos_lsbyte
         + ((obj.size + 1) * instance);
}

/*!
 * @brief  Look up object type from report ID
 * @param  mxt Maxtouch Device
 * @param  report_id Report ID
 * @return Object type number
 */
uint16_t report_id_to_type(struct mxt_device *mxt, int report_id)
{
  return (mxt->report_id_map[report_id].object_type);
}
