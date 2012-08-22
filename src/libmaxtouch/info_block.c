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

#include "log.h"
#include "libmaxtouch.h"

#include "info_block.h"

/*! Information Block structure. */
info_block_t info_block;

/*! Pointer to report ID look-up table. */
static report_id_map_t *report_id_map;

/*! Command processor start position. */
uint16_t command_processor_address;

/*!
 * @brief Information block checksum return function.
 * @return 24-bit checksum value in 32-bit integer.
 */
uint32_t info_block_crc(crc_t * crc)
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
static int info_block_checksum(uint32_t read_crc, uint8_t *idb_base_addr,
                               uint16_t crc_area_size)
{
  uint32_t calc_crc = 0; /* Checksum calculated by the driver code */
  uint16_t crc_byte_index = 0;

  /* Call the CRC function crc24() iteratively to calculate the CRC,
   * passing it two characters at a time.
   */

  while (crc_byte_index < ((crc_area_size % 2) ? (crc_area_size - 1) : crc_area_size))
  {
    calc_crc = crc24(calc_crc, *(idb_base_addr + crc_byte_index), *(idb_base_addr + crc_byte_index + 1));
    crc_byte_index += 2;
  }

  /* Call crc24() for the final byte, plus an extra
   *  0 value byte to make the sequence even if it's odd
   */
  if (crc_area_size % 2)
  {
    calc_crc = crc24(calc_crc, *(idb_base_addr + crc_byte_index), 0);
  }

  calc_crc &= calc_crc & 0x00FFFFFF; /* Mask 32-bit calculated checksum to 24-bit */

  /* Compare the read checksum with calculated checksum */
  if (read_crc != calc_crc)
  {
    return -1;
  }

  return 0;
}

/*!
 * @brief  Reads the information block from the chip.
 * @return Zero on success, negative for error.
 */
int read_information_block()
{
  int ret = -1;

  int memory_offset = 0;

  /*
   * Offset to the member of ID Information block containing
   * information about number of objects in object table
   */
  const uint8_t ID_OBJECTS_OFFSET = 6;

  uint8_t no_of_bytes;
  const uint8_t NUM_ID_BYTES = 7; /* Number of bytes of information in ID block */
  uint8_t num_declared_objects;

  static const int CRC_LENGTH = 3;
  uint32_t read_crc = 0; /* Checksum value as read from the chip */
  uint16_t crc_area_size; /* Size of data for CRC calculation */
  uint8_t *idb_base_addr;

  unsigned char *info_block_shadow;

  /* Read "num_declared_objects" field of ID Information block */
  ret = mxt_read_register((unsigned char *)&num_declared_objects, ID_OBJECTS_OFFSET, 1);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read information about number of objects");
    return -1;
  }

  LOG(LOG_VERBOSE, "Successfully read and stored the ID Information");

  /* Determine the number of data bytes in Information Block for checksum calculation */
  crc_area_size = NUM_ID_BYTES + num_declared_objects * sizeof(object_t);

  /* Allocate space to read Information Block AND Checksum from the chip */
  no_of_bytes = crc_area_size + CRC_LENGTH;

  info_block_shadow = (unsigned char *) malloc(no_of_bytes);
  if (info_block_shadow == NULL)
  {
    LOG(LOG_ERROR, "Failed to allocate %d bytes for the Information Block data", no_of_bytes);
    return -1;
  }
  LOG(LOG_VERBOSE, "Allocated %d bytes to store Information Block data", no_of_bytes);

  /* Read the Information Block from the chip */
  ret = mxt_read_register(info_block_shadow, memory_offset, no_of_bytes);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read the Information Block");
    return -1;
  }

  LOG(LOG_VERBOSE, "Successfully read and stored the Information Block data");

  /*
   * id, objects, and crc pointers should be pointing
   * to the memory areas storing these values from the chip
   */
  info_block.id = (info_id_t *) info_block_shadow;
  info_block.objects = (object_t *) ((uint8_t *)(info_block_shadow) + NUM_ID_BYTES);
  info_block.crc = (crc_t *) ((uint8_t *)(info_block_shadow) + crc_area_size);

  /* Read CRC for checksum comparision */
  read_crc = ((uint32_t)info_block.crc->CRC);
  read_crc += ((uint32_t)info_block.crc->CRC_hi << 16u);

  /* assign byte pointer the base address of Information Block */
  idb_base_addr = (uint8_t *)info_block.id;

  /* Calculate and compare Information Block Checksum */
  ret = info_block_checksum(read_crc, idb_base_addr, crc_area_size);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Information Block Checksum mismatch");
    return -1;
  }

  LOG(LOG_VERBOSE, "Successfully matched and stored the Information Block Checksum");

  return 0;

}

/*!
 * @brief  Populates a look-up table for the report IDs.
 * @return Zero on success, negative for error.
 */
int calc_report_ids()
{
  /* Report ID zero is reserved - start from one */
  int num_report_ids = 1;
  int report_id_count = 1;

  int element_index;
  int instance_index;
  int report_index;
  int no_of_bytes;

  object_t element;

  /* Calculate the number of report IDs */
  for (element_index = 0; element_index < info_block.id->num_declared_objects; element_index++)
  {
    element = info_block.objects[element_index];
    num_report_ids += (element.instances + 1) * element.num_report_ids;
  }

  /* Allocate memory for report ID look-up table */
  no_of_bytes = num_report_ids * sizeof(report_id_map_t);
  report_id_map = malloc(no_of_bytes);
  if (report_id_map == NULL)
  {
    LOG(LOG_ERROR, "Failed to allocate %d bytes for the Report ID LUT", no_of_bytes);
    return -1;
  }
  LOG(LOG_VERBOSE, "Allocated %d bytes to store the Report ID LUT", no_of_bytes);

  /* Store the object and instance for each report ID */
  for (element_index = 0; element_index < info_block.id->num_declared_objects; element_index++)
  {
    element = info_block.objects[element_index];

    for (instance_index = 0; instance_index < (element.instances + 1); instance_index++)
    {
      for (report_index = 0; report_index < element.num_report_ids; report_index++)
      {
        report_id_map[report_id_count].object_type = element.object_type;
        report_id_map[report_id_count].instance = instance_index;
        report_id_count++;
      }
    }
  }

  LOG(LOG_VERBOSE, "Created a look-up table of %d Report IDs", report_id_count);

  return 0;
}

int get_firmware_build()
{
  if (info_block.id == NULL)
    return -1;

  return info_block.id->build;
}

/*!
 * @brief  Logs information about the chip.
 */
void display_chip_info()
{
  object_t element;
  int element_index;
  int no_of_touch_instances = 0;

  /* Display ID information */
  LOG(LOG_INFO, "Family ID = 0x%02X", info_block.id->family_id);
  LOG(LOG_INFO, "Variant ID = 0x%02X", info_block.id->variant_id);
  LOG(LOG_INFO, "Version = %d.%d", (info_block.id->version & 0xF0) >> 4, (info_block.id->version & 0x0F));
  LOG(LOG_INFO, "Build = 0x%02X", info_block.id->build);
  LOG(LOG_INFO, "Matrix X Size = %d", info_block.id->matrix_x_size);
  LOG(LOG_INFO, "Matrix Y Size = %d", info_block.id->matrix_y_size);
  LOG(LOG_INFO, "Number of elements in the Object Table = %d", info_block.id->num_declared_objects);

  /* Display information about specific objects */
  for (element_index = 0; element_index < info_block.id->num_declared_objects; element_index++)
  {
    element = info_block.objects[element_index];

    LOG(LOG_INFO, "T%u size:%u instances:%u address:%u",
      element.object_type, element.size + 1,
      element.instances + 1, get_start_position(element));

    switch (element.object_type)
    {
      case GEN_MESSAGEPROCESSOR_T5:
      {
        LOG(LOG_INFO, "Message processor address = %u", get_start_position(element));
        LOG(LOG_INFO, "Maximum message length = %d", element.size + 1);
        break;
      }
      case GEN_COMMANDPROCESSOR_T6:
      {
        command_processor_address = get_start_position(element);
        LOG(LOG_INFO, "Command processor address = %u", command_processor_address);
        break;
      }
      case TOUCH_MULTITOUCHSCREEN_T9:
      case TOUCH_SINGLETOUCHSCREEN_T10:
      case TOUCH_XSLIDER_T11:
      case TOUCH_YSLIDER_T12:
      case TOUCH_XWHEEL_T13:
      case TOUCH_YWHEEL_T14:
      case TOUCH_KEYARRAY_T15:
      case TOUCH_PROXIMITY_T23:
      case TOUCH_KEYSET_T31:
      case TOUCH_XSLIDERSET_T32:
      {
        no_of_touch_instances += element.instances + 1;
        break;
      }
      default:
        /* Do nothing */
        break;
    }
  }

  LOG(LOG_INFO, "Instances of touch objects = %d", no_of_touch_instances);
}

/*!
 * @param  object_type Object ID number.
 * @param  instance Instance number of the object.
 *
 * @brief  Returns the start address of the selected object and instance number
 *         in the chip's memory map.
 * @return Object address, or OBJECT_NOT_FOUND if object/instance not found.
 */
uint16_t get_object_address(uint8_t object_type, uint8_t instance)
{
  int element_index = 0;
  object_t element;

  for (element_index = 0; element_index < info_block.id->num_declared_objects; element_index++)
  {
    element = info_block.objects[element_index];

    /* Does object type match? */
    if (element.object_type == object_type)
    {
      /* Are there enough instances defined in the firmware? */
      if (element.instances >= instance)
      {
        return get_start_position(element) + ((element.size + 1) * instance);
      }
      else
      {
        LOG(LOG_WARN, "Warning: T%u instance %u does not exist",
                      object_type, instance);
        return OBJECT_NOT_FOUND;
      }
    }
  }

  LOG(LOG_WARN, "Warning: T%u does not exist", object_type);
  return OBJECT_NOT_FOUND;
}

/*!
 * @param object_type Object ID number.
 *
 * @brief  Returns the size of the specified type in the object table.
 * @return Object size, or OBJECT_NOT_FOUND if object not found.
 */
uint8_t get_object_size(uint8_t object_type)
{
  int element_index = get_object_table_num(object_type);
  if (element_index == 255)
  {
    return OBJECT_NOT_FOUND;
  }

  return info_block.objects[element_index].size + 1;
}

/*!
 * @param object_type Object ID number.
 *
 * @brief  Returns the index of the specified type in the object table.
 * @return Element index, or 255 if object type not found.
 */
uint8_t get_object_table_num(uint8_t object_type)
{
  int element_index;

  for (element_index = 0; element_index < info_block.id->num_declared_objects; element_index++)
  {
    if (info_block.objects[element_index].object_type == object_type)
    {
      return element_index;
    }
  }

  LOG(LOG_ERROR, "Specified object type not found by %s()", __func__);
  return 255;
}

/*!
 * @param element Object table element.
 *
 * @brief  Returns the start position for the specified object element by
 *         combining the least significant and most significant bytes.
 * @return Start position as a single value.
 */
uint16_t get_start_position(object_t element)
{
  return (element.start_pos_msbyte * 256) + element.start_pos_lsbyte;
}

/*!
 * @brief  Look up object type from report ID
 * @param  report_id Report ID
 * @return Object type number
 */
uint16_t report_id_to_type(int report_id)
{
  return (report_id_map[report_id].object_type);
}
