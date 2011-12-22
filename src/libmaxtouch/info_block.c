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

/*! Pointer to Information Block structure. */
info_block_t *info_block;

/*! Pointer to report ID look-up table. */
static report_id_map_t *report_id_map;

/*! Command processor start position. */
uint16_t command_processor_address;

/*! Pointer to info block structure. */
info_id_t *id;

/*! Pointer to object table structure. */
object_t *object_table;

/*!
 * @brief  Reads the information block from the chip.
 * @return Zero on success, negative for error.
 */
int read_information_block()
{
  int ret = -1;

  int memory_offset = 0;
  int no_of_bytes;

  static const int crc_length = 3;
  uint8_t crc[crc_length];

  /* Allocate space to read object table */
  no_of_bytes = sizeof(info_id_t);
  id = (info_id_t *) malloc(no_of_bytes);
  if (id == NULL)
  {
    LOG(LOG_ERROR, "Failed to allocate %d bytes to store the ID Information", no_of_bytes);
    return -1;
  }
  LOG(LOG_DEBUG, "Allocated %d bytes to store the ID Information", no_of_bytes);

  /* Read the ID information fields */
  ret = mxt_read_register((unsigned char *)id, memory_offset, no_of_bytes);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read ID Information");
    return -1;
  }
  memory_offset += no_of_bytes;

  LOG(LOG_VERBOSE, "Successfully read and stored the ID Information");

  /* Allocate space to read object table */
  no_of_bytes = id->num_declared_objects * sizeof(object_t);
  object_table = (object_t *) malloc(no_of_bytes);
  if (object_table == NULL)
  {
    LOG(LOG_ERROR, "Failed to allocate %d bytes for the Object Table data", no_of_bytes);
    return -1;
  }
  LOG(LOG_DEBUG, "Allocated %d bytes to store Object Table data", no_of_bytes);

  /* Read the object table */
  ret = mxt_read_register((unsigned char *)object_table, memory_offset, no_of_bytes);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read the Object Table");
    return -1;
  }
  memory_offset += no_of_bytes;

  LOG(LOG_VERBOSE, "Successfully read and stored the Object Table data");

  /* Read CRC */
  ret = mxt_read_register((unsigned char *)crc, memory_offset, crc_length);
  if (ret < 0)
  {
    LOG(LOG_ERROR, "Failed to read the Information Block Checksum");
    return -1;
  }

  LOG(LOG_VERBOSE, "Successfully read and stored the Information Block Checksum");

  /* Assign pointers to parent structure */
  no_of_bytes = sizeof(info_block_t);
  info_block = malloc(no_of_bytes);
  if (info_block == NULL)
  {
    LOG(LOG_ERROR, "Failed to allocate %d bytes to store the Information Block structure", no_of_bytes);
    return -1;
  }
  LOG(LOG_DEBUG, "Allocated %d bytes to store the Information Block structure", no_of_bytes);

  info_block->info_id = *id;
  info_block->objects = object_table;
  info_block->CRC = (crc[1] * 256) + crc[0];
  info_block->CRC_hi = crc[2];

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
  for (element_index = 0; element_index < id->num_declared_objects; element_index++)
  {
    element = object_table[element_index];
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
  LOG(LOG_DEBUG, "Allocated %d bytes to store the Report ID LUT", no_of_bytes);

  /* Store the object and instance for each report ID */
  for (element_index = 0; element_index < id->num_declared_objects; element_index++)
  {
    element = object_table[element_index];

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
  if (id == NULL)
    return -1;

  return id->build;
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
  LOG(LOG_INFO, "Family ID = 0x%02X", id->family_id);
  LOG(LOG_INFO, "Variant ID = 0x%02X", id->variant_id);
  LOG(LOG_INFO, "Version = %d.%d", (id->version & 0xF0) >> 4, (id->version & 0x0F));
  LOG(LOG_INFO, "Build = 0x%02X", id->build);
  LOG(LOG_INFO, "Matrix X Size = %d", id->matrix_x_size);
  LOG(LOG_INFO, "Matrix Y Size = %d", id->matrix_y_size);
  LOG(LOG_INFO, "Number of elements in the Object Table = %d", id->num_declared_objects);

  /* Display information about specific objects */
  for (element_index = 0; element_index < id->num_declared_objects; element_index++)
  {
    element = object_table[element_index];

    LOG(LOG_INFO, "T%u size:%u instances:%u address:%u",
      element.object_type, element.size,
      element.instances, get_start_position(element));

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

  for (element_index = 0; element_index < id->num_declared_objects; element_index++)
  {
    element = object_table[element_index];

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
        LOG(LOG_ERROR, "Specified instance not found by %s()", __func__);
        return OBJECT_NOT_FOUND;
      }
    }
  }

  LOG(LOG_ERROR, "Specified object type not found by %s()", __func__);
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

  return object_table[element_index].size + 1;
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

  for (element_index = 0; element_index < id->num_declared_objects; element_index++)
  {
    if (object_table[element_index].object_type == object_type)
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

