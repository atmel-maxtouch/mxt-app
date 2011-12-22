#pragma once
//------------------------------------------------------------------------------
/// \file   info_block_driver.h
/// \brief  Header file defining the info block datatypes & structs needed in
//          driver.
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

//   Info block type definition for driver SW. Similar to info_block.h
//   in standard firmware files, but with slight differences:
//   1) info_id fields are not const (they can't be since they are not
//      known at struct init time)
//   2) info_block struct is defined (even if the compiler is not IAR),
//      but the object table is pointer to array instead of array, since
//      the size of that array is not known before the info block is
//      read from touch chip.
//   
//   This file needs to be kept in sync with firmware info_block.h!
//   
//------------------------------------------------------------------------------

#include <stdint.h>

/*----------------------------------------------------------------------------
  type definitions
----------------------------------------------------------------------------*/

/*! \brief Object table element struct. */
typedef struct
{
   uint8_t object_type;      /*!< Object type ID. */
   uint8_t start_pos_lsbyte; /*!< LSByte of the start address of the obj config structure. */
   uint8_t start_pos_msbyte; /*!< MSByte of the start address of the obj config structure. */
   uint8_t size;             /*!< Byte length of the obj config structure -1.*/
   uint8_t instances;        /*!< Number of objects of this obj. type -1. */
   uint8_t num_report_ids;   /*!< The max number of touches in a screen,
                              *  max number of sliders in a slider array, etc.*/
} object_t;



/*! \brief Info ID struct. */
typedef struct
{
   uint8_t family_id;            /* address 0 */
   uint8_t variant_id;           /* address 1 */

   uint8_t version;              /* address 2 */
   uint8_t build;                /* address 3 */

   uint8_t matrix_x_size;        /* address 4 */
   uint8_t matrix_y_size;        /* address 5 */

   /*! Number of entries in the object table. The actual number of objects 
    * can be different if any object has more than one instance. */
   uint8_t num_declared_objects; /* address 6 */
} info_id_t;


/*! \brief Info block struct holding ID and object table data and their CRC sum.
 * 
 * Info block struct. Similar to one in info_block.h, but since 
 * the size of object table is not known beforehand, it's pointer to an
 * array instead of an array. This is not defined in info_block.h unless
 * we are compiling with IAR AVR or AVR32 compiler (__ICCAVR__ or __ICCAVR32__
 * is defined). If this driver is compiled with those compilers, the
 * info_block.h needs to be edited to not include that struct definition.
 * 
 * CRC is 24 bits, consisting of CRC and CRC_hi; CRC is the lower 16 bits and
 * CRC_hi the upper 8. 
 * 
 */

typedef struct
{
   /*! Info ID struct. */
   info_id_t info_id;

   /*! Pointer to an array of objects. */
   object_t *objects;

   /*! CRC field, low bytes. */
   uint16_t CRC;

   /*! CRC field, high byte. */
   uint8_t CRC_hi;
} info_block_t;
