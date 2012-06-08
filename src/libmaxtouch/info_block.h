#pragma once
//------------------------------------------------------------------------------
/// \file   info_block.h
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

#include <stdint.h>

#include "info_block_driver.h"

/*! Object types */
#define RESERVED_T0                               0u
#define RESERVED_T1                               1u
#define DEBUG_DELTAS_T2                           2u
#define DEBUG_REFERENCES_T3                       3u
#define DEBUG_SIGNALS_T4                          4u
#define GEN_MESSAGEPROCESSOR_T5                   5u
#define GEN_COMMANDPROCESSOR_T6                   6u
#define GEN_POWERCONFIG_T7                        7u
#define GEN_ACQUISITIONCONFIG_T8                  8u
#define TOUCH_MULTITOUCHSCREEN_T9                 9u
#define TOUCH_SINGLETOUCHSCREEN_T10               10u
#define TOUCH_XSLIDER_T11                         11u
#define TOUCH_YSLIDER_T12                         12u
#define TOUCH_XWHEEL_T13                          13u
#define TOUCH_YWHEEL_T14                          14u
#define TOUCH_KEYARRAY_T15                        15u
#define PROCG_SIGNALFILTER_T16                    16u
#define PROCI_LINEARIZATIONTABLE_T17              17u
#define SPT_COMMSCONFIG_T18                       18u
#define SPT_GPIOPWM_T19                           19u
#define PROCI_GRIPFACESUPPRESSION_T20             20u
#define RESERVED_T21                              21u
#define PROCG_NOISESUPPRESSION_T22                22u
#define TOUCH_PROXIMITY_T23                       23u
#define PROCI_ONETOUCHGESTUREPROCESSOR_T24        24u
#define SPT_SELFTEST_T25                          25u
#define DEBUG_CTERANGE_T26                        26u
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27        27u
#define SPT_CTECONFIG_T28                         28u
#define SPT_GPI_T29                               29u
#define SPT_GATE_T30                              30u
#define TOUCH_KEYSET_T31                          31u
#define TOUCH_XSLIDERSET_T32                      32u
#define RESERVED_T33                              33u
#define GEN_MESSAGEBLOCK_T34                      34u
#define SPT_GENERICDATA_T35                       35u
#define RESERVED_T36                              36u
#define DEBUG_DIAGNOSTIC_T37                      37u
#define SPT_USERDATA_T38                          38u
#define SPARE_T39                                 39u
#define PROCI_GRIPSUPPRESSION_T40                 40u
#define PROCI_PALMSUPPRESSION_T41                 41u
#define PROCI_TOUCHSUPPRESSION_T42                42u
#define SPT_DIGITIZER_T43                         43u
#define SPT_MESSAGECOUNT_T44                      44u
#define PROCI_VIRTUALKEY_T45                      45u
#define SPT_CTECONFIG_T46                         46u
#define PROCI_STYLUS_T47                          47u
#define PROCG_NOISESUPPRESSION_T48                48u
#define GEN_DUALPULSE_T49                         49u
#define SPARE_T50                                 50u
#define SPT_SONY_CUSTOM_T51                       51u
#define TOUCH_PROXKEY_T52                         52u
#define GEN_DATASOURCE_T53                        53u
#define PROCG_NOISESUPPRESSION_T54                54u
#define PROCI_ADAPTIVETHRESHOLD_T55               55u
#define PROCI_SHIELDLESS_T56                      56u
#define PROCI_EXTRATOUCHSCREENDATA_T57            57u
#define SPT_EXTRANOISESUPCTRLS_T58                58u
#define SPT_FASTDRIFT_T59                         59u
#define SPT_TIMER_T61                             61u
#define PROCG_NOISESUPPRESSION_T62                62u
#define PROCI_ACTIVESTYLUS_T63                    63u
#define SPT_REFERENCERELOAD_T64                   64u
#define PROCI_LENSBENDING_T65                     65u
#define SPT_GOLDENREFERENCES_T66                  66u
#define PROCI_CUSTOMGESTUREPROCESSOR_T67          67u
#define SERIAL_DATA_COMMAND_T68                   68u
#define RESERVED_T255                             255u

/*! Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0u

/* Function prototypes */
int read_information_block(void);
int calc_report_ids(void);
void display_chip_info(void);
uint16_t get_object_address(uint8_t object_type, uint8_t instance);
uint8_t get_object_size(uint8_t object_type);
uint8_t get_object_table_num(uint8_t object_type);
uint16_t get_start_position(object_t element);
int get_firmware_build(void);
uint32_t info_block_crc(crc_t *);
uint16_t report_id_to_type(int report_id);

/*!
 * @brief Struct holding the object type / instance info.
 *
 * Struct holding the object type / instance info. An array of these maps
 * report id's to object type / instance (array index = report id).  Note
 * that the report ID number 0 is reserved.
 */
typedef struct
{
   uint8_t object_type;  /*!< Object type. */
   uint8_t instance;     /*!< Instance number. */
} report_id_map_t;

/* Global variables - documented in the .c file */
extern uint16_t command_processor_address;
extern info_block_t info_block;
