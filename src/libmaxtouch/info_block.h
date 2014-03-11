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

struct mxt_device;

#define MXT_INSTANCES(o) ((uint16_t)((o).instances_minus_one) + 1)
#define MXT_SIZE(o) ((uint16_t)((o).size_minus_one) + 1)

/*! \brief Checksum element struct */
struct mxt_raw_crc
{
  /*! CRC field */
  uint16_t CRC;

  /*! CRC field: higher byte */
  uint8_t CRC_hi;
};

/*! \brief Object table element struct */
struct mxt_object
{
  uint8_t type;                  /*!< Object type ID */
  uint8_t start_pos_lsb;         /*!< LSByte of the start address of the obj config structure */
  uint8_t start_pos_msb;         /*!< MSByte of the start address of the obj config structure */
  uint8_t size_minus_one;        /*!< Byte length of the obj config structure - 1 */
  uint8_t instances_minus_one;   /*!< Number of objects of this obj. type - 1 */
  uint8_t num_report_ids;        /*!< The max number of touches in a screen,
                                  *  max number of sliders in a slider array, etc.*/
};

/*! \brief ID Information fields in the Information Block*/
struct mxt_id_info
{
  uint8_t family;           /*!< Device family */
  uint8_t variant;          /*!< Device variant */

  uint8_t version;          /*!< Firmware version (Major/minor nibbles) */
  uint8_t build;            /*!< Firmware build number */

  uint8_t matrix_x_size;    /*!< Matrix X Size */
  uint8_t matrix_y_size;    /*!< Matrix Y Size */

  /*! Number of elements in the object table. The actual number of objects
   * can be different if any object has more than one instance. */
  uint8_t num_objects;
};

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
struct mxt_info
{
  /*! Pointer to the struct containing ID Information. */
  struct mxt_id_info *id;

  /*! Pointer to an array of objects */
  struct mxt_object *objects;

  /*! Information block checksum */
  uint32_t crc;

  /*! Raw info block data */
  uint8_t *raw_info;
};

/*!
 * @brief Struct holding the object type / instance info.
 *
 * Struct holding the object type / instance info. An array of these maps
 * report id's to object type / instance (array index = report id).  Note
 * that the report ID number 0 is reserved.
 */
struct mxt_report_id_map
{
   uint16_t object_type;  /*!< Object type */
   uint8_t instance;      /*!< Instance number */
};

/*! Object types */
enum mxt_object_type {
  RESERVED_T0 = 0,
  RESERVED_T1 = 1,
  DEBUG_DELTAS_T2 = 2,
  DEBUG_REFERENCES_T3 = 3,
  DEBUG_SIGNALS_T4 = 4,
  GEN_MESSAGEPROCESSOR_T5 = 5,
  GEN_COMMANDPROCESSOR_T6 = 6,
  GEN_POWERCONFIG_T7 = 7,
  GEN_ACQUISITIONCONFIG_T8 = 8,
  TOUCH_MULTITOUCHSCREEN_T9 = 9,
  TOUCH_SINGLETOUCHSCREEN_T10 = 10,
  TOUCH_XSLIDER_T11 = 11,
  TOUCH_YSLIDER_T12 = 12,
  TOUCH_XWHEEL_T13 = 13,
  TOUCH_YWHEEL_T14 = 14,
  TOUCH_KEYARRAY_T15 = 15,
  PROCG_SIGNALFILTER_T16 = 16,
  PROCI_LINEARIZATIONTABLE_T17 = 17,
  SPT_COMMSCONFIG_T18 = 18,
  SPT_GPIOPWM_T19 = 19,
  PROCI_GRIPFACESUPPRESSION_T20 = 20,
  RESERVED_T21 = 21,
  PROCG_NOISESUPPRESSION_T22 = 22,
  TOUCH_PROXIMITY_T23 = 23,
  PROCI_ONETOUCHGESTUREPROCESSOR_T24 = 24,
  SPT_SELFTEST_T25 = 25,
  DEBUG_CTERANGE_T26 = 26,
  PROCI_TWOTOUCHGESTUREPROCESSOR_T27 = 27,
  SPT_CTECONFIG_T28 = 28,
  SPT_GPI_T29 = 29,
  SPT_GATE_T30 = 30,
  TOUCH_KEYSET_T31 = 31,
  TOUCH_XSLIDERSET_T32 = 32,
  RESERVED_T33 = 33,
  GEN_MESSAGEBLOCK_T34 = 34,
  SPT_PROTOTYPE_T35 = 35,
  RESERVED_T36 = 36,
  DEBUG_DIAGNOSTIC_T37 = 37,
  SPT_USERDATA_T38 = 38,
  SPARE_T39 = 39,
  PROCI_GRIPSUPPRESSION_T40 = 40,
  PROCI_PALMSUPPRESSION_T41 = 41,
  PROCI_TOUCHSUPPRESSION_T42 = 42,
  SPT_DIGITIZER_T43 = 43,
  SPT_MESSAGECOUNT_T44 = 44,
  PROCI_VIRTUALKEY_T45 = 45,
  SPT_CTECONFIG_T46 = 46,
  PROCI_STYLUS_T47 = 47,
  PROCG_NOISESUPPRESSION_T48 = 48,
  GEN_DUALPULSE_T49 = 49,
  SPARE_T50 = 50,
  SPT_SONY_CUSTOM_T51 = 51,
  TOUCH_PROXKEY_T52 = 52,
  GEN_DATASOURCE_T53 = 53,
  PROCG_NOISESUPPRESSION_T54 = 54,
  PROCI_ADAPTIVETHRESHOLD_T55 = 55,
  PROCI_SHIELDLESS_T56 = 56,
  PROCI_EXTRATOUCHSCREENDATA_T57 = 57,
  SPT_EXTRANOISESUPCTRLS_T58 = 58,
  SPT_FASTDRIFT_T59 = 59,
  SPT_TIMER_T61 = 61,
  PROCG_NOISESUPPRESSION_T62 = 62,
  PROCI_ACTIVESTYLUS_T63 = 63,
  SPT_REFERENCERELOAD_T64 = 64,
  PROCI_LENSBENDING_T65 = 65,
  SPT_GOLDENREFERENCES_T66 = 66,
  PROCI_CUSTOMGESTUREPROCESSOR_T67 = 67,
  SERIAL_DATA_COMMAND_T68 = 68,
  PROCI_PALMGESTUREPROCESSOR_T69 = 69,
  SPT_DYNAMICCONFIGURATIONCONTROLLER_T70 = 70,
  SPT_DYNAMICCONFIGURATIONCONTAINER_T71 = 71,
  PROCG_NOISESUPPRESSION_T72 = 72,
  PROCI_ZONEINDICATION_T73 = 73,
  PROCG_SIMPLEGESTUREPROCESSOR_T74 = 74,
  MOTION_SENSING_OBJECT_T75 = 75,
  PROCI_MOTION_GESTURES_T76 = 76,
  SPT_CTESCANCONFIG_T77 = 77,
  PROCI_GLOVEDETECTION_T78 = 78,
  SPT_TOUCHEVENTTRIGGER_T79 = 79,
  PROCI_RETRANSMISSIONCOMPENSATION_T80 = 80,
  PROCI_UNLOCKGESTURE_T81 = 81,
  SPT_NOISESUPEXTENSION_T82 = 82,
  ENVIRO_LIGHTSENSING_T83 = 83,
  PROCI_GESTUREPROCESSOR_T84 = 84,
  PEN_ACTIVESTYLUSPOWER_T85 = 85,
  PROCG_NOISESUPACTIVESTYLUS_T86 = 86,
  PEN_ACTIVESTYLUSDATA_T87 = 87,
  PEN_ACTIVESTYLUSRECEIVE_T88 = 88,
  PEN_ACTIVESTYLUSTRANSMIT_T89 = 89,
  PEN_ACTIVESTYLUSWINDOW_T90 = 90,
  DEBUG_CUSTOMDATACONFIG_T91 = 91,
  PROCI_TOUCHSEQUENCELOGGER_T93 = 93,
  TOUCH_MULTITOUCHSCREEN_T100 = 100,
  SPT_TOUCHSCREENHOVER_T101 = 101,
  SPT_SELFCAPHOVERCTECONFIG_T102 = 102,
  PROCI_SCHNOISESUPPRESSION_T103 = 103,
  SPT_AUXTOUCHCONFIG_T104 = 104,
  SPT_DRIVENPLATEHOVERCONFIG_T105 = 105,
  SPT_ACTIVESTYLUSMMBCONFIG_T106 = 106,
  PROCI_ACTIVESTYLUS_T107 = 107,
  PROCG_NOISESUPSELFCAP_T108 = 108,
  SPT_SELFCAPGLOBALCONFIG_T109 = 109,
  SPT_SELFCAPTUNINGPARAMS_T110 = 110,
  SPT_SELFCAPCONFIG_T111 = 111,
  GEN_INFOBLOCK16BIT_T254 = 254,
  RESERVED_T255 = 255,
};

/*! Returned by get_object_address() if object is not found */
#define OBJECT_NOT_FOUND   0u

#define MXT_FW_VER_LEN     10u

/* Function prototypes */
int mxt_read_info_block(struct mxt_device *dev);
int mxt_calc_report_ids(struct mxt_device *dev);
void mxt_display_chip_info(struct mxt_device *dev);
uint16_t mxt_get_object_address(struct mxt_device *dev, uint16_t object_type, uint8_t instance);
uint8_t mxt_get_object_size(struct mxt_device *dev, uint16_t object_type);
uint8_t mxt_get_object_table_num(struct mxt_device *dev, uint16_t object_type);
uint16_t mxt_get_start_position(struct mxt_object obj, uint8_t instance);
int mxt_get_firmware_version(struct mxt_device *dev, char *version_str);
uint16_t mxt_report_id_to_type(struct mxt_device *dev, int report_id);
uint8_t mxt_get_object_instances(struct mxt_device *mxt, uint16_t object_type);
