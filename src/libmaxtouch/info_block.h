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
#define MXT_OBJECT_SIZE_MAX 256

/*! \brief Checksum element struct */
struct mxt_raw_crc
{
  /*! CRC field */
  uint16_t CRC;

  /*! CRC field: higher byte */
  uint8_t CRC_hi;
} __attribute__((packed));

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
} __attribute__((packed));

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
} __attribute__((packed));

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
#define F_ENUM(x, y) x = y,
#define F_SWITCH(x, y) case x: return ( #x );

#define OBJECT_LIST(f) \
  f(RESERVED_T0, 0) \
  f(RESERVED_T1, 1) \
  f(DEBUG_DELTAS_T2, 2) \
  f(DEBUG_REFERENCES_T3, 3) \
  f(DEBUG_SIGNALS_T4, 4) \
  f(GEN_MESSAGEPROCESSOR_T5, 5) \
  f(GEN_COMMANDPROCESSOR_T6, 6) \
  f(GEN_POWERCONFIG_T7, 7) \
  f(GEN_ACQUISITIONCONFIG_T8, 8) \
  f(TOUCH_MULTITOUCHSCREEN_T9, 9) \
  f(TOUCH_SINGLETOUCHSCREEN_T10, 10) \
  f(TOUCH_XSLIDER_T11, 11) \
  f(TOUCH_YSLIDER_T12, 12) \
  f(TOUCH_XWHEEL_T13, 13) \
  f(TOUCH_YWHEEL_T14, 14) \
  f(TOUCH_KEYARRAY_T15, 15) \
  f(PROCG_SIGNALFILTER_T16, 16) \
  f(PROCI_LINEARIZATIONTABLE_T17, 17) \
  f(SPT_COMMSCONFIG_T18, 18) \
  f(SPT_GPIOPWM_T19, 19) \
  f(PROCI_GRIPFACESUPPRESSION_T20, 20) \
  f(RESERVED_T21, 21) \
  f(PROCG_NOISESUPPRESSION_T22, 22) \
  f(TOUCH_PROXIMITY_T23, 23) \
  f(PROCI_ONETOUCHGESTUREPROCESSOR_T24, 24) \
  f(SPT_SELFTEST_T25, 25) \
  f(DEBUG_CTERANGE_T26, 26) \
  f(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, 27) \
  f(SPT_CTECONFIG_T28, 28) \
  f(SPT_GPI_T29, 29) \
  f(SPT_GATE_T30, 30) \
  f(TOUCH_KEYSET_T31, 31) \
  f(TOUCH_XSLIDERSET_T32, 32) \
  f(RESERVED_T33, 33) \
  f(GEN_MESSAGEBLOCK_T34, 34) \
  f(SPT_PROTOTYPE_T35, 35) \
  f(RESERVED_T36, 36) \
  f(DEBUG_DIAGNOSTIC_T37, 37) \
  f(SPT_USERDATA_T38, 38) \
  f(SPARE_T39, 39) \
  f(PROCI_GRIPSUPPRESSION_T40, 40) \
  f(PROCI_PALMSUPPRESSION_T41, 41) \
  f(PROCI_TOUCHSUPPRESSION_T42, 42) \
  f(SPT_DIGITIZER_T43, 43) \
  f(SPT_MESSAGECOUNT_T44, 44) \
  f(PROCI_VIRTUALKEY_T45, 45) \
  f(SPT_CTECONFIG_T46, 46) \
  f(PROCI_STYLUS_T47, 47) \
  f(PROCG_NOISESUPPRESSION_T48, 48) \
  f(GEN_DUALPULSE_T49, 49) \
  f(SPARE_T50, 50) \
  f(SPT_SONY_CUSTOM_T51, 51) \
  f(TOUCH_PROXKEY_T52, 52) \
  f(GEN_DATASOURCE_T53, 53) \
  f(PROCG_NOISESUPPRESSION_T54, 54) \
  f(PROCI_ADAPTIVETHRESHOLD_T55, 55) \
  f(PROCI_SHIELDLESS_T56, 56) \
  f(PROCI_EXTRATOUCHSCREENDATA_T57, 57) \
  f(SPT_EXTRANOISESUPCTRLS_T58, 58) \
  f(SPT_FASTDRIFT_T59, 59) \
  f(SPT_TIMER_T61, 61) \
  f(PROCG_NOISESUPPRESSION_T62, 62) \
  f(PROCI_ACTIVESTYLUS_T63, 63) \
  f(SPT_REFERENCERELOAD_T64, 64) \
  f(PROCI_LENSBENDING_T65, 65) \
  f(SPT_GOLDENREFERENCES_T66, 66) \
  f(PROCI_CUSTOMGESTUREPROCESSOR_T67, 67) \
  f(SERIAL_DATA_COMMAND_T68, 68) \
  f(PROCI_PALMGESTUREPROCESSOR_T69, 69) \
  f(SPT_DYNAMICCONFIGURATIONCONTROLLER_T70, 70) \
  f(SPT_DYNAMICCONFIGURATIONCONTAINER_T71, 71) \
  f(PROCG_NOISESUPPRESSION_T72, 72) \
  f(PROCI_ZONEINDICATION_T73, 73) \
  f(PROCG_SIMPLEGESTUREPROCESSOR_T74, 74) \
  f(MOTION_SENSING_OBJECT_T75, 75) \
  f(PROCI_MOTION_GESTURES_T76, 76) \
  f(SPT_CTESCANCONFIG_T77, 77) \
  f(PROCI_GLOVEDETECTION_T78, 78) \
  f(SPT_TOUCHEVENTTRIGGER_T79, 79) \
  f(PROCI_RETRANSMISSIONCOMPENSATION_T80, 80) \
  f(PROCI_UNLOCKGESTURE_T81, 81) \
  f(SPT_NOISESUPEXTENSION_T82, 82) \
  f(ENVIRO_LIGHTSENSING_T83, 83) \
  f(PROCI_GESTUREPROCESSOR_T84, 84) \
  f(PEN_ACTIVESTYLUSPOWER_T85, 85) \
  f(PROCG_NOISESUPACTIVESTYLUS_T86, 86) \
  f(PEN_ACTIVESTYLUSDATA_T87, 87) \
  f(PEN_ACTIVESTYLUSRECEIVE_T88, 88) \
  f(PEN_ACTIVESTYLUSTRANSMIT_T89, 89) \
  f(PEN_ACTIVESTYLUSWINDOW_T90, 90) \
  f(DEBUG_CUSTOMDATACONFIG_T91, 91) \
  f(PROCI_SYMBOLGESTUREPROCESSOR_T92, 92) \
  f(PROCI_TOUCHSEQUENCELOGGER_T93, 93) \
  f(SPT_PTCCONFIG_T95, 95) \
  f(SPT_PTCTUNINGPARAMS_T96, 96) \
  f(TOUCH_PTCKEYS_T97, 97) \
  f(PROCG_PTCNOISESUPPRESSION_T98, 98) \
  f(PROCI_KEYGESTUREPROCESSOR_T99, 99) \
  f(TOUCH_MULTITOUCHSCREEN_T100, 100) \
  f(SPT_TOUCHSCREENHOVER_T101, 101) \
  f(SPT_SELFCAPHOVERCTECONFIG_T102, 102) \
  f(PROCI_SCHNOISESUPPRESSION_T103, 103) \
  f(SPT_AUXTOUCHCONFIG_T104, 104) \
  f(SPT_DRIVENPLATEHOVERCONFIG_T105, 105) \
  f(SPT_ACTIVESTYLUSMMBCONFIG_T106, 106) \
  f(PROCI_ACTIVESTYLUS_T107, 107) \
  f(PROCG_NOISESUPSELFCAP_T108, 108) \
  f(SPT_SELFCAPGLOBALCONFIG_T109, 109) \
  f(SPT_SELFCAPTUNINGPARAMS_T110, 110) \
  f(SPT_SELFCAPCONFIG_T111, 111) \
  f(PROCI_SELFCAPGRIPSUPPRESSION_T112, 112) \
  f(SPT_PROXMEASURECONFIG_T113, 113) \
  f(SPT_ACTIVESTYLUSMEASCONFIG_T114, 114) \
  f(PROCI_SYMBOLGESTURE_T115, 115) \
  f(SPT_SYMBOLGESTURECONFIG_T116, 116) \
  f(GEN_INFOBLOCK16BIT_T254, 254) \
  f(SPT_PROTOTYPE_T220, 220) \
  f(SPT_PROTOTYPE_T221, 221) \
  f(SPT_PROTOTYPE_T222, 222) \
  f(SPT_PROTOTYPE_T223, 223) \
  f(SPT_PROTOTYPE_T224, 224) \
  f(SPT_PROTOTYPE_T225, 225) \
  f(SPT_PROTOTYPE_T226, 226) \
  f(SPT_PROTOTYPE_T227, 227) \
  f(SPT_PROTOTYPE_T228, 228) \
  f(SPT_PROTOTYPE_T229, 229) \
  f(SPT_PROTOTYPE_T230, 230) \
  f(SPT_PROTOTYPE_T231, 231) \
  f(SPT_PROTOTYPE_T232, 232) \
  f(SPT_PROTOTYPE_T233, 233) \
  f(SPT_PROTOTYPE_T234, 234) \
  f(SPT_PROTOTYPE_T235, 235) \
  f(SPT_PROTOTYPE_T236, 236) \
  f(SPT_PROTOTYPE_T237, 237) \
  f(SPT_PROTOTYPE_T238, 238) \
  f(SPT_PROTOTYPE_T239, 239) \
  f(RESERVED_T255, 255)

enum mxt_object_type
{
  OBJECT_LIST(F_ENUM)
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
