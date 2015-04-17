//------------------------------------------------------------------------------
/// \file   checksum.c
/// \brief  Functions for accessing the mXT config file(xcfg or RAW).
/// \author Ace Yang
//------------------------------------------------------------------------------
// Copyright 2014 Atmel Corporation. All rights reserved.
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

#include "libmaxtouch.h"
#include "info_block.h"

uint32_t config_crc;
#define OBP_RAW_MAGIC      "OBP_RAW V1"

/*!
 * @brief  Configure file Checksum algorithm.
 * @return Calculated Configure Block Checksum.
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
 * @brief  Calculates and reports the configure file Checksum.
 * @return #mxt_rc
 */
static int calculate_checkcrc(uint32_t read_crc,
                         uint8_t *base_addr, size_t size)
{
  uint32_t calc_crc = 0; /* Checksum calculated by the driver code */
  uint16_t crc_byte_index = 0;

  /* Call the CRC function crc24() iteratively to calculate the CRC,
   * passing it two characters at a time.  */
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
  calc_crc &= calc_crc & 0x00FFFFFF;

  /* A zero CRC indicates a communications error */
  if (calc_crc == 0) {
    fprintf(stderr, "Configure File Checksum zero\n");
    return MXT_ERROR_IO;
  }

  /* Compare the read checksum with calculated checksum */
  if (read_crc != calc_crc) {
    fprintf(stderr, "Configure File Checksum error calc=%06X, read for file=%06X\n",
            calc_crc, read_crc);
    return MXT_ERROR_INFO_CHECKSUM_MISMATCH;
  }

  fprintf(stderr, "Configure File Checksum verified %06X\n", calc_crc);
  return MXT_SUCCESS;
}

/*!
 * @brief  Calculates and reports the XCFG Config file Checksum.
 * @return #mxt_rc
 */
static int xcfg_file(char *filename){
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
	char ignore_line = false;
	int offset;
	int width;
	int ret;
	uint8_t buf[4096];
	int buf_size = 0;
	
	//fprintf(stderr,"Opening config file %s...", filename);
	
	fp = fopen(filename, "r");
	if (fp == NULL)
	{
		fprintf(stderr,"Error opening %s: %s\n", filename, strerror(errno));
		ret = mxt_errno_to_rc(errno);
		return ret;
	}
	
	while (1)
	{
		/* First character is expected to be '[' - skip empty lines and spaces  */
		c = getc(fp);
		while ((c == '\n') || (c == '\r') || (c == 0x20))
		{
			c = getc(fp);
		}
		
		if (c != '[')
		{
			if (c == EOF)
				break;
			
			/* If we are ignoring the current section then look for the next section 
			 */
			if (ignore_line)
			{
				continue;
			}
			
			fprintf(stderr,"Parse error: expected '[', read ascii char %c!", c);
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		if (fscanf(fp, "%[^] ]", object) != 1)
		{
			fprintf(stderr,"Object parse error\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		/* Ignore the comments and file header sections */
		if (!strcmp(object, "COMMENTS")
			|| !strcmp(object, "APPLICATION_INFO_HEADER"))
		{
			ignore_line = true;
			continue;
		}
		
		ignore_line = false;
		
		if( !strcmp(object, "VERSION_INFO_HEADER")) {
			
			while(ignore_line == false) {
				if (fscanf(fp, "%s", object) != 1) {
					fprintf(stderr,"Object address parse error\n");
					ret = MXT_ERROR_FILE_FORMAT;
					goto close;
				}
				if (!strncmp(object, "CHECKSUM",8)) {
					sscanf(object, "%[^'=']=%x",tmp, &config_crc);
					ignore_line = true;
					continue;
				}
			}
			continue; 
		}

		if (fscanf(fp, "%s", tmp) != 1)
		{
			fprintf(stderr,"Instance parse error\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		if (strcmp(tmp, "INSTANCE"))
		{
			fprintf(stderr,"Parse error, expected INSTANCE\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		if (fscanf(fp, "%d", &instance) != 1)
		{
			fprintf(stderr,"Instance number parse error\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		/* Read rest of header section */
		while(c != ']')
		{
			c = getc(fp);
			if (c == '\n')
			{
				fprintf(stderr,"Parse error, expected ] before end of line\n");
				ret = MXT_ERROR_FILE_FORMAT;
				goto close;
			}
		}
		
		while(c != '\n')
		{
			c = getc(fp);
		}
		
		while ((c != '=') && (c != EOF))
		{
			c = getc(fp);
		}
		
		if (fscanf(fp, "%d", &object_address) != 1)
		{
			fprintf(stderr,"Object address parse error\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		c = getc(fp);
		while((c != '=') && (c != EOF))
		{
			c = getc(fp);
		}
		
		if (fscanf(fp, "%d", &object_size) != 1)
		{
			fprintf(stderr,"Object size parse error\n");
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		c = getc(fp);
		
		/* Find object type ID number at end of object string */
		substr = strrchr(object, '_');
		if (substr == NULL || (*(substr + 1) != 'T'))
		{
			fprintf(stderr,"Parse error, could not find T number in %s\n", object);
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		if (sscanf(substr + 2, "%d", &object_id) != 1)
		{
			fprintf(stderr,"Unable to get object type ID for %s\n", object);
			ret = MXT_ERROR_FILE_FORMAT;
			goto close;
		}
		
		while (1)
		{
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
			if (fscanf(fp, "%d", &offset) != 1)
			{
				fprintf(stderr,"Address parse error\n");
				ret = MXT_ERROR_FILE_FORMAT;
				goto close;
			}
			
			/* Read byte count of this register (max 2) */
			if (fscanf(fp, "%d", &width) != 1)
			{
				fprintf(stderr,"Byte count parse error\n");
				ret = MXT_ERROR_FILE_FORMAT;
				goto close;
			}
			
			while((c != '=') && (c != EOF))
			{
				c = getc(fp);
			}
			
			if (fscanf(fp, "%d", &data) != 1)
			{
				fprintf(stderr,"Data parse error\n");
				ret = MXT_ERROR_FILE_FORMAT;
				goto close;
			}
			c = getc(fp);
			if(object_id != 38 &&  object_id != 37 &&  object_id != 68) {
				switch (width)
				{
					case 1:
						buf[buf_size] = (char) data;
						buf_size++;
						break;
					case 2:
						buf[buf_size] = (char) data & 0xFF;
						buf[buf_size + 1] = (char) ((data >> 8) & 0xFF);
						buf_size+=2;
						break;
					case 4:
						buf[buf_size] = (char) data & 0xFF;
						buf[buf_size + 1] = (char) ((data >> 8) & 0xFF);
						buf[buf_size + 2] = (char) ((data >> 16) & 0xFF);
						buf[buf_size + 3] = (char) ((data >> 24) & 0xFF);
						buf_size+=4;
						break;	
					default:
						fprintf(stderr,"Only 32-bit / 16-bit / 8-bit config values supported!\n");
						ret = MXT_ERROR_FILE_FORMAT;
						goto close;
				}
			}
		}
		
	}

	if(buf_size & 0x1) {
		buf[buf_size] = 0;
		buf_size--;
	}
	ret = calculate_checkcrc(config_crc , buf , buf_size);		
	
close:
	fclose(fp);
	return ret;
}
//******************************************************************************
/// \brief  Structure containing configuration data for a particular object
struct mxt_object_config {
  uint32_t type;
  uint8_t instance;
  uint32_t size;
  uint8_t *data;
};
//******************************************************************************
/*!
 * @brief  Calculates and reports the RAW config file Checksum.
 * @return #mxt_rc
 */
static int raw_file(char *filename){
	struct mxt_id_info cfg_info;
	FILE *fp;
	int ret = 0;
	size_t i;
	uint32_t info_crc;
	//size_t reg;
	char line[2048];
	uint8_t buf[4096];
	int buf_size = 0;
	
	memset(buf , 0x0 , 4096);
	fp = fopen(filename, "r");
	if (fp == NULL)
	{
		fprintf(stderr,"Error opening %s: %s\n", filename, strerror(errno));
		ret = mxt_errno_to_rc(errno);
		return ret;
	}
	
	if (fgets(line, sizeof(line), fp) == NULL)
	{
		fprintf(stderr,"Unexpected EOF\n");
	}
	
	if (strncmp(line, OBP_RAW_MAGIC, strlen(OBP_RAW_MAGIC)))
	{
		fprintf(stderr,"Not in OBP_RAW format\n");
		ret = -1;
		goto close;
	} else {
		fprintf(stderr,"Loading OBP_RAW file\n");
	}
	
	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_id_info); i++) {
		ret = fscanf(fp, "%hhx", (unsigned char *)&cfg_info + i);
		if (ret != 1) {
			fprintf(stderr,"Bad format\n");
			ret = -1;
			goto close;
		}
	}
	
	/* Read CRCs */
	ret = fscanf(fp, "%x", &info_crc);
	if (ret != 1) {
		fprintf(stderr,"Bad format: failed to parse Info CRC\n");
		ret = -1;
		goto close;
	}
	
	ret = fscanf(fp, "%x", &config_crc);
	if (ret != 1) {
		fprintf(stderr,"Bad format: failed to parse Config CRC\n");
		ret = -1;
		goto close;
	}
	
	while (1)
	{
		struct mxt_object_config cfg;
		
		/* Read type, instance, length */
		ret = fscanf(fp, "%x %" SCNx8 " %x", &cfg.type, &cfg.instance, &cfg.size);
		if (ret == EOF) {
			break;
		} else if (ret != 3) {
			fprintf(stderr,"Bad format: failed to parse object\n");
			ret = -1;
			goto close;
		}
		
		/* Update bytes from file */
		for (i = 0; i < cfg.size; i++) {
			uint8_t val;
			ret = fscanf(fp, "%hhx", &val);
			if (ret != 1) {
				fprintf(stderr,"Parse error in T%d", cfg.type);
				ret = -1;
				free(cfg.data);
				goto close;
			}
			
			if(cfg.type != 38 &&  cfg.type != 37 &&  cfg.type != 68)
				buf[buf_size + i] = val;
			
		}
		
		if(cfg.type != 38 &&  cfg.type != 37 &&  cfg.type != 68) 
			buf_size = buf_size + i;
		
	}

	if(buf_size & 0x1) {
		buf[buf_size] = 0;
		buf_size--;
	}
	ret = calculate_checkcrc(config_crc , buf , buf_size);
close:
	fclose(fp);
	return ret;
}

int mxt_checkcrc(char *filename){
  int ret;
  char *extension = strrchr(filename, '.');

  if (extension && (!strcmp(extension, ".xcfg") || !strcmp(extension, ".XCFG")))
    ret = xcfg_file(filename);
  else
    ret = raw_file(filename);
  
  return ret ;
}