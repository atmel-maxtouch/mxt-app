//------------------------------------------------------------------------------
/// \file   freq_sweep.c
/// \brief  Frequency Sweep Tool
/// \author Michael Gong
//------------------------------------------------------------------------------
// Copyright 2024 Atmel Corporation. All rights reserved.
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
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/msg.h"

#include "mxt_app.h"
#include "freq_sweep.h"


int mxt_fs_insert_data (struct mxt_device *mxt, FILE *fp, uint8_t *msg_buf,
						struct freq_sweep_options *fs_opts)
{
	float mintchthr = 0;
	float noiselvl = 0;
	float nlthr = 0;
	float pknltdlvl = 0;

	if (fs_opts->sweep_object) {
	/* Calculation means for Mintchthr, noiselvl, nlthr */
		
		if (msg_buf[8] != 0x00) {
			mintchthr = msg_buf[3] / msg_buf[8];
			noiselvl =  msg_buf[5] / msg_buf[8];
			nlthr =  msg_buf[6] / msg_buf[8];
		}

		mxt_dbg(mxt->ctx, "%d,%2.2f,%2.2f,%d,%2.2f,%d,%d\n", 
			fs_opts->curr_freq, mintchthr, noiselvl, msg_buf[4], nlthr, fs_opts->adcperx, msg_buf[7]);

		fprintf(fp, "%d,%2.2f,%2.2f,%d,%2.2f,%d,%d\n", 
			fs_opts->curr_freq, mintchthr, noiselvl, msg_buf[4], nlthr, fs_opts->adcperx, msg_buf[7]);
	} else {

		if (msg_buf[8] != 0x00) {
			pknltdlvl = msg_buf[4] / msg_buf[8];
		}

		mxt_dbg(mxt->ctx, "T108: %d,%2.6f,%d,%d\n", fs_opts->curr_freq, pknltdlvl, fs_opts->adcperx, msg_buf[7]);
		fprintf(fp, "%d,%2.6f,%d,%d\n", fs_opts->curr_freq, pknltdlvl, fs_opts->adcperx, msg_buf[7]);
	}

	return MXT_SUCCESS;

}

int mxt_get_options(struct mxt_device *mxt, const char *input_file,
	struct freq_sweep_options *fs_opts)
{
	bool ignore_line = false;
	char in_str[255];
	char object[255];
	char tmp_str[255];
	int file_read = 0;
	int ret = 0;
	FILE *fp;
	int c;

	/* Get the input file, parse test parameters */
	fp = fopen(input_file, "r");
	if (fp == NULL) {
		mxt_err(mxt->ctx, "\nError opening %s: %s", input_file, strerror(errno));
		return mxt_errno_to_rc(errno);
	} else {
		mxt_info(mxt->ctx, "\nFound input parameter file %s", input_file);
	}

	while (!file_read) {
		c = getc(fp);

		/* Search for beginning bracket */
		while ((c == '\n') || (c == '\r') || (c == 0x20))
			c = getc(fp);

		if (c != '[') {
			if (c == EOF)
				break;

			/* skip characters until finding next beginning bracket */
			if (ignore_line) {
				continue;
			}

      		mxt_err(mxt->ctx, "Parse error: expected '[', read ascii char %c!", c);
      		ret = MXT_ERROR_FILE_FORMAT;
      		goto close;
		}

		/* Grab string before ending bracket */
		if (fscanf(fp, "%[^] ]", in_str) != 1) {
      		mxt_err(mxt->ctx, "Object parse error");
      		ret = MXT_ERROR_FILE_FORMAT;
      		goto close;
    	}

    	if (!strcmp(in_str, "COMMENTS")) {
    		ignore_line = true;
    		mxt_dbg(mxt->ctx, "Skipping [%s] section", in_str);
    		continue;
    	}

    	ignore_line = false;

    	if (!strcmp(in_str, "Frequency_Sweep_Parameters")) {
			c = getc(fp);	/* Get the last ']' before next line*/
			break;
		}

	}	/* While loop - file_read */

	if (fscanf(fp, "%s", in_str) != 1)  {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Frequency_Start", 15)) {
		sscanf(in_str, "%[^'=']=%d", object, &fs_opts->freq_start);
	}

	if (fscanf(fp, "%s", in_str) != 1)  {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Frequency_End", 13)) {
		sscanf(in_str, "%[^'=']=%d", object, &fs_opts->freq_end);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Frequency_Step_Size", 19)) {

		sscanf(in_str, "%[^'=']=%d", object, &fs_opts->step_size);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Frequency_Duration", 18)) {
			
		sscanf(in_str, "%[^'=']=%ds", object, &fs_opts->duration);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Frames", 6)) {
		sscanf(in_str, "%[^'=']=%d", object, &fs_opts->frames);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Constant_ADC_Per_X_Value", 24)) {
		sscanf(in_str, "%[^'=']=%d", object, &fs_opts->adcperx);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Variable_Value", 14)) {
		sscanf(in_str, "%[^'=']={%d,%d}", object, 
			&fs_opts->var1, &fs_opts->var2);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Single_X_NLGAIN", 15)) {

		sscanf(in_str, "%[^'=']=%d", object, 
			&fs_opts->sx_nlgain);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Dual_X_NLGAIN", 13)) {
		
		sscanf(in_str, "%[^'=']=%d", object, 
			&fs_opts->dx_nlgain);
	}

	if (fscanf(fp, "%s", in_str) != 1) {
		mxt_err(mxt->ctx, "Invalid string format");
		goto close;
	}

	if (!strncmp(in_str, "Noise_Suppression_Object", 24)) {

		sscanf(in_str, "%[^'=']=%s", object, tmp_str);

		if (!strncmp(tmp_str, "Mutual_T(72)", 12)) {
			fs_opts->sweep_object = 1;
		} else if (!strncmp(tmp_str, "Self_Cap_(T108)", 15)) {
			fs_opts->sweep_object = 0;
		}
	}

close:
	fclose(fp);
	return ret;
}

int mxt_save_obj_parameters(struct mxt_device *mxt, struct freq_sweep_options *fs_opts)
{
	int ret = 0;
	int i;

	/* Get T72 obj_addr and obj_size */
	fs_opts->t72_addr = mxt_get_object_address(mxt, PROCG_NOISESUPPRESSION_T72, 0);
	if (fs_opts->t72_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

	fs_opts->t72_size = mxt_get_object_size(mxt, PROCG_NOISESUPPRESSION_T72);

	/* Get T108 obj_addr and obj_size */
	fs_opts->t108_addr = mxt_get_object_address(mxt, PROCG_NOISESUPSELFCAP_T108, 0);
	if (fs_opts->t108_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

	fs_opts->t108_size = mxt_get_object_size(mxt, PROCG_NOISESUPSELFCAP_T108);

	/* Get T100 obj_addr and obj_size */
	fs_opts->t100_addr = mxt_get_object_address(mxt, TOUCH_MULTITOUCHSCREEN_T100, 0);
	if (fs_opts->t100_addr == OBJECT_NOT_FOUND) return MXT_ERROR_OBJECT_NOT_FOUND;

	fs_opts->t100_size = mxt_get_object_size(mxt, TOUCH_MULTITOUCHSCREEN_T100);

	/* Allocate storage buffers */
	fs_opts->t72_buf = calloc(fs_opts->t72_size, sizeof(uint8_t));
	if (!fs_opts->t72_buf)
		return MXT_ERROR_NO_MEM;

	fs_opts->t108_buf = calloc(fs_opts->t108_size, sizeof(uint8_t));
	if (!fs_opts->t108_buf)
		return MXT_ERROR_NO_MEM;

	fs_opts->t100_buf = calloc(fs_opts->t100_size, sizeof(uint8_t));
	if (!fs_opts->t100_buf)
		return MXT_ERROR_NO_MEM;

	/* Save content of objects */
	for (i = 0; i < fs_opts->t72_size; i++) {
		ret = mxt_read_register(mxt, &fs_opts->t72_buf[i], fs_opts->t72_addr + i, 1);
		if (ret) {
			mxt_err(mxt->ctx, "Failed to save T72 object registers");
			return MXT_ERROR_IO;
		}
	}

	for (i = 0; i < fs_opts->t108_size; i++) {
		ret = mxt_read_register(mxt, &fs_opts->t108_buf[i], fs_opts->t108_addr + i, 1);
		if (ret) {
			mxt_err(mxt->ctx, "Failed to save T108 object registers");
			return MXT_ERROR_IO;
		}
	}

	for (i = 0; i < fs_opts->t100_size; i++) {
		ret = mxt_read_register(mxt, &fs_opts->t100_buf[i], fs_opts->t100_addr + i, 1);
		if (ret) {
			mxt_err(mxt->ctx, "Failed to save T100 object registers");
			return MXT_ERROR_IO;
		}
	}

	return MXT_SUCCESS;
}

int mxt_set_test_parameters(struct mxt_device *mxt, struct freq_sweep_options *fs_opts)
{
	uint8_t backup_cmd = BACKUPNV_COMMAND;
	uint8_t noisctrl;
	uint8_t tmp_byte;
	int ret = 0;
	int i;

	/* Enable Peak deltas in T100 TCHAUX byte only */

	tmp_byte = T100_TCHAUX_PEAK_BIT;
	
	ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t100_addr + T100_TCHAUX_OFFSET, 1);
	if (ret) {
		mxt_err(mxt->ctx, "Failed to write T100 TCHAUX register");
		goto failed_wr_rd;
	}

	/* Setup T72 noise suppression object registers */
	if (fs_opts->sweep_object == true) {

		/* Setup NL GAIN */
		if (fs_opts->sx_nlgain != 0) {

			tmp_byte = fs_opts->sx_nlgain & 0xFF;

			ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_SX_NLGAIN_OFFSET, 1);

			/* Enable recal and disable auto */
			noisctrl = 0x80;
		} else {

			tmp_byte = fs_opts->dx_nlgain & 0xFF;

			ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_DX_NLGAIN_OFFSET, 1);

			/* Enable recal, disable auto, set DX mode */
			noisctrl = 0x88;
		}

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write to T72 NLGAIN");
			goto failed_wr_rd;
		}

		/* Setup Noise state */
		ret = mxt_write_register(mxt, &noisctrl, fs_opts->t72_addr +
			T72_NOISCTRL_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write T72 NOISCTRL");
			goto failed_wr_rd;
		}

		tmp_byte = fs_opts->adcperx & 0xFF;

		/* Setup ADC per X, touch */
		for (i = 0; i < 5; i++) {
			ret = mxt_write_register(mxt, &tmp_byte, (fs_opts->t72_addr +
				T72_NOISE_TCH_ADPX + i), 1);
		}

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write touch ADC per X");
			goto failed_wr_rd;
		}

		/* Setup ADC per X, no touch */
		for (i = 0; i < 5; i++) {
			ret = mxt_write_register(mxt, &tmp_byte, (fs_opts->t72_addr +
				T72_NOISE_NOTCH_ADPX + i), 1);
		}

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write no touch ADC per X");
			goto failed_wr_rd;
		}

		/* Clear all noise thresholds */
		tmp_byte = 0x00;

		for (i = 0; i < 3; i++) {
			ret = mxt_write_register(mxt, &tmp_byte, (fs_opts->t72_addr + 
				T72_NOISE_THR_OFFSET + i), 1);
		}

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write noise threshold register");
			goto failed_wr_rd;
		}

		/* Disable NL freq hop and state change */
		ret = mxt_read_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CFG3_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to read from T72 CFG3 register");
			goto failed_wr_rd;
		}

		tmp_byte &= ~(T72_NOIS_HOP_BITS);

		ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CFG3_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write to T72 CFG3 register");
			goto failed_wr_rd;
		}

		/* Force Noisy Mode */
		ret = mxt_read_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CALCFG1_OFFSET, 1);
		if (ret) {
			mxt_err(mxt->ctx, "Failed to read CALCFG1 register");
			goto failed_wr_rd;
		}

		tmp_byte |= T72_NOISY_STATE_BIT;

		ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CALCFG1_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write to T72 CALCFG1 register");
			goto failed_wr_rd;
		}

		ret = mxt_read_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CFG1_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to read T72 CFG1 register");
			goto failed_wr_rd;
		}

		tmp_byte |= T72_NOISY_STATE_BIT;
		
		ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_CFG1_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write T72 CFG1 register");
			goto failed_wr_rd;
		}

		/* Set hop count to 1 */

		tmp_byte = 0x01;

		ret = mxt_write_register(mxt, &tmp_byte, fs_opts->t72_addr +
			T72_HOPCNT_OFFSET, 1);

		if (ret) {
			mxt_err(mxt->ctx, "Failed to write T72 HOPCNT register");
			goto failed_wr_rd;
		}

	} else { /* T108 setup */

		/* Setup up T108 registers */
	}

	ret = mxt_backup_config(mxt, backup_cmd);

	if (ret) {
		mxt_err(mxt->ctx, "Failed to backup configuration\n");
	}

	return MXT_SUCCESS;

failed_wr_rd:

	return MXT_ERROR_IO;
}

int mxt_fs_handle_messages(struct mxt_device *mxt, uint8_t *msg,
						   void *context, uint8_t size, uint8_t msg_count)
{
	unsigned int object_type = mxt_report_id_to_type(mxt, msg[0]);
	bool t72_msg_found = false;
	bool t108_msg_found = false;
	time_t start_time = time(NULL);
	uint8_t *msg_type = context;
	time_t now;

	mxt_dbg(mxt->ctx, "Received message from T%u", object_type);

	// msg_type fields:
	// msg_type[0] - msg[0] - Msg type (i.e T72, T100, T6, etc)
	// msg_type[1] - msg[1] - Status1 byte 
	// msg_type[2] - msg[2] - Status2 byte
	// msg_type[3] - msg[3] - Mean MINTCHTHR value
	// msg_type[4] - msg[4] - PKNOISELVL value T72 and T108
	// msg_type[5] - msg[5] - Mean NOISELVL value
	// msg_type[6] - msg[6] - NLTHR value
	// msg_type[7] - msg[7] - Peak delta of touch on screen
	// msg_type[8] - msg[8] - T72 message count
	// msg_type[9] - Read msg start time, if zero, reading by frames

	if (object_type == PROCG_NOISESUPPRESSION_T72) {

		t72_msg_found = true;

		msg_type[8] += 1;	/* Inc. T72 msg count upon entry */

		msg_type[0] = object_type;
		msg_type[1] = msg[1];	/* Status1 */
		msg_type[2] = msg[2];	/* Status2 */

		/* mintchthr */
		msg_type[3] += msg[3];

		/* Max pknoise lvl */
		if (msg[4] >= msg_type[4])
			msg_type[4] = msg[4];

		/* noiselvl */
		msg_type[5] += msg[5];

		/* NLTHR */
		msg_type[6] += msg[6];

	} else if (object_type == PROCG_NOISESUPSELFCAP_T108) {
		
		t108_msg_found = true;

		msg_type[9] += 1;	/* Inc. T108 msg count upon entry */

		/* PKNLTDLVL */
		msg_type[4] = (msg_type[4] + msg[4]) / (msg_type[9]);

	} else if (object_type == TOUCH_MULTITOUCHSCREEN_T100) { 
		
		/* Find max peak threshold*/
		if (msg[6] >= msg_type[7])
			msg_type[7] = msg[6];

	} else if (object_type == GEN_COMMANDPROCESSOR_T6) {
		/* Debug only */
		// print_t6_status(msg[1]);
	}

	/* Get current time */
	now = time(NULL);

	/* Verify if time buffer has start time from read msg */
	/* Otherwise reading by frames */
	if (msg_type[9] != 0) {
		if ((now - start_time) > msg_type[9]) {
       		mxt_info(mxt->ctx, "%d has elapsed, proceeding to next freq.");
        	return MXT_SUCCESS;
    	}
    } else {
    	if ((msg_count == 0) && (t72_msg_found || t108_msg_found))	/* No more msgs in current cycle */
			return MXT_SUCCESS;
	}

	return MXT_MSG_CONTINUE;	/* Look for next msg, 1s timeout */
}

int mxt_enable_noise_obj(struct mxt_device *mxt, struct freq_sweep_options *fs_opts)
{
	uint8_t tmp_byte;
	uint16_t offset;
	uint16_t addr;
	int ret = 0;

	/* Starting parameters */
	if (fs_opts->sweep_object) {
		offset = T72_NOISFREQ_OFFSET;
		addr = fs_opts->t72_addr;
	} else {
		offset = T108_NOISFREQ_OFFSET;;
		addr = fs_opts->t108_addr;
	}

	/* Turn on T72 */
	tmp_byte = 0xFF;

	ret = mxt_write_register(mxt, &tmp_byte, addr +	T72_T108_CTRL_OFFSET, 1);

	if (ret) {
		mxt_err(mxt->ctx, "Error writing to T72/T08 offset register");
	}

	return ret;
}

int mxt_disable_noise_obj(struct mxt_device *mxt, struct freq_sweep_options *fs_opts)
{
	uint8_t tmp_byte;
	uint16_t offset;
	uint16_t addr;
	int ret = 0;

	/* Starting parameters */
	if (fs_opts->sweep_object) {
		offset = T72_NOISFREQ_OFFSET;
		addr = fs_opts->t72_addr;
	} else {
		offset = T108_NOISFREQ_OFFSET;;
		addr = fs_opts->t108_addr;
	}

	/* Turn on T72 */
	tmp_byte = 0x00;

	ret = mxt_write_register(mxt, &tmp_byte, addr +	T72_T108_CTRL_OFFSET, 1);

	if (ret) {
		mxt_err(mxt->ctx, "Error writing to T72/T08 offset register");
	}

	return ret;
}


int mxt_fs_sweep_start(struct mxt_device *mxt, struct freq_sweep_options *fs_opts, const char *output_file)
{
	uint8_t msg_buf[10] = {0};
	uint16_t addr;
	uint16_t offset;
	int flag = false;
	time_t t1;
	time_t t2;
	int diff;
	FILE *fp;
	int i, j;
	int ret = 0;

	fp = fopen(output_file, "w");
	if (fp == NULL) {
		mxt_err(mxt->ctx, "Failed to create %s file\n", output_file);
		ret = MXT_ERROR_IO;
		return ret;
	} else {
		mxt_info(mxt->ctx, "Output file %s created\n", output_file);
	}

	/* Create the freq sweep header */
	ret = mxt_print_fs_header(mxt, fs_opts, fp);
	if (ret) {
		mxt_err(mxt->ctx, "Failed to create sweep header in %s file", output_file);
		goto fs_close;
	}

	/* Starting parameters */
	if (fs_opts->sweep_object) {
		offset = T72_NOISFREQ_OFFSET;
		addr = fs_opts->t72_addr;
	} else {
		offset = T108_NOISFREQ_OFFSET;;
		addr = fs_opts->t108_addr;
	}

	/* Assign start frequency */
	fs_opts->curr_freq = fs_opts->freq_start;

	/* Clear msg content for next freq */
	memset(msg_buf, 0, sizeof(msg_buf));

	do { 	/* Loop until freq end */
		/* Capture based on frames or duration */
		mxt_info(mxt->ctx, "Testing frequency %d", fs_opts->curr_freq);

		/* Write current frequency into NOISY FREQ registers */
		for (j = 0; j < 5; j++) {
			ret = mxt_write_register(mxt, (uint8_t *) &fs_opts->curr_freq, 
				(addr + j +	offset), 1);

			if (ret) {
				mxt_err(mxt->ctx, "Error writing current freq to register");
				goto fs_close;
			}
		}

		if (fs_opts->frames != 0x00) {

			for (i = 0; i < fs_opts->frames; i++) {

				mxt_info(mxt->ctx, "Testing frame %d", i);

				ret = mxt_disable_noise_obj(mxt, fs_opts);

				if (ret) {
					mxt_err(mxt->ctx, "Error disabling T72/T108 object");
					goto fs_close;
				} else {
					mxt_dbg(mxt->ctx, "Disabling T72/T108");
				}

				msleep(100);

				ret = mxt_enable_noise_obj(mxt, fs_opts);

				if (ret) {
					mxt_err(mxt->ctx, "Error enabling T72/T108 object");
					goto fs_close;
				} else {
					mxt_dbg(mxt->ctx, "Enabling T72/T108");
				}
				
				/* Capture the return message, timeout 5s */
				ret = mxt_read_messages(mxt, 10, &msg_buf, mxt_fs_handle_messages, &flag);

				if (ret == MXT_ERROR_TIMEOUT) {
					mxt_warn(mxt->ctx, "WARN: timed out waiting for return msg");
				}
			}	/* Frames */
		} else { /* Capture by time */

		 	mxt_info(mxt->ctx, "Test time duration = %ds", fs_opts->duration);
			/* Pass time to handle messages function */
			msg_buf[9] = fs_opts->duration;
			
			ret = mxt_disable_noise_obj(mxt, fs_opts);

			if (ret) {
				mxt_err(mxt->ctx, "Error disabling T72/T108 object");
				goto fs_close;
			} else {
				mxt_dbg(mxt->ctx, "Disabling T72/T108");
			}

			ret = mxt_enable_noise_obj(mxt, fs_opts);

			if (ret) {
				mxt_err(mxt->ctx, "Error enabling T72/T108 object");
				goto fs_close;
			} else {
				mxt_dbg(mxt->ctx, "Enabling T72/T108");
			}

			ret = mxt_read_messages(mxt, fs_opts->duration, &msg_buf, mxt_fs_handle_messages, &flag);

			if (ret == MXT_ERROR_TIMEOUT) {
				mxt_info(mxt->ctx, "%d seconds has elapsed\n", fs_opts->duration);
			}
		}

		/* Insert data, regardless if zeros */
		ret = mxt_fs_insert_data(mxt, fp, msg_buf, fs_opts);
		if (ret) {
			mxt_err(mxt->ctx, "Failed to insert data");
		}

		/* Clear msg content for next freq */
		memset(msg_buf, 0, sizeof(msg_buf));

		/* Increment to next frequency */

		fs_opts->curr_freq += fs_opts->step_size;

	} while (fs_opts->curr_freq <= fs_opts->freq_end);

	/* Write back all the data in the config */
	
	for (i = 0; i < fs_opts->t72_size; i++) {
		ret = mxt_write_register(mxt, &fs_opts->t72_buf[i], (fs_opts->t72_addr +
				i), 1);
	}

	for (i = 0; i < fs_opts->t108_size; i++) {
		ret = mxt_write_register(mxt, &fs_opts->t108_buf[i], (fs_opts->t108_addr +
				i), 1);
	}

	for (i = 0 ;i < fs_opts->t100_size; i++ ) {
		ret = mxt_write_register(mxt, &fs_opts->t100_buf[i], (fs_opts->t100_addr + i), 1);
	}

	if (ret) {
		mxt_err(mxt->ctx, "Failed to restore config settings");
		goto fs_close;
	}

	ret = mxt_backup_config(mxt, BACKUPNV_COMMAND);

fs_close:

	free(fs_opts->t72_buf);
	free(fs_opts->t108_buf);
	free(fs_opts->t100_buf);
	fclose(fp);
	return ret;
}

int mxt_print_fs_header(struct mxt_device *mxt, struct freq_sweep_options *fs_opts, FILE *fp) {

	struct mxt_id_info *id = mxt->info.id;
	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	uint8_t tmp_byte;
	uint16_t t8_addr;
	uint16_t t56_addr;
	uint16_t t111_addr;
	int ret = 0;

	fprintf(fp, "Frequency Sweep Log:\n");
	fprintf(fp, "TimeStamp        : ");
	fprintf(fp, "%d-%02d-%02d %02d:%02d:%02d\n", tm.tm_year + 1900, tm.tm_mon + 1,
		tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

	fprintf(fp, "Chip Information\n");
	fprintf(fp, "Family ID        : ");
	fprintf(fp, "%02X\n", id->family);
	fprintf(fp, "Variant ID       : ");
	fprintf(fp, "%02X\n", id->variant);
	fprintf(fp, "Firmware Version : ");
	fprintf(fp, "%02X\n", id->version);
	fprintf(fp, "Build Number     : ");
	fprintf(fp, "%02X\n", id->build);
	fprintf(fp, "XMatrixSize      : ");
	fprintf(fp, "%02X\n", id->matrix_x_size);
	fprintf(fp, "YMatrixSize      : ");
	fprintf(fp, "%02X\n\n", id->matrix_y_size);

	fprintf(fp, "Test Settings\n");
	fprintf(fp, "Frequency Start         : ");
	fprintf(fp, "%d\n", fs_opts->freq_start);
	fprintf(fp, "Frequency End           : ");
	fprintf(fp, "%d\n", fs_opts->freq_end);
	fprintf(fp, "Frequency Step Size     : ");
	fprintf(fp, "%d\n", fs_opts->step_size);
	fprintf(fp, "Frequency Step Duration : ");
	fprintf(fp, "%d seconds\n", fs_opts->duration);
	fprintf(fp, "Num of capture frames   : ");
	fprintf(fp, "%d\n", fs_opts->frames);
	fprintf(fp, "ADC's Per X\n");
	fprintf(fp, "Constant ADC. Value     : ");
	fprintf(fp, "%d\n", fs_opts->adcperx);
	fprintf(fp, "Noise Suppression Object    : ");
	if (fs_opts->sweep_object) {
		fprintf(fp, "Mutual_T(72)\n");
	} else {
		fprintf(fp, "Self Cap (T108)\n");
	}

	t8_addr = mxt_get_object_address(mxt, GEN_ACQUISITIONCONFIG_T8, 0);
	t56_addr = mxt_get_object_address(mxt, PROCI_SHIELDLESS_T56, 0);
	t111_addr = mxt_get_object_address(mxt, SPT_SELFCAPCONFIG_T111, 0);

	fprintf(fp, "Mutual T8 Charge Time       : ");
	ret = mxt_read_register(mxt, &tmp_byte, t8_addr + T8_CHRGTIME_OFFSET, 1);
	fprintf(fp, "%d\n", tmp_byte);

	ret = mxt_read_register(mxt, &tmp_byte, t56_addr + T56_INTTIME_OFFSET, 1);
	fprintf(fp, "T56 Integration Time        : ");
	fprintf(fp, "%d\n", tmp_byte);

	ret = mxt_read_register(mxt, &tmp_byte, t111_addr + T111_INTTIME_OFFSET, 1);
	fprintf(fp, "T111[0] Integration Time    : ");
	fprintf(fp, "%d\n", tmp_byte);

	ret = mxt_read_register(mxt, &tmp_byte, t111_addr + T111_DELAYTIME_OFFSET, 1);
	fprintf(fp, "T111[0] Delay Time          : ");
	fprintf(fp, "%d\n", tmp_byte);

	if (ret) {
		mxt_err(mxt->ctx, "Error reading header parameters");
		return MXT_ERROR_IO;
	}

	if (fs_opts->sweep_object) {
		fprintf(fp, "BASEFREQ,MEAN MINTCHTHR,MEAN NOISELVL,MAX PKNOISELVL,"
			"MEAN NLTHR,ADCS PER X,PEAK DELTAS\n");
	} else {
		fprintf(fp, "BASEFREQ,MAX PKNOISELVL,ADCS PER X,PEAK DELTAS\n");
	}
	
	return MXT_SUCCESS;
}

int mxt_freq_sweep(struct mxt_device *mxt, const char *input_file,
	const char *output_file, struct freq_sweep_options *fs_opts)
{
	int ret = 0;
	int i;

	ret = mxt_get_options(mxt, input_file, fs_opts);
	if (ret) {
		mxt_err(mxt->ctx, "Error getting test parameters\n");
		goto fs_end;
	} else {
		mxt_info(mxt->ctx, "Successfully read input paramaters\n");
	}

	ret = mxt_save_obj_parameters(mxt, fs_opts);
	if (ret) {
		mxt_err(mxt->ctx, "Failed to save object parameters\n");
		goto fs_end;
	} else {
		mxt_dbg(mxt->ctx, "Finished reading the T72, T108 and T100 data\n");
	}

	ret = mxt_set_test_parameters(mxt, fs_opts);
	if (ret) {
		mxt_err(mxt->ctx, "Failed to set test parameters\n");
		goto fs_end;
	} else {
		mxt_info(mxt->ctx, "Successfully set test parameters\n");
	}

	/* Start the frequency sweep process */
	ret = mxt_fs_sweep_start(mxt, fs_opts, output_file);
	if (ret) {
		mxt_err(mxt->ctx, "Failed to start frequency sweep\n");
		goto fs_end;
	} else {
		mxt_info(mxt->ctx, "Frequency Sweep Completed\n");
	}

fs_end:

	return ret;
}