//------------------------------------------------------------------------------
/// \file   freq_sweep.h
/// \brief  Frequency Sweep Tool
/// \author Michael Gong
//------------------------------------------------------------------------------
// Copyright 2024 Microchip Technology. All rights reserved.
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


/* Frequency Sweep definitions */
//#define BROKEN_LINE_PATTERN_ITO       0
//#define BROKEN_LINE_PATTERN_XSENSE    1


//******************************************************************************
/// \brief Frequency sweep tool options
/// \fs_opts name
struct freq_sweep_options {
  struct fs_msg *msg_output;
  bool dualx;
  uint32_t freq_start;
  uint32_t freq_end;
  uint32_t curr_freq;
  uint32_t step_size;
  uint32_t duration;
  uint32_t adcperx;
  uint32_t frames;
  uint32_t var1;
  uint32_t var2;
  uint32_t sx_nlgain;
  uint32_t dx_nlgain;
  bool sweep_object;  /* 0 for T108, 1 for T72 */
  uint16_t t72_addr;
  uint16_t t108_addr;
  uint8_t t72_size;
  uint8_t t108_size;
  uint16_t t100_addr;
  uint8_t t100_size;
  uint8_t *t100_buf;
  uint8_t *t72_buf;
  uint8_t *t108_buf;
  uint8_t t100_msg_cnt;
  uint8_t t72_t108_msg_cnt;
};

struct fs_msg {
  uint8_t status1;
  uint8_t status2;
  uint8_t mintchthr;
  uint8_t pknoiselvl;
  uint8_t noiselvl;
  uint8_t nlthr;
  int peak_deltas;
};

int mxt_get_options(struct mxt_device *mxt, const char *input_file, struct freq_sweep_options *fs_opts);
int mxt_set_test_parameters(struct mxt_device *mxt, struct freq_sweep_options *fs_opts);
int mxt_fs_sweep_start(struct mxt_device *mxt, struct freq_sweep_options *fs_opts, const char *outputfile);
int mxt_print_fs_header(struct mxt_device *mxt, struct freq_sweep_options *fs_opts, FILE *fp);
int mxt_save_obj_parameters(struct mxt_device *mxt, struct freq_sweep_options *fs_opts);
int mxt_freq_sweep(struct mxt_device *mxt, const char *input_file, const char *output_file,
  struct freq_sweep_options *fs_opts);
int mxt_enable_noise_obj(struct mxt_device *mxt, struct freq_sweep_options *fs_opts);
int mxt_disable_noise_obj(struct mxt_device *mxt, struct freq_sweep_options *fs_opts);
