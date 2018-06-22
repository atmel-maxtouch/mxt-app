//------------------------------------------------------------------------------
/// \file   broken_line.h
/// \brief  Broken line detection
/// \author Steven Swann
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

/* Broken Line Detection BROKEN_LINE_PATTERN Options */
#define BROKEN_LINE_PATTERN_ITO       0
#define BROKEN_LINE_PATTERN_XSENSE    1

/* Broken Line Detection Default Options */
#define BROKEN_LINE_DEFAULT_THRESHOLD 20

//******************************************************************************
/// \brief Broken line detection context options
struct broken_line_options
{
  bool dualx;
  uint8_t x_center_threshold;
  uint8_t x_border_threshold;
  uint8_t y_center_threshold;
  uint8_t y_border_threshold;
  uint8_t pattern;
};

int mxt_broken_line(struct mxt_device *mxt, struct broken_line_options *bl_opts);
