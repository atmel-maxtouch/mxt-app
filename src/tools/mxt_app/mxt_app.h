#pragma once
//------------------------------------------------------------------------------
/// \file   mxt_app.h
/// \brief  mxt-app header file
/// \author Nick Dyer
//------------------------------------------------------------------------------
// Copyright 2012 Atmel Corporation. All rights reserved.
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

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
           _a < _b ? _a : _b; })

/* T6 Debug Diagnostics Commands */
#define PAGE_UP           0x01
#define PAGE_DOWN         0x02
#define DELTAS_MODE       0x10
#define REFS_MODE         0x11

/* T25 Self Test Commands */
#define SELF_TEST_ANALOG       0x01
#define SELF_TEST_PIN_FAULT    0x11
#define SELF_TEST_PIN_FAULT_2  0x12
#define SELF_TEST_AND_GATE     0x13
#define SELF_TEST_SIGNAL_LIMIT 0x17
#define SELF_TEST_GAIN         0x20
#define SELF_TEST_OFFSET       0x21
#define SELF_TEST_ALL          0xFE
#define SELF_TEST_INVALID      0xFD
#define SELF_TEST_TIMEOUT      0xFC

int mxt_flash_firmware(const char *filename, const char *new_version,
                       int adapter, int bootloader_address);
int mxt_socket_server(uint16_t port);
int mxt_socket_client(char *ip_address, uint16_t port);
int mxt_debug_dump(int mode, const char *csv_file, uint16_t frames);
void mxt_dd_menu(void);
int mxt_store_golden_refs(void);
int mxt_menu(void);
uint8_t self_test_handler(void);
int run_self_tests(uint8_t cmd);
int mxt_serial_data_upload(const char *filename, uint16_t datatype);
int print_raw_messages(void);
int print_raw_messages_t44(void);
void event_printer(void);
void print_t6_state(uint8_t state);
