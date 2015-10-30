//------------------------------------------------------------------------------
/// \file   self_test.c
/// \brief  T25 Self Test functions
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/info_block.h"
#include "libmaxtouch/utilfuncs.h"
#include "libmaxtouch/log.h"

#include "mxt_app.h"

#define T25_TIMEOUT   10

//******************************************************************************
/// \brief Handle messages from the self test object
/// \return #mxt_rc
static int self_test_handle_messages(struct mxt_device *mxt, uint8_t *msg,
                                     void *context, uint8_t size)
{
  unsigned int object_type = mxt_report_id_to_type(mxt, msg[0]);
  int ret;

  mxt_verb(mxt->ctx, "Received message from T%u", object_type);

  if (object_type == SPT_SELFTEST_T25) {
    switch (msg[1]) {
    case SELF_TEST_ALL:
      mxt_info(mxt->ctx, "PASS: All tests passed");
      ret = MXT_SUCCESS;
      break;
    case SELF_TEST_INVALID:
      mxt_err(mxt->ctx, "FAIL: Invalid or unsupported test command");
      ret = MXT_ERROR_NOT_SUPPORTED;
      break;
    case SELF_TEST_TIMEOUT:
      mxt_err(mxt->ctx, "FAIL: Test timeout");
      ret = MXT_ERROR_TIMEOUT;
      break;
    case SELF_TEST_ANALOG:
      mxt_err(mxt->ctx, "FAIL: AVdd Analog power is not present");
      ret = MXT_ERROR_SELF_TEST_ANALOG;
      break;
    case SELF_TEST_PIN_FAULT:
      mxt_err(mxt->ctx, "FAIL: Pin fault");
      ret = MXT_ERROR_SELF_TEST_PIN_FAULT;
      break;
    case SELF_TEST_PIN_FAULT_2:
      if (msg[3] == 0 && msg[4] == 0)
        mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM=%d driven shield line failed");
      else if (msg[3] > 0)
        mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM=%d X%d", msg[2], msg[3] - 1);
      else if (msg[4] > 0)
        mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM=%d Y%d", msg[2], msg[4] - 1);

      ret = MXT_ERROR_SELF_TEST_PIN_FAULT;
      break;
    case SELF_TEST_AND_GATE:
      mxt_err(mxt->ctx, "FAIL: AND Gate Fault");
      ret = MXT_ERROR_SELF_TEST_AND_GATE;
      break;
    case SELF_TEST_SIGNAL_LIMIT:
      mxt_err(mxt->ctx, "FAIL: Signal limit fault in T%d[%d]", msg[2], msg[3]);
      ret = MXT_ERROR_SELF_TEST_SIGNAL_LIMIT;
      break;
    case SELF_TEST_GAIN:
      mxt_err(mxt->ctx, "FAIL: Gain error");
      ret = MXT_ERROR_SELF_TEST_GAIN;
      break;
    default:
      mxt_err(mxt->ctx, "FAIL: status %02X", msg[1]);
      ret = MXT_ERROR_UNEXPECTED_DEVICE_STATE;
      break;
    }
  } else {
    ret = MXT_MSG_CONTINUE;
  }

  return ret;
}

//******************************************************************************
/// \brief Print T25 limits for enabled touch object instances
static int print_touch_object_limits(struct mxt_device *mxt, uint16_t t25_addr,
                                     uint16_t object_type, int *touch_object)
{
  uint8_t buf[4];
  uint16_t upsiglim;
  uint16_t losiglim;
  bool enabled;
  int instance;
  int ret;

  for (instance = 0; (instance < mxt_get_object_instances(mxt, object_type));
       instance++) {
    ret = mxt_read_register(mxt, (uint8_t *)&buf,
                            mxt_get_object_address(mxt, object_type, instance), 1);
    if (ret)
      return ret;

    enabled = buf[0] & 0x01;

    mxt_info(mxt->ctx, "%s[%d] %s",
             mxt_get_object_name(object_type),
             instance,
             enabled ? "enabled":"disabled");


    if (enabled) {
      ret = mxt_read_register(mxt, (uint8_t *)&buf,
                              t25_addr + 2 + *touch_object * 4, 4);
      if (ret)
        return ret;

      upsiglim = (uint16_t)((buf[1] << 8u) | buf[0]);
      losiglim = (uint16_t)((buf[3] << 8u) | buf[2]);

      mxt_info(mxt->ctx, "  UPSIGLIM:%d", upsiglim);
      mxt_info(mxt->ctx, "  LOSIGLIM:%d", losiglim);
    }

    (*touch_object)++;
  }
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Print T25 limits for each enabled touch object
static int print_t25_limits(struct mxt_device *mxt, uint16_t t25_addr)
{
  int touch_object = 0;
  int ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_MULTITOUCHSCREEN_T9,
                                  &touch_object);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_MULTITOUCHSCREEN_T100,
                                  &touch_object);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_PROXKEY_T52,
                                  &touch_object);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_KEYARRAY_T15,
                                  &touch_object);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_PROXIMITY_T23,
                                  &touch_object);
  if (ret)
    return ret;

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Disable noise suppression objects
static void disable_noise_suppression(struct mxt_device *mxt)
{
  uint16_t addr;
  uint8_t disable = 0;

  addr = mxt_get_object_address(mxt, PROCG_NOISESUPPRESSION_T22, 0);
  if (addr != OBJECT_NOT_FOUND) {
    mxt_write_register(mxt, &disable, addr, 1);
  }

  addr = mxt_get_object_address(mxt, PROCG_NOISESUPPRESSION_T48, 0);
  if (addr != OBJECT_NOT_FOUND) {
    mxt_write_register(mxt, &disable, addr, 1);
  }

  addr = mxt_get_object_address(mxt, PROCG_NOISESUPPRESSION_T54, 0);
  if (addr != OBJECT_NOT_FOUND) {
    mxt_write_register(mxt, &disable, addr, 1);
  }

  addr = mxt_get_object_address(mxt, PROCG_NOISESUPPRESSION_T62, 0);
  if (addr != OBJECT_NOT_FOUND) {
    mxt_write_register(mxt, &disable, addr, 1);
  }
}

//******************************************************************************
/// \brief Run self test
int run_self_tests(struct mxt_device *mxt, uint8_t cmd)
{
  uint16_t t25_addr;
  uint8_t enable = 3;
  int ret;

  mxt_msg_reset(mxt);

  // Enable self test object & reporting
  t25_addr = mxt_get_object_address(mxt, SPT_SELFTEST_T25, 0);
  mxt_info(mxt->ctx, "Enabling self test object");
  mxt_write_register(mxt, &enable, t25_addr, 1);

  mxt_info(mxt->ctx, "Disabling noise suppression");
  disable_noise_suppression(mxt);

  ret = print_t25_limits(mxt, t25_addr);
  if (ret)
    return ret;

  switch (cmd) {
  case SELF_TEST_ANALOG:
    mxt_info(mxt->ctx, "Running Analog power test");
    break;
  case SELF_TEST_PIN_FAULT:
    mxt_info(mxt->ctx, "Running Pin fault test");
    break;
  case SELF_TEST_PIN_FAULT_2:
    mxt_info(mxt->ctx, "Running Pin fault 2 test");
    break;
  case SELF_TEST_AND_GATE:
    mxt_info(mxt->ctx, "Running AND Gate test");
    break;
  case SELF_TEST_SIGNAL_LIMIT:
    mxt_info(mxt->ctx, "Running Signal Limit test");
    break;
  case SELF_TEST_GAIN:
    mxt_info(mxt->ctx, "Running Gain test");
    break;
  case SELF_TEST_OFFSET:
    mxt_info(mxt->ctx, "Running Offset test");
    break;
  case SELF_TEST_ALL:
    mxt_info(mxt->ctx, "Running all tests");
    break;
  default:
    mxt_info(mxt->ctx, "Writing %02X to CMD register", cmd);
    break;
  }

  mxt_write_register(mxt, &cmd, t25_addr + 1, 1);

  return mxt_read_messages_sigint(mxt, T25_TIMEOUT, NULL, self_test_handle_messages);
}

//******************************************************************************
/// \brief Run self test
uint8_t self_test_menu(struct mxt_device *mxt)
{
  int self_test;
  uint8_t cmd;

  while (1) {
    cmd = 0;

    printf("Self-test menu:\n\
      Enter 1 for running Analog power test\n\
      Enter 2 for running Pin fault test\n\
      Enter 3 for running Pin fault 2 test\n\
      Enter 4 for running AND Gate test\n\
      Enter 5 for running Signal Limit test\n\
      Enter 6 for running Gain test\n\
      Enter 7 for running all the above tests\n\
      Enter 255 to get out of the self-test menu\n");

    if (scanf("%d", &self_test) != 1) {
      printf("Input error\n");
      return MXT_ERROR_BAD_INPUT;
    }

    switch(self_test) {
    case 1:
      cmd = SELF_TEST_ANALOG;
      break;
    case 2:
      cmd = SELF_TEST_PIN_FAULT;
      break;
    case 3:
      cmd = SELF_TEST_PIN_FAULT_2;
      break;
    case 4:
      cmd = SELF_TEST_AND_GATE;
      break;
    case 5:
      cmd = SELF_TEST_SIGNAL_LIMIT;
      break;
    case 6:
      cmd = SELF_TEST_GAIN;
      break;
    case 7:
      cmd = SELF_TEST_ALL;
      break;
    case 255:
      return MXT_SUCCESS;
      break;
    default:
      printf("Invalid option\n");
      break;
    }

    if (cmd)
      run_self_tests(mxt, cmd);
  }

  return MXT_SUCCESS;
}
