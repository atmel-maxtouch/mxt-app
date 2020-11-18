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

#define SELFTEST_TIMEOUT   10

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
        mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM=%d, X%d", msg[2], msg[3] - 1);
      else if (msg[4] > 0)
        mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM=%d, Y%d", msg[2], msg[4] - 1);

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
  } else if (object_type == SPT_SELFTESTCONTROL_T10) {
  	
  	  mxt_info(mxt->ctx, "Test Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", msg[1], msg[2], msg[3], msg[4], msg[5]);
  	  
  	  switch (msg[1]) {
  	  case OND_TEST_ALL_PASS:
  	    mxt_info(mxt->ctx, "PASS: All On-Demand tests have passed");
  	    ret = MXT_SUCCESS;
  	    break;
  	  case POST_TEST_ALL_PASS:
  	  	mxt_info(mxt->ctx, "PASS: All POST tests have passed");
  	  	ret = MXT_SUCCESS;
  	  	break;
  	  case BIST_TEST_ALL_PASS:
  	  	mxt_info(mxt->ctx, "PASS: All BIST tests have passed");
  	  	ret = MXT_SUCCESS;
  	  	break;
  	  case BIST_TEST_OVERRUN:
  	  	mxt_info(mxt->ctx, "OVERRUN: BIST test cycle overrun");
  	  	ret = MXT_BIST_OVERRUN;
  	  	break;
  	  case OND_TEST_INVALID:
  	    mxt_err(mxt->ctx, "FAIL: Invalid or unsupported test command");
  	    ret = MXT_ERROR_NOT_SUPPORTED;
  	    break;
  	  case POST_TEST_FAILED:
  	  case BIST_TEST_FAILED:
  	  case OND_TEST_FAILED:

  	    if (msg[1] == POST_TEST_FAILED) {
  	  	  mxt_err(mxt->ctx, "FAIL: A POST test failure was detected");
  	    } else if (BIST_TEST_FAILED) {
  	  	  mxt_err(mxt->ctx, "FAIL: A BIST test failure was detected");
  	    } else {
  	    	mxt_err(mxt->ctx, "FAIL: On-Demand test failure was detected");
  	    }

  	    switch(msg[2]){
  	    case CLOCK_FAILURE:
  	  	  mxt_err(mxt->ctx, "FAIL: Clock related failure occurred");
  	  	  ret = MXT_CLOCK_FAILURE;
  	  	  break;
  	    case FLASH_MEM_FAILURE:
  	  	  mxt_err(mxt->ctx, "Flash memory related failure occurred");
  	  	  ret = MXT_FLASH_MEM_FAILURE;
  	  	  break;
  	    case RAM_MEM_FAILURE:
  	  	  mxt_err(mxt->ctx, "RAM memory related failure occurred");
  	  	  ret = MXT_MEM_RAM_FAILURE;
  	  	  break;
  	    case CTE_TEST_FAILURE:
  	  	  mxt_err(mxt->ctx, "CTE related failure occurred");
  	  	  ret = MXT_CTE_TEST_FAILURE;
  	  	  break;
  	    case SIGNAL_LIMIT_FAILURE:
  	  	  mxt_err(mxt->ctx, "Signal Limit relatd failure occurred");
  	  	  mxt_err(mxt->ctx, "T%d instance[%d] failed signal limit test", msg[2], msg[3]);
  	  	  ret = MXT_ERROR_SELF_TEST_SIGNAL_LIMIT;
  	  	  break;
  	    case POWER_TEST_FAILURE:
  	  	  mxt_err(mxt->ctx, "Power related failure occurred");
  	  	  ret = MXT_POWER_FAILURE;
  	  	  break;
  	    case PIN_FAULT_FAILURE:
  	  	  mxt_err(mxt->ctx, "Pin Fault related failure occurred");
  	  	  ret = MXT_ERROR_SELF_TEST_PIN_FAULT;
  	  	  
  	  	  if (msg[4] == 0 && msg[5] == 0)
            mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM = %d driven shield line failed");
      	  else if (msg[4] > 0)
        	mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM = %d X%d", msg[3], msg[4] - 1);
      	  else if (msg[5] > 0)
        	mxt_err(mxt->ctx, "FAIL: Pin fault SEQ_NUM = %d Y%d", msg[3], msg[5] - 1);

  	  	  break;
  	  
  	  	default:
  	  	  mxt_err(mxt->ctx, "Test Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x", msg[1], msg[2], msg[3], msg[4], msg[5]);

	  	}

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
static int print_touch_object_limits(struct mxt_device *mxt, uint16_t selftest_addr,
                                     uint16_t object_type, int *touch_object, bool selftest_type)
{
  uint8_t buf[8];
  uint16_t upsiglim;
  uint16_t losiglim;
  uint16_t rangesiglim;
  uint8_t singlex_gain;
  uint8_t dualx_gain;
  bool enabled;
  int instance;
  int ret;
  uint16_t t100_addr;

  if (selftest_type == 0){
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
                              selftest_addr + 2 + *touch_object * 4, 4);
      if (ret)
        return ret;

      upsiglim = (uint16_t)((buf[1] << 8u) | buf[0]);
      losiglim = (uint16_t)((buf[3] << 8u) | buf[2]);

      mxt_info(mxt->ctx, "  UPSIGLIM:%d", upsiglim);
      mxt_info(mxt->ctx, "  LOSIGLIM:%d", losiglim);
    }

    (*touch_object)++;
  }
} else {

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
                              selftest_addr + 1, 8); /* skip reserved byte */
      if (ret)
        return ret;

      singlex_gain = buf[0];
      dualx_gain = buf[1];
      losiglim = (uint16_t)((buf[3] << 8u) | buf[2]);
      upsiglim = (uint16_t)((buf[5] << 8u) | buf[4]);
      rangesiglim = (uint16_t)((buf[7] << 8u) | buf[6]);

      mxt_info(mxt->ctx, "  SingleX GAIN:%d", singlex_gain);
      mxt_info(mxt->ctx, "  DualX GAIN:%d", dualx_gain);
      mxt_info(mxt->ctx, "  UPSIGLIM:%d", upsiglim);
      mxt_info(mxt->ctx, "  LOSIGLIM:%d", losiglim);
      mxt_info(mxt->ctx, "  RANGESIGLIM: %d\n", rangesiglim);

	}
  
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
                                  &touch_object, 0);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_MULTITOUCHSCREEN_T100,
                                  &touch_object, 0);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_PROXKEY_T52,
                                  &touch_object, 0);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_KEYARRAY_T15,
                                  &touch_object, 0);
  if (ret)
    return ret;

  ret = print_touch_object_limits(mxt, t25_addr, TOUCH_PROXIMITY_T23,
                                  &touch_object, 0);
  if (ret)
    return ret;

  return MXT_SUCCESS;
}


//******************************************************************************
/// \brief Print T25 limits for each enabled touch object
static int print_t12_limits(struct mxt_device *mxt, uint16_t t12_addr)
{
  int touch_object = 0;
  int ret;

  ret = print_touch_object_limits(mxt, t12_addr, TOUCH_MULTITOUCHSCREEN_T100,
                                  &touch_object, 1);
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
int run_self_tests(struct mxt_device *mxt, uint8_t cmd, bool type)
{
  uint16_t t25_addr;
  uint16_t t10_addr;
  uint16_t t12_addr;
  uint8_t enable = 3;
  uint8_t disable_t10 = 0;
  int ret;

 if (type == 0){

    t25_addr = mxt_get_object_address(mxt, SPT_SELFTEST_T25, 0);
    
    if (t25_addr == OBJECT_NOT_FOUND) {
      mxt_info(mxt->ctx, "T25 Self Test Object not Found ... Exiting\n");
      return MXT_SUCCESS;
    }

   // Enable self test object & reporting

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

   mxt_msg_reset(mxt);

   mxt_dump_messages(mxt);

   mxt_write_register(mxt, &cmd, t25_addr + 1, 1);
 } else {

   // Enable self test object & reporting
    t10_addr = mxt_get_object_address(mxt, SPT_SELFTESTCONTROL_T10, 0);
    
    if (t10_addr == OBJECT_NOT_FOUND) {
      mxt_info(mxt->ctx, "T10 Self Test Object not Found ... Exiting\n");
      return MXT_SUCCESS;
    }

   t12_addr = mxt_get_object_address(mxt, SPT_SELFTESTSIGLIMIT_T12,  0);
   t10_addr = mxt_get_object_address(mxt, SPT_SELFTESTCONTROL_T10,  0);
 	
   mxt_info(mxt->ctx, "Enabling T10 self test object");
   mxt_info(mxt->ctx, "Disable POST and BIST periodic tests");
   mxt_write_register(mxt, &disable_t10, t10_addr, 1);

   usleep(20000);	//* Delay 20ms */

   mxt_info(mxt->ctx, "Enable T10 Reporting");
   mxt_write_register(mxt, &enable, t10_addr, 1);

   mxt_info(mxt->ctx, "Disabling noise suppression\n");
   disable_noise_suppression(mxt);

   ret = print_t12_limits(mxt, t12_addr);
   if (ret)
     return ret;

   switch (cmd) {
   case OND_POWER_TEST:
     mxt_info(mxt->ctx, "Running power test");
     break;
   case OND_PIN_FAULT_TEST:
     mxt_info(mxt->ctx, "Running Pin fault test");
     break;
   case OND_SIGNAL_LIMIT_TEST:
     mxt_info(mxt->ctx, "Running signal limit test");
     break;
   case OND_RUN_ALL_TEST:
     mxt_info(mxt->ctx, "Running all tests");
     break;
   default:
     mxt_info(mxt->ctx, "Writing %02X to CMD register", cmd);
     break;
   }

  mxt_msg_reset(mxt);

  mxt_dump_messages(mxt);

  mxt_write_register(mxt, &cmd, t10_addr + 1, 1);

 }

   return mxt_read_messages_sigint(mxt, SELFTEST_TIMEOUT, NULL, self_test_handle_messages);
}

//******************************************************************************
/// \brief Run self test
uint8_t self_test_t25_menu(struct mxt_device *mxt)
{
  int self_test;
  uint8_t cmd;
  int err;

  while (1) {
    cmd = 0;

    printf("\nSelf-test menu:\n\
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
      run_self_tests(mxt, cmd, 0);

    if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {

      err = debugfs_set_irq(mxt, true);

      if (err)
        mxt_dbg(mxt->ctx, "Could not disable IRQ");
    }
  }

  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Run self test
uint8_t self_test_t10_menu(struct mxt_device *mxt)
{
  int self_test;
  uint8_t cmd;
  int err;
  uint16_t t10_addr = 0x0000;


    t10_addr = mxt_get_object_address(mxt, SPT_SELFTESTCONTROL_T10, 0);
    
    if (t10_addr == OBJECT_NOT_FOUND) {
      mxt_info(mxt->ctx, "T10 Self Test Object not Found ... Exiting\n");
      return MXT_SUCCESS;
    } 

  while (1) {
    cmd = 0;

    printf("\nOn Demand Self Test menu:\n\
      Enter 1 for running Power test\n\
      Enter 2 for running Pin fault test\n\
      Enter 3 for running Signal Limit test\n\
      Enter 7 for running all the above tests\n\
      Enter 255 to get out of the self-test menu\n");

    if (scanf("%d", &self_test) != 1) {
      printf("Input error\n");
      return MXT_ERROR_BAD_INPUT;
    }

    switch(self_test) {
    case 1:
      cmd = OND_POWER_TEST;
      break;
    case 2:
      cmd = OND_PIN_FAULT_TEST;
      break;
    case 3:
      cmd = OND_SIGNAL_LIMIT_TEST;
      break;
    case 7:
      cmd = OND_RUN_ALL_TEST;
      break;
    case 255:
      return MXT_SUCCESS;
      break;
    default:
      printf("Invalid option\n");
      break;
    }

    if (cmd)
      run_self_tests(mxt, cmd, 1);

  	/* Reset chip to normal config values */

  	 err = mxt_reset_chip(mxt, false, 0);
      
     if (err) {
       mxt_err(mxt->ctx, "Error resetting");
     } else {
         mxt_info(mxt->ctx, "Chip reset");
     }

    if (mxt->conn->type == E_I2C_DEV && mxt->debug_fs.enabled == true) {

      err = debugfs_set_irq(mxt, true);

      if (err)
        mxt_dbg(mxt->ctx, "Could not disable IRQ");
    }
  }

  return MXT_SUCCESS;
}


//******************************************************************************
/// \brief Run self test
uint8_t self_test_main_menu(struct mxt_device *mxt)
{
  int self_test;
  uint8_t cmd;
  int err;
  uint16_t t25_addr = 0x0000;

  while (1) {
    cmd = 0;

 printf("\nSelf-test main menu:\n\
      Enter 1 for T25 Self test menu\n\
      Enter 2 for T10 On-Demand test menu\n\
      Enter 255 to get out of the self-test menu\n");

    if (scanf("%d", &self_test) != 1) {
      printf("Input error\n");
      return MXT_ERROR_BAD_INPUT;
    }

    switch(self_test) {
    case 1:
      self_test_t25_menu(mxt);
      return MXT_SUCCESS;
      break;
    case 2:
      self_test_t10_menu(mxt);
      break;
    case 255:
      return MXT_SUCCESS;
      break;
    default:
      printf("Invalid option\n");
      break;
    }
  }

  return MXT_SUCCESS;
}

