//------------------------------------------------------------------------------
/// \file   jniinterface.c
/// \brief  JNI functions for libmaxtouch
/// \author Nick Dyer
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

#include <string.h>
#include <stdbool.h>

#include "com_atmel_Maxtouch_MaxtouchJni.h"
#include "jni.h"
#include "libmaxtouch/libmaxtouch.h"

//******************************************************************************
/// \brief  JNI initialisation function
jint JNI_OnLoad(JavaVM *vm, void *reserved)
{
  JNIEnv *env;

  if ( (*vm)->GetEnv(vm, (void **) &env, JNI_VERSION_1_4) )
    return JNI_ERR;

  return JNI_VERSION_1_4;
}

//******************************************************************************
/// \brief  Scan for device
/// \return device found true/false
JNIEXPORT jboolean JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_Scan
  (JNIEnv *env, jobject this)
{
  int ret;

  ret = mxt_scan();

  if (ret != 1)
  {
    return JNI_FALSE;
  }
  else
  {
    return JNI_TRUE;
  }
}

//******************************************************************************
/// \brief  Read info block
/// \return success true/false
JNIEXPORT jboolean JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_GetInfo
  (JNIEnv *env, jobject this)
{
  int ret;

  ret = mxt_get_info();

  if (ret == 0)
  {
    return JNI_TRUE;
  }
  else
  {
    return JNI_FALSE;
  }
}

//******************************************************************************
/// \brief  Read registers from MXT chip
JNIEXPORT jbyteArray JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_ReadRegister
( JNIEnv *env, jobject this, jint start_register, jint count)
{
  int ret;
  unsigned char* buf;
  jbyteArray jb = 0;

  buf = (char *)malloc(count * sizeof(char));

  if (buf == NULL)
    return NULL;

  ret = mxt_read_register(buf, start_register, count);

  if (ret == 0)
  {
    jb=(*env)->NewByteArray(env, count);
    (*env)->SetByteArrayRegion(env, jb, 0, count, buf);
  }

  free(buf);
  return jb;
}

//******************************************************************************
/// \brief  Write registers to MXT chip
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_WriteRegister
  (JNIEnv *env, jobject this, jint start_register, jbyteArray data)
{
  int ret;
  int count;
  unsigned char *buf;

  /* transfer contents of byte array into buffer */
  count=(*env)->GetArrayLength(env, data);
  buf = (unsigned char *)malloc(count * sizeof(unsigned char));

  if (buf == NULL)
    return -1;

  (*env)->GetByteArrayRegion(env, data, 0, count, buf);

  ret = mxt_write_register(buf, start_register, count);

  free(buf);
  return ret;
}

//******************************************************************************
/// \brief Enable/disable debug output
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_SetDebugEnable
  (JNIEnv *env, jobject this, jboolean debugState)
{
  bool bDebugState;

  bDebugState = debugState == JNI_TRUE ? true : false;

  return mxt_set_debug(bDebugState);
}

//******************************************************************************
/// \brief Get debug state
JNIEXPORT jboolean JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_GetDebugEnable
  (JNIEnv *end, jobject this)
{
  bool bDebugState;

  bDebugState = mxt_get_debug();

  return (bDebugState == true ? JNI_TRUE : JNI_FALSE);
}

//******************************************************************************
/// \brief Enable/disable phone touchscreen
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_SetPause
  (JNIEnv *env, jobject this, jboolean pauseState)
{
  bool bPauseState;

  bPauseState = pauseState == JNI_TRUE ? true : false;

  return mxt_set_pause(bPauseState);
}

//******************************************************************************
/// \brief Get phone touchscreen enable/disable state
JNIEXPORT jboolean JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_GetPause
  (JNIEnv *env, jobject this)
{
  bool bPauseState;

  bPauseState = mxt_get_pause();

  return (bPauseState == true ? JNI_TRUE : JNI_FALSE);
}

//******************************************************************************
/// \brief Load config file
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_LoadConfigFile
  (JNIEnv *env, jobject this, jstring filename)
{
  const char *szFilename = (*env)->GetStringUTFChars(env, filename, 0);
  int ret;

  ret = mxt_load_config_file(szFilename, false);

  (*env)->ReleaseStringUTFChars(env, filename, szFilename);

  return ret;
}

//******************************************************************************
/// \brief  Get debug messages
/// \return Array of java string objects
JNIEXPORT jobjectArray JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_GetDebugMessages
  (JNIEnv *env, jobject this)
{
  int count, i;
  jobjectArray stringarray;
  jclass stringClass;
  char *szMessage;

  count = mxt_get_debug_messages();

  // Create JNI array of strings to return
  stringClass = NULL;
  stringClass = (*env)->FindClass(env, "java/lang/String");
  stringarray= (*env)->NewObjectArray(env, count, stringClass, NULL);

  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      szMessage = (char *)mxt_retrieve_message();
      (*env)->SetObjectArrayElement(env, stringarray, i, (*env)->NewStringUTF(env, szMessage));
    }
  }

  return stringarray;
}

//******************************************************************************
/// \brief  Get location of interface in sysfs
/// \return directory path
JNIEXPORT jstring JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_GetSysfsDirectory
  (JNIEnv *env, jobject this)
{
  jclass stringClass;
  jstring sysfsLocation;

  char *szLocation;

  szLocation = (char *)sysfs_get_directory();

  stringClass = NULL;
  stringClass = (*env)->FindClass(env, "java/lang/String");
  sysfsLocation = (*env)->NewStringUTF(env, szLocation);

  return sysfsLocation;
}

//******************************************************************************
/// \brief  Backup configuration to non-volatile memory
/// \return Zero on success, or negative error
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_BackupConfig
  (JNIEnv *env, jobject this)
{
  return mxt_backup_config();
}

//******************************************************************************
/// \brief  Reset chip
/// \return Zero on success, or negative error
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_ResetChip
  (JNIEnv *env, jobject this)
{
  return mxt_reset_chip(false);
}

//******************************************************************************
/// \brief  Calibrate chip
/// \return Zero on success, or negative error
JNIEXPORT jint JNICALL Java_com_atmel_Maxtouch_MaxtouchJni_CalibrateChip
  (JNIEnv *env, jobject this)
{
  return mxt_calibrate_chip();
}
