//------------------------------------------------------------------------------
/// \file   dmesg.c
/// \brief  Functions to read kernel message buffer
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

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/klog.h>

#ifndef SYSLOG_ACTION_READ_ALL
#define SYSLOG_ACTION_READ_ALL      (3)
#endif

#ifndef SYSLOG_ACTION_SIZE_BUFFER
#define SYSLOG_ACTION_SIZE_BUFFER   (10)
#endif

#define MAX_DMESG_COUNT       (500)
#define MAX_DMESG_BUFSIZE     (10E6)

#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"
#include "sysfs_device.h"

#include "dmesg.h"

//******************************************************************************
/// Define for size of message buffer
#define BUFFERSIZE 256

//******************************************************************************
/// \brief  Linked list item structure
struct dmesg_item {
  unsigned long sec;
  unsigned long msec;
  char msg[BUFFERSIZE];
  struct dmesg_item *next;
};

//******************************************************************************
/// \brief  Add new node to linked list of dmesg items
/// \param  mxt  Maxtouch Device
/// \param  sec  Seconds value of item
/// \param  msec Milliseconds value of item
/// \param  msg  Message string data
static void dmesg_list_add(struct mxt_device *mxt, unsigned long sec,
                           unsigned long msec, char *msg)
{
  // create new node
  struct dmesg_item* new_node = (struct dmesg_item *)calloc(1, sizeof(struct dmesg_item));

  if (!new_node) return;

  strncpy(new_node->msg, msg, sizeof(new_node->msg));
  new_node->msg[sizeof(new_node->msg) - 1] = '\0';
  new_node->sec = sec;
  new_node->msec = msec;

  new_node->next = mxt->sysfs.dmesg_head;
  mxt->sysfs.dmesg_head = new_node;

  mxt->sysfs.dmesg_count++;
}

//******************************************************************************
/// \brief  Remove all items from the linked list
/// \param  mxt  Maxtouch Device
static void dmesg_list_empty(struct mxt_device *mxt)
{
  if (mxt->sysfs.dmesg_head == NULL)
    return;

  // reset
  struct dmesg_item *old_node = mxt->sysfs.dmesg_head;
  mxt->sysfs.dmesg_head = NULL;
  mxt->sysfs.dmesg_count = 0;

  // release memory
  struct dmesg_item *next_node = NULL;
  while (old_node->next != NULL) {
    next_node = old_node->next;
    free(old_node);

    old_node = next_node;
  }
  free(old_node);

  return;
}

//******************************************************************************
/// \brief  Get messages
/// \param  mxt  Maxtouch Device
/// \param  count Number of messages available
/// \param  init_timestamp Read newest dmesg line and initialise timestamp
/// \return #mxt_rc
int dmesg_get_msgs(struct mxt_device *mxt, int *count, bool init_timestamp)
{
  char msg[BUFFERSIZE];
  int ep, sp;
  int ret = MXT_SUCCESS;
  unsigned long sec, msec, lastsec = 0, lastmsec = 0;

  // Read entire kernel log buffer
  ep = klogctl(SYSLOG_ACTION_READ_ALL, mxt->sysfs.debug_msg_buf,
               mxt->sysfs.debug_msg_buf_size);
  // Return if no bytes read
  if (ep < 0) {
    mxt_warn(mxt->ctx, "klogctl error %d (%s)", errno, strerror(errno));
    ret = mxt_errno_to_rc(errno);
  } else {
    // null terminate
    mxt->sysfs.debug_msg_buf[ep] = 0;
    sp = ep;

    if (!init_timestamp)
      dmesg_list_empty(mxt);

    // Search for next new line character
    while (true) {
      sp--;
      while (sp >= 0 && *(mxt->sysfs.debug_msg_buf + sp) != '\n')
        sp--;

      if (sp <= 0)
        break;

      // Try to parse dmesg line
      if (sscanf(mxt->sysfs.debug_msg_buf+sp+1, "< %*c>[ %lu.%06lu] %255[^\n]",
                 &sec, &msec, msg) == 3) {
        if (init_timestamp) {
          mxt->sysfs.timestamp = sec;
          mxt->sysfs.mtimestamp = msec;
          mxt_verb(mxt->ctx, "%s - init [%5lu.%06lu]", __func__, sec, msec);
          break;
        }

        // Store time of last message in buffer
        if (lastsec == 0) {
          lastsec = sec;
          lastmsec = msec;
        }

        // Only 500 at a time, otherwise we overrun JNI reference limit.
        // Timestamp must be greater than previous messages, slightly
        // complicated by seconds and microseconds
        if ((mxt->sysfs.dmesg_count > MAX_DMESG_COUNT) ||
            (sec == mxt->sysfs.timestamp && msec <= mxt->sysfs.mtimestamp) ||
            (sec < mxt->sysfs.timestamp)) {
          mxt->sysfs.timestamp = lastsec;
          mxt->sysfs.mtimestamp = lastmsec;
          break;
        }

        char* msgptr;
        msg[sizeof(msg) - 1] = '\0';
        msgptr = strstr(msg, "MXT MSG");
        if (msgptr)
          dmesg_list_add(mxt, sec, msec, msgptr);
      }
    }

    if (!init_timestamp) {
      *count = mxt->sysfs.dmesg_count;
      mxt->sysfs.dmesg_ptr = mxt->sysfs.dmesg_head;
    }
  }

  return ret;
}


//******************************************************************************
/// \brief  Update the timestamp from the klog messages
/// \param  mxt  Maxtouch Device
/// \return #mxt_rc
static int dmesg_update_timestamp(struct mxt_device *mxt)
{
  return dmesg_get_msgs(mxt, NULL, true);
}

//******************************************************************************
/// \brief  Get the next debug message
/// \param  mxt  Maxtouch Device
/// \return Message string
char *dmesg_get_msg_string(struct mxt_device *mxt)
{
  char *msg_string;

  msg_string = mxt->sysfs.dmesg_ptr->msg;

  if (mxt->sysfs.dmesg_ptr != NULL) {
    // Get next record in linked list
    mxt->sysfs.dmesg_ptr = mxt->sysfs.dmesg_ptr->next;
  }

  if (!strncasecmp("MXT MSG:FF", msg_string, 10))
    return NULL;

  return msg_string;
}

//******************************************************************************
/// \brief  Get debug message as byte array
/// \param  mxt  Maxtouch Device
/// \param  buf  Pointer to buffer
/// \param  buflen  Length of buffer
/// \param  count number of bytes read
/// \return #mxt_rc
int dmesg_get_msg_bytes(struct mxt_device *mxt, unsigned char *buf,
                        size_t buflen, int *count)
{
  unsigned int bufidx = 0;
  int offset;
  char *message;

  message = dmesg_get_msg_string(mxt);

  if (!message)
    return MXT_ERROR_NO_MESSAGE;

  /* Check message begins with prefix */
  if (strncmp(MSG_PREFIX, message, strlen(MSG_PREFIX))) {
    *count = 0;
    return MXT_SUCCESS;
  }

  message += strlen(MSG_PREFIX);

  while (1 == sscanf(message, "%hhx%n", buf + bufidx, &offset)) {
    message += offset;
    bufidx++;

    if (bufidx >= buflen)
      break;
  }

  *count = bufidx;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief  Reset the timestamp counter
int dmesg_reset(struct mxt_device *mxt)
{
  int ret;

  mxt->sysfs.dmesg_head = NULL;
  mxt->sysfs.mtimestamp = 0;
  mxt->sysfs.timestamp  = 0;
  ret = dmesg_update_timestamp(mxt);

  return ret;
}

//******************************************************************************
/// \brief Allocate kernel log buffer
int dmesg_alloc_buffer(struct mxt_device *mxt)
{
  int size;

  size = klogctl(SYSLOG_ACTION_SIZE_BUFFER, NULL, 0);
  if (size == -1) {
    mxt_err(mxt->ctx, "klogctl error %d (%s)", errno, strerror(errno));
    return mxt_errno_to_rc(errno);
  }

  if (size > MAX_DMESG_BUFSIZE)
    size = MAX_DMESG_BUFSIZE;

  mxt_dbg(mxt->ctx, "sysfs.debug_msg_buf_size: %d bytes", size);

  // Allocate buffer space
  mxt->sysfs.debug_msg_buf = (char *)calloc(size, sizeof(char));
  if (mxt->sysfs.debug_msg_buf == NULL) {
    mxt_err(mxt->ctx, "Error allocating debug_msg_buf %d bytes", size);
    return mxt_errno_to_rc(errno);
  }

  mxt->sysfs.debug_msg_buf_size = size;
  return MXT_SUCCESS;
}

//******************************************************************************
/// \brief Allocate kernel log buffer
void dmesg_free_buffer(struct mxt_device *mxt)
{
  free(mxt->sysfs.debug_msg_buf);
  mxt->sysfs.debug_msg_buf = NULL;
}
