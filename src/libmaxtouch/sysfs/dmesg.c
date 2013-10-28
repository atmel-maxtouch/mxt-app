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

#ifndef KLOG_READ_ALL
#define KLOG_READ_ALL 3
#endif

#include "libmaxtouch/log.h"
#include "libmaxtouch/libmaxtouch.h"
#include "sysinfo.h"
#include "sysfs_device.h"

#include "dmesg.h"

int dmesg_count = 0;
dmesg_item *dmesg_head = NULL;
dmesg_item *dmesg_ptr = NULL;
dmesg_item *dmesg_tail = NULL;

unsigned long timestamp = 0;
unsigned long mtimestamp = 0;

//******************************************************************************
/// \brief  Add new node to linked list of dmesg items
static void dmesg_list_add(unsigned long sec, unsigned long msec, char *msg)
{
  // create new node
  dmesg_item* new_node = (dmesg_item *)calloc(1, sizeof(dmesg_item));

  if (!new_node) return;

  strncpy(new_node->msg, msg, sizeof(new_node->msg));
  new_node->msg[sizeof(new_node->msg) - 1] = '\0';
  new_node->sec = sec;
  new_node->msec = msec;
  new_node->next = NULL;

  // find node
  if(dmesg_head != NULL)
  {
    dmesg_tail->next = new_node;
    dmesg_tail = new_node;
  }
  else
  {
    dmesg_head = new_node;
    dmesg_tail = new_node;
  }

  dmesg_count++;
}

//******************************************************************************
/// \brief  Remove all items from the linked list
static void dmesg_list_empty(void)
{
  if (dmesg_head == NULL)
    return;

  // reset
  dmesg_item *old_node = dmesg_head;
  dmesg_head = NULL;
  dmesg_tail = NULL;
  dmesg_count = 0;

  // release memory
  dmesg_item *next_node = NULL;
  while (old_node->next != NULL)
  {
    next_node = old_node->next;
    free(old_node);

    old_node = next_node;
  }
  free(old_node);

  return;
}

//******************************************************************************
/// \brief  Get debug messages
/// \return Number of messages
int sysfs_get_msg_count(void)
{
  char buffer[KLOG_BUF_LEN + 1];
  char msg[BUFFERSIZE];
  char *msgptr;
  char *lineptr;
  int op, sp, ep;
  unsigned long sec, msec;

  // Read entire kernel log buffer
  op = KLOG_READ_ALL;
  op = klogctl(op, buffer, KLOG_BUF_LEN);
  // Return if no bytes read
  if (op < 0) {
    LOG(LOG_INFO, "klogctl error %d (%s)", errno, strerror(errno));
    return -1;
  }

  buffer[op] = 0;
  sp = 0;

  dmesg_list_empty();

  // Search for next new line character
  while((lineptr = strstr(buffer+sp, "\n" )) != NULL)
  {
    // Set up start point for next line
    ep = lineptr - buffer - sp;

    if(ep > BUFFERSIZE)
      ep = BUFFERSIZE;

    sp = sp + ep + 1;

    // Try to parse dmesg line
    if (sscanf(buffer+sp, "<%*c>[%lu.%06lu%*s %255[^\n]",
        &sec, &msec, (char*)&msg) == 3)
    {
      // Timestamp must be greater than previous messages, slightly complicated by
      // seconds and microseconds
      if ((sec > timestamp) || ((sec == timestamp) && (msec > mtimestamp)))
      {
        msg[sizeof(msg) - 1] = '\0';
        msgptr = strstr(msg, "MXT MSG");

        if (msgptr)
        {
            dmesg_list_add(sec, msec, msgptr);
        }
      }
    }

    // Only 500 at a time, otherwise we overrun JNI reference limit.
    if (dmesg_count > 500)
      break;

    // end of buffer
    if(sp >= op)
      break;
  }

  // Set the timestamp to the last message
  timestamp = sec;
  mtimestamp = msec;

  // Reset pointer to first record
  dmesg_ptr = dmesg_head;

  return dmesg_count;
}

//******************************************************************************
/// \brief  Get the next debug message
/// \return Message string
char *sysfs_get_msg_string()
{
  char *msg_string;

  msg_string = dmesg_ptr->msg;

  if (dmesg_ptr != NULL)
  {
    // Get next record in linked list
    dmesg_ptr = dmesg_ptr->next;
  }

  return msg_string;
}

//******************************************************************************
/// \brief  Get debug message as byte array
/// \param  buf  Pointer to buffer
/// \param  buflen  Length of buffer
/// \return number of bytes read
int sysfs_get_msg_bytes(unsigned char *buf, size_t buflen)
{
   unsigned int bufidx = 0;
   int offset;
   char *message;

   message = sysfs_get_msg_string();

   /* Check message begins with prefix */
   if (strncmp(MSG_PREFIX, message, strlen(MSG_PREFIX)))
   {
      return 0;
   }

   message += strlen(MSG_PREFIX);

   while (1 == sscanf(message, "%hhx%n", buf + bufidx, &offset))
   {
      message += offset;
      bufidx++;

      if (bufidx >= buflen)
         break;
   }

   return bufidx;
}

//******************************************************************************
/// \brief  Reset the timestamp counter
int sysfs_msg_reset()
{
  int ret;

  mtimestamp = 0;
  ret = get_uptime(&timestamp);

  return ret;
}
