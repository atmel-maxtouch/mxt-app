//------------------------------------------------------------------------------
/// \file   bridge.c
/// \brief  Connect to the wifi bridge client
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/dmesg.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"

#include "bridge.h"

#define BUF_SIZE 1024

char buf[BUF_SIZE];
char hexbuf[BUF_SIZE];

//******************************************************************************
/// \brief Read command on a single line
static int readline(int fd, char *str, int maxlen)
{
  int n;
  int readcount;
  char c;

  for (n = 1; n < maxlen; n++) {
    /* read 1 character at a time */
    readcount = read(fd, &c, 1);
    if (readcount == 1)
    {
      if ((c == '\n') || (c == '\r'))
        break;
      *str = c;
      str++;
    }
    else if (readcount == 0)
    {
      if (n == 1)
        return (0);
      else
        break;
    }
    else
      return (-1);
  }

  /* null-terminate the buffer */
  *str=0;

  return (n);
}

//******************************************************************************
/// \brief Read MXT messages and send them to other end
static void handle_messages(int sockfd)
{
  int count, i;

  count = mxt_get_debug_messages();

  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      snprintf(buf, BUF_SIZE, "%s\n", (char *)mxt_retrieve_message());
      if (write(sockfd, buf, strlen(buf)) < 0)
      {
        printf("write error\n");
        return;
      }
    }
  }
}

//******************************************************************************
/// \brief Read and deal with incoming command
static int handle_cmd(int sockfd)
{
  int ret;
  int i;
  uint16_t address;
  uint8_t count;
  unsigned char databuf[BUF_SIZE];

  ret = readline(sockfd, buf, BUF_SIZE);
  if (ret < 0) {
    LOG(LOG_ERROR, "error reading");
    return (ret);
  }

  if (!strcmp(buf, "")) {
    return 0;
  }

  LOG(LOG_INFO, "%s", buf);

  if (!strcmp(buf, "SAT")) {
    printf("Server attached\n");
  } else if (!strcmp(buf, "SDT")) {
    printf("Server detached\n");
  } else if (sscanf(buf, "REA %hu %hhu", &address, &count) == 2) {
    ret = mxt_read_register(&databuf[0], address, count);
    if (ret < 0) {
      snprintf(buf, BUF_SIZE, "RRP ERR");
    } else {
      snprintf(buf, BUF_SIZE, "RRP ");

      for (i = 0; i < count; i++) {
        sprintf(hexbuf, "%02X", databuf[i]);
        strncat(buf, hexbuf, BUF_SIZE - 1);
      }
      buf[BUF_SIZE - 1] = '\0';
    }
    LOG(LOG_INFO, "%s", buf);
    strncat(buf, "\n", BUF_SIZE);
    ret = write(sockfd, buf, strlen(buf));
    if (ret < 0) {
      printf("write error\n");
      return -1;
    }
  } else if (sscanf(buf, "WRI %hd %s", &address, (char *)&hexbuf) == 2) {
    ret = mxt_convert_hex(&hexbuf[0], &databuf[0], &count, BUF_SIZE);
    if (ret < 0) {
      snprintf(buf, BUF_SIZE, "WRP ERR");
    } else {
      ret = mxt_write_register(&databuf[0], address, count);
      if (ret < 0) {
        snprintf(buf, BUF_SIZE, "WRP ERR");
      } else {
        snprintf(buf, BUF_SIZE, "WRP OK");
      }
    }

    LOG(LOG_INFO, "%s", buf);
    strncat(buf, "\n", BUF_SIZE);
    ret = write(sockfd, buf, strlen(buf));
    if (ret < 0) {
      printf("write error\n");
      return -1;
    }
  } else {
    printf("Unrecognised cmd %s\n", buf);
    return -1;
  }

  return 0;
}

//******************************************************************************
/// \brief Main bridge function to handle a single connection
static int bridge(struct hostent *server, uint16_t portno)
{
  int sockfd;
  int ret;
  struct sockaddr_in serv_addr;
  fd_set readfds;
  struct timeval tv;
  int fopts = 0;

  /* Open socket */
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    LOG(LOG_ERROR, "ERROR opening socket");
    return -1;
  }

  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
      (char *)&serv_addr.sin_addr.s_addr,
      server->h_length);
  serv_addr.sin_port = htons(portno);
  if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
    LOG(LOG_ERROR, "ERROR connecting");

  /* set up select timeout */
  tv.tv_sec = 0;
  tv.tv_usec = 100000; /* 0.1 seconds */

  while (1)
  {
    FD_SET(sockfd, &readfds);

    ret = select(sockfd + 1, &readfds, NULL, NULL, &tv);
    if (ret == -1 && errno == EINTR) {
      LOG(LOG_DEBUG, "interrupted");
      continue;
    } else if (ret == -1) {
      LOG(LOG_ERROR, "select error");
      goto close;
    }

    if (fcntl(sockfd, F_GETFL, &fopts) < 0) {
      LOG(LOG_DEBUG, "Closing");
      goto close;
    }

    if (FD_ISSET(sockfd, &readfds)) {
      ret = handle_cmd(sockfd);
      if (ret < 0) {
        LOG(LOG_DEBUG, "handle_cmd failure");
        goto close;
      }
    }

    handle_messages(sockfd);
  }

close:
  close(sockfd);
  return ret;
}

//******************************************************************************
/// \brief Bridge client
int mxt_socket_client(char *ip_address, uint16_t port)
{
  struct hostent *server;
  int ret;

  printf("Bridge tool for Atmel maXTouch chips version: %s\n\n",
      __GIT_VERSION);

  server = gethostbyname(ip_address);
  if (server == NULL) {
    printf("Error, no such host\n");
    return -1;
  }

  ret = mxt_dmesg_reset();
  if (ret)
    LOG(LOG_ERROR, "Failure to reset dmesg timestamp");

  ret = bridge(server, port);
  if (ret < 0)
    LOG(LOG_ERROR, "Failure in bridge, ret %d", ret);

  return 0;
}
