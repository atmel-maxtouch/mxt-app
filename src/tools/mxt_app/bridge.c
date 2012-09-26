//------------------------------------------------------------------------------
/// \file   bridge.c
/// \brief  TCP bridge functions
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
        return 0;
      else
        break;
    }
    else
    {
      LOG(LOG_DEBUG, "read returned %d (%s)", errno, strerror(errno));
      return -1;
    }
  }

  /* null-terminate the buffer */
  *str=0;

  return n;
}

//******************************************************************************
/// \brief Read MXT messages and send them to other end
static int handle_messages(int sockfd)
{
  int count, i;
  int ret;

  count = mxt_get_msg_count();

  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      snprintf(buf, BUF_SIZE, "%s\n", mxt_get_msg_string());
      ret = write(sockfd, buf, strlen(buf));
      if (ret < 0)
      {
        printf("write returned %d (%s)", errno, strerror(errno));
        return ret;
      }
    }
  }
  return 0;
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
  if (ret <= 0) {
    LOG(LOG_DEBUG, "error reading or peer closed socket");
    return -1;
  }

  if (!strcmp(buf, "")) {
    return 0;
  }

  LOG(LOG_VERBOSE, "%s", buf);

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
static int bridge(int sockfd)
{
  int ret;
  fd_set readfds;
  struct timeval tv;
  int fopts = 0;


  /* set up select timeout */
  tv.tv_sec = 0;
  tv.tv_usec = 100000; /* 0.1 seconds */

  LOG(LOG_INFO, "Connected");

  ret = mxt_msg_reset();
  if (ret)
    LOG(LOG_ERROR, "Failure to reset dmesg timestamp");

  while (1)
  {
    FD_ZERO(&readfds);
    FD_SET(sockfd, &readfds);

    ret = select(sockfd + 1, &readfds, NULL, NULL, &tv);
    if (ret == -1 && errno == EINTR) {
      LOG(LOG_DEBUG, "interrupted");
      continue;
    } else if (ret == -1) {
      ret = -errno;
      LOG(LOG_ERROR, "select returned %d (%s)", errno, strerror(errno));
      goto disconnect;
    }

    if (fcntl(sockfd, F_GETFL, &fopts) < 0) {
      ret = 0;
      goto disconnect;
    }

    if (FD_ISSET(sockfd, &readfds)) {
      ret = handle_cmd(sockfd);
      if (ret < 0)
        goto disconnect;
    }

    ret = handle_messages(sockfd);
    if (ret < 0)
      goto disconnect;
  }

disconnect:
  LOG(LOG_INFO, "Disconnected");
  return ret;
}

//******************************************************************************
/// \brief Bridge client
int mxt_socket_client(char *ip_address, uint16_t port)
{
  struct hostent *server;
  int sockfd;
  int ret;
  struct sockaddr_in serv_addr;


  server = gethostbyname(ip_address);
  if (server == NULL) {
    printf("Error, no such host\n");
    return -1;
  }

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    LOG(LOG_ERROR, "socket returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  /* Set up socket options */
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr,
      (char *)&serv_addr.sin_addr.s_addr,
      server->h_length);
  serv_addr.sin_port = htons(port);

  /* Connect */
  LOG(LOG_INFO, "Connecting to %s:%u", ip_address, port);
  ret = connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));
  if (ret < 0)
  {
    LOG(LOG_DEBUG, "connect returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  ret = bridge(sockfd);

  close(sockfd);
  return ret;
}

//******************************************************************************
/// \brief Bridge server
int mxt_socket_server(uint16_t portno)
{
  int serversock;
  int clientsock;
  int ret;
  struct sockaddr_in server_addr, client_addr;
  socklen_t sin_size = sizeof(client_addr);

  /* Create endpoint */
  serversock = socket(AF_INET, SOCK_STREAM, 0);
  if (serversock < 0)
  {
    LOG(LOG_ERROR, "socket returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  /* Set up socket options */
  bzero((char *) &server_addr, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(portno);

  /* Bind name to socket */
  ret = bind(serversock, (struct sockaddr *) &server_addr, sizeof(server_addr));
  if (ret < 0)
  {
    LOG(LOG_ERROR, "bind returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  /* Start listening */
  ret = listen(serversock, BUF_SIZE);
  if (ret < 0)
  {
    LOG(LOG_DEBUG, "listen returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  /* Wait for connection */
  LOG(LOG_INFO, "Waiting for connection");
  clientsock = accept(serversock, (struct sockaddr *) &client_addr, &sin_size);
  if (clientsock < 0)
  {
    LOG(LOG_DEBUG, "accept returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  ret = bridge(clientsock);

  close(serversock);
  return ret;
}
