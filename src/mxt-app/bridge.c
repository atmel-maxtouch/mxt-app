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
#include <netinet/tcp.h>
#include <netdb.h>
#include <inttypes.h>
#include <poll.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"

#include "mxt_app.h"
#include "buffer.h"

#define MAX_LINESIZE 10000

//******************************************************************************
/// \brief Read command on a single line
static int readline(struct mxt_device *mxt, int fd, struct mxt_buffer *linebuf)
{
  int ret;
  int n;
  int readcount;
  char c;

  mxt_buf_reset(linebuf);

  for (n = 1; n < MAX_LINESIZE; n++) {
    /* read 1 character at a time */
    readcount = read(fd, &c, 1);
    if (readcount == 1)
    {
      if ((c == '\n') || (c == '\r'))
        break;

      ret = mxt_buf_add(linebuf, c);
      if (ret < 0)
        return ret;
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
      mxt_dbg(mxt->ctx, "read returned %d (%s)", errno, strerror(errno));
      return -1;
    }
  }

  /* null-terminate the buffer */
  ret = mxt_buf_add(linebuf, '\0');
  if (ret < 0)
    return ret;

  return n;
}

//******************************************************************************
/// \brief Read MXT messages and send them to other end
static int handle_messages(struct mxt_device *mxt, int sockfd)
{
  int count, i;
  char *msg;
  int ret;

  count = mxt_get_msg_count(mxt);

  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      msg = mxt_get_msg_string(mxt);
      if (msg == NULL) {
        mxt_warn(mxt->ctx, "failed to retrieve message");
        return 0;
      }

      ret = write(sockfd, msg, strlen(msg));
      if (ret < 0)
      {
        mxt_err(mxt->ctx, "write returned %d (%s)", errno, strerror(errno));
        return ret;
      }

      ret = write(sockfd, "\n", 1);
      if (ret < 0)
      {
        mxt_err(mxt->ctx, "write returned %d (%s)", errno, strerror(errno));
        return ret;
      }

    }
  }
  return 0;
}

//******************************************************************************
/// \brief Handle bridge read
static int bridge_rea_cmd(struct mxt_device *mxt, int sockfd,
                          uint16_t address, uint16_t count)
{
  int ret;
  uint8_t *databuf;
  char *response;
  const char * const PREFIX = "RRP ";
  size_t response_len;
  int i;

  databuf = calloc(count, sizeof(uint8_t));
  if (!databuf)
  {
    mxt_err(mxt->ctx, "Failed to allocate memory");
    return -1;
  }

  /* Allow for newline/null byte */
  response_len = strlen(PREFIX) + count*2 + 1;
  response = calloc(response_len, sizeof(uint8_t));
  if (!response)
  {
    mxt_err(mxt->ctx, "Failed to allocate memory");
    ret = -1;
    goto free_databuf;
  }

  strcpy(response, PREFIX);
  mxt_info(mxt->ctx, "reading %d %d", address, count);
  ret = mxt_read_register(mxt, databuf, address, count);
  if (ret < 0) {
    mxt_warn(mxt->ctx, "RRP ERR");
    strcpy(response + strlen(PREFIX), "ERR\n");
    response_len = strlen(response);
  } else {
    for (i = 0; i < count; i++) {
      sprintf(response + strlen(PREFIX) + i*2, "%02X", databuf[i]);
    }
    mxt_info(mxt->ctx, "%s", response);
    response[response_len - 1] = '\n';
  }

  ret = write(sockfd, response, response_len);
  if (ret < 0) {
    mxt_err(mxt->ctx, "Socket write error %d (%s)", errno, strerror(errno));
    goto free;
  }

free:
  free(response);
free_databuf:
  free(databuf);
  return ret;
}

//******************************************************************************
/// \brief Handle bridge write
static int bridge_wri_cmd(struct mxt_device *mxt, int sockfd, uint16_t address,
                          char *hex, uint16_t bytes)
{
  int ret;
  uint16_t count;
  const char * const FAIL = "WRP ERR\n";
  const char * const PASS = "WRP OK\n";
  const char *response;
  uint8_t *databuf;

  databuf = calloc(bytes, sizeof(uint8_t));
  if (!databuf)
  {
    mxt_err(mxt->ctx, "Failed to allocate memory");
    return -1;
  }

  ret = mxt_convert_hex(hex, databuf, &count, bytes);
  if (ret < 0) {
    response = FAIL;
  } else {
    ret = mxt_write_register(mxt, databuf, address, count);
    if (ret < 0) {
      mxt_verb(mxt->ctx, "WRI OK");
      response = FAIL;
    } else {
      response = PASS;
    }
  }

  ret = write(sockfd, response, strlen(response));
  if (ret < 0) {
    mxt_err(mxt->ctx, "Socket write error %d (%s)", errno, strerror(errno));
  }

  free(databuf);
  return ret;
}

//******************************************************************************
/// \brief Read and deal with incoming command
static int handle_cmd(struct mxt_device *mxt, int sockfd)
{
  int ret;
  uint16_t address;
  uint16_t count;
  struct mxt_buffer linebuf;
  char *line;
  int offset;

  ret = mxt_buf_init(&linebuf);
  if (ret)
    return ret;

  ret = readline(mxt, sockfd, &linebuf);
  if (ret <= 0) {
    mxt_dbg(mxt->ctx, "error reading or peer closed socket");
    ret = -1;
    goto free;
  }

  line = (char *)linebuf.data;
  if (strlen(line) == 0) {
    ret = 0;
    goto free;
  }

  mxt_verb(mxt->ctx, "%s", line);

  if (!strcmp(line, "SAT")) {
    mxt_info(mxt->ctx, "Server attached");
    ret = 0;
  } else if (!strcmp(line, "SDT")) {
    mxt_info(mxt->ctx, "Server detached");
    ret = 0;
  } else if (sscanf(line, "REA %" SCNu16 " %" SCNu16, &address, &count) == 2) {
    ret = bridge_rea_cmd(mxt, sockfd, address, count);
  } else if (sscanf(line, "WRI %" SCNu16 "%n", &address, &offset) == 1) {
    /* skip space */
    offset += 1;

    ret = bridge_wri_cmd(mxt, sockfd, address,
                         line + offset,
                         strlen(line) - offset);
  } else {
    mxt_info(mxt->ctx, "Unrecognised cmd");
    ret = 0;
  }

free:
  mxt_buf_free(&linebuf);
  return ret;
}

//******************************************************************************
/// \brief Main bridge function to handle a single connection
static int bridge(struct mxt_device *mxt, int sockfd)
{
  int ret;
  struct pollfd fds[2];
  int fopts = 0;
  int debug_ng_fd = 0;
  int numfds = 1;
  int timeout;

  mxt_info(mxt->ctx, "Connected");

  ret = mxt_msg_reset(mxt);
  if (ret)
    mxt_err(mxt->ctx, "Failure to reset msgs");

  fds[0].fd = sockfd;
  fds[0].events = POLLIN | POLLERR;

  while (1)
  {
    debug_ng_fd = mxt_get_msg_poll_fd(mxt);
    if (debug_ng_fd)
    {
      fds[1].fd = debug_ng_fd;
      fds[1].events = POLLPRI;
      numfds = 2;
      timeout = -1;
    } else {
      timeout = 100; // milliseconds
    }

    ret = poll(fds, numfds, timeout);
    if (ret == -1 && errno == EINTR) {
      mxt_dbg(mxt->ctx, "interrupted");
      continue;
    } else if (ret == -1) {
      ret = -errno;
      mxt_err(mxt->ctx, "poll returned %d (%s)", errno, strerror(errno));
      goto disconnect;
    }

    /* Detect socket disconnect */
    if (fcntl(sockfd, F_GETFL, &fopts) < 0) {
      ret = 0;
      goto disconnect;
    }

    if (fds[0].revents) {
      ret = handle_cmd(mxt, sockfd);
      if (ret < 0)
        goto disconnect;
    }

    /* If timeout or msg poll fd event */
    if (ret == 0 || fds[1].revents) {
      ret = handle_messages(mxt, sockfd);
      if (ret < 0)
        goto disconnect;
    }
  }

disconnect:
  mxt_info(mxt->ctx, "Disconnected");
  return ret;
}

//******************************************************************************
/// \brief Bridge client
int mxt_socket_client(struct mxt_device *mxt, char *ip_address, uint16_t port)
{
  struct hostent *server;
  int sockfd;
  int ret;
  struct sockaddr_in serv_addr;

  server = gethostbyname(ip_address);
  if (server == NULL) {
    mxt_err(mxt->ctx, "Error, no such host");
    return -1;
  }

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    mxt_err(mxt->ctx, "socket returned %d (%s)", errno, strerror(errno));
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
  mxt_info(mxt->ctx, "Connecting to %s:%u", ip_address, port);
  ret = connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr));
  if (ret < 0)
  {
    mxt_dbg(mxt->ctx, "connect returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  ret = bridge(mxt, sockfd);

  close(sockfd);
  return ret;
}

//******************************************************************************
/// \brief Bridge server
int mxt_socket_server(struct mxt_device *mxt, uint16_t portno)
{
  int serversock;
  int clientsock;
  int ret;
  int one = 1;
  struct sockaddr_in server_addr, client_addr;
  socklen_t sin_size = sizeof(client_addr);

  /* Create endpoint */
  serversock = socket(AF_INET, SOCK_STREAM, 0);
  if (serversock < 0)
  {
    mxt_err(mxt->ctx, "socket returned %d (%s)", errno, strerror(errno));
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
    mxt_err(mxt->ctx, "bind returned %d (%s)", errno, strerror(errno));
    close(serversock);
    return -errno;
  }

  ret = setsockopt(serversock, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  if (ret < 0)
  {
    mxt_err(mxt->ctx, "setsockopt returned %d (%s)", errno, strerror(errno));
    close(serversock);
    return -errno;
  }

  /* Start listening */
  ret = listen(serversock, 1);
  if (ret < 0)
  {
    mxt_dbg(mxt->ctx, "listen returned %d (%s)", errno, strerror(errno));
    close(serversock);
    return -errno;
  }

  /* This string is used by ADB bridge client to signal it can connect */
  printf("AWAITING_CONNECTION\n");
  clientsock = accept(serversock, (struct sockaddr *) &client_addr, &sin_size);
  if (clientsock < 0)
  {
    mxt_dbg(mxt->ctx, "accept returned %d (%s)", errno, strerror(errno));
    close(serversock);
    return -errno;
  }

  printf("CONNECTED\n");

  ret = bridge(mxt, clientsock);

  close(serversock);

  printf("DISCONNECTED\n");

  return ret;
}