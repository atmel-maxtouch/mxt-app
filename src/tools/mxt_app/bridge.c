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
#include <inttypes.h>
#include <poll.h>

#include "libmaxtouch/libmaxtouch.h"
#include "libmaxtouch/sysfs/sysfs_device.h"
#include "libmaxtouch/log.h"
#include "libmaxtouch/utilfuncs.h"

#include "mxt_app.h"
#include "buffer.h"

#define MAX_LINESIZE 10000

//******************************************************************************
/// \brief Read command on a single line
static int readline(int fd, struct mxt_buffer *linebuf)
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
      LOG(LOG_DEBUG, "read returned %d (%s)", errno, strerror(errno));
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
static int handle_messages(int sockfd)
{
  int count, i;
  char *msg;
  int ret;

  count = mxt_get_msg_count();

  if (count > 0)
  {
    for (i = 0; i < count; i++)
    {
      msg = mxt_get_msg_string();
      ret = write(sockfd, msg, strlen(msg));
      if (ret < 0)
      {
        LOG(LOG_ERROR, "write returned %d (%s)", errno, strerror(errno));
        return ret;
      }

      ret = write(sockfd, "\n", 1);
      if (ret < 0)
      {
        LOG(LOG_ERROR, "write returned %d (%s)", errno, strerror(errno));
        return ret;
      }

    }
  }
  return 0;
}


//******************************************************************************
/// \brief Handle bridge read
static int bridge_rea_cmd(int sockfd, uint16_t address, uint16_t count)
{
  int ret;
  uint8_t *databuf;
  char *response;
  const char * const PREFIX = "RRP ";
  size_t response_len;
  int i;

  databuf = malloc(count);
  if (!databuf)
  {
    LOG(LOG_ERROR, "Failed to allocate memory");
    return -1;
  }

  /* Allow for newline/null byte */
  response_len = strlen(PREFIX) + count*2 + 1;
  response = malloc(response_len);
  if (!response)
  {
    LOG(LOG_ERROR, "Failed to allocate memory");
    ret = -1;
    goto free_databuf;
  }

  strcpy(response, PREFIX);
  LOG(LOG_INFO, "reading %d %d", address, count);
  ret = mxt_read_register(databuf, address, count);
  if (ret < 0) {
    LOG(LOG_WARN, "RRP ERR");
    strcpy(response + strlen(PREFIX), "ERR\n");
    response_len = strlen(response);
  } else {
    for (i = 0; i < count; i++) {
      sprintf(response + strlen(PREFIX) + i*2, "%02X", databuf[i]);
    }
    LOG(LOG_INFO, "%s", response);
    response[response_len - 1] = '\n';
  }

  ret = write(sockfd, response, response_len);
  if (ret < 0) {
    LOG(LOG_ERROR, "Socket write error %d (%s)", errno, strerror(errno));
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
static int bridge_wri_cmd(int sockfd, uint16_t address,
                          char *hex, uint16_t bytes)
{
  int ret;
  uint16_t count;
  const char * const FAIL = "WRP ERR\n";
  const char * const PASS = "WRP OK\n";
  const char *response;
  uint8_t *databuf;

  databuf = malloc(bytes);
  if (!databuf)
  {
    LOG(LOG_ERROR, "Failed to allocate memory");
    return -1;
  }

  ret = mxt_convert_hex(hex, databuf, &count, bytes);
  if (ret < 0) {
    response = FAIL;
  } else {
    ret = mxt_write_register(databuf, address, count);
    if (ret < 0) {
      LOG(LOG_VERBOSE, "WRI OK");
      response = FAIL;
    } else {
      response = PASS;
    }
  }

  ret = write(sockfd, response, strlen(response));
  if (ret < 0) {
    LOG(LOG_ERROR, "Socket write error %d (%s)", errno, strerror(errno));
  }

  free(databuf);
  return ret;
}

//******************************************************************************
/// \brief Read and deal with incoming command
static int handle_cmd(int sockfd)
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

  ret = readline(sockfd, &linebuf);
  if (ret <= 0) {
    LOG(LOG_DEBUG, "error reading or peer closed socket");
    ret = -1;
    goto free;
  }

  line = (char *)linebuf.data;
  if (strlen(line) == 0) {
    ret = 0;
    goto free;
  }

  LOG(LOG_VERBOSE, "%s", line);

  if (!strcmp(line, "SAT")) {
    LOG(LOG_INFO, "Server attached");
    ret = 0;
  } else if (!strcmp(line, "SDT")) {
    LOG(LOG_INFO, "Server detached");
    ret = 0;
  } else if (sscanf(line, "REA %" SCNu16 " %" SCNu16, &address, &count) == 2) {
    ret = bridge_rea_cmd(sockfd, address, count);
  } else if (sscanf(line, "WRI %" SCNu16 "%n", &address, &offset) == 1) {
    /* skip space */
    offset += 1;

    ret = bridge_wri_cmd(sockfd, address,
                         line + offset,
                         strlen(line) - offset);
  } else {
    LOG(LOG_INFO, "Unrecognised cmd");
    ret = 0;
  }

free:
  mxt_buf_free(&linebuf);
  return ret;
}

//******************************************************************************
/// \brief Main bridge function to handle a single connection
static int bridge(int sockfd)
{
  int ret;
  struct pollfd fds[2];
  int fopts = 0;
  int debug_ng_fd = 0;
  int numfds = 1;

  LOG(LOG_INFO, "Connected");

  ret = mxt_msg_reset();
  if (ret)
    LOG(LOG_ERROR, "Failure to reset dmesg timestamp");

  fds[0].fd = sockfd;
  fds[0].events = POLLRDNORM | POLLERR;

  while (1)
  {
    if (sysfs_has_debug_ng())
    {
      debug_ng_fd = sysfs_get_debug_ng_fd();
      fds[1].fd = debug_ng_fd;
      fds[1].events = POLLPRI;
      numfds = 2;
    }

    ret = poll(fds, numfds, -1);
    if (ret == -1 && errno == EINTR) {
      LOG(LOG_DEBUG, "interrupted");
      continue;
    } else if (ret == -1) {
      ret = -errno;
      LOG(LOG_ERROR, "poll returned %d (%s)", errno, strerror(errno));
      goto disconnect;
    }

    /* Detect socket disconnect */
    if (fcntl(sockfd, F_GETFL, &fopts) < 0) {
      ret = 0;
      goto disconnect;
    }

    if (fds[0].revents) {
      ret = handle_cmd(sockfd);
      if (ret < 0)
        goto disconnect;
    }

    if (fds[1].revents) {
      ret = handle_messages(sockfd);
      if (ret < 0)
        goto disconnect;
    }
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
    LOG(LOG_ERROR, "Error, no such host");
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
  ret = listen(serversock, 1);
  if (ret < 0)
  {
    LOG(LOG_DEBUG, "listen returned %d (%s)", errno, strerror(errno));
    return -errno;
  }

  /* This string is used by ADB bridge client to signal it can connect */
  printf("AWAITING_CONNECTION\n");
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
