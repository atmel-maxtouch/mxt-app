//------------------------------------------------------------------------------
/// \file   mxt_bridge.c
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

#define BUF_SIZE 1024

char buf[BUF_SIZE];
char hexbuf[BUF_SIZE];

/*!
 * @brief  Display usage information for the config_loader utility.
 */
static void display_usage(void)
{
  printf
  (
    "Usage: mxt_bridge address\n"
    "address  IP address or hostname to connect to.\n"
  );
}

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

static char to_digit(char hex)
{
    char decimal;

    if (hex >= '0' && hex <= '9')
        decimal = hex - '0';
    else if (hex >= 'A' && hex <= 'F')
        decimal = hex - 'A' + 10;
    else if (hex >= 'a' && hex <= 'f')
        decimal = hex - 'a' + 10;
    else
        decimal = 0;

    return decimal;
}

static int mxt_convert_hex(char *hex, unsigned char *databuf, int *count)
{
    int pos = 0;
    int datapos = 0;
    char highnibble;
    char lownibble;

    while (1) {
        highnibble = *(hex + pos);
        lownibble = *(hex + pos + 1);

        if (lownibble == '\0' || lownibble == '\n'
            || highnibble == '\0' || highnibble == '\n')
            break;

        *(databuf + datapos) = (to_digit(highnibble) << 4) 
                                | to_digit(lownibble);
        datapos++;

        pos += 2;
        if (pos > BUF_SIZE)
            return -1;
    }

    *count = datapos;
    return 0;
}

static void handle_messages(int sockfd)
{
    int count, i;

    count = mxt_get_debug_messages();

    if (count > 0)
    {
        for (i = 0; i < count; i++)
        {
            snprintf(buf, BUF_SIZE, "%s\n", (char *)mxt_retrieve_message());
            write(sockfd, buf, strlen(buf));
        }
    }
}

static int handle_cmd(int sockfd)
{
  int ret;
  int address, count, i;
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
  } else if (sscanf(buf, "REA %d %d", &address, &count) == 2) {
      ret = mxt_read_register(&databuf[0], address, count);
      if (ret < 0) {
        snprintf(buf, BUF_SIZE, "RRP ERR");
      } else {
        snprintf(buf, BUF_SIZE, "RRP ");

        for (i = 0; i < count; i++) {
          sprintf(hexbuf, "%02X", databuf[i]);
          strncat(buf, hexbuf, BUF_SIZE);
        }
      }
      LOG(LOG_INFO, "%s", buf);
      strncat(buf, "\n", BUF_SIZE);
      write(sockfd, buf, strlen(buf));
  } else if (sscanf(buf, "WRI %d %s", &address, (char *)&hexbuf) == 2) {
      ret = mxt_convert_hex(&hexbuf[0], &databuf[0], &count);
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
      write(sockfd, buf, strlen(buf));
  } else {
      printf("Unrecognised cmd %s\n", buf);
      return -1;
  }

  return 0;
}

static int bridge(struct hostent *server)
{
    int sockfd;
    int ret;
    int portno = 4000;
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

/*!
 * @brief  Entry point for the config_loader utility.
 * @return Zero on success, negative for error.
 */
int main (int argc, char *argv[])
{
  struct hostent *server;
  int ret;

  printf("Bridge tool for Atmel maXTouch chips\n\n");

  /* Parse input arguments */
  if (argc != 2)
  {
    display_usage();
    return -1;
  }

  /* Find an mXT device and read the info block */
  switch (mxt_scan())
  {
    case 1:
      /* Device found - continue */
      break;
    case 0:
      printf("Could not find a device, exiting...\n");
      return -1;
    default:
      printf("Error initializing, exiting...\n");
      return -1;
  }
  
  if (mxt_get_info() < 0)
  {
    printf("Error reading info block, exiting...\n");
    return -1;
  }

  server = gethostbyname(argv[1]);
  if (server == NULL) {
    printf("Error, no such host\n");
    mxt_release();
    return -1;
  }

  mxt_set_debug(true);
  ret = mxt_dmesg_reset();
  if (ret)
    LOG(LOG_ERROR, "Failure to reset dmesg timestamp");

  ret = bridge(server);
  if (ret < 0)
    LOG(LOG_ERROR, "Failure in bridge, ret %d", ret);

  mxt_set_debug(false);
  mxt_release();

  return 0;
}
