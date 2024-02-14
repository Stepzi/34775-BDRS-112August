/*  
 * 
 * Copyright © 2022 DTU, 
 * Author:
 * Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include <string>
#include <string.h>
#include <sys/types.h>
#include "usocket.h"
#include <stdio.h>


USocket::USocket(const char * host, const char * port)
{ // set parameters
//   std::string portStr = std::to_string(port);
  // create client to vision server in python
  int res;
  if ((res = getaddrinfo(host, port, nullptr, &servinfo)) != 0)
  { // failed
    fprintf(stderr,"# Getting address info failed: %s\n", gai_strerror(res));
  }
  else
  {
    if ((sockfd = socket(servinfo->ai_family, servinfo->ai_socktype, servinfo->ai_protocol)) == -1)
    { // failed
      fprintf(stderr,"# Failed socket creation to %s:%s\n", host, port);
    }
    // try to connect
    if (connect(sockfd, servinfo->ai_addr, servinfo->ai_addrlen) == -1)
    { // failed
  //     std::perror("# USocket:: connect failed");
      connected = false;
      close(sockfd);
      th1 = nullptr;
    }
    else
    { // connection established
      connected = true;
      // start read thread
      th1 = new std::thread(runObj, this);
    }
  }
}

void USocket::terminate()
{ // wait for receive thread to finish
  stop = true;
  if (th1 != nullptr)
    th1->join();
}

bool USocket::sendCommand(std::string command)
{
  bool sendOk = false;
  if (connected)
  {
    if (not command.ends_with('\n'))
      command += '\n';
    int n = command.size();
    int m = send(sockfd, command.c_str(), n, 0);
    if (n == m)
    {
      sendOk = true;
      txCnt++;
      txTime.now();
    }
  }
  return sendOk;
}

void USocket::run()
{
  const int MAX_RX_CNT = 2000;
  char rxBuf[MAX_RX_CNT];
  int rxCnt = 0;
  while (connected and not stop)
  { // receive 1 character
    char recvChar;
    int e = recv(sockfd, &recvChar, 1, MSG_DONTWAIT);
    if (e == 1)
    { /// got a character
      // Check accepted chars
      if (recvChar >=' ' or recvChar == '\n' or recvChar == '\t')
      { // collect to a string (a fixed array of characters for speed)
        rxBuf[rxCnt] = recvChar;
        // If the line ends, terminate string and decode
        if (rxBuf[rxCnt] == '\n')
        { // terminate string (replacing the newline '\n')
          rxBuf[rxCnt] = '\0';
          // unpack this line
          reply = rxBuf;
          // mark command as handled
          cmdSend = false;
          replyCnt++;
          rxTime.now();
          // ready for next message
          rxCnt = 0;
        }
        else if (rxCnt < MAX_RX_CNT)
        { // Increment string length
          rxCnt++;
        }
        else
        { // Buffer overflow
          printf("USocket:: Listen loop overflow (discards the buffer)\n");
          rxCnt = 0;
        }
      }
    }
    else if (e < 0 and errno != EAGAIN)
    { // lost connection with hardware
      // shut down
      printf("### lost hardware connection (errno=%d) ###\n", errno);
      connected = false;
      close(sockfd);
    }
    else
    { // no data, wait a bit
      usleep(900);
    }
  }
  if (connected)
  {
    connected = false;
    close(sockfd);
  }
}


std::string USocket::waitForReply(float timeoutMs)
{
  std::string r = "";
  UTime t;
  t.now();
  while (replyCnt == replyCntLast and t.getTimePassed() < timeoutMs/1000.0)
  { // wait a ms
    usleep(1000);
  }
  if (replyCnt != replyCntLast)
  { // got a reply
    r = reply;
    replyCntLast = replyCnt;
  }
  return r;
}


