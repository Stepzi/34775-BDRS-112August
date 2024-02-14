/*  
 * 
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
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


#ifndef USOCKET_H
#define USOCKET_H

// #include <iostream>
// #include <sys/time.h>
// #include <cstdlib>
#include <sys/types.h>
// #include <mutex>
// #include <condition_variable>
#include <sys/types.h>
// #include <sys/stat.h>
#include <unistd.h>
// #include <math.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <thread>

#include "utime.h"

using namespace std;
// forward declaration

class USocket
{
public:
  USocket(const char * host, const char * port);
  /**
   * Listen to socket from python vision app */
  void run();
  /** decode an unpacked incoming messages
   * \returns true if the request is send OK */
  bool sendCommand(std::string command);
  /**
   * terminate */
  void terminate();
  /**
   * Wait for a reply from vision
   * */
  std::string waitForReply(float timeoutMs);

public:
  int txCnt = 0;
  int replyCnt = 0;
  UTime txTime, rxTime;
  bool connected = false;
  std::string reply;


private:
  std::string host;
  int port;
  addrinfo * servinfo = nullptr; /// socket info
  int sockfd; /// Socket file descriptor
  //
  std::string command;
  int replyCntLast = 0;
  bool cmdSend = false;
  //
  static void runObj(USocket * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  // support variables
  std::thread * th1;
  bool stop = false;
};


#endif
