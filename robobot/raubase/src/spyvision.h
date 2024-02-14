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


#ifndef SPYVISION_H
#define SPYVISION_H

#include <unistd.h>

#include "utime.h"
#include "usocket.h"

using namespace std;

/**
 * Class for interface with vision
 * written in Python
 * The interface connects to a python socket server.
 * Requests are send to the Ptyhon server and
 * this class listens to the result */
class SPyVision
{
public:
  /** setup and connect to server */
  void setup();
  /**
   * Listen to socket from python vision app */
  void run();
  /** send a command to python socket server
   * \returns true if the request is send OK */
  bool sendCommand(const char* command);
  /**
   * terminate */
  void terminate();
  /**
   * Wait for a reply from vision
   * */
   bool waitForAruco(float timeoutMs);

private:
  /**
   * Decode reply from vision */
  void decodeReply(const char * reply);

public: // data reply
  // aruco
  bool aruco_valid = false;
  float aruco_x;
  float aruco_y;
  float aruco_h;
  int aruco_ID;
  int aruco_updateCnt = 0;
  int aruco_updateCntLast = 0;
  // golf
  bool golf_valid = false;
  int golf_count = 0;
  // missing a vector of found ball positions
  // or something similar

private:
  USocket * sock = nullptr;
  //
  void toLogRx(const char * got);
  void toLogTx(const char * cmd);
  bool toConsole = false;
  FILE * logfile = nullptr;
  //
  static void runObj(SPyVision * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  // support variables
  std::thread * th1 = nullptr;

};

/**
 * Make this visible to the rest of the software */
extern SPyVision pyvision;

#endif
