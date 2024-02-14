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


#ifndef SENCODER_H
#define SENCODER_H

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <sys/types.h>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>

#include "utime.h"

using namespace std;

/**
 * Class to receive the motor encoder values.
 * */
class SEncoder
{
public:
  /** setup and request data */
  void setup();
  /**
   * regular update tick */
  void tick();
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();

public:
//   mutex dataLock; // ensure consistency
  int updateCnt = false;
  UTime encTime, encTimeLast;
  int64_t enc[2] = {0};

private:
  void toLog();
  int64_t encLast[2] = {0};
  bool firstEnc = true;
  bool encoder_reversed = true;
  bool toConsole = false;
  FILE * logfile = nullptr;
//   std::condition_variable_any nd; // new data service
};

/**
 * Make this visible to the rest of the software */
extern SEncoder encoder;

#endif
