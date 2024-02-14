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


#pragma once

#include <thread>

#include "medge.h"
#include "utime.h"
#include "upid.h"

using namespace std;

/**
 * Class to do motor velocity control.
 * This class uses the encoder values as velocity measurement.
 * The desired motor velocity reference is received from
 * the mixer module.
 * */
class CEdge
{
public:
  /** setup and request data */
  void setup();
  /**
   * thread to do updates, when new data is available */
  void run();
  /**
   * terminate */
  void terminate();

public:
  /// controller output limit (same value positive and negative)
  float maxTurnrate;
  /** edge follow variables */
  bool followLeft = false;
  // Mid-robot offset from line edge (positive is left)
  float followOffset = 0.0;
  // should control be enabled (default is off)
//   bool enabled = false;

private:
  /// private stuff
  static void runObj(CEdge * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  void toLog();
  /**
   * PID controller */
  UPID pid;
  float u;
  bool limited = false;
  //
  // support variables
  FILE * logfileCtrl = {nullptr};
  FILE * logfile = {nullptr};
  bool toConsole;
  //   mutex dataLock; // data consistency lock, should not be needed
  std::thread * th1;
  bool stop = false;
  float measuredValue;
};

/**
 * Make this visible to the rest of the software */
extern CEdge cedge;

