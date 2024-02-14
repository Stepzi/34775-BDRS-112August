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


#ifndef MEDGE_H
#define MEDGE_H

#include "sedge.h"
#include "utime.h"

using namespace std;

/**
 * Class that extrach edge position of the line sensor
 * as well as crossing lines.
 * An updateCnt is incremented at every update
 * */
class MEdge
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

protected:
  /**
   * Find left and right edge
   * detect crossing line */
  void findEdge();

public:
  /// PC time of last update
  UTime updTime;
  int updateCnt = 0;
  // calbration
  int calibWhite[8];
  int calibBlack[8];
  float sensorWidth;
  bool calibrationValid = true;
  // white value in per-mille (1/1000) (integer)
  int whiteThresholdPm;
  float width = 0.0;
  bool edgeValid = false;
  float leftEdge = 0.0;
  float rightEdge = 0.0;
  // flag for doing a white line sensor calibration
  bool sensorCalibrateWhite = false;
  bool sensorCalibrateBlack = false;

private:
  /// private stuff
  static void runObj(MEdge * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  /**
   * log and print to console */
  void toLog();
  //
  int ls[8] = {0};
  int lineUpdateCnt = 0;
  // debug print
  bool toConsole = false;
  FILE * logfile = nullptr;
  FILE * logfileNorm = nullptr;
  std::thread * th1;
  // mostly debug
  int eeL, ddL, eeR, ddR;
  int l, r;

  const int sensorCalibrateSamples = 100;
  int sensorCalibrateCount = 0;
  int sensorCalibrateValue[8] = {0};

};

/**
 * Make this visible to the rest of the software */
extern MEdge medge;

#endif
