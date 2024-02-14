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


#ifndef SIRDIST_H
#define SIRDIST_H


#include "utime.h"

/**
 * Class to receive the IR (sharp 2Y0A21) sensor
 * using the calibration function build into the Teensy sensor processing,
 * the calibration values are send to the Teensy.
 */
class SIrDist
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
  int updateCnt = false;
  UTime updTime;
  float dist[2];
  int distAD[2];
  int ir13cm[2];
  int ir50cm[2];
  float urm09factor;
  enum sensortypes {sharp, URM09};
  sensortypes sensortype[2];

public:
//   mutex dataLock; // ensure consistency
  UTime updTimeLast;
  void calibrate(int sensor, int distance_cm);
  bool inCalibration = false;
private:
  void toLog();
  bool toConsole = false;
  FILE * logfile = nullptr;
  //
  int calibSensor;
  int calibDist;
  int calibSum = 0;
  const static int calibCountMax = 20;
  int calibCount = 0;
};

/**
 * Make this visible to the rest of the software */
extern SIrDist dist;

#endif
