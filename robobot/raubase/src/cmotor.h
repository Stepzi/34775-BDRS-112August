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


#ifndef UMOTOR_H
#define UMOTOR_H

#include <thread>

#include "sencoder.h"
#include "utime.h"
#include "upid.h"

using namespace std;

/**
 * Class to do motor velocity control.
 * This class uses the encoder values as velocity measurement.
 * The desired motor velocity reference is received from
 * the mixer module.
 * */
class CMotor
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
  /** velocity controller - left and right
   * PID values */
  float kp;
  float taui;
  float taud;
  float alpha;
  /// controller output limit (same value positive and negative)
  float maxMotV;
  //
public:
  // is output limited, this may be valuable for other controllers.
  bool limited = false;

private:
  /// private stuff
  static void runObj(CMotor * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  void logfileLeadText(FILE * f, const char * side);
  /// velocity control loop on Teense (else here)
//   bool useTeensyControl = true;
  /**
   * PID controllers, one each wheel */
  UPID pid[2];
  //
  float sampleTime;
  /// old values for PID
  float ep1[2] = {0}, up1[2] = {0}, ui1[2] = {0};
  /// pre-calculated lead values
  float le0, le1, lu1;
  /// pre-calculated integrator values
  float ie;
  // controller output
  float u[2];
  // support variables
  FILE * logfile[2] = {nullptr};
//   mutex dataLock; // data consistency lock, should not be needed
  std::thread * th1;
  bool stop = false;
  int dataCnt = 0;
  /// old mixer update count
  int mixerUpdateCnt = 0;
  int poseUpdateCnt = 0;
};

/**
 * Make this visible to the rest of the software */
extern CMotor motor;

#endif
