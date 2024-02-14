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


#ifndef MPOSE_H
#define MPOSE_H

#include "sencoder.h"
#include "utime.h"
#include "thread"

using namespace std;

/**
 * Class that update robot based on wheel encoder update.
 * The result is odometry coordinate update
 *   x,y position
 *   h (heading)
 *   time of last encoder update (poseTime)
 *   wheel velocity (eheelVel)
 * An updateCnt is incremented at every update
 * */
class MPose
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
  /**
   * Set pose to 0,0,0 */
  void resetPose();

protected:
  // robot geometry
  float gear = 10.0;
  float wheelDiameter = 0.09;
  float encTickPerRev = 64;
  float distPerTick = (wheelDiameter * M_PI) / gear / encTickPerRev;
  // distance between driving wheels
  float wheelBase = 0.22;

public:
  /** calculated pose
   * x (forward), y (left), h (heading) in odometry coordinates */
  float x = 0.0, y = 0.0, h = 0.0;
  float dist = 0;
  float turned = 0;
  /// PC time of last update
  UTime poseTime;
  //  Calculated wheel velocity
  float wheelVel[2] = {0.0};
  float turnrate = 0.0;
  float turnRadius = 0.0;
  float robVel = 0.0;
  // new pose is calculated count
  int updateCnt = 0;

private:
  /// private stuff
  static void runObj(MPose * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  /**
   * print to console and logfile */
  void toLog();
  // support variables
  bool firstEnc = true;
  /// Debug print
  bool toConsole = false;
  /// Logfile - most details
  FILE * logfile = nullptr;
  // just absolute pose (and distance)
  FILE * logAbs = nullptr;
  std::thread * th1;
  // source data iteration
  int encoderUpdateCnt = 0;
  /// pose that can't be reset (for debug/map use)
  float x2 = 0.0, y2 = 0.0, h2 = 0.0;
  float dist2 = 0;
  float turned2 = 0;
};

/**
 * Make this visible to the rest of the software */
extern MPose pose;

#endif
