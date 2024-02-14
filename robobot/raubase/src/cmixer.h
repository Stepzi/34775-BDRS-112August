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


#ifndef MMIXER_H
#define MMIXER_H

#include "cmotor.h"
#include "utime.h"
#include "cheading.h"
#include "mpose.h"

using namespace std;

/**
 * The mixer translates linear and rotation reference
 * values to velocity for each motor.
 * */
class CMixer
{
public:
  /** setup and initialize parameters */
  void setup();
  /**
   * close down */
  void terminate();
  /**
   * Velocity control in automnomous mode
   * \param linearVelocity in meter per second
   * */
  void setVelocity(float linearVelocity);// m/s
  /**
   * Set heading mode to turnrate control.
   * \param turnVelocity is the desired turnrate */
  void setTurnrate(float turnVelocity);  // rad/sec
  /**
   * Set turnrate, when turnrate is controlled by a
   * specifig controller, e.g. edge follow controller .
   * \param turnVelocity is the turn actuator value */
  void setInModeTurnrate(float turnVelocity);
  /**
   * Set drive mode to heading control, and aim for this heading
   * relative to measured heading in pose.h.
   * \param heading is the desired heading. */
  void setDesiredHeading(float heading); // in radians
  /**
   * Manual control, if a gamepad is connected
   * \param manual if true, then override any automatic guide.
   * \param linVel desired linear velocity (m/s) (valid if manual==true only)
   * \param rotVel desired rotation velocity (rad/s) (valid if manual==true only) */
  void setManualControl(bool manual, float linVel, float rotVel);
  /**
   * Set drive mode to edge control
   * \param leftEdge follow left edge of line, else right edge.
   * \param offset minor offset relative to the edge (m), positive is left */
  void setEdgeMode(bool leftEdge, float offset);

  /**
   * Get wheel velocity */
  inline float * getWheelVelocityArray()  { return wheelVelRef; }
  /**
   * are we in autonomous mode, i.e. not in manual override */
  inline bool autonomous()  {  return not manualOverride;  }

  /**
   * Translate fro linear velocity and turnrate to wheel velocity */
  void updateWheelVelocity();

public:
  /// Mixer update cnt
  int updateCnt = 0;
  UTime updateTime;
  // when not in turnrate mode, then try to keep
  // this desired heading (compared to pose.h)
  float desiredHeading = 0;
  // turnrate mode for automatic drive
//   bool turnrateMode = true;
  // heading mode
  enum {HM_TURNRATE, HM_ABS_HEADING, HM_EDGE} headingMode = HM_TURNRATE;

private:
  /// private stuff
  /**
   * Du mixing and transfer result to velocity and heading controller */
  void updateVelocities();
  /** log data for this module */
  void toLog();
  //
  FILE * logfile = nullptr;
  bool toConsole = false;
  /// Turnrate in radians per second
//   float turnrateRef = 0; // desired
  /// Linear velocity (m/s)
  float linVel = 0;
  // manual override mode
  bool manualOverride = false;
  float manualLinVel = 0;
  float manualTurnrateRef = 0;
  /// for autonomous drive
  float autoLinVel = 0;
  float autoTurnrateRef = 0;
  //
  float wheelbase;
  float velDif; // desired velocity difference
  float turnRadius; // desired turn radius
  // velocity ref for left and right wheel
  float wheelVelRef[2] = {0};
};

/**
 * Make this visible to the rest of the software */
extern CMixer mixer;

#endif
