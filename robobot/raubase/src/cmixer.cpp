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
#include <thread>
#include <math.h>
#include "sencoder.h"
#include "cmixer.h"
#include "cmotor.h"
#include "cedge.h"
#include "steensy.h"
#include "uservice.h"

// create value
CMixer mixer;


/// mixer class combines drive orders to desired wheel velocity
void CMixer::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("mixer") or not ini["mixer"].has("print"))
  { // no data yet, so generate some default values
    ini["mixer"]["log"] = "true";
    ini["mixer"]["print"] = "false";
  }
  // get values from ini-file
  //
  wheelbase = strtof(ini["pose"]["wheelbase"].c_str(), nullptr);
//   turnrateControl = ini["heading"]["enabled"] == "true";
  // wheelbase must not be zero or negative
  if (wheelbase < 0.005)
    wheelbase = 0.22;
  //
  toConsole = ini["mixer"]["print"] == "true";
  if (ini["mixer"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_mixer.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mixer logfile\n");
    fprintf(logfile, "%% Wheel base used in calculation: %g m\n", wheelbase);
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tmanual override mode (0= automatic, 1=manuel mode)\n");
    fprintf(logfile, "%% 3 \tLinear velocity (m/s)\n");
    fprintf(logfile, "%% 4 \tHeading mode (0=turnrate, 1=heading, 2=edge)\n");
    fprintf(logfile, "%% 5 \tDesired heading (heading mode, compared to pose.h)\n");
    fprintf(logfile, "%% 6 \tTurnrate reference (rad/sec) positive is CCV\n");
    fprintf(logfile, "%% 7 \tTurnrate after heading control (rad/sec) positive is CCV\n");
    fprintf(logfile, "%% 8 \tDesired left wheel velocity (m/s)\n");
    fprintf(logfile, "%% 9 \tDesired right wheel velocity (m/s)\n");
    fprintf(logfile, "%% 10 \tCalculated commanded turn radius (999 if straight) (m)\n");
  }
}

void CMixer::terminate()
{
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}

void CMixer::setDesiredHeading(float heading)
{
  desiredHeading = heading;
  headingMode = HM_ABS_HEADING;
  updateVelocities();
}

void CMixer::setVelocity(float linearVelocity)
{
  autoLinVel = linearVelocity;
  updateVelocities();
}

void CMixer::setTurnrate(float turnVelocity)
{
  autoTurnrateRef = turnVelocity;
  headingMode = HM_TURNRATE;
  updateVelocities();
}

void CMixer::setInModeTurnrate(float turnVelocity)
{
  autoTurnrateRef = turnVelocity;
  updateVelocities();
}

void CMixer::setManualControl(bool manual, float linVel, float rotVel)
{
  manualOverride = manual;
  manualLinVel = linVel;
  manualTurnrateRef = rotVel;
  updateVelocities();
}

void CMixer::setEdgeMode(bool leftEdge, float offset)
{
  headingMode = HM_EDGE;
  // inform edge control of new settings
  // follow left or right edge
  cedge.followLeft = leftEdge;
  // offset by (to the left)
  cedge.followOffset = offset;
}


void CMixer::updateVelocities()
{ // trigger new calculation
//   printf("# CMixer:: update\n");
  if (manualOverride)
  {
    linVel = manualLinVel;
    heading.setRef(true, manualTurnrateRef, desiredHeading);
  }
  else
  {
    linVel = autoLinVel;
    heading.setRef(headingMode != HM_ABS_HEADING, autoTurnrateRef, desiredHeading);
  }
  //
  updateWheelVelocity();
  //
  updateTime.now();
  toLog();
}

void CMixer::updateWheelVelocity()
{ // velocity difference to get the desired turn rate.
  velDif = wheelbase * heading.getTurnrate();
  float v0; // left
  float v1; // right
  // adjust each wheel with half difference
  // positive turn-rate (CCV) makes right wheel
  // turn faster forward
  v1 = linVel + velDif/2;
  v0 = v1 - velDif;
  // turn radius (for logging only)
  //
  // linvel = (v0+v1)/2
  // linvel = turnRadius * turnrate
  // turnRadius = linvel / turnrate
  //
  const float minTurnrate = 0.001; // rad/s
  if (heading.getTurnrate() > minTurnrate or heading.getTurnrate() < -minTurnrate)
    turnRadius = linVel / heading.getTurnrate();
  else if (velDif > 0)
    turnRadius = linVel / minTurnrate;
  else
    turnRadius = linVel / -minTurnrate;
  // implement result
  wheelVelRef[0] = v0;
  wheelVelRef[1] = v1;
  updateCnt++;
  updateTime.now();
  toLog();
}

void CMixer::toLog()
{
  if (service.stop)
    return;
  if (logfile != nullptr)
  { // add to log after update
    fprintf(logfile, "%lu.%04ld %d %.3f %d %.4f %.4f %.4f %.3f %.3f %.2f\n",
            updateTime.getSec(), updateTime.getMicrosec()/100,
            manualOverride, linVel, headingMode, desiredHeading,
            heading.getTurnrateRef(), heading.getTurnrate(),
            wheelVelRef[0], wheelVelRef[1], turnRadius);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %.3f %d %.4f %.4f %.4f %.3f %.3f %.2f\n",
           updateTime.getSec(), updateTime.getMicrosec()/100,
           manualOverride, linVel, headingMode, desiredHeading,
           heading.getTurnrateRef(), heading.getTurnrate(),
           wheelVelRef[0], wheelVelRef[1], turnRadius);
  }
}
