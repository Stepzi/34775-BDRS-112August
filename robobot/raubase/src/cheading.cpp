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
#include <math.h>
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
#include "mpose.h"
#include "cmixer.h"

#include "cheading.h"

// create value
CHeading heading;


void CHeading::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("heading"))
  { // motor block is OK, but control parameters are needed too
    ini["heading"]["kp"] = "10.0"; // unit is (turnrate (m/sec) per angle error (rad))
    ini["heading"]["lead"] = "0.0 1.0"; // tau_d (sec) and alpha, tau_d = 0.0 means no function
    ini["heading"]["taui"] = "0.0"; // tau_i (sec) 0.0 is no integrator function
    ini["heading"]["maxTurnrate"] = "3.0"; // (rad/s)
    ini["heading"]["log"] = "true";
    ini["heading"]["print"] = "false";
  }
  //
  // get values from ini-file
  float kp = strtof(ini["heading"]["kp"].c_str(), nullptr);
  const char * p1 = ini["heading"]["lead"].c_str();
  // lead
  float taud = strtof(p1, (char**)&p1);
  float alpha = strtof(p1, (char**)&p1);
  // integrator
  float taui = strtof(ini["heading"]["taui"].c_str(), nullptr);
  // output limit
  maxTurnrate = strtof(ini["heading"]["maxTurnrate"].c_str(), nullptr);
  // sample time from encoder module
  float sampleTime = strtof(ini["encoder"]["rate_ms"].c_str(), nullptr) / 1000.0;
  //
  pid.setup(sampleTime, kp, taud, alpha, taui);
  pid.doAngleFolding(true);
  // should debug print be enabled
  pid.toConsole = ini["heading"]["print"] == "true";
  // initialize logfile
  if (ini["heading"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_heading.txt";
    logfile = fopen(fn.c_str(), "w");
    logfileLeadText(logfile);
    pid.logPIDparams(logfile, false);
  }
  th1 = new std::thread(runObj, this);
}

void CHeading::logfileLeadText(FILE * f)
{
    fprintf(f, "%% Heading control logfile\n");
    fprintf(f, "%% 1 \tTime (sec)\n");
    fprintf(f, "%% 2 \tReference for desired heading (rad)\n");
    fprintf(f, "%% 3 \tMeasured heading (rad)\n");
    fprintf(f, "%% 4 \tValue after Kp (rad/s)\n");
    fprintf(f, "%% 5 \tValue after Lead (rad/s)\n");
    fprintf(f, "%% 6 \tIntegrator value (rad/s)\n");
    fprintf(f, "%% 7 \tAfter controller (u) (rad/s)\n");
    fprintf(f, "%% 8 \tIs output limited (1=limited)\n");
}

void CHeading::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}

void CHeading::setRef(bool useTurnrate, float turnrate, float absHeading)
{
  turnrateControl = useTurnrate;
  turnrateRef = turnrate;
  headingRef = absHeading;
}



void CHeading::run()
{
  int loop = 0;
  while (not service.stop)
  {
    if (pose.updateCnt != poseUpdateCnt)
    { // do constant rate control
      // that is; every time new encoder data is available,
      // and therefore pose.updateCnt increased,
      // then new motor control values should be calculated.
      poseUpdateCnt = pose.updateCnt;
      // do control.
      // got new encoder data
      float dt = pose.poseTime - lastPose;
      lastPose = pose.poseTime;
      // calculate new reference turnrate
      if (turnrateControl)
        desiredHeading += turnrateRef * dt;
      else
      {
        desiredHeading = headingRef;
      }
      if (dt < 1.0)
      { // valid control timing
        u = pid.pid(desiredHeading, pose.h, limited);
        // test for output limiting
        if (fabsf(u) > maxTurnrate or motor.limited)
        { // don't turn too fast
          limited = true;
          if (u > maxTurnrate)
            u = maxTurnrate;
          else if (u < -maxTurnrate)
            u = -maxTurnrate;
        }
        else
          limited = false;
      }
      // log control values
      pid.saveToLog(logfile, pose.poseTime);
      // finished calculating turn rate
      mixer.updateWheelVelocity();
    }
//     else
//     { // no control - rely on motor velocity controller
//       poseUpdateCnt = pose.updateCnt;
//       mixer.setInModeTurnrate(turnrateRef);
//       mixer.translateToWheelVelocity();
//     }
    loop++;
    usleep(2000);
  }
}


