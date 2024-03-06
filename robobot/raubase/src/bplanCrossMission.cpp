/*  
 * 
 * Copyright © 2023 DTU,
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
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"
#include "sdist.h"

#include "bplanCrossMission.h"

// create class object
BPlanCrossMission planCrossMission;


void BPlanCrossMission::setup()
{ // ensure there is default values in ini-file
  if (not ini["PlanCrossMission"].has("log"))
  { // no data yet, so generate some default values
    ini["PlanCrossMission"]["log"] = "true";
    ini["PlanCrossMission"]["run"] = "true";
    ini["PlanCrossMission"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["PlanCrossMission"]["print"] == "true";
  //
  if (ini["PlanCrossMission"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_PlanCrossMission.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission PlanCrossMission logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlanCrossMission::~BPlanCrossMission()
{
  terminate();
}

void BPlanCrossMission::run_StartToFirstCross()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.09;

  float f_Line_LeftOffset = 0.03;
  //float f_Line_RightOffset = -0.03;
  bool b_Line_HoldLeft = true;
  //bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.25; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
 
  //
  toLog("PlanCrossMission started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify.
        if(medge.width > f_LineWidth_MinThreshold) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.25"); //Some parse of float to log, Villiams :)
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          mixer.setVelocity(f_Velocity_DriveForward); 
          state = 2;
        }
        else if(medge.width < f_LineWidth_NoLine)
        {
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.01);//Drive slowly and turn i circle
          mixer.setTurnrate(1.0);
        }
        else if(t.getTimePassed() > f_Time_Timeout)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(medge.width > f_LineWidth_Crossing) 
        { 
          // start driving
          toLog("First split found");
          pose.dist=0;
          finished = true;   
        }
        //If line missed, reverse and it is tried to be detected again with a lower line width measure. If missed again this continues.
        else if(abs(pose.dist) > f_Distance_FirstCrossMissed)
        {
            pose.dist=0;
            mixer.setVelocity(f_Velocity_DriveBackwards); 
            f_LineWidth_Crossing -= 0.005;
        }
        break;

      default:
        toLog("Default Start to Cross");
        lost = true;
      break;
    }
    if (state != oldstate)
    { // C-type string print
      snprintf(s, MSL, "State change from %d to %d", oldstate, state);
      toLog(s);
      oldstate = state;
      t.now();
    }
    // wait a bit to offload CPU (4000 = 4ms)
    usleep(4000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}


void BPlanCrossMission::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlanCrossMission::toLog(const char* message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
           oldstate,
           message);
  }
}
