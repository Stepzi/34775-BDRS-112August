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

#include "bplan21.h"

// create class object
BPlan21 plan21;


void BPlan21::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan21"].has("log"))
  { // no data yet, so generate some default values
    ini["plan21"]["log"] = "true";
    ini["plan21"]["run"] = "false";
    ini["plan21"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan21"]["print"] == "true";
  //
  if (ini["plan21"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan21.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan21 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan21::~BPlan21()
{
  terminate();
}

void BPlan21::run()
{
  if (not setupDone)
    setup();
  if (ini["plan21"]["run"] == "false")
    return;
  //
  UTime t;
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  //
  toLog("Plan21 started");
  int turns = 0;
  //
  while (not finished and not lost and not service.stop)
  { // run a square with 4 90 deg turns - CCV
    switch (state)
    {
      case 10:
        if (pose.dist >= 1.0 or turns == 0)
        { // done, and then
          if (turns >= 4)
          {
            finished = true;
            mixer.setVelocity(0);
            mixer.setTurnrate(0);
            break;
          }
          toLog("now turn to -pi/2 rad (30deg)");
          // reset turned angle and distance
          pose.resetPose();
          mixer.setVelocity(0.3);
          mixer.setDesiredHeading(M_PI/2);
          t.now();
          turns++;
        }
        else if (t.getTimePassed() > 10)
          lost = true;
        break;
      default:
        toLog("Unknown state");
        lost = true;
        break;
    }
    if (state != oldstate)
    {
      const int MSL = 100;
      char s[MSL];
      snprintf(s, MSL, "state change %d -> %d", oldstate, state);
      oldstate = state;
      toLog(s);
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan21 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan21 finished");
}

void BPlan21::terminate()
{ // just close logfile
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan21::toLog(const char* message)
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

