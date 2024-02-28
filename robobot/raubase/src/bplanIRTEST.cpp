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

#include "bplanIRTEST.h"

// create class object
BPlanIRTEST planIRTEST;


void BPlanIRTEST::setup()
{ // ensure there is default values in ini-file
  if (not ini["PlanIRTEST"].has("log"))
  { // no data yet, so generate some default values
    ini["PlanIRTEST"]["log"] = "true";
    ini["PlanIRTEST"]["run"] = "true";
    ini["PlanIRTEST"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["PlanIRTEST"]["print"] == "true";
  //
  if (ini["PlanIRTEST"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_PlanIRTEST.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission PlanIRTEST logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlanIRTEST::~BPlanIRTEST()
{
  terminate();
}

void BPlanIRTEST::run()
{
  if (not setupDone)
    setup();
  if (ini["PlanIRTEST"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  toLog("PlanIRTEST started");;
  toLog("Time stamp, IR dist 0, IR dist 1");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 1: 
        float irDist0;
        float irDist1;

        while(t.getTimePassed() < 10){
            irDist0 = dist.dist[0];
            irDist1 = dist.dist[1];
            std::string temp_str = std::to_string(t.getTimePassed()) 
                + " " + std::to_string(irDist0)
                + " " + std::to_string(irDist1);
            toLog(temp_str.c_str());
        };
        finished = true;    
        break;

      default:
        toLog("Default Mission 0");
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
    toLog("PlanIRTEST got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanIRTEST finished");
}


void BPlanIRTEST::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlanIRTEST::toLog(const char* message)
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
