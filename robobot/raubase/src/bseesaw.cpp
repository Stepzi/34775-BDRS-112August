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

#include "bseesaw.h"
#include <iostream>
#include "simu.h"

double normalSpeed        =  0.3;   //speed under normal conditions
double lineWidth          =  0.02;  //width to determine if we are on the line
double lineGone           =  0.1;   //width to determine if the line was lost
double lineOffset         =  0;     //offset for line edge detection

// create class object
BSeesaw seesaw;

void BSeesaw::setup()
{ // ensure there is default values in ini-file
  if (not ini["seesaw"].has("log"))
  { // no data yet, so generate some default values
    ini["seesaw"]["log"] = "true";
    ini["seesaw"]["run"] = "true";
    ini["seesaw"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["seesaw"]["print"] == "true";
  //
  if (ini["seesaw"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_seesaw.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission seesaw logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BSeesaw::~BSeesaw()
{
  terminate();
}

void BSeesaw::run()
{
  if (not setupDone)
    setup();
  if (ini["seesaw"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false; 
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  toLog("seesaw started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {  
      case 1: // Start Position, assume we are on a line but verify.
        if(medge.width > lineWidth) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.2");
          mixer.setEdgeMode(true /* right */, lineOffset /* offset */);
          mixer.setVelocity(0.1);
          state = 2;
        }
        else if(medge.width < lineGone)
        {
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.0);//Drive slowly and turn in a circle
          mixer.setTurnrate(0.2);
        }
        else if(t.getTimePassed() > 10)
        {
          toLog("Never found Line");
          lost = true;
        }
      break;

      case 2:
        mixer.setVelocity(0.05);
      break;

    default:
      toLog("Default Seesaw");
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
    toLog("seesaw got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("seesaw finished");
}

void BSeesaw::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BSeesaw::toLog(const char* message)
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
