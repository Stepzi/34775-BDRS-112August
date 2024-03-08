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
#include "mgolfball.h"

#include "bgolfballtest.h"

// create class object
bgolfballtest golfballtest;


void bgolfballtest::setup()
{ // ensure there is default values in ini-file
  if (not ini["golfballtest"].has("log"))
  { // no data yet, so generate some default values
    ini["golfballtest"]["log"] = "true";
    ini["golfballtest"]["run"] = "false";
    ini["golfballtest"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["golfballtest"]["print"] == "true";
  //
  if (ini["golfballtest"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_golfballtest.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission golfballtest logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

bgolfballtest::~bgolfballtest()
{
  terminate();
}


void bgolfballtest::run()
{
  if (not setupDone)
    setup();
  if (ini["golfballtest"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  //
  toLog("golfballtest started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission
      case 10:

        int center = {0,0};
        if(Mgolfball::findGolfball(center)){
            
            const int MSL = 200;
            char s[MSL];

            snprintf(s, MSL, "Golfball found at X = %d, Y = %d", center[0], center[1]);
            toLog(s);

            // turn kp times error from center
        }
      case 11: // wait for distance
        if (pose.dist >= 1.0)
        { // done, and then
          finished = true;
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
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("golfballtest got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("golfballtest finished");
}


void bgolfballtest::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void bgolfballtest::toLog(const char* message)
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
