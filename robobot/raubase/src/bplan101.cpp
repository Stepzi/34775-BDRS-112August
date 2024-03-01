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
#include <opencv2/calib3d.hpp>
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
#include "maruco.h"
#include "scam.h"


#include "bplan101.h"

// create class object
BPlan101 plan101;


void BPlan101::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan101"].has("log"))
  { // no data yet, so generate some default values
    ini["plan101"]["log"] = "true";
    ini["plan101"]["run"] = "false";
    ini["plan101"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan101"]["print"] == "true";
  //
  if (ini["plan101"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan101.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan101 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan101::~BPlan101()
{
  terminate();
}



void BPlan101::run()
{
  if (not setupDone)
    setup();
  if (ini["plan101"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  //
  toLog("Plan101 started");
  int count = 0;
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // Test ArUco plan
      case 10:
      { // brackets to allow local variables

        int n = aruco.IDs.size();
        
        // save as local copy
        std::vector<cv::Vec3d> pos_m = aruco.pos_m;
        std::vector<cv::Vec3d> rot_m = aruco.rot_m;
        std::vector<int> IDs = aruco.IDs;
        UTime fixTime = aruco.fixTime;

        const int MSL = 200;
        char s[MSL];
        snprintf(s, MSL, "# Last Aruco codes found: %lu.%04lu",fixTime.getSec(),fixTime.getMicrosec()/100);
        toLog(s);

        for (int i = 0; i < n; i++)
        { 
          if (logfile != nullptr or toConsole)
          {
            const int MSL = 200;
            char s[MSL];
            snprintf(s, MSL, "# ArUco (%d, %d) in robot coordinates (x,y,z) = (%g %g %g)", i, IDs[i], pos_m[i][0], pos_m[i][1], pos_m[i][2]);
            toLog(s);
            snprintf(s, MSL, "# Aruco angles in robot coordinates (roll = %.1f deg, pitch = %.1f deg, yaw = %.1f deg)", rot_m[i][0], rot_m[i][1], rot_m[i][2]);
            toLog(s);
          }
        }
        if (n == 0 and (logfile != nullptr or toConsole))        {

          const int MSL = 200;
          char s[MSL];
          snprintf(s, MSL, "# No Aruco code found yet");
          toLog(s);

          
        }
        count++;
        // repeat 4 times (to get some statistics)
        // if (count > 3)
        //   finished = true;
        break;
      }
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
    usleep(1000000); //Sleep increased
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan101 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan101 finished");
}


void BPlan101::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan101::toLog(const char* message)
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

