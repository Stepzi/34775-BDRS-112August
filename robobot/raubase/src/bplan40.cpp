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

#include "bplan40.h"

// create class object
BPlan40 plan40;


void BPlan40::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan40"].has("log"))
  { // no data yet, so generate some default values
    ini["plan40"]["log"] = "true";
    ini["plan40"]["run"] = "false";
    ini["plan40"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan40"]["print"] == "true";
  //
  if (ini["plan40"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan40.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan40 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan40::~BPlan40()
{
  terminate();
}

void BPlan40::run()
{
  if (not setupDone)
    setup();
  if (ini["plan40"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 5;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  toLog("Plan40 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 5: // wait for Regbot, then go forward
        if (dist.dist[0] < 0.25)
        { // something is close, assume it is the Regbot
          // start driving
          pose.resetPose();
          toLog("forward 0.1 m/sec");
          mixer.setVelocity(0.25);
          mixer.setTurnrate(0);
          state = 12;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("Gave up waiting for Regbot");
          lost = true;
        }
        break;
      case 12: // forward until distance, then look for edge
        if (pose.dist > 0.3)
        {
          toLog("Continue until edge is found");
          state = 20;
          pose.dist = 0;
        }
        else if (t.getTimePassed() > 10)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec");
          lost = true;
        }
        break;
      case 20: // forward looking for line, then turn
        if (medge.width > 0.05)
        {
          toLog("found line, turn left");
          // set to edge control, left side and 0 offset
          mixer.setVelocity(0.2); // slow
          mixer.setTurnrate(1.0); // rad/s
          state = 30;
          pose.dist = 0;
          pose.turned = 0;
        }
        else if (t.getTimePassed() > 10 or pose.dist > 0.6)
        { // line should be found within 10 seconds, else lost
          toLog("failed to find line after 10 sec / 30cm");
          lost = true;
        }
        break;
      case 30: // Continue turn until right edge is almost reached, then follow right edge
        if (medge.edgeValid and medge.rightEdge > -0.04 and pose.turned > 0.3)
        {
          toLog("Line detected, that is OK to follow");
          mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
          mixer.setVelocity(0.3);
          state = 40;
          pose.dist = 0;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("Time passed, no crossing line");
          lost = true;
        }
        else if (pose.dist > 1.0)
        {
          toLog("Driven too long");
          state = 90;
        }
        break;
      case 40: // follow edge until crossing line, the go straight
        if (medge.width > 0.075 and pose.dist > 0.2)
        { // go straight
          mixer.setTurnrate(0);
          pose.dist = 0;
          state = 50;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          finished = true;
        }
        else if (not medge.edgeValid)
        {
          toLog("Lost line");
          state = 80;
        }
        break;
      case 50: // continue straight until wall is close
        if (dist.dist[0] < 0.15)
        { // wall found
          toLog("wall found");
          mixer.setVelocity(0);
          finished = true;
        }
        else if (t.getTimePassed() > 10)
        {
          toLog("too long time");
          lost = true;
        }
        else if (pose.dist > 1.5)
        {
          toLog("too far");
          lost = true;
        }
        break;
      default:
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
    toLog("Plan40 got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan40 finished");
}


void BPlan40::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan40::toLog(const char* message)
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
