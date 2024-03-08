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

#include "baxe.h"

double normalSpeed        =  0.3;   //speed under normal conditions
double lineWidth          =  0.02;  //width to determine if we are on the line
double lineGone           =  0.1;   //width to determine if the line was lost
double lineOffset         =  0;  //offset for line edge detection
double intersectionWidth  =  0.05;  //used to detect intersections

double stopDistance       = 0.2;   //distance to assume that there is something in front of the robot
double intersectionToAxe  = 0.9;    //distance from intersection to axe start
double axeLenght          = 2;      //distance from start to finish of mission Axe

// create class object
BAxe axe;


void BAxe::setup()
{ // ensure there is default values in ini-file
  if (not ini["axe"].has("log"))
  { // no data yet, so generate some default values
    ini["axe"]["log"] = "true";
    ini["axe"]["run"] = "true";
    ini["axe"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["axe"]["print"] == "true";
  //
  if (ini["axe"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_axe.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission axe logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BAxe::~BAxe()
{
  terminate();
}

void BAxe::run()
{
  if (not setupDone)
    setup();
  if (ini["axe"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  //
  toLog("axe started");
  //
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
          mixer.setVelocity(0.0);//Drive slowly and turn i circle
          mixer.setTurnrate(0.2);
        }
        else if(t.getTimePassed() > 10)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;

      //finding an intersection
      case 2:
        if (medge.width > intersectionWidth)
        {
          toLog("found intersection");
          mixer.setVelocity(0);
          //mixer.setEdgeMode(true, lineOffset);
          mixer.setVelocity(0.1);
          pose.dist = 0;
          state = 3;
        }

        else
        {
          if (pose.dist > 0.1)
          {
            mixer.setVelocity(normalSpeed);
          }
        }
        break;

    // getting the robot to the axe
    // based on the driven distance.
    // the distance sensor is for 
    // additional safety
      case 3:
        if (dist.dist[0] < stopDistance or pose.dist > intersectionToAxe)
        {
          pose.resetPose();
          toLog("robot in front of axe");
          mixer.setVelocity(0);
          state = 4;
        }
        break;

      // waiting for axe
      case 4:
        //float currentDistToAxe = dist.dist[0];
        bool tempBool = true;

        mixer.setVelocity(0);
        while (dist.dist[0] >= stopDistance * 2)  //nothing in front - waiting for axe to appear
        {
          if (tempBool) 
          {
            toLog ("waiting for axe");
            tempBool = false;
          }
        } 
        
        tempBool = true;
        while (dist.dist[0] <= stopDistance)        //axe in front - waiting for it to be gone
        {
          if (tempBool) 
          {
            toLog ("waiting for axe to pass");
            tempBool = false;
          }
        }

        toLog ("axe is gone");                      //yeet
        mixer.setVelocity(0.5);
        state = 5;
        break;

      // deciding when to finish the mission
      case 5:
        if(pose.dist > axeLenght) //lenght of the axe
        {
          mixer.setVelocity(0);
          mixer.setTurnrate(0);
          finished = true;
        }
        //probably not needed
        else if (t.getTimePassed() > 30)
        {
          toLog("Gave up waiting for Regbot");
          lost = true;
        }
        break;

      default:
        toLog("Default Axe");
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
    toLog("axe got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("axe finished");
}

void BAxe::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BAxe::toLog(const char* message)
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
