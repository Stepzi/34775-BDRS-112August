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
#include <iostream>

double normalSpeed        =  0.3;   //speed under normal conditions
double lineWidth          =  0.02;  //width to determine if we are on the line
double lineGone           =  0.1;   //width to determine if the line was lost
double lineOffset         =  0;     //offset for line edge detection
double intersectionWidth  =  0.05;  //used to detect intersections

double axeStop            = 0.4;    //distance to assume that axe is in front of the robot
double axeToIntersection  = 0.8;    //distance from intersection to axe start
double axeLenght          = 2;      //distance from start to finish of mission Axe
//float  axeGoneTimer       = 1;      //time to start driving after the axe is gone

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

  std::string distSens1;
  //bool tempBool1;
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
        if (dist.dist[0] < axeStop or pose.dist > axeToIntersection)
        {
          pose.resetPose();
          toLog("robot in front of axe");
          mixer.setVelocity(0);
          state = 4;
        }
      break;

      // waiting for axe
      case 4:
        mixer.setVelocity(0);

        while (dist.dist[1] >= axeStop)    //nothing in front - waiting for axe to appear
        {
          toLog ("waiting for axe");
        } 

        while (dist.dist[1] < axeStop)         //axe in front - waiting for it to be gone
        {
          toLog ("waiting for axe to pass");
        }

        toLog ("axe is gone");                     //yeet
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

      //testing the loops
      case 11:
        mixer.setVelocity(0);
        //tempBool1 = true;

        while (dist.dist[1] >= axeStop)    //nothing in front - waiting for axe to appear
        {
          toLog ("waiting for axe");
          /*if (tempBool1)
          {
            toLog ("waiting for axe");
            tempBool1 = false;
          }
          else
          {
            //wait
          }*/
        } 

        //tempBool1 = true;
        while (dist.dist[1] < axeStop)         //axe in front - waiting for it to be gone
        {
          toLog ("waiting for axe to pass");
          /*if (tempBool1)
          {
            toLog ("waiting for axe to pass");
            tempBool1 = false;
          }*/
        }
        
        toLog ("axe is gone");                      //yeet
        finished = true;
      break;

      //testing the ir sensor
      case 111: 
        mixer.setVelocity(0);
        distSens1 = std::to_string(dist.dist[1]);
        toLog(const_cast<char*>(distSens1.c_str()));
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
