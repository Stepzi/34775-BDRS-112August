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
#include "upid.h"
#include "bracetrack.h"

// create class object
BRaceTrack racetrack;


void BRaceTrack::setup()
{ // ensure there is default values in ini-file
  if (not ini["racetrack"].has("log"))
  { // no data yet, so generate some default values
    ini["racetrack"]["log"] = "true";
    ini["racetrack"]["run"] = "true";
    ini["racetrack"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["racetrack"]["print"] == "true";
  //
  if (ini["racetrack"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_racetrack.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission racetrack logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BRaceTrack::~BRaceTrack()
{
  terminate();
}

void BRaceTrack::run()
{
  if (not setupDone)
    setup();
  if (ini["racetrack"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;

  const int MSL = 100;
  char s[MSL];
  float speed = 0;
  float maxSpeed = 1.2; //1.5
  float maxDist = 5.0; //around 4
  float offset = 0.0;
  state = 1;//TESTING
  pose.dist = maxDist;//+1.0;//TESTING
  oldstate = state; 
  //
  toLog("racetrack started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      //SET PID kp = 5.0, lead = 0.6 0.15, taui = 0.0
      case 1: // Start Position, assume we are on a line but verify.
        if(medge.width > 0.02) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          cedge.changePID(8.0, 5.0, 0.6, 0.15, 0.0);
          mixer.setEdgeMode(true /* right */,  0.00 /* offset */);
          mixer.setVelocity(0.6);
          pose.dist = 0;
          state = 2;
        }
        else if(medge.width < 0.01)
        {
          pose.resetPose();
          mixer.setVelocity(0.15);//Drive slowly and turn i circle
          mixer.setTurnrate(0.2);
        }
        else if(t.getTimePassed() > 5)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;
      //Add identification of curves in the road. For localizing while doing the race track.
      case 2:

        if(speed < maxSpeed)
        {
          speed = speed + 0.005;
          mixer.setVelocity(0.5 + speed);
        }
        else if(speed >= maxSpeed)
        {
          toLog("Speed up");
          state = 3;
        }
      break;
      case 3:
        //toLog(std::to_string(abs(pose.dist)).c_str());
        if(pose.dist > maxDist)
        {
          toLog("Reaeched Turn");
          toLog("Change PID: KP 25, Lead: 0.6 , 0.15");
          //cedge.changePID(10.0, 25.0, 0.3, 0.3, 0.0);
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          mixer.setEdgeMode(true,0.03);
          toLog(std::to_string(pose.dist).c_str());
          pose.dist = 0.0;
          state = 4;
        }
        break;  
      case 4:
        if (pose.dist > 1.8){
          toLog(std::to_string(pose.dist).c_str());
          toLog("Drive straight");
//          cedge.changePID(10.0, 30.0, 0.4, 0.2, 0.0);
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          mixer.setEdgeMode(true,0.02);
          mixer.setVelocity(0.6);
          toLog(std::to_string(offset).c_str());
          toLog(std::to_string(speed).c_str());
          pose.dist = 0;
          offset = 0;
          speed = 0;
          state = 5; 
        }
        else if (offset < 0.03){
           offset = offset + 0.00005;
           mixer.setEdgeMode(true,offset);
        }
        else if(!medge.edgeValid){
          //lost = true;
        }
        if(speed > 0.6)
        {
          speed = speed - 0.003;
          mixer.setVelocity(speed);
        }
      break;
      case 5:
          if (pose.dist > 1.8){
          toLog("Second Turn");
          //cedge.changePID(10.0, 40.0, 0.6, 0.15, 0.0);
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          mixer.setVelocity(0.6);
          mixer.setEdgeMode(true,0.03);
          pose.dist = 0;
          state = 6;
          }
        //   else if (offset < 0.03){
        //   offset = offset + 0.0001;
        //   //mixer.setEdgeMode(false,offset);
        // }
      break;
      case 6:
          if (pose.dist > 1.5){
          toLog("Drive straight");
//          cedge.changePID(10.0, 20.0, 0.6, 0.15, 0.0);
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          //mixer.setEdgeMode(false,0.00);
          mixer.setVelocity(0.6);
          pose.dist = 0;
          state = 7; 
        }
      break;
      case 7:
        if (!medge.edgeValid && medge.width < 0.015)//Gotta test the distance messured. && medge.valid == false) //A Large number will trigger on the ramp and gates
        { // something is close, assume it is the goal
          // start driving
          pose.resetPose();
          toLog("Reached finish line? que? hombre >:)");
          mixer.setVelocity(0);
          finished = true;
        }
        else if(t.getTimePassed() > 20)
        {
          toLog("I am probably driving very fast somewhere I should not. I am very lost");
          lost = true;
        }
        break;
      default:
        toLog("Default Mission RaceTrack");
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
    toLog("racetrack got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("racetrack finished");
}


void BRaceTrack::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BRaceTrack::toLog(const char* message)
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
