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
  float maxSpeed = 1; //1.5
  float maxDist = 5.5; //around 4
  float offset = 0.0;
  state = 3;//TESTING
  pose.dist = maxDist+1.0;//TESTING
  oldstate = state; 
  //
  toLog("racetrack started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 1: // Start Position, assume we are on a line but verify.
        if(medge.width > 0.02) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          mixer.setEdgeMode(false /* right */,  0.0 /* offset */);
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 2;
        }
        else if(medge.width < 0.01)
        {
          pose.resetPose();
          mixer.setVelocity(0.0);//Drive slowly and turn i circle
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

        if(pose.dist < maxDist  && speed < maxSpeed)
        {
          speed = speed + 0.01;
          mixer.setVelocity(0.5 + speed);
        }
        else if(speed >= maxSpeed)
        {
          toLog("Reached Turn");
          state = 3;
        }
      break;
      case 3:
        //toLog(std::to_string(abs(pose.dist)).c_str());
        if(abs(pose.dist) > maxDist)
        {
          mixer.setVelocity(0.5);
          toLog("Change PID: KP 40, Lead: 0.6 , 0.15");
          cedge.changePID(10.0, 40.0, 0.3, 0.5, 0.0);
          //pid.setup();
          toLog(std::to_string(pose.dist).c_str());
          pose.dist = 0.0;
          state = 4;
        }
        break;  
      case 4:
        if (pose.dist > 2.5){
          toLog(std::to_string(pose.dist).c_str());
          toLog("Drive straight");
          cedge.changePID(10.0, 10.0, 0.6, 0.3, 0.0);
          mixer.setEdgeMode(false,0.00);
          mixer.setVelocity(1.0);
          pose.dist = 0;
          offset = 0;
          speed = 0;
          state = 5; 
        }
        else if (offset < 0.02){
          offset = offset + 0.0001;
          mixer.setEdgeMode(false,offset);
        }
        else if(!medge.edgeValid){
          //lost = true;
        }
      break;
      case 5:
          if (pose.dist > 1.5){
          toLog("Second Turn");
          cedge.changePID(10.0, 40.0, 0.6, 0.3, 0.0);

          mixer.setVelocity(0.5);
          mixer.setEdgeMode(false,0.02);
          pose.dist = 0;
          state = 6;
          }
      break;
      case 6:
          if (pose.dist > 1.5){
          toLog("Drive straight");
          cedge.changePID(10.0, 10.0, 0.6, 0.3, 0.0);
          mixer.setEdgeMode(false,0.00);
          mixer.setVelocity(0.8);
          pose.dist = 0;
          state = 7; 
        }
      break;
      case 7:
        if (!medge.edgeValid)//Gotta test the distance messured. && medge.valid == false) //A Large number will trigger on the ramp and gates
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
