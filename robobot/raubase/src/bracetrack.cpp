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
  float turnSpeed = 0.8; //0.8
  float toTurn = 0.0;
  float maxSpeed = 1.2; //1.5
  float maxDist = 5.3; //around 4
  float offset = 0.0;
  float Turn90Deg = 3.14 / 2.0;
  state = 0;//TESTING
  pose.dist = maxDist;//+1.0;//TESTING
  oldstate = state; 
  //
  toLog("racetrack started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 0: //Drive to end of tape and turn around.
        if(medge.edgeValid)
        {
          pose.turned = 0.0;
          pose.dist = 0.0;
          toLog("Go to end of racetrack.");
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          mixer.setEdgeMode(false, 0.00);
          mixer.setVelocity(0.3);
          state = 1;
        }
        else{
          toLog("Expected to start on a line. Did not find the Line");
          lost = true;
        }
        break;
      case 1:
        if(!medge.edgeValid && pose.dist > 0.5)
        {
          toLog("Assume end of raceTrack is reached");
          pose.turned = 0.0;
          pose.dist = 0.0;
          mixer.setVelocity(0.0);
          mixer.setDesiredHeading(3.14);

          state = 2;
        }
      break;    
      case 2:
          //toLog(std::to_string(pose.turned).c_str());
          if(abs(pose.turned) > 3.0)
          {
            mixer.setVelocity(0.3);
            mixer.setEdgeMode(true /* lest */,  0.00 /* offset */);
            state = 3;
          }
      break;
      //SET PID kp = 5.0, lead = 0.6 0.15, taui = 0.0
      case 3: // Start Position, assume we are on a line but verify.
        if(pose.dist > 0.5 && medge.edgeValid == true) //We should be on a line 
        {
          toLog("Started on Line");
          cedge.changePID(8.0, 5.0, 0.6, 0.15, 0.0);
          mixer.setEdgeMode(true /* lest */,  0.00 /* offset */);
          mixer.setVelocity(0.0);
          pose.dist = 0;
          state = 4;
        }
        else if(t.getTimePassed() > 5)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;
      //Add identification of curves in the road. For localizing while doing the race track.
      case 4:
        if(speed < maxSpeed)
        {
          speed = speed + 0.005;
          mixer.setVelocity(0.5 + speed);
        }
        else if(speed >= maxSpeed)
        {
          toLog("Speed up");
          state = 5;
        }
        else if(!medge.edgeValid && medge.width < 0.015){
          state = 101;
        }
      break;
      case 5:
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
          pose.turned = 0.0;
          state = 6;
        }
        else if(!medge.edgeValid && medge.width < 0.015){
          pose.dist = 0.0;
          state = 101;
        }
        break;  
      case 6: 
        toLog(std::to_string(medge.edgeValid).c_str());
        if (abs(pose.turned) > Turn90Deg){//Turn 90 degrees //pose.dist > 1.8){
          toLog(std::to_string(pose.dist).c_str());
          toLog("Drive straight");
//          cedge.changePID(10.0, 30.0, 0.4, 0.2, 0.0);
          cedge.changePID(8.0, 10.0, 0.3, 0.3, 0.0);
          mixer.setEdgeMode(true,0.02);
          mixer.setVelocity(1.2);
          toLog(std::to_string(offset).c_str());
          toLog(std::to_string(speed).c_str());
          pose.dist = 0;
          pose.turned = 0;
          offset = 0;
          speed = 0;
          state = 7; 
        }
        else if (offset < 0.03){ //CHANGE OFFSET BEFORE TURN
           offset = offset + 0.00005;
           mixer.setEdgeMode(true,offset);
        }
        if(!medge.edgeValid && medge.width < 0.015){
          pose.dist = 0.0;
          state = 101;
        }
        if(speed > turnSpeed) //RAMP DOWN BEFORE TURN
        {
          speed = speed - 0.003;
          mixer.setVelocity(speed);
        }
      break;
      case 7:
          if (pose.dist > 1.8){
          toLog("Second Turn");
          //cedge.changePID(10.0, 40.0, 0.6, 0.15, 0.0);
          cedge.changePID(8.0, 30.0, 0.3, 0.3, 0.0);
          mixer.setVelocity(0.8);
          mixer.setEdgeMode(true,0.03);
          pose.dist = 0;
          pose.turned = 0;
          state = 8;
          }
        //   else if (offset < 0.03){
        //   offset = offset + 0.0001;
        //   //mixer.setEdgeMode(false,offset);
        // }
      break;
      case 8:
          if (pose.dist > 1.5 || abs(pose.turned) > Turn90Deg){
          toLog("Drive straight");
//          cedge.changePID(10.0, 20.0, 0.6, 0.15, 0.0);
          cedge.changePID(8.0, 20.0, 0.3, 0.3, 0.0);
          //mixer.setEdgeMode(false,0.00);
          mixer.setVelocity(1.0);
          pose.dist = 0;
          state = 9; 
        }
      break;
      case 9:
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
          state = 101;
        }
        break;
      case 101:
        //LOST LINE. TURN 180 Degrees and folllow line to the right. 
        if(pose.dist > 0.25){
          toLog(std::to_string(pose.turned).c_str());
          // if(pose.turned <= 0)
          //   toTurn = pose.turned + (3.14);
          // else{
          //   toTurn = pose.turned - (3.14);
          // }
          pose.turned = 0;
          toLog(std::to_string(toTurn).c_str());
          mixer.setDesiredHeading(3.14);
          mixer.setVelocity(0.1);
          mixer.setDesiredHeading(toTurn);
          pose.dist = 0;
          toTurn = 0.0;
          state = 102;
        }
        break;
      case 102:
        if(medge.edgeValid && medge.width > 0.06)
        {
          toLog("Found Line! I am vinkelret on the line");
          mixer.setVelocity(0.0);
          pose.turned = 0;
          pose.dist = 0;
          toTurn = -(3.14/2.0);
          mixer.setDesiredHeading((toTurn));
          t.clear();
          state = 103;
        }
        else if (medge.edgeValid && medge.width > 0.03){
          toLog("Found Line! I am not coming straight onto the line");
          mixer.setVelocity(0.0);
          pose.turned = 0;
          pose.dist = 0;
          toTurn = -0.5;
          mixer.setDesiredHeading((toTurn));
          t.clear();
          state = 103;
        }
        else if (medge.edgeValid && medge.width > 0.02){
          toLog("Found Line! I am straight on the line");
          mixer.setVelocity(0.0);
          pose.turned = 0;
          pose.dist = 0;
          toTurn = -(3.14/2.0);
          mixer.setDesiredHeading(toTurn);
          t.clear();
          state = 103;
        }
        else if(pose.dist > 1)
        {
          toLog("Did not Find Line after 1 M. Look for ArUcO codes.");
          lost = true;
          
        }
        break;
      case 103:
        if(pose.turned >= toTurn && (t.getTimePassed() > 1.0))
        {
          state = 104;
        }
        else if(t.getTimePassed() > 1.5)
        {
          state = 104;
        }
      break;
        case 104:
        if(!medge.edgeValid){
          mixer.setVelocity(0.1);
          mixer.setDesiredHeading(0.2);
        }
        else if(medge.edgeValid)
        {
          state = 0;
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