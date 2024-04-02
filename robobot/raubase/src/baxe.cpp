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
#include "simu.h"

double normalSpeed        =  0.3;   //speed under normal conditions
double lineWidth          =  0.02;  //width to determine if we are on the line
double lineGone           =  0.1;   //width to determine if the line was lost
double lineOffset         =  0;     //offset for line edge detection
double intersectionWidth  =  0.07;  //used to detect intersections

int    startSide          = 21;     //select the side we start from - 21 from the roundabout and 22 from the stairs
double axeStop            = 0.50;    //distance to assume that axe is in front of the robot
double axeToIntersection  = 0.7;    //distance from intersection to axe start
double axeLenght          = 2.0;      //distance from start to finish of mission Axe
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

  float speed = 0.0;

  std::string distSens1;
  std::string pose1; 
  std::string gs0;
  std::string gs1; 
  std::string gs2;

  float waitTime;
  float distanceToAxe;
  float accValue1;

  int    numberOfSamples    = 0;
  float distanceAverage[10]  = {0};
  float totalAverage = 0.0;
  toLog("axe started");
  
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
          state = startSide;
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

      //finding an intersection - roundabout side
      case 21:
        if (medge.width > intersectionWidth)
        {
          toLog("found intersection");
          mixer.setVelocity(0);
          mixer.setVelocity(0.1);
          pose.dist = 0;
          state = 50;
        }
        else
        {                                                             
          if (pose.dist > 0.1)
          {
            mixer.setVelocity(normalSpeed);
          }
        }
      break;

      //finding an intersection - goal side
      case 22:
        if (medge.width > intersectionWidth)
        {
          mixer.setEdgeMode(true, lineOffset);
          toLog("preparing to turn around");
          pose.dist = 0;
          pose.turned = 0;
          state = 23;
        }
        else
        {                                                             
          if (pose.dist > 0.1)
          {
            mixer.setVelocity(normalSpeed);
          }
        }
      break;

      //going past the intersection and then turning around
      case 23:
        if (pose.dist > 0.3)
        {
          //mixer.setVelocity(0.05);
          toLog("turning around");
          mixer.setVelocity(0);
          mixer.setTurnrate(0.5);
          //finished = true;
          state = 24;
        }
        else
        {
          mixer.setVelocity(0.3);
        }
      break;

      //continuing the track as if we started from the roundabout side
      case 24:
      mixer.setTurnrate(0.9);
        if (pose.turned > 2.8)
        {
          //finished = true;
          mixer.setTurnrate(0);
          mixer.setVelocity(0.2);
          mixer.setEdgeMode(true, lineOffset);
          state = 21;
        }
      break;

      // getting the robot to the axe.
      // based on the driven distance.

      //IF we drive at 0.1 we can drive for 0.8 meters before hitting the axe if they dont move it. at a period time of 7 seconds.
      case 50:
        if(dist.dist[1] < 0.80)
        {
          totalAverage += dist.dist[1];
          numberOfSamples += 1;
        }
        else if(dist.dist[1] > 0.8) 
        {
          numberOfSamples = 0;
          totalAverage = 0.0;
        }
        if(numberOfSamples >= 10)
        { 
          mixer.setVelocity(0.0);
          toLog("Found axe with 10 samples, average = ");
          totalAverage = totalAverage/10.0;
          toLog(std::to_string(totalAverage).c_str());
          numberOfSamples = 0.0;
          state = 51;
        }
      break;      
      case 51:
        if(totalAverage <= 0.20){
          toLog("Calculated a average distance too close to the axe.");
          mixer.setVelocity(-0.1);
          pose.dist = 0.0;
          state = 53;
        }
        else if(totalAverage >= 0.20){
          toLog("Calculated distance to far from the axe");
          mixer.setVelocity(0.1);
          pose.dist = 0.0;
          state = 52;
        }
      break;

      case 52:
        if(abs(pose.dist) >  (totalAverage - 0.20))
        {
          mixer.setVelocity(0.0);
          pose.dist = 0.0;
          totalAverage = 0.0;
          state = 41;
        }
      break;

      case 53:
          if(abs(pose.dist) >  (abs(totalAverage) - 0.20))
          {
            mixer.setVelocity(0.0);
            pose.dist = 0.0;
            totalAverage = 0.0;
            state = 41;
          }
        break;

      break;
      case 3:
        if (pose.dist > axeToIntersection )
        {
          toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
          pose.dist = 0;
          toLog("robot in front of axe");
          mixer.setVelocity(0);
          toLog ("axe timer");
          t.clear();
          state = 40;
        }
      break;

      // waiting to make sure to detect the axe
      case 40:
        waitTime = t.getTimePassed();
        //toLog(const_cast<char*>((std::to_string(waitTime)).c_str()));
        if (waitTime > 3)
        {
          toLog ("waiting for axe to appear");
          state = 41;
        }
      break;
      
      //waiting for axe to appear
      case 41: 
        distSens1 = std::to_string(dist.dist[1]);
        toLog(const_cast<char*>(distSens1.c_str()));
        if (dist.dist[1] >= axeStop )       //nothing in front 
        {
          mixer.setVelocity(0);
          distSens1 = std::to_string(dist.dist[1]);
          toLog(const_cast<char*>(distSens1.c_str()));
          numberOfSamples = 0;
        } 
        else if(dist.dist[1] < axeStop)
        {
          numberOfSamples += 1;
        }
        if(numberOfSamples > 10)
        {
          toLog ("waiting for axe to pass");
          numberOfSamples = 0;
          state = 42;
        }
      break;

      //waiting for it to be gone
      case 42:
        if (dist.dist[1] < axeStop)         //axe in front
        {
          mixer.setVelocity(0);
          distSens1 = std::to_string(dist.dist[1]);
          toLog(const_cast<char*>(distSens1.c_str()));
          numberOfSamples = 0;
        }
        else if(dist.dist[1] >= axeStop)
        {
          toLog(std::to_string(numberOfSamples).c_str());
          numberOfSamples += 1;
        }
        if (numberOfSamples >= 10)
        {
          toLog ("axe is gone");
          state = 43;
        }
      break;

      case 43:
        if(speed < 0.5) //RAMP UP
        {
          speed = speed + 0.01;
          mixer.setVelocity(speed);
        }
        else
        {
          state = 5;
          speed = 0.0;
        }
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

      //TESTS FOR DIFFERENT FUNCTIONS
      
      //testing the ir sensor
      case 111: 
        mixer.setVelocity(0);
        distSens1 = std::to_string(dist.dist[1]);
        toLog(const_cast<char*>(distSens1.c_str()));
      break;

      //testing the turn function
    case 222:
      //mixer.setEdgeMode(true, lineOffset);
      pose.turned = 0;
      pose.dist = 0;
      mixer.setVelocity(0.1);
      state = 223;
    break;

    case 223:
      if (pose.dist < 0.1)
      {
        mixer.setVelocity(0.1);
      }
      else 
      {
        finished = true;
        mixer.setVelocity(0);
        //mixer.setDesiredHeading(3); //1.5 = 90 degree turn, 3 = 180
        mixer.setTurnrate(0.5);
        state = 224;
      }
    break;

    case 224:
      mixer.setTurnrate(0.5);
      pose1 = std::to_string(pose.turned);
      toLog(const_cast<char*>(pose1.c_str()));
        if (pose.turned > 1.45)
        {
          /*mixer.setVelocity(0.2);
          mixer.setTurnrate(0.5);
          mixer.setEdgeMode(true, lineOffset);*/
          finished = true;
        }
    break;

    /*case 999:
      imu.gyro[0] = 0;
      imu.gyro[1] = 0;
      imu.gyro[2] = 0;
      state = 9999;
    break;*/

    case 999: 
      gs0 = std::to_string(imu.acc[1]);
      /*gs1 = std::to_string(imu.gyro[1]);
      gs2 = std::to_string(imu.gyro[2]);
      har a = const_cast<char*>(gs0.c_str());
      char b = const_cast<char*>(gs1.c_str());
      char c = const_cast<char*>(gs2.c_str());*/
      toLog(const_cast<char*>(gs0.c_str()));
      mixer.setVelocity(0);
      mixer.setTurnrate(2);
    break;

    case 1001:
      mixer.setEdgeMode(true /* right */, lineOffset /* offset */);
      mixer.setVelocity(0.1);
      state = 1002;
    break;

    case 1002:
        if (medge.width > intersectionWidth+0.05)
        {
          toLog("found intersection");
          mixer.setVelocity(0);
          pose.dist = 0;
          pose.turned = 0;
          state = 1003;
        }
        else
        {                                                             
          if (pose.dist > 0.1)
          {
            mixer.setVelocity(normalSpeed);
          }
        }
    break;

    case 1003:
      mixer.setTurnrate(0.9);
        if (pose.turned > 1.4)
        {
          //finished = true;
          mixer.setTurnrate(0);
          mixer.setVelocity(0.2);
          mixer.setEdgeMode(true, lineOffset);
          accValue1 = imu.acc[1];
          state = 1004;
        }
    break;   

    case 1004:
      if (abs(imu.acc[1]) > accValue1 + 0.2)
      {
        toLog("step detected");
        finished = true;
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
