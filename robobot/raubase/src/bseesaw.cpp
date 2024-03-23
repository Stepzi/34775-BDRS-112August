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

bool leftEdge  = true;
bool rightEdge = false;

float normalSpeed        =  0.3;   //speed under normal conditions
float lineWidth          =  0.02;  //width to determine if we are on the line
float lineGone           =  0.01;   //width to determine if the line was lost
float lineOffset         =  0;     //offset for line edge detection
float intersectionWidth  =  0.06;  //used to detect intersections
float intersectionToEdge =  0.23;
float edgeToSeesaw       =  0.84;   // distance from edge to tilting point
float edgeWidth          =  0.098;  

float speed              = 0;
float maxSpeed           = 0.5;

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

  float waitTime;
  
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
          toLog("Follow Line with velocity 0.1");
          mixer.setEdgeMode(leftEdge, lineOffset);
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
        if (medge.width > intersectionWidth)
          {
            mixer.setEdgeMode(leftEdge, lineOffset);
            toLog("found intersection");
            mixer.setVelocity(0.07);
            pose.dist = 0;
            state = 3;
          }
          else
          {    
            mixer.setVelocity(normalSpeed);                                                         
            /*-if (pose.dist > 0.1)
            {
              mixer.setVelocity(normalSpeed);
            }*/
          }
      break;

      case 3:
        if (pose.dist > intersectionToEdge)
        {
          //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("robot on the edge");
          mixer.setVelocity(0.02);
          pose.dist = 0;
          state = 4;
        }
      break;

      case 4:
        if (pose.dist > edgeWidth)
          {
            //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
            toLog("coming to tilting point");
            mixer.setEdgeMode(leftEdge, lineOffset);
            mixer.setVelocity(0.1);
            pose.dist = 0;
            state = 5;
          }
      break;

      case 5:
        if (pose.dist > edgeToSeesaw)
        {
          //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
          toLog("robot on the tilting point");
          mixer.setVelocity(0);
          t.clear();
          state = 6;
        }
      break;
      
      case 6:
        //servo.setServo(1, 1, -500, 200);
        
        waitTime = t.getTimePassed();
        if (waitTime>3){
        state = 7;}
      break;

      case 7:
        /*if (servo.servo_position[1] < -400)
        {*/
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("going down");
          mixer.setVelocity(0.05);
          state = 9;
        
      break;

      case 8:
         if(speed < maxSpeed)
        {
          speed = speed + 0.005;
          mixer.setVelocity(0.5 + speed);
          //toLog(speed);
        }
        else
        {
          toLog("Speed up");
          state = 9;
        }
        
      break;

      case 9:
        if(medge.width < lineGone)
        {
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.1);
          state = 10;
        }
      break;

      case 10:
        if(medge.width > lineWidth)
        {
          mixer.setVelocity(0);          
          pose.turned = 0;
          state = 11;
        }
      break;

      case 11:
        mixer.setTurnrate(-0.5);
        if (pose.turned < -1.3)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("back on the line");
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 12;
        }
      break;

      case 12:
        if (pose.dist > 0.3)
        {
          finished = true;
        }
      break;

      //test cases

      case 100:
      mixer.setVelocity(0);
        pose.dist = 0;
        pose.turned = 0;        
        state = 101;
      break;
      case 101:
        toLog(const_cast<char*>((std::to_string(pose.turned)).c_str()));
        mixer.setTurnrate(-0.5);
        if (pose.turned < -1.4)
        {
          state = 102;
        }

        break;

        case 102:
          finished = true;
        break;
        
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
