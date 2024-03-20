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

#include "bStairs.h"

// create class object
BStairs stairs;


void BStairs::setup()
{ // ensure there is default values in ini-file
  if (not ini["stairs"].has("log"))
  { // no data yet, so generate some default values
    ini["stairs"]["log"] = "true";
    ini["stairs"]["run"] = "true";
    ini["stairs"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["stairs"]["print"] == "true";
  //
  if (ini["stairs"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_stairs.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission stairs logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BStairs::~BStairs()
{
  terminate();
}


void BStairs::run(bool entryDirectionStart, bool exitDirectionStart)
{
  if (not setupDone)
    setup();
  if (ini["stairs"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  if(entryDirectionStart == true)
  {
    state = 0;
  }
    else
  {
    state = 0;
  }
  float f_LineWidth_Crossing = 0.05;

  float f_Line_LeftOffset = 0;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;
  float f_Velocity_DriveForward = 0.25; 
  float f_Velocity_DriveSlow = 0.15;
  float f_Velocity_DriveBack = -0.15;
  int servoDown = -200;
  int servoSpeed = 400;
  oldstate = state;

  //
  toLog("Plan Stairs started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission
      case 0:
        if(medge.edgeValid)
        {
          toLog("Started on a line");
          state = 1;
        }
        else if(!medge.edgeValid)
        {
          toLog("Did not find Line");
          lost = true; 
        }
        break;
      case 1:
          toLog("Start Driving - Start-side from Start intersection");
          mixer.setEdgeMode(b_Line_HoldRight, 0 );
          mixer.setVelocity(f_Velocity_DriveForward); 
          pose.dist = 0.0;
          pose.turned = 0.0;
          state = 2;
        break;
      case 2:
          if(medge.edgeValid && (medge.width > f_LineWidth_Crossing))
          {
            toLog("Crossed intersection to seesaw - continue until next intersection");
            mixer.setVelocity(f_Velocity_DriveSlow);
            state = 3;
          }
        break;
      case 3:
          if(medge.edgeValid && (medge.width > f_LineWidth_Crossing))
          {
            toLog("Reached Stairs Intersection");
            pose.dist = 0;
            pose.turned = 0;
            mixer.setEdgeMode(b_Line_HoldLeft, 0);
            state = 4;
          }
        break;
      case 4:
        if(pose.dist > 0.3){
          toLog("Reached start of Stairs, put down servo");
          mixer.setVelocity( 0.0 );
          servo.setServo(1, 1, servoDown, servoSpeed);
          t.clear();
          state = 5;
        }
        break;
      case 5:
        if(t.getTimePassed() > 2 || servo.servo_position[1] <= servoDown){
          toLog("Servo Is Down, drive forward");
          mixer.setVelocity(f_Velocity_DriveSlow);
          pose.dist = 0;
          state = 6;
        }
        break;
      case 6:
        if(pose.dist > 0.3 /*|| imu_data z axis*/)
        {
          toLog("Down First Step, drive Back");
          mixer.setVelocity(f_Velocity_DriveBack);
          state = 7;
          t.clear();
        }        
        break;
      case 7:
        if(t.getTimePassed() > 1){
          toLog("Backed up aginst first step");
          mixer.setVelocity(0.0);
          finished = true;
        }
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
    toLog("Plan Stairs got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan Stairs finished");
}


void BStairs::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BStairs::toLog(const char* message)
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

