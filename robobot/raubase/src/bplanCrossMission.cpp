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
#include "simu.h"

#include "bplanCrossMission.h"

// create class object
BPlanCrossMission planCrossMission;


void BPlanCrossMission::setup()
{ // ensure there is default values in ini-file
  if (not ini["PlanCrossMission"].has("log"))
  { // no data yet, so generate some default values
    ini["PlanCrossMission"]["log"] = "true";
    ini["PlanCrossMission"]["run"] = "true";
    ini["PlanCrossMission"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["PlanCrossMission"]["print"] == "true";
  //
  if (ini["PlanCrossMission"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_PlanCrossMission.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission PlanCrossMission logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlanCrossMission::~BPlanCrossMission()
{
  terminate();
}

void BPlanCrossMission::run_StartToFirstCross()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.09;

  float f_Line_LeftOffset = 0.02;
  //float f_Line_RightOffset = -0.03;
  bool b_Line_HoldLeft = true;
  //bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.3; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;

  int woodWhite = 600;
  int blackWhite = 350;
  
  //
  toLog("PlanCrossMission started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify.
        if(medge.width > f_LineWidth_MinThreshold) //We should be on a line 
        {
          medge.updateCalibBlack(medge.calibBlack,8);
          medge.updatewhiteThreshold(blackWhite);
          servo.setServo(2, true, -900, 200);
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.25"); //Some parse of float to log, Villiams :)
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          mixer.setVelocity(f_Velocity_DriveForward); 
          state = 2; // #########################################################################
        }
        else if(medge.width < f_LineWidth_NoLine)
        {
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.01);//Drive slowly and turn i circle
          mixer.setTurnrate(1.0);
        }
        else if(t.getTimePassed() > f_Time_Timeout)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;

      case 20:
        if(abs(pose.turned) > M_PI){
          state = 21;
          pose.turned = 0;
        }

        break;

      case 21:
        pose.resetPose();
        mixer.setVelocity(0);
        heading.setMaxTurnRate(0.7);
        mixer.setTurnrate(0.5);
        usleep(10000);
        state = 22;
        break;

      case 22:
      // std::cout << pose.turned/M_PI << std::endl;
        if(pose.turned > M_PI*0.9 && medge.edgeValid){
          pose.resetPose();
          heading.setMaxTurnRate(5);
          mixer.setEdgeMode(b_Line_HoldLeft, 0);
          mixer.setVelocity(f_Velocity_DriveForward/2); 
          state = 20;
        }
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(medge.width > f_LineWidth_Crossing) 
        { 
          // start driving
          toLog("First split found");
          pose.dist=0;
          finished = true;   
        }
        //If line missed, reverse and it is tried to be detected again with a lower line width measure. If missed again this continues.
        else if(abs(pose.dist) > f_Distance_FirstCrossMissed)
        {
            pose.dist=0;
            mixer.setVelocity(f_Velocity_DriveBackwards); 
            f_LineWidth_Crossing -= 0.005;
        }
        break;

      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}


void BPlanCrossMission::run_RoundaboutToAxe()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 1;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.013;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.09;

  float f_Line_LeftOffset = 0.03;
  float f_Line_RightOffset = -0.03;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.30; 
  //float f_Velocity_DriveBackwards = -0.15; 
  //float f_Distance_FirstCrossMissed = 10;
 
  //
  toLog("PlanCrossMission2 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify.
        toLog(std::to_string(medge.width).c_str());
        if(medge.width > f_LineWidth_MinThreshold) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.25"); //Some parse of float to log, Villiams :)
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          mixer.setVelocity(f_Velocity_DriveForward); 
          state = 2;
        }
        else if(medge.width < f_LineWidth_NoLine)
        {
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.01);//Drive slowly and turn i circle
          mixer.setTurnrate(1.0);
        }
        else if(t.getTimePassed() > f_Time_Timeout)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(medge.width > f_LineWidth_Crossing) 
        { 
          // start driving
          toLog("First split found");
          pose.turned = 0;
          mixer.setVelocity(0);
          mixer.setTurnrate(1);
          state = 3;
        }
        //If line missed, reverse and it is tried to be detected again with a lower line width measure. If missed again this continues.
        break;
      case 3: 
        toLog(std::to_string(pose.turned).c_str());
        if(pose.turned > 0.5) 
        { 
          // start driving
          toLog("Turning done");
          mixer.setVelocity(0.3);
          state = 4;
        }
        //If line missed, reverse and it is tried to be detected again with a lower line width measure. If missed again this continues.
        break;
      case 4:
        if(medge.width > f_LineWidth_Crossing) 
        { 
          // start driving
          toLog("Second split found");
          pose.dist=0;
          finished = true;   
        }
      break;

      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}

void BPlanCrossMission::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlanCrossMission::toLog(const char* message)
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

void BPlanCrossMission::run_AxeToTunnel()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 3;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.09;

  // Untested But reads calibration values from medge module
  
  // wood = medge.calibWood;
  // black = medge.calibBlack;
  
  int wood[8]  = {384, 479, 495, 467, 506, 506, 463, 391};
  int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

  int woodWhite = 600;
  int blackWhite = 350;

  float f_Line_LeftOffset = 0;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.3; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
 
  //
  toLog("PlanCrossMission started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify
        pose.dist = 0;
        pose.turned = 0;
        medge.updateCalibBlack(medge.calibWood,8);
        medge.updatewhiteThreshold(woodWhite);
        usleep(1000);
        mixer.setDesiredHeading(0.8);
        state = 2;
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(pose.turned > 0.78) 
        { 
          mixer.setVelocity(0.3);
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          pose.dist = 0;
          state = 3;
        }
      break;

      case 3:
          mixer.setVelocity(0.3);
          heading.setMaxTurnRate(3);
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          mixer.setEdgeMode(b_Line_HoldLeft,f_Line_LeftOffset);
          state = 4;
      break;

      case 4:
        if(medge.width > f_LineWidth_Crossing) //We should be on a line 
        {
          mixer.setVelocity(0);
          pose.resetPose();
          heading.setMaxTurnRate(1);
          mixer.setDesiredHeading(-1.2);
          state = 5;
        }
      break;

      case 5:
        if(abs(pose.turned) > 1.2-0.02) //We should be on a line 
        {
          pose.dist = 0;
          mixer.setVelocity(0.3);
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight, f_Line_LeftOffset);
          state = 6;
        }
      break;

      case 6:
        if(abs(pose.dist) > 0.3) //We should be on a line 
        {
          mixer.setVelocity(0);
          pose.resetPose();
          pose.turned = 0;
          heading.setMaxTurnRate(1);
          mixer.setDesiredHeading(3.2);
          state = 7;
        }
      break;

      case 7:
        if(abs(pose.turned) > 3) //We should be on a line 
        {
          pose.dist = 0;
          heading.setMaxTurnRate(3);
          mixer.setVelocity(0);
          finished = true;
        }
      break;
      // case 8:
      //   if(abs(pose.dist) > 0.2) //We should be on a line 
      //   {
      //     mixer.setVelocity(0.1);
      //     finished = true;
      //   }
      // break;
      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}
  

void BPlanCrossMission::run_AxeToRace()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 3;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.09;
  
  int wood[8]  = {384, 479, 495, 467, 506, 506, 463, 391};
  int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

  int woodWhite = 600;
  int blackWhite = 350;

  float f_Line_LeftOffset = 0;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.3; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
 
  //
  toLog("PlanCrossMission started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify
        pose.dist = 0;
        pose.turned = 0;
        medge.updateCalibBlack(medge.calibWood,8);
        medge.updatewhiteThreshold(woodWhite);

        mixer.setDesiredHeading(0.8);
        state = 2;
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(pose.turned > 0.78) 
        { 
          mixer.setVelocity(0.2);
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          pose.dist = 0;
          state = 3;
        }
      break;

      case 3:
          mixer.setVelocity(0.2);
          heading.setMaxTurnRate(3);
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          mixer.setEdgeMode(b_Line_HoldLeft,f_Line_LeftOffset);
          state = 4;
      break;

      case 4:
        if(medge.width > f_LineWidth_Crossing) //We should be on a line 
        {
          mixer.setVelocity(0);
          pose.resetPose();
          heading.setMaxTurnRate(1);
          mixer.setDesiredHeading(-1.2);
          state = 5;
        }
      break;

      case 5:
        if(abs(pose.turned) > 1.2-0.02) //We should be on a line 
        {
          pose.dist = 0;
          mixer.setVelocity(0.1);
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight, f_Line_LeftOffset);
          state = 6;
        }
      break;

      case 6:
        if(abs(pose.dist) > 0.7) //We should be on a line 
        {
          mixer.setVelocity(0);
          pose.resetPose();
          pose.turned = 0;
          heading.setMaxTurnRate(1);
          mixer.setDesiredHeading(3.2);
          state = 7;
        }
      break;

      case 7:
        if(abs(pose.turned) > 3) //We should be on a line 
        {
          pose.dist = 0;
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight, 0);
          mixer.setVelocity(0.2);
          state = 8;
        }
      break;
      
      case 8:
          if(medge.width > 0.06){
             toLog("Found crossing, change line sensor thresholds");
            pose.dist = 0;
            pose.turned = 0;
            state = 101;
          }
      break;

      case 101:
        if(pose.dist > 0.30)
        {
          toLog("30 cm after crossing, i am on black floor now");
          medge.updateCalibBlack(medge.calibBlack,8);
          medge.updatewhiteThreshold(blackWhite);
          pose.turned = 0;
          pose.dist = 0;
          finished = true;
        }
      break;
      // case 8:
      //   if(abs(pose.dist) > 0.2) //We should be on a line 
      //   {
      //     mixer.setVelocity(0.1);
      //     finished = true;
      //   }
      // break;
      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}




void BPlanCrossMission::run_RaceEndToTunnel()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 0;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.07;
  
 int wood[8]  = {317, 397, 424, 401, 418, 414, 395, 344};
  int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

  int woodWhite = 600;
  int blackWhite = 350;

  float f_Line_LeftOffset = 0;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.3; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
  //
  toLog("Plan go back to tunnel - started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {

      case 0:
          toLog("Start Race End to Tunnel");
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          sleep(1);
          state = 1;  
      break;
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify
        pose.dist = 0;
        pose.turned = 0;
        pose.resetPose();
        heading.setMaxTurnRate(1);
        mixer.setDesiredHeading(3.1);
        state = 2;
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(abs(pose.turned) > 3.1-0.02) 
        { 
          mixer.setVelocity(0.2);
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight, -0.02);
          pose.dist = 0;
          state = 3;
        }
      break;

      case 3:
        if(pose.dist > 0.5) //We should be on a line 
        {
          mixer.setVelocity(0.35);
          state = 4;
        }
      break;

      case 4:
      toLog(std::to_string(pose.dist).c_str());
        if(medge.width > f_LineWidth_Crossing && pose.dist > 4.9) //We should be on a line 
        {
          pose.dist = 0; //DONT REMOVE, DIST USED FROM STATE 2 IN CLOSE GATE
          pose.turned = 0;
          finished = true;
        }
      break;

      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}



void BPlanCrossMission::run_TunnelToGoal()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 0;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.07;
  
  int wood[8]  = {384, 479, 495, 467, 506, 506, 463, 391};
  int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

  int woodWhite = 600;
  int blackWhite = 350;

  float f_Line_LeftOffset = 0.02;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;
  int noLine = 0;
  //Postion and velocity data
  float f_Velocity_DriveForward = 0.3; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
 
  medge.updateCalibBlack(medge.calibWood,8);
  medge.updatewhiteThreshold(woodWhite);
  sleep(1);

  //
  toLog("Plan go back to goal from tunnel - started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {

      case 0:
          toLog("Start Race End to Tunnel");
          state = 1;  
      break;
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0.3);
        heading.setMaxTurnRate(3);
        mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
        state = 2;
        break;

      //Case 2 - first crossing on the track
      case 2:
        if(medge.width > 0.05) 
        { 
          toLog("Found first crossing");
          mixer.setVelocity(0.3);
          pose.dist = 0;
          state = 10;

        }
      break;

      case 10:
        if(pose.dist > 1.0)
        {
          pose.dist = 0.0;
          pose.turned = 0.0;
          mixer.setVelocity(0.25);
          mixer.setEdgeMode(f_Line_RightOffset,0);
          state = 3;
        }
      break;

      case 3:
        if(!medge.edgeValid && medge.width < 0.02 && pose.dist > 2) //We should be on a line 
        {
          toLog("Lost Line assume I am between goal and racetrack End");
          // Modified by SEB
          // mixer.setVelocity(0.2);
          finished = true;
          // state = 4;
        }
      break;

      case 4:
        if(medge.edgeValid && medge.width > 0.09)
        {
          toLog("Found Line in frong of Goal");
          pose.dist = 0;
          pose.turned = 0;
          state = 5;
        }
      break;

      case 5:
        if(abs(pose.dist) > 0.05) //We should be on a line 
        {
          pose.resetPose();
          heading.setMaxTurnRate(1);
          mixer.setVelocity(0.0);
          mixer.setDesiredHeading(1.57);
          state = 6;
        }
      break;

      case 6:
        if(abs(pose.turned) > 1.57 - 0.02)
        {
          heading.setMaxTurnRate(3);
          toLog("In front of goal");
          finished = true;
        }
      break;

      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}


void BPlanCrossMission::run_GoalToFirstCross()
{
  if (not setupDone)
    setup();
  if (ini["PlanCrossMission"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 0;
          // mixer.setVelocity(0.3);
          // mixer.setEdgeMode(true, 0.02);
          // pose.dist = 2.8;
          // medge.updateCalibBlack(medge.calibBlack,8);
          // medge.updatewhiteThreshold(350);

  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  float f_LineWidth_MinThreshold = 0.02;
  float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.07;
  
  int wood[8]  = {384, 479, 495, 467, 506, 506, 463, 391};
  int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

  int woodWhite = 600;
  int blackWhite = 350;

  float f_Line_LeftOffset = 0.02;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  float f_Time_Timeout = 10.0;
  int noLine = 0;
  //Postion and velocity data
  float f_Velocity_DriveForward = 0.4; 
  float f_Velocity_DriveBackwards = -0.15; 
  float f_Distance_FirstCrossMissed = 10;
 
  medge.updateCalibBlack(medge.calibWood,8);
  medge.updatewhiteThreshold(woodWhite);
  sleep(1);

  //
  toLog("Plan go back to goal from tunnel - started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {

      case 0:
          toLog("Goal to First Cross");
          state = 1;  
      break;
      //Case 1 - Starting with error handling if no line found
      case 1: // Start Position, assume we are on a line but verify
        pose.resetPose();
        mixer.setVelocity(0.15);
        // heading.setMaxTurnRate(3);
        heading.setMaxTurnRate(0.8);
        mixer.setDesiredHeading(3.14/4*1.1);
        // mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
        state = 2;
        break;

      case 2:
        if(pose.turned > 3.14/4*0.7) 
        { 
          toLog("turned");
          heading.setMaxTurnRate(3);
          mixer.setVelocity(0.07);
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
          

          // mixer.setVelocity(f_Velocity_DriveForward);
          state = 3;

        }  
      break;

      case 3:
        if(medge.width > 0.06) 
        { 
          toLog("intercepted line after goal");
          pose.dist = 0;

          state = 302;

        }
      break;
      // case 2:
      //   if((medge.width > 0.06) && (pose.dist > 0.1)) 
      //   { 
      //     toLog("intercepted line after goal");
      //     heading.setMaxTurnRate(1);
      //     mixer.setVelocity(0.1);
      //     pose.resetPose();

      //     // mixer.setVelocity(f_Velocity_DriveForward);
      //     state = 20;

      //   }
      // break;

      // case 20:
      //   if(pose.dist > 0.05){
      //     mixer.setVelocity(0);
      //     pose.resetPose();
      //     mixer.setDesiredHeading(3.1415*0.9);
      //     state = 30;
      //   }

      // break;


      // case 30:
      // if(pose.turned > 3.1415/2 *0.9 && medge.edgeValid){
      //   heading.setMaxTurnRate(3);
      //   pose.resetPose();
      //   state = 3;
      //   pose.dist = 0;
      // }

      // break;

      // //Case 3 - second crossing on the track
      // case 3:
      //   mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset);
      //   state = 301;
      // break;

      // case 301:
      //    mixer.setVelocity(0.1);
      //    pose.dist = 0;
      //    state = 302;
        
      // break;

      case 302:
        if(pose.dist > 0.4){
          //  finished = true;
          mixer.setVelocity(f_Velocity_DriveForward);
          state = 4;
        }

      break;
  

      case 4:      
        if(pose.dist > 1 || (imu.acc[0] > 0.6 && t.getTimePassed() > 1)){
            toLog("On Ramp");
            medge.updateCalibBlack(medge.calibBlack,8);
            medge.updatewhiteThreshold(blackWhite);
            pose.dist = 0;
            state = 42;
        }
        break;

      case 42:
      // toLog(std::to_string(medge.width).c_str());

        if(medge.width > 0.06){
          toLog("Stairs Intersection");
          pose.dist = 0;
          state = 43;
        }

      break;

      case 43:
        if(medge.width > 0.06 && pose.dist > 0.5){
          toLog("Seesaw Intersection");
          pose.dist = 0;
          state = 40;
        }

      break;

      
       case 40:
        toLog(std::to_string(pose.dist).c_str());
        if(abs(pose.dist) > 2.7)
          {
            pose.dist = 0.0;
            toLog("Change Calibration - Wood");
            medge.updateCalibBlack(medge.calibWood,8);
            medge.updatewhiteThreshold(woodWhite);
            mixer.setVelocity(0.25);
            state = 400;
            pose.dist = 0;
          }

      break; 

      case 400:
        // toLog(std::to_string(pose.dist).c_str());
        if(pose.dist > 1.5)
          {
            pose.dist = 0.0;
            toLog("Change Calibration - Black");
            medge.updateCalibBlack(medge.calibBlack,8);
            medge.updatewhiteThreshold(blackWhite);
            mixer.setVelocity(f_Velocity_DriveForward);
            state = 41;
          }

      break;

      case 41:
        if(pose.dist > 3){
          toLog("Drive 3m forward, open loop through gate");
          mixer.setVelocity(0);
          usleep(4000);
          mixer.setTurnrate(0);
          // pose.resetPose();
          state = 411;
        }
        break;

      case 411:
        pose.h = 0;
        pose.turned = 0;
        // finished = true;     
        state = 4121;
        break;

      case 4121:
        mixer.setTurnrate(-0.5);  
        state = 412;
      break;

      case 412:
      // toLog(std::to_string(medge.width).c_str());
      // toLog(std::to_string(pose.turned).c_str());
        if(abs(pose.turned) > 3.1415*0.9 && medge.edgeValid && medge.width > 0.01){
          // pose.resetPose();
          mixer.setTurnrate(0);         
          state = 413;
          
        }

        break;
      case 413:
        // finished = true;
          heading.setMaxTurnRate(3);
          mixer.setVelocity(0.07);
          pose.dist = 0;
          mixer.setEdgeMode(b_Line_HoldLeft, -f_Line_LeftOffset/2);
          state  = 414;

        break;

      case 414:
        if(pose.dist > 0.2){
          mixer.setVelocity(f_Velocity_DriveForward);
          pose.turned = 0;
          pose.dist = 0;
          state =  5;
        }

        break;

      case 5:
        toLog(std::to_string(pose.turned).c_str());
        if(pose.turned > 3.1415/4 || pose.dist > 2.5){
          pose.dist = 0.0;
          toLog("Change Calibration - Wood");
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          mixer.setVelocity(0.25);
          state = 6;
          pose.dist = 0;
        }
      break;

      case 6:
        // toLog(std::to_string(pose.dist).c_str());
        if(pose.dist > 1.2)
        {
          pose.dist = 0.0;
          toLog("Change Calibration - Black");
          medge.updateCalibBlack(medge.calibBlack,8);
          medge.updatewhiteThreshold(blackWhite);
          mixer.setVelocity(f_Velocity_DriveForward);
          state = 7;
        }

      break;

      case 7:
        // toLog(std::to_string(medge.width).c_str());
        toLog(std::to_string(pose.dist).c_str());
        if(medge.width > 0.06 && pose.dist > 2) 
        { 
          toLog("Found seesaw intersection");
          // pose.resetPose();
          mixer.setVelocity(0);
          mixer.setTurnrate(0);
          pose.dist = 0;
          state = 8;
          // finished = true;
        }
      break;

      case 8:
        mixer.setVelocity(-0.07);
        state = 9;
        break;

      case 9:
        if(pose.dist < -0.05){
          mixer.setVelocity(0);
          finished = true;
        }
        break;





      

      // case 41:
      // // toLog(std::to_string(pose.dist).c_str());
      //   if(medge.width > 0.06 && pose.dist > 3)
      //   {
      //     pose.dist = 0;
      //     state = 5;
      //   }
      // break;

      // case 5:
      //   if(abs(pose.dist) > 0.17)
      //   {
      //     t.clear();
      //     mixer.setVelocity(0);
      //     heading.setMaxTurnRate(1);
      //     state = 51;
      //   }
      // break;

      // case 51:
      //   if(t.getTimePassed() > 0.4)
      //   {
      //     pose.turned = 0.0;
      //     pose.resetPose();
      //     mixer.setDesiredHeading(3.3);
      //     state = 6;
      //   }
      // break;

      // case 6:
      // toLog(std::to_string(pose.turned).c_str());
      //   if(abs(pose.turned) > (2.9))
      //   {
      //     pose.dist = 0.0;
      //     heading.setMaxTurnRate(3);
      //     mixer.setVelocity(-0.25);
      //     //mixer.setEdgeMode(b_Line_HoldRight, 0);
      //     //mixer.setEdgeMode(f_Line_RightOffset,0);
      //     state = 61;
      //   }
      // break;
      
      // case 61:
      //   if(abs(pose.dist) > 0.15)
      //   {
      //       mixer.setVelocity(0);
      //       t.clear();
      //       state = 62;
      //   }
      //   break; 
      
      // case 62:
      //   if(t.getTimePassed() > 0.5){
      //       mixer.setVelocity(0.25);
      //       mixer.setEdgeMode(f_Line_RightOffset,0);
      //       state = 7;
      //   }
      // break;

      // case 7:
      //   //toLog(std::to_string(medge.width).c_str());
      //   //toLog(std::to_string(pose.dist).c_str());
      //   if(pose.dist > 2) 
      //   { 
      //     mixer.setVelocity(f_Velocity_DriveForward);
      //     mixer.setEdgeMode(b_Line_HoldRight, -0.03);
      //     pose.dist = 0;
      //     state = 71;

      //   }
      // break;

      // case 71:
      //   //toLog(std::to_string(medge.width).c_str());
      //   //toLog(std::to_string(pose.dist).c_str());
      //   if(medge.width > 0.055) 
      //   { 
      //     toLog("Found third crossing again");
      //     mixer.setVelocity(f_Velocity_DriveForward);
      //     pose.dist = 0;
      //     state = 8;

      //   }
      // break;


      // case 8:
      //   if(pose.dist > 0.1)
      //   {
      //     mixer.setEdgeMode(b_Line_HoldLeft, 0);
      //     state = 9;
      //   }
      // break;

      // case 9:
      //   if(medge.width > 0.1)
      //   {
      //     pose.dist = 0.0;
      //     pose.turned = 0.0;
      //     mixer.setVelocity(0);
      //     finished = true;
      //   }
      // break;


      default:
        toLog("Default Start to Cross");
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
    toLog("PlanCrossMission got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanCrossMission finished");
}
