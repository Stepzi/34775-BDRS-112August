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

#include "bplanIRTEST.h"

// create class object
BPlanIRTEST planIRTEST;


void BPlanIRTEST::setup()
{ // ensure there is default values in ini-file
  if (not ini["PlanIRTEST"].has("log"))
  { // no data yet, so generate some default values
    ini["PlanIRTEST"]["log"] = "true";
    ini["PlanIRTEST"]["run"] = "true";
    ini["PlanIRTEST"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["PlanIRTEST"]["print"] == "true";
  //
  if (ini["PlanIRTEST"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_PlanIRTEST.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission PlanIRTEST logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlanIRTEST::~BPlanIRTEST()
{
  terminate();
}

void BPlanIRTEST::run(bool entryDirectionStart, bool exitDirectionStart)
{
  if (not setupDone)
    setup();
  if (ini["PlanIRTEST"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  
  if(entryDirectionStart == true)
  {
    state = 1;
  }
  else
  {
    state = 11;
  }

  oldstate = state;
  const int MSL = 100;
  char s[MSL];
  
  //Hardcoded Line data
  //float f_LineWidth_MinThreshold = 0.02;
  //float f_LineWidth_NoLine = 0.01;
  float f_LineWidth_Crossing = 0.07;

  float f_Line_LeftOffset = 0;
  float f_Line_RightOffset = 0;
  bool b_Line_HoldLeft = true;
  bool b_Line_HoldRight = false;

  //Hardcoded time data
  //float f_Time_Timeout = 10.0;

  //Postion and velocity data
  float f_Velocity_DriveForward = 0.25; 
  //float f_Velocity_DriveBackwards = -0.15; 
  //float f_Distance_FirstCrossMissed = 1.5;
  float f_Distance_LeftCrossToRoundabout = 0.85;
  bool b_Flag_Once = true;
  //
  toLog("PlanIRTEST started");;
  toLog("Time stamp, IR dist 0, IR dist 1");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      /********************************************************************/
      /******************** Coming from the start-side ********************/
      /********************************************************************/
      //Case 1 - first crossing on the track and forward

      case 0: 
        
        if(b_Flag_Once){
          mixer.setVelocity(0);
          pose.resetPose();
          mixer.setTurnrate(0.5);
          b_Flag_Once = false;
        }
      break;


      case 1:
          // start driving
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset );
          mixer.setVelocity(f_Velocity_DriveForward); //Already driving
          toLog("Starting from split - Start-side of roundabout");
          pose.dist=0;   
          state = 2;
        break;
      
      //Case 2 - Drive 0.9 still following line. Then start turning.
      case 2:  // Stop goal reached case
        if(pose.dist > f_Distance_LeftCrossToRoundabout)
        { 
          pose.dist=0;   
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset );
          toLog("Ready to enter the roundabout from the start-side");
          mixer.setVelocity(0);
          pose.resetPose();
          mixer.setDesiredHeading(1.67); // (3*pi) / 8
          state = 3;
        }
        break;
      
      //Case 3 - Stop turning after some angle
      case 3:
        toLog(std::to_string(pose.turned).c_str());
        if(abs(pose.turned) > 1.65){
          mixer.setTurnrate(0);
          state = 21;
        }
      break;

      /******************************************************************/
      /******************** Coming from the axe-side ********************/
      /********************************************************************/
      //Case 11 - Drive from the Axe-side
      case 11:  // Stop goal reached case
        if(pose.dist > f_Distance_LeftCrossToRoundabout)
        { 
          toLog("Ready to enter the roundabout from the start-side");
          mixer.setVelocity(0);
          pose.dist = 0;
          toLog("Ready to enter the roundabout from the start-side");
          pose.turned = 0;
          mixer.setVelocity(0.01);//Drive slowly and turn i circle
          mixer.setTurnrate(0.5);
          state = 12;
        }
        break;
      
      //Case 12 - Stop turning after some angle
      case 12:
        if(pose.turned > -0.3){
          mixer.setTurnrate(0);
          state = 21;
        }
      break;

      /******************************************************************/
      /**** Both sides collected behaviour from waiting on the robot ****/
      /********************************************************************/
      //Case 21 - If robot is seen, clear time and wait until case 6
      case 21: 
        if(dist.dist[1] < 0.25){
          toLog("Robot seen!");
          t.clear();
          pose.dist = 0;
          state = 22;
        }
      break;

      //Case 22 - After 2 seconds, reset distance, set follow line mode and drive forward slowly.
      case 22: 
        //toLog(std::to_string(t.getTimePassed()).c_str());
        if(t.getTimePassed() > 5)
        {
          pose.dist = 0;
          mixer.setVelocity(0.2);
          state = 95;
        }
      break;

      case 95:
        if(pose.dist > 0.4)
        {
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset );
          state = 23;
        }
      break;

      //Case 23 - After 0.6 driven, set up speed, follow right with new offset and go to case 8
      case 23:
        if(pose.dist > 0.6)
        {
          mixer.setVelocity(0.25);
          mixer.setEdgeMode(b_Line_HoldRight, f_Line_RightOffset );
          state = 24;
        }

      break;

      //Case 24 - At crossing, attempt to go into roundabout
      case 24:
      if(medge.width > f_LineWidth_Crossing) //0.07
        { 
          
          mixer.setVelocity(0);
          //pose.resetPose();
          toLog("Make 3 test");
          pose.resetPose();
          mixer.setDesiredHeading(1.3);
          //state = 25;
          state = 25;
        }
        break;
      
      //Case 25 - After turning some angle, go to state 10
      case 25:
      //TEST IF WE CAN DRIVE NEGATIVE AND GO BACKWARDS
      //toLog("Turned");
      //toLog(std::to_string(pose.turned).c_str());
      //toLog("Turnerate");
      //toLog(std::to_string(pose.turnrate).c_str());


        if(abs(pose.turned) > 1.28){
          mixer.setVelocity(-0.3);
          state = 31;
        }
      break;

      /********************************************************************/
      /************************ On the roundabout *************************/
      /********************************************************************/
      //Case 31 - after some distance, stop and turn for the circle
      case 31:
        if(abs(pose.dist) > 0.55)
        {
          pose.resetPose();
          mixer.setVelocity(0); //TEST THIS!!!!!
          mixer.setDesiredHeading(1.5);
          state = 32;
        } 
      break;

      //Case 32 - After turning some angle, ready for turning around on the roundabout. 
      case 32:
      //TEST IF WE CAN DRIVE NEGATIVE AND GO BACKWARDS
        if(abs(pose.turned) > 1.48)
        {
          mixer.setDesiredHeading(0);
          pose.turned = 0;
          
          mixer.setTurnrate(0.55);
          mixer.setVelocity(0.2);
          state = 33;
        } 
      break;

      //Case 27 - After turning some angle, ready for turning around on the roundabout. 
      case 33:
      //TEST IF WE CAN DRIVE NEGATIVE AND GO BACKWARDS
        if(abs(pose.turned) > 6.3)
        {
          mixer.setVelocity(0);
          mixer.setTurnrate(0);
          pose.resetPose();
          mixer.setDesiredHeading(-1.6);
          /*pose.resetPose();
          mixer.setTurnrate(0.75);
          mixer.setVelocity(0.4);
          t.clear();*/
          state = 34;
        } 
      break;

      case 34:
      //TEST IF WE CAN DRIVE NEGATIVE AND GO BACKWARDS
        if(abs(pose.turned) > 1.58)
        {
          mixer.setVelocity(0.1);
          state = 35;
        } 
      break;

      case 35:
      //TEST IF WE CAN DRIVE NEGATIVE AND GO BACKWARDS
        if(abs(pose.dist) > 0.08)
        {
          mixer.setVelocity(0);
          state = 41;
        } 
      break;

        //else if(t.getTimePassed() > 5)
        //{
          //some fail saving code.
          //mixer.setVelocity(-0.2);
        //}
      break;

      /********************************************************************/
      /*************************** Exit strategy **************************/
      /********************************************************************/
      //Case 41 - If robot is seen, clear time and wait until case 6
      case 41: 
      toLog(std::to_string(dist.dist[1]).c_str());
        if(dist.dist[1] < 0.25)
        {
          toLog("Robot seen!");
          t.clear();
          state = 42;
        }
      break;

      //Case 42 - If robot is seen, clear time and wait until case 6
      case 42:
        if(t.getTimePassed() > 2)
        {
          mixer.setVelocity(0.1);
          state = 43;
        }
      break;


      case 43:
      if(medge.width > f_LineWidth_Crossing) 
        { 
          // start driving
          toLog("First line found");
          pose.dist = 0;
          state = 44;  
        }
      break;

      case 44:
      if((medge.width > f_LineWidth_Crossing) && (pose.dist >= 0.05)) 
        { 
          // start driving
          toLog("Second line found");
          pose.resetPose();
          if(exitDirectionStart)
          {
            mixer.setDesiredHeading(-1.6);
          }
          else
          {
            mixer.setDesiredHeading(1.6);
          }
          state = 45;  
        }
      break;

      case 45:
        if(abs(pose.turned) > 1.58)
        {
          mixer.setVelocity(0.2);
          mixer.setEdgeMode(b_Line_HoldLeft, f_Line_LeftOffset );
          state = 46;
        }
      break;

      case 46:
        if(exitDirectionStart)
        {
          if(medge.width > f_LineWidth_Crossing)
          {
            mixer.setVelocity(0.2);
            pose.dist = 0;
            mixer.setEdgeMode(b_Line_HoldRight, f_Line_RightOffset );
            state = 47;
          }
        }
        else
        {
          if(medge.width > f_LineWidth_Crossing)
          {
            mixer.setVelocity(0);
            toLog("FINISH at axe :)");
            finished = true;
          }
        }
      break;
      
      case 47:
      if((medge.width > f_LineWidth_Crossing) && (pose.dist >= 0.05)) 
        { 
          mixer.setVelocity(0);
          toLog("FINISH at Start :)");
          finished = true;
        }
      break;

      case 99: // IR dist case 
        float irDist0;
        float irDist1;

        while(t.getTimePassed() < 10){
            irDist0 = dist.dist[0];
            irDist1 = dist.dist[1];
            std::string temp_str = std::to_string(t.getTimePassed()) 
                + " " + std::to_string(irDist0)
                + " " + std::to_string(irDist1);
            toLog(temp_str.c_str());
        };
        finished = true;    
        break;

      default:
        toLog("Default Mission 0");
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
    toLog("PlanIRTEST got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("PlanIRTEST finished");
}


void BPlanIRTEST::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlanIRTEST::toLog(const char* message)
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
