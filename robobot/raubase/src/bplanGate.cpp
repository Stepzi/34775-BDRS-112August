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
  #include "cheading.h"

  #include "bplanGate.h"

  // create class object
  BPlanGate planGate;


  void BPlanGate::setup()
  { // ensure there is default values in ini-file
    if (not ini["PlanGate"].has("log"))
    { // no data yet, so generate some default values
      ini["PlanGate"]["log"] = "true";
      ini["PlanGate"]["run"] = "true";
      ini["PlanGate"]["print"] = "true";
    }
    // get values from ini-file
    toConsole = ini["PlanGate"]["print"] == "true";
    //
    if (ini["PlanGate"]["log"] == "true")
    { // open logfile
      std::string fn = service.logPath + "log_PlanGate.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Mission PlanGate logfile\n");
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tMission state\n");
      fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
    }
    setupDone = true;
  }

  BPlanGate::~BPlanGate()
  {
    terminate();
  }

  void BPlanGate::runOpen()
  {
    if (not setupDone)
      setup();
    if (ini["PlanGate"]["run"] == "false")
      return;
    UTime t("now");
    bool finished = false;
    bool lost = false;
    

    state = 0;
    oldstate = state;


    const int MSL = 100;
    char s[MSL];
    
    //Hardcoded Line data
    //float f_LineWidth_MinThreshold = 0.02;
    //float f_LineWidth_NoLine = 0.01;
    float f_LineWidth_Crossing = 0.07;

    //float f_Line_LeftOffset = 0;
    float f_Line_RightOffset = 0;
    //bool b_Line_HoldLeft = true;
    bool b_Line_HoldRight = false;


    int wood[8]  = {352, 436, 468, 461, 503, 499, 460, 391};
    int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

    int woodWhite = 550;
    int blackWhite = 350;
    //Hardcoded time data
    //float f_Time_Timeout = 10.0;

    //Postion and velocity data
    //float f_Velocity_DriveForward = 0.25; 
    //float f_Velocity_DriveBackwards = -0.15; 
    //float f_Distance_FirstCrossMissed = 1.5;
    //float f_Distance_LeftCrossToRoundabout = 0.85;
    
    toLog("PlanGate started");;
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
          toLog("Start Open Gate");
          pose.dist = 0;
          pose.turned = 0;
          medge.updateCalibBlack(wood,8);
          medge.updatewhiteThreshold(woodWhite);
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
          mixer.setVelocity(0.15);
          state = 1;  
      break;
      
      case 1:
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
          medge.updateCalibBlack(black,8);
          medge.updatewhiteThreshold(blackWhite);
          pose.turned = 0;
          pose.dist = 0;
          state = 2;
        }
      break;
      
      case 2:
       toLog(std::to_string(dist.dist[1]).c_str());
          if(dist.dist[1] < 0.2)
          {
              mixer.setVelocity(0);
              pose.dist = 0.0;
              pose.turned = 0.0;
              heading.setMaxTurnRate(1);
              mixer.setDesiredHeading(-1.25);
              state = 3;
          }
      break;

      case 3:
        toLog(std::to_string(pose.turned).c_str());
          if(abs(pose.turned) > 1.5-0.02){
              pose.dist = 0;
              pose.turned = 0;
              mixer.setVelocity(0.15);
              state = 4;
          }
      break;

      case 4:
          if(dist.dist[0] > 0.5){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 5;
          }
      break; 

      case 5:
          if(pose.dist > 0.2){
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.5);
              state = 6;
          }
      break; 

      case 6:
          if(abs(pose.turned) > 1.6-0.02){
              pose.dist = 0;
              pose.turned = 0;
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 7;
          }
      break; 

      case 7:
          if(pose.dist > 0.5){
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.6);
              state = 8;
          }
      break; 

      case 8:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 9;
          }
      break; 

      case 9:
          if(dist.dist[1] < 0.2){
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.6);
              state = 10;
          }
      break; 

      case 10:
          if(abs(pose.turned) > 1.60-0.02){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 11;
          }
      break; 

      case 11:
        if(dist.dist[1] < 0.15){
              pose.resetPose();
              mixer.setVelocity(0.6);
              state = 12;
          }
      break;

      case 12:
          if(pose.dist > 0.45){
              t.clear();
              pose.turned = 0;
              pose.resetPose();
              mixer.setVelocity(0);
              state = 13;
          }
      break; 

        //Case 22 - After 2 seconds, reset distance, set follow line mode and drive forward slowly.
        case 13: 
          //toLog(std::to_string(t.getTimePassed()).c_str());
          if(t.getTimePassed() > 1)
          {
            mixer.setDesiredHeading(-1.6);
            state = 14;
          }
        break;

      case 14:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 15;
          }
      break; 

      case 15:
          if(pose.dist > 0.7){
              pose.resetPose();
              mixer.setVelocity(-0.1);
              state = 16;
          }
      break; 

      case 16:
        if(medge.width > f_LineWidth_Crossing) 
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.4);
              state = 17;  
            }
      break;

      case 17:
        if(abs(pose.turned) > 1.4 - 0.02)
            { 
              mixer.setVelocity(0.2);
              state = 18;
            }
      break;

      case 18:
        if(abs(pose.dist) > 0.2)
            { 
              heading.setMaxTurnRate(3);
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
              state = 19;
            }
      break;
      
      case 19:
        if(dist.dist[1] < 0.15){
              pose.resetPose();
              state = 20;
          }
      break;

      case 20:
        if(abs(pose.dist) > 0.4)
            { 
              mixer.setVelocity(0);
              heading.setMaxTurnRate(1);
              mixer.setDesiredHeading(-1.6);
              state = 21;
            }
      break;

      case 21:
        if(abs(pose.turned) > 1.8 - 0.02)
            { 
              pose.dist = 0;
              mixer.setVelocity(0.1);
              state = 22;
            }
      break;

      case 22:
        if(abs(pose.dist) > 0.4)
            { 
              mixer.setVelocity(-0.1);
              state = 23;
            }
      break;

      case 23:
        if(medge.width > f_LineWidth_Crossing)
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.3);
              state = 24;
            }
      break;

      case 24:
        if(abs(pose.turned) > 1.3 - 0.02)
            { 
              pose.dist = 0;
              heading.setMaxTurnRate(3);
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
              state = 25;
            }
      break;

      case 25:
      toLog(std::to_string(dist.dist[0]).c_str());
        if((abs(pose.dist)) > 1.3 )
            { 
              mixer.setVelocity(0);
              finished = true;
            }
      break;

        default:
          toLog("Default Mission 0q");
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
      toLog("PlanGate got lost - stopping");
      mixer.setVelocity(0);
      mixer.setTurnrate(0);
    }
    else
      toLog("PlanGate finished");
  }

  void BPlanGate::runClose()
  {
    if (not setupDone)
      setup();
    if (ini["PlanGate"]["run"] == "false")
      return;
    UTime t("now");
    bool finished = false;
    bool lost = false;
    

    state = 1;
    oldstate = state;


    const int MSL = 100;
    char s[MSL];
    
    //Hardcoded Line data
    //float f_LineWidth_MinThreshold = 0.02;
    //float f_LineWidth_NoLine = 0.01;
    float f_LineWidth_Crossing = 0.07;

    //float f_Line_LeftOffset = 0;
    float f_Line_RightOffset = 0;
    //bool b_Line_HoldLeft = true;
    bool b_Line_HoldRight = false;

    //Hardcoded time data
    //float f_Time_Timeout = 10.0;

    //Postion and velocity data
    //float f_Velocity_DriveForward = 0.25; 
    //float f_Velocity_DriveBackwards = -0.15; 
    //float f_Distance_FirstCrossMissed = 1.5;
    //float f_Distance_LeftCrossToRoundabout = 0.85;
    
    toLog("PlanGate started");;
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

      case 1:
          pose.resetPose();
          mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
          mixer.setVelocity(0.15);
          state = 2;  
      break;
      
      case 2:
        if(medge.width > f_LineWidth_Crossing-0.02) 
        { 
            pose.resetPose();
            mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
            mixer.setVelocity(0.15); 
            state = 3;  
          }
      break;

      case 3:
          if(pose.dist > 0.25){
            pose.resetPose();
            mixer.setVelocity(0);
            mixer.setDesiredHeading(1.6);
            state = 4;
          }
      break;

      case 4:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 5;
          }
      break; 

      case 5:
          if(pose.dist > 0.4){
            pose.resetPose();
            mixer.setVelocity(0);
            mixer.setDesiredHeading(-1.6);
            state = 6;
          }
      break; 

      case 6:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 7;
          }
      break; 

      case 7:
        if(dist.dist[0] < 0.3){
          pose.resetPose();
          mixer.setVelocity(0.15);
          state = 8;
        }
      break; 

      case 8:
          if(pose.dist > 0.9){
            pose.resetPose();
            mixer.setDesiredHeading(-1.6);
            state = 9;
          }
      break; 

      case 9:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.3);
              state = 10;
          }
      break; 

      case 10: // here now
          if(medge.width > f_LineWidth_Crossing-0.02) { 
            mixer.setVelocity(0);
            mixer.setDesiredHeading(0.8);
            finished = true;
          }
      break; 

      case 11:
        if(dist.dist[1] < 0.15){
              pose.resetPose();
              mixer.setVelocity(0.6);
              state = 12;
          }
      break;

      case 12:
          if(pose.dist > 0.45){
              t.clear();
              pose.turned = 0;
              pose.resetPose();
              mixer.setVelocity(0);
              state = 13;
          }
      break; 

        //Case 22 - After 2 seconds, reset distance, set follow line mode and drive forward slowly.
        case 13: 
          //toLog(std::to_string(t.getTimePassed()).c_str());
          if(t.getTimePassed() > 1)
          {
            mixer.setDesiredHeading(1.2);
            state = 14;
          }
        break;

      case 14:
          if(abs(pose.turned) > 1.2-0.02){
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 15;
          }
      break; 

      case 15:
          if(pose.dist > 0.7){
              pose.resetPose();
              mixer.setVelocity(-0.1);
              state = 16;
          }
      break; 

      case 16:
        if(medge.width > f_LineWidth_Crossing) 
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.4);
              state = 17;  
            }
      break;

      case 17:
        if(abs(pose.turned) > 1.4 - 0.02)
            { 
              mixer.setVelocity(0.2);
              state = 18;
            }
      break;

      case 18:
        if(abs(pose.dist) > 0.2)
            { 
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
              state = 19;
            }
      break;
      
      case 19:
        if(dist.dist[1] < 0.15){
              pose.resetPose();
              state = 20;
          }
      break;

      case 20:
        if(abs(pose.dist) > 0.5)
            { 
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.6);
              state = 21;
            }
      break;

      case 21:
        if(abs(pose.turned) > 1.6 - 0.02)
            { 
              pose.dist = 0;
              mixer.setVelocity(0.1);
              state = 22;
            }
      break;

      case 22:
        if(abs(pose.dist) > 0.4)
            { 
              mixer.setVelocity(-0.1);
              state = 23;
            }
      break;

      case 23:
        if(medge.width > f_LineWidth_Crossing)
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.3);
              state = 24;
            }
      break;

      case 24:
        if(abs(pose.turned) > 1.3 - 0.02)
            { 
              pose.dist = 0;
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
              state = 25;
            }
      break;

      case 25:
        if(abs(pose.dist) > 0.5)
            { 
              mixer.setVelocity(0);
              finished = true;
            }
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
      toLog("PlanGate got lost - stopping");
      mixer.setVelocity(0);
      mixer.setTurnrate(0);
    }
    else
      toLog("PlanGate finished");
  }



  void BPlanGate::terminate()
  { //
    if (logfile != nullptr)
      fclose(logfile);
    logfile = nullptr;
  }

  void BPlanGate::toLog(const char* message)
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
