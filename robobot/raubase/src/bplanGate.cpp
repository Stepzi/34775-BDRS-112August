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

    float f_Line_LeftOffset = 0;
    float f_Line_RightOffset = 0;
    bool b_Line_HoldLeft = true;
    bool b_Line_HoldRight = false;


    int wood[8]  = {199, 249, 261, 264, 300, 317, 302, 254};
    int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

    int woodWhite = 500;
    int blackWhite = 400;
    //Hardcoded time data
    //float f_Time_Timeout = 10.0;

    //Postion and velocity data
    //float f_Velocity_DriveForward = 0.25; 
    //float f_Velocity_DriveBackwards = -0.15; 
    //float f_Distance_FirstCrossMissed = 1.5;
    //float f_Distance_LeftCrossToRoundabout = 0.85;

    //Wall following
    float wall_1 = 0;
    float wall_2 = 0;
    int wall_sum_cnt = 10;
    float wall_drive_dist = 0.1;
    float correctionAngle = 0;

    float speed = 0;
    
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
          mixer.setEdgeMode(b_Line_HoldRight,-0.02);
          mixer.setVelocity(0.3);
          state = 1;  
      break;
      
      case 1:
          if(medge.width > 0.07){
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
              heading.setMaxTurnRate(1);
              mixer.setVelocity(0);
              pose.dist = 0.0;
              pose.turned = 0.0;
              pose.resetPose();
              mixer.setDesiredHeading(1.6);
              state = 3;
          }
      break;

      case 3:
        toLog(std::to_string(pose.turned).c_str());
          if(abs(pose.turned) > 1.6-0.02){
              pose.dist = 0;
              pose.turned = 0;
              mixer.setVelocity(0.25);
              state = 4;
          }
      break;

      case 4:
        toLog(std::to_string(dist.dist[0]).c_str());
          if(dist.dist[0] > 0.5){
              pose.resetPose();
              mixer.setVelocity(0.3);
              state = 5;
          }
      break; 

      case 5:
          if(pose.dist > 0.25){
              pose.resetPose();
              pose.turned = 0;
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.6);
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
              pose.resetPose();
              pose.turned = 0;
              mixer.setDesiredHeading(-1.6);
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
      toLog(std::to_string(dist.dist[1]).c_str());
          if(dist.dist[1] < 0.1){
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.75);
              state = 10;
          }
      break; 

      case 10:
          if(abs(pose.turned) > 1.75-0.02){
              pose.resetPose();
              wall_sum_cnt = 100;
              state = 11;
          }
      break; 

      case 11:
        //Sum the distances and count down
        wall_sum_cnt = wall_sum_cnt - 1;
        wall_1 = wall_1 + dist.dist[0];
        if( wall_sum_cnt <= 0)
        {
          //Average
          wall_1 = wall_1 / 100;
          toLog("Wall 1: ");
          toLog(std::to_string(wall_1).c_str());
          pose.resetPose();
          mixer.setVelocity(-0.15);
          state = 12;
        }
      break;

      case 12:
        if(abs(pose.dist) > wall_drive_dist)
        {
          wall_sum_cnt = 100;
          state = 13;
        }
      break;

      case 13:
        //Sum the distances and count down
        wall_sum_cnt = wall_sum_cnt - 1;
        wall_2 = wall_2 + dist.dist[0];
        if( wall_sum_cnt <= 0)
        {
          //Average
          wall_2 = wall_2 / 100;
          toLog("Wall 2: ");
          toLog(std::to_string(wall_2).c_str());
          mixer.setVelocity(0.15);
          state = 14;
        }
      break;

      case 14:

        if(abs(pose.dist) > wall_drive_dist)
        {
          t.clear();
          correctionAngle = atan((wall_2-wall_1) / wall_drive_dist);
          toLog("Correction angle: ");
          toLog(std::to_string(correctionAngle).c_str());
          state = 15;
        }
      break;
      
      case 15:
        if(t.getTimePassed() > 1)
        {
          pose.resetPose();
          pose.turned = 0;
          mixer.setDesiredHeading(correctionAngle);
          t.clear();
          state = 16;
        }
      break;

      case 16:
        if(t.getTimePassed() > 0.5)
        { 
          speed  = 0;
          pose.resetPose();
          pose.dist = 0;
          state = 17;
        }
      break;


      case 17:
        if(speed > -0.45) //RAMP DOWN BEFORE TURN
        {
          speed = speed - 0.003;
          mixer.setVelocity(speed);
        }
        else
        {
          state = 171;
        }
      break;


      case 171:
          if(abs(pose.dist) > 0.6){
              t.clear();
              pose.turned = 0;
              pose.resetPose();
              mixer.setVelocity(0);
              state = 18;
          }
      break; 
/*

      case 10:
          if(abs(pose.turned) > 1.75-0.02){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 11;
          }
      break; 

      case 11:
        if(pose.dist > 0.1){
              pose.resetPose();
              mixer.setVelocity(0.6);
              state = 12;
          }
      break;








      case 11:
        if(dist.dist[1] < 0.11){
              pose.resetPose();
              mixer.setVelocity(0.6);
              state = 12;
          }
      break;

      case 12:
          if(pose.dist > 0.55){
              t.clear();
              pose.turned = 0;
              pose.resetPose();
              mixer.setVelocity(0);
              state = 13;
          }
      break; */

        //Case 22 - After 2 seconds, reset distance, set follow line mode and drive forward slowly.
        case 18: 
          //toLog(std::to_string(t.getTimePassed()).c_str());
          if(t.getTimePassed() > 1)
          {
            mixer.setDesiredHeading(-1.6);
            state = 19;
          }
        break;

      case 19:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 20;
          }
      break; 

      case 20:
          if(pose.dist > 0.7){
              pose.resetPose();
              mixer.setVelocity(-0.1);
              state = 151;
          }
      break; 

      case 151:
      if(medge.width > f_LineWidth_Crossing) 
            { 
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 152;  
            }
      break;

      case 152:
        if(pose.dist > 0.05) 
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.6);
              state = 21;  
            }
      break;

      

      case 21:
        if(abs(pose.turned) > 1.6 - 0.02)
            { 
              pose.dist = 0;
              pose.resetPose();
              mixer.setVelocity(0.2);
              state = 22;
            }
      break;

      case 22:
        if(abs(pose.dist) > 0.15)
            { 
              heading.setMaxTurnRate(3);
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldLeft,f_Line_LeftOffset);
              state = 23;
            }
      break;
      
      case 23:
        if(dist.dist[1] < 0.15){
              pose.resetPose();
              mixer.setVelocity(0.35);
              state = 24;
          }
      break;

      case 24:
        if(abs(pose.dist) > 0.55)
            { 
              mixer.setVelocity(0);
              heading.setMaxTurnRate(1);
              mixer.setDesiredHeading(1.6);
              state = 25;
            }
      break;

      case 25:
        if(abs(pose.turned) > 1.6 - 0.02)
            { 
              pose.dist = 0;
              mixer.setVelocity(0.1);
              state = 26;
            }
      break;

      case 26:
        if(abs(pose.dist) > 0.4)
            { 
              mixer.setVelocity(-0.1);
              state = 27;
            }
      break;

      case 27:
        if(medge.width > f_LineWidth_Crossing)
            { 
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.3);
              state = 28;
            }
      break;

      case 28:
        if(abs(pose.turned) > 1.3 - 0.02)
            { 
              pose.dist = 0;
              heading.setMaxTurnRate(3);
              mixer.setVelocity(0.2);
              mixer.setEdgeMode(b_Line_HoldRight,f_Line_RightOffset);
              state = 29;
            }
      break;

      case 29:
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
    

    state = 0;
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


    int wood[8]  = {352, 436, 468, 461, 503, 499, 460, 391};
    int black[8] = {34, 33, 40, 44, 52, 52, 49, 46};

    int woodWhite = 550;
    int blackWhite = 250;
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

     /* case 1:
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
      break;*/
      case 0:
          toLog("Start Close Gate");
          pose.dist = 0;
          pose.turned = 0;
          medge.updateCalibBlack(wood,8);
          medge.updatewhiteThreshold(woodWhite);
          heading.setMaxTurnRate(3);
          mixer.setEdgeMode(b_Line_HoldRight,-0.03);
          mixer.setVelocity(0.15);
          state = 1;  
      break;

      case 1:
          if(medge.width > 0.09){
             toLog("Found crossing, change line sensor thresholds");
            pose.dist = 0;
            pose.turned = 0;
            state = 2;
          }
      break;

      case 2:
        if(pose.dist > 0.30)
        {
          toLog("30 cm after crossing, i am on black floor now");
          medge.updateCalibBlack(black,8);
          medge.updatewhiteThreshold(blackWhite);
          pose.turned = 0;
          pose.dist = 0;
          state = 3;
        }
      break;

      case 3:
      toLog(std::to_string(dist.dist[0]).c_str());
          if(dist.dist[0] < 0.18){
            heading.setMaxTurnRate(1);
            pose.resetPose();
            mixer.setVelocity(-0.1);
            state = 4;
          }
      break;

      case 4:
          if(abs(pose.dist) > 0.1){
              pose.resetPose();
              mixer.setVelocity(0);
              mixer.setDesiredHeading(1.6);
              state = 5;
          }
      break;

      case 5:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 6;
          }
      break; 

      case 6:
          if(pose.dist > 0.5){
            pose.resetPose();
            mixer.setVelocity(0);
            mixer.setDesiredHeading(-1.6);
            state = 7;
          }
      break; 

      case 7:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              pose.dist = 0;
              mixer.setVelocity(0.15);
              state = 8;
          }
      break; 

      case 8:
        if((dist.dist[1] < 0.3) || (pose.dist > 1.3))
        {
          pose.resetPose();
          mixer.setVelocity(0);
          mixer.setDesiredHeading(-1.6);
          state = 9;
        }
      break; 
// Fix door detection !!!! 
            /*case 19:
        if(dist.dist[1] < 0.3)
        {
          //If front sensor sees door
          pose.dist = 0;
          state = 200;
        }
        else if (abs(pose.dist) > 1.3)
        {
          //If front sensor didn't see the door
          pose.resetPose();
          mixer.setVelocity(0);
          mixer.setDesiredHeading(-1.6);
          state = 201;
        }
      break; *7

      /*case 8:
          if(pose.dist > 0.9){
            pose.resetPose();
            mixer.setDesiredHeading(-1.6);
            state = 9;
          }
      break;*/ 

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
            mixer.setDesiredHeading(1.6);
            state = 11;
          }
      break; 

      case 11:
        if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(-0.01);
              state = 12;
          }
      break;

      case 12:
          if(abs(pose.dist) > 0.2){
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
          mixer.setVelocity(0.1);
          state = 14;
        }
      break;

      case 14:
          if(abs(pose.dist) > 0.1){
              pose.resetPose();
              mixer.setDesiredHeading(-1.6);
              state = 15;
          }
      break; 

      case 15:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.1);
              state = 16;
          }
      break; 

      case 16:
        toLog(std::to_string(dist.dist[0]).c_str());
          if(dist.dist[0] > 0.5){
              pose.resetPose();
              mixer.setVelocity(0.15);
              state = 17;
          }
      break; 

      case 17:
          if(pose.dist > 0.3){
              pose.resetPose();
              pose.turned = 0;
              mixer.setVelocity(0);
              mixer.setDesiredHeading(-1.6);
              state = 18;
          }
      break; 


      case 18:
          if(abs(pose.turned) > 1.6-0.02){
              pose.resetPose();
              mixer.setVelocity(0.25);
              //state = 19;
          }
      break; 

      case 19:
        if(dist.dist[1] < 0.3)
        {
          //If front sensor sees door
          pose.dist = 0;
          state = 200;
        }
        else if (abs(pose.dist) > 1.3)
        {
          //If front sensor didn't see the door
          pose.resetPose();
          mixer.setVelocity(0);
          mixer.setDesiredHeading(-1.6);
          state = 201;
        }
      break; 

      case 200:
        if(pose.dist > 0.3){
            pose.resetPose();
            pose.turned = 0;
            mixer.setVelocity(0);
            mixer.setDesiredHeading(-1.6);
            state = 18;
        }
      break;



      /*case 16:
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
*/
      /*case 18:
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
      break;*/

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
