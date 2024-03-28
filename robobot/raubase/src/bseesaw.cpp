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
#include "mgolfball.h"

#include "bseesaw.h"
#include <iostream>
#include "simu.h"

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
  bool leftEdge  = true;
  bool rightEdge = false;

  float normalSpeed        =  0.3;    //speed under normal conditions
  float lineWidth          =  0.02;   //width to determine if we are on the line
  float lineGone           =  0.01;   //width to determine if the line was lost
  float lineOffset         =  0.03;      //offset for line edge detection
  float intersectionWidth  =  0.1;   //used to detect intersections

  //seesaw variables
  float intersectionToEdge =  0.23;   //seesaw - distance from the intersection to the step
  float edgeToSeesaw       =  0.84;   //seesaw - distance from the edge to the tilting point
  float edgeWidth          =  1.098;  //seesaw - distance to assume the robot went down the step

  //ramp variables
  float rampUpDistance     = 2;       //distance from getting back on line to the start of the turn on the plateu
  float rampUpToHole       = 0.3;     //distance from the start of the turn to the golf ball hole

  float speed              = 0;
  float maxSpeed           = 0.5;

  int woodWhite = 600;
  int blackWhite = 350;
  
  toLog("seesaw started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {  
      case 1: // Start Position, assume we are on a line but verify.
      servo.setServo(2, true, -900, 500);
      medge.updateCalibBlack(medge.calibBlack,8);
      medge.updatewhiteThreshold(blackWhite);
      heading.setMaxTurnRate(3);
      usleep(1000);

      mixer.setEdgeMode(leftEdge, lineOffset);

      // toLog(const_cast<char*>((std::to_string(medge.width)).c_str()));
      // std::cout << medge.width << std::endl;
      if(medge.edgeValid && (medge.width > intersectionWidth)) //We should be on a line 
      {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("found intersection");
          mixer.setVelocity(0.07);
          pose.dist = 0;
          servo.setServo(2, true, 350, 700);
          state = 3;
      }
      else if(medge.edgeValid && (medge.width > lineWidth))
      {
        // pose.resetPose();
        toLog("Follow Line with velocity 0.1");
        // mixer.setEdgeMode(leftEdge, lineOffset);
        mixer.setVelocity(0.1);
        // state = 2;
      }else{
        lost = true;
      }
       
      break;

      case 2:
        if (medge.width > intersectionWidth)
          {
            mixer.setEdgeMode(leftEdge, 0.05);
            toLog("found intersection");
            mixer.setVelocity(0.07);
            pose.dist = 0;
            
            state = 3;
          }
          else
          {    
            mixer.setVelocity(0.1);
          }
      break;

      case 3:
      {
        

        if (pose.dist > intersectionToEdge)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);

          mixer.setVelocity(0.2);
          pose.dist = 0;
          state = 4;
          
        }

        break;
      }

      case 4:
      {
        float accel = imu.acc[2];

        if (pose.dist > edgeWidth  || (accel < -1.4) )
        {
          const int MSL = 100;
          char s[MSL];
          snprintf(s, MSL, "detected shock: %f \n", accel);
          toLog(s);
          servo.setServo(2, true, -900, 500);
          //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
          // toLog("coming to the tilting point");
          mixer.setEdgeMode(leftEdge, lineOffset);
          mixer.setVelocity(0.1);
          pose.dist = 0;
          state = 5;
        }
        break;
      }

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
        //servo.setServo(2, 1, -500, 200);
        
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
          medge.updateCalibBlack(medge.calibBlack,8);
          medge.updatewhiteThreshold(blackWhite);
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.1);
          mixer.setEdgeMode(rightEdge, 0);
          state = 10;
        }
      break;

      case 10:
        if(medge.width > lineWidth)
        {
          toLog("Found Line again");
          mixer.setVelocity(0.07);
          pose.dist = 0;          
          state = 11;
        }
        break;

      case 11:
        if (pose.dist > 0.2)
        {
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          state = 12;
        }
      break;

      case 12:
        // mixer.setVelocity(normalSpeed);
        if (medge.width > intersectionWidth)
          {
            toLog("found intersection - going straight");
            pose.resetPose();
            mixer.setVelocity(0.1);
            state = 14;
          }
        // else
        //   {    
        //     mixer.setVelocity(normalSpeed);
        //   }
      break;

      case 14:
        if(pose.dist > 0.1)
        {
          toLog("going 10cm straight");
          mixer.setEdgeMode(rightEdge, 0);
          mixer.setVelocity(0.1);          
          state = 15;
        }
      break;

      case 15:
        if(medge.edgeValid && (medge.width > lineWidth)){
          mixer.setVelocity(0.03);
          state = 17;
        }
        
        break;

      case 16:
        // if(medge.edgeValid && (medge.width > lineWidth)){
        //   mixer.setVelocity(0.03);
        //   state = 16;
        // }
        
      break;

      case 17:
      {
        finished = true;
      }
      break;

/*
      case 16:
        if (medge.width > intersectionWidth)
          {
            toLog("found intersection");
            mixer.setVelocity(0.05);
            pose.dist = 0;
            state = 17;
          }
        else
          {    
            mixer.setVelocity(normalSpeed);
          }
      break;

      case 17:
        if (pose.dist > 0.2)
        {
          state = 18; //coming to the ball
        }
      break;

      case 18:
        mixer.setTurnrate(0.5);
        if (pose.turned < -1.3)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("back on the line");
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 12;
        }
      break;

      case 18:
        //servo.setServo(2, 1, 500, 200);

        waitTime = t.getTimePassed(); 
          if (waitTime > 3)
          {
            pose.turned = 0;
            state = 19;
          } 

      break;

      case 19:
        mixer.setTurnrate(-0.5);
        if (pose.turned < -1.3)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("back on the line");
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 12;
        }
      break;*/




      //test cases

      case 99:
        std::cout << servo.servo_position[1] << std::endl;
        servo.setServo(2,true,-0,100);
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

void BSeesaw::run_withGolf()
{
  if (not setupDone)
    setup();
  if (ini["seesaw"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false; 
  bool lost = false;
  state = 5;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];

  float waitTime;
  bool leftEdge  = true;
  bool rightEdge = false;

  float normalSpeed        =  0.3;    //speed under normal conditions
  float lineWidth          =  0.02;   //width to determine if we are on the line
  float lineGone           =  0.01;   //width to determine if the line was lost
  float lineOffset         =  0.03;      //offset for line edge detection
  float intersectionWidth  =  0.1;   //used to detect intersections

  //seesaw variables
  float intersectionToEdge =  0.23;   //seesaw - distance from the intersection to the step
  float edgeToSeesaw       =  0.84;   //seesaw - distance from the edge to the tilting point
  float edgeWidth          =  1.098;  //seesaw - distance to assume the robot went down the step

  //ramp variables
  float rampUpDistance     = 2;       //distance from getting back on line to the start of the turn on the plateu
  float rampUpToHole       = 0.3;     //distance from the start of the turn to the golf ball hole

  float speed              = 0;
  float maxSpeed           = 0.5;

  int woodWhite = 600;
  int blackWhite = 350;

  std::vector<int> center{0,0};
  
  toLog("seesaw started");
  
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {  
      case 1: // Start Position, assume we are on a line but verify.
      servo.setServo(2, true, -900, 500);
      medge.updateCalibBlack(medge.calibBlack,8);
      medge.updatewhiteThreshold(blackWhite);
      heading.setMaxTurnRate(3);
      usleep(1000);

      mixer.setEdgeMode(leftEdge, lineOffset);

      // toLog(const_cast<char*>((std::to_string(medge.width)).c_str()));
      // std::cout << medge.width << std::endl;
      if(medge.edgeValid && (medge.width > intersectionWidth)) //We should be on a line 
      {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("found intersection");
          mixer.setVelocity(0.07);
          pose.dist = 0;
          servo.setServo(2, true, 350, 700);
          state = 3;
      }
      else if(medge.edgeValid && (medge.width > lineWidth))
      {
        // pose.resetPose();
        toLog("Follow Line with velocity 0.1");
        // mixer.setEdgeMode(leftEdge, lineOffset);
        mixer.setVelocity(0.1);
        // state = 2;
      }else{
        lost = true;
      }
       
      break;

      case 2:
        if (medge.width > intersectionWidth)
          {
            mixer.setEdgeMode(leftEdge, 0.05);
            toLog("found intersection");
            mixer.setVelocity(0.07);
            pose.dist = 0;
            
            state = 3;
          }
          else
          {    
            mixer.setVelocity(0.1);
          }
      break;

      case 3:
      {
        

        if (pose.dist > intersectionToEdge)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);

          mixer.setVelocity(0.2);
          pose.dist = 0;
          state = 4;
          
        }

        break;
      }

      case 4:
      {
        float accel = imu.acc[2];

        if (pose.dist > edgeWidth  || (accel < -1.4) )
        {
          const int MSL = 100;
          char s[MSL];
          snprintf(s, MSL, "detected shock: %f \n", accel);
          toLog(s);
          servo.setServo(2, true, -900, 500);
          //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
          // toLog("coming to the tilting point");
          mixer.setEdgeMode(leftEdge, lineOffset);
          mixer.setVelocity(0.1);
          pose.dist = 0;
          state = 5;
        }
        break;
      }

      case 5:
        toLog("Looking for golfball");
        if(golfball.findGolfball(center, nullptr)){
            char s[MSL];
            // next need to modify the reference...since my C++ knowledge are shit I guess
            // that we can just return the center as a whole (as in the function to return std::vector)
            snprintf(s, MSL, "Golfball found at X = %d, Y = %d", center[0], center[1]);
            toLog(s);
            state = 51;

        }else{
          toLog("No Golfball Found");
          lost = true;
        }
        finished = true;

        // if (pose.dist > edgeToSeesaw)
        // {
        //   //toLog(const_cast<char*>((std::to_string(pose.dist)).c_str()));
        //   toLog("robot on the tilting point");
        //   mixer.setVelocity(0);
        //   t.clear();
        //   state = 6;
        // }
      break;

      case 51:
      {

        break;
      }
      
      case 6:
        //servo.setServo(2, 1, -500, 200);
        
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
          medge.updateCalibBlack(medge.calibBlack,8);
          medge.updatewhiteThreshold(blackWhite);
          pose.resetPose();
          toLog("No Line");
          mixer.setVelocity(0.1);
          mixer.setEdgeMode(rightEdge, 0);
          state = 10;
        }
      break;

      case 10:
        if(medge.width > lineWidth)
        {
          toLog("Found Line again");
          mixer.setVelocity(0.07);
          pose.dist = 0;          
          state = 11;
        }
        break;

      case 11:
        if (pose.dist > 0.2)
        {
          medge.updateCalibBlack(medge.calibWood,8);
          medge.updatewhiteThreshold(woodWhite);
          state = 12;
        }
      break;

      case 12:
        // mixer.setVelocity(normalSpeed);
        if (medge.width > intersectionWidth)
          {
            toLog("found intersection - going straight");
            pose.resetPose();
            mixer.setVelocity(0.1);
            state = 14;
          }
        // else
        //   {    
        //     mixer.setVelocity(normalSpeed);
        //   }
      break;

      case 14:
        if(pose.dist > 0.1)
        {
          toLog("going 10cm straight");
          mixer.setEdgeMode(rightEdge, 0);
          mixer.setVelocity(0.1);          
          state = 15;
        }
      break;

      case 15:
        if(medge.edgeValid && (medge.width > lineWidth)){
          mixer.setVelocity(0.03);
          state = 17;
        }
        
        break;

      case 16:
        // if(medge.edgeValid && (medge.width > lineWidth)){
        //   mixer.setVelocity(0.03);
        //   state = 16;
        // }
        
      break;

      case 17:
      {
        finished = true;
      }
      break;

/*
      case 16:
        if (medge.width > intersectionWidth)
          {
            toLog("found intersection");
            mixer.setVelocity(0.05);
            pose.dist = 0;
            state = 17;
          }
        else
          {    
            mixer.setVelocity(normalSpeed);
          }
      break;

      case 17:
        if (pose.dist > 0.2)
        {
          state = 18; //coming to the ball
        }
      break;

      case 18:
        mixer.setTurnrate(0.5);
        if (pose.turned < -1.3)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("back on the line");
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 12;
        }
      break;

      case 18:
        //servo.setServo(2, 1, 500, 200);

        waitTime = t.getTimePassed(); 
          if (waitTime > 3)
          {
            pose.turned = 0;
            state = 19;
          } 

      break;

      case 19:
        mixer.setTurnrate(-0.5);
        if (pose.turned < -1.3)
        {
          mixer.setEdgeMode(leftEdge, lineOffset);
          toLog("back on the line");
          mixer.setVelocity(0.5);
          pose.dist = 0;
          state = 12;
        }
      break;*/




      //test cases

      case 99:
        std::cout << servo.servo_position[1] << std::endl;
        servo.setServo(2,true,-0,100);
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
  
  // servo.setServo(2,0);
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
