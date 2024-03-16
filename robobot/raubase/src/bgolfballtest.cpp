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
#include "mgolfball.h"

#include "bgolfballtest.h"

// create class object
bgolfballtest golfballtest;


void bgolfballtest::setup()
{ // ensure there is default values in ini-file
  if (not ini["golfballtest"].has("log"))
  { // no data yet, so generate some default values
    ini["golfball"]["log"] = "true";
    ini["golfball"]["run"] = "false";
    ini["golfball"]["print"] = "true";
    ini["golfball"]["deadband_x"] = "10";
    ini["golfball"]["deadband_y"] = "10";
    ini["golfball"]["k_x"] = "0.5";
    ini["golfball"]["k_y"] = "0.5";
    ini["golfball"]["target_x"] = "640";
    ini["golfball"]["target_y"] = "650";
    ini["golfball"]["dist_y"] = "0.05";
    ini["golfball"]["servo_velocity"] = "200";
    ini["golfball"]["servo_down"] = "0";
  }
  //
  if (ini["golfball"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_golfballtest.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission golfballtest logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

bgolfballtest::~bgolfballtest()
{
  terminate();
}


void bgolfballtest::run()
{
  // setup
  if (not setupDone)
    setup();
  if (ini["golfballtest"]["run"] == "false")
    return;

  // get values from ini-file
  
  int deadband_x = strtol(ini["golfballtest"]["deadband_x"].c_str(), nullptr, 10);
  int deadband_y = strtol(ini["golfballtest"]["deadband_y"].c_str(), nullptr, 10);
  float k_x = strtol(ini["golfballtest"]["k_x"].c_str(), nullptr);
  float k_y = strtol(ini["golfballtest"]["k_y"].c_str(), nullptr);
  int target_x = strtol(ini["golfballtest"]["target_x"].c_str(), nullptr, 10);
  int target_y = strtol(ini["golfballtest"]["target_y"].c_str(), nullptr, 10);
  float dist_y = strtof(ini["golfballtest"]["dist_y"].c_str(), nullptr);
  int servo_down = strtol(ini["golfballtest"]["servo_down"].c_str(), nullptr, 10);
  int servo_velocity = strtol(ini["golfballtest"]["servo_velocity"].c_str(), nullptr, 10);
  //

  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 2;
  oldstate = state;
  const int MSL = 200;
  std::vector<int> center{0,0};

  //
  toLog("golfballtest started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission


      case 2:
        if(medge.edgeValid && (medge.width > 0.02)) //We should be on a line 
        {          
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.2");
          mixer.setEdgeMode(true /*left*/, 0.00 /* offset */);
          mixer.setVelocity(0.4);
          state = 3;
        }else{
          lost = true;
        }

        break;
      case 3:
        if(medge.edgeValid && (medge.width > 0.02)) //We should be on a line 
        {
          const int MSL = 200;
          char s[MSL];
          snprintf(s, MSL, "Following Line, driven %f m", pose.dist);
          toLog(s);

          if(pose.dist >= 1){
            toLog("Driven far enough");
            pose.resetPose();
            mixer.setDesiredHeading(1.570796);
            state = 4;
          }
          
        }else{
          lost = true;
        }

        break;

      case 4:
       if(abs(pose.turned-1.570796) < 0.1){
          pose.resetPose();
          mixer.setTurnrate(0);
          toLog("Finished Turn");
          state = 10;
        }else{
          toLog("Wait to finish turn");
        }        
        break;
    
      case 10:
        
        if(golfball.findGolfball(center, nullptr)){
            char s[MSL];
            snprintf(s, MSL, "Golfball found at X = %d, Y = %d", center[0], center[1]);
            toLog(s);
            state = 11;

        }else{
          toLog("No Golfball Found");
          lost = true;
        }

        break;

      case 11: 
        // Lateral Error
        
        int error_x = target_x - center[0];
        if(abs(error_x) > deadband_x){
          mixer.setVelocity(0);
          mixer.setTurnrate(k_x*((error_x > 0) ? 1 : -1));
          toLog("Correcting  x-offset");
          state = 10;
        }else{
          // Goldfball on line
          mixer.setTurnrate(0);
          state = 12;
        }
        break;

      case 12:
        // Forward Error
        int error_y = target_y - center[1];
        if(abs(error_y) > deadband_y){
          //pose.resetPose();
          //mixer.setDesiredHeading(k*error);
          mixer.setTurnrate(0);
          mixer.setVelocity(k_y*((error_y > 0) ? 1 : -1));
          toLog("Correcting  y-offset");
          state = 10;
        }else{
          mixer.setVelocity(0);
          state = 13;
        }

        break;

      case 13:
        // Goldfball on line close to robot    
        mixer.setTurnrate(0);
        pose.resetPose();

        mixer.setVelocity(0.1);
        toLog("Drive Forward Open-Loop");
        state = 14;

        break;

      case 14:
        if(pose.dist >= dist_y){
          //set servo down
          servo.setServo(1,true,servo_down,servo_velocity);
          toLog("Set Servo down");
          state = 15;
        }
        break;

      case 15:
        if(servo.servo_position - servo_down < 10){
          pose.resetPose();
          mixer.setDesiredHeading(1.570796);
          toLog("Servo reached down position");
          toLog("Start Turning 90 deg");
          state = 16;
        }else{
          toLog("Wait for servo to reach down position");
        }
        
        
        break;

      case 16:
        if(abs(pose.turned-1.570796) < 0.1){
          toLog("Finished Turn, finish Program");
          finished = true;
        }else{
          toLog("Wait to finish turn");
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
    toLog("golfballtest got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("golfballtest finished");
}


void bgolfballtest::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void bgolfballtest::toLog(const char* message)
{
  toConsole = ini["golfballtest"]["print"] == "true";
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

