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
    ini["golfballtest"]["log"] = "true";
    ini["golfballtest"]["run"] = "false";
    ini["golfballtest"]["print"] = "true";
    ini["golfballtest"]["deadband_x"] = "10";
    ini["golfballtest"]["deadband_y"] = "10";
    ini["golfballtest"]["k_x"] = "1";
    ini["golfballtest"]["k_y"] = "1";
    ini["golfballtest"]["target_x"] = "640";
    ini["golfballtest"]["target_y"] = "650";
    ini["golfballtest"]["dist_y"] = "0.05";
    ini["golfballtest"]["servo_velocity"] = "200";
    ini["golfballtest"]["servo_down"] = "0";
  }
  // get values from ini-file
  toConsole = ini["golfballtest"]["print"] == "true";
  int deadband_x = strtol(ini["golfballtest"]["deadband_x"].c_str(), nullptr, 10);
  int deadband_y = strtol(ini["golfballtest"]["deadband_y"].c_str(), nullptr, 10);
  int k_x = strtol(ini["golfballtest"]["k_x"].c_str(), nullptr, 10);
  int k_y = strtol(ini["golfballtest"]["k_y"].c_str(), nullptr, 10);
  int target_x = strtol(ini["golfballtest"]["target_x"].c_str(), nullptr, 10);
  int target_y = strtol(ini["golfballtest"]["target_y"].c_str(), nullptr, 10);
  float dist_y = strtof(ini["golfballtest"]["dist_y"].c_str(), nullptr, 10);
  int servo_down = strtol(ini["golfballtest"]["servo_down"].c_str(), nullptr, 10);
  int servo_velocity = strtol(ini["golfballtest"]["servo_velocity"].c_str(), nullptr, 10);

  //
  if (ini["golfballtest"]["log"] == "true")
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
  if (not setupDone)
    setup();
  if (ini["golfballtest"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 5;
  oldstate = state;

  int center = {0,0};

  //
  toLog("golfballtest started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { // make a shift in heading-mission


      case 5:
        toLog("On branch facing up");
        


        break;
    
      case 10:
        
        if(Mgolfball::findGolfball(center)){
            const int MSL = 200;
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
        
        int error = target_x - center[0];
        if(abs(error) > deadband_x){
          //pose.resetPose();
          //mixer.setDesiredHeading(k*error);
          mixer.setVelocity(0);
          mixer.setTurnrate(k_x*((error > 0) ? 1 : ((error < 0) ? -1 : 0))));
          toLog("Correcting  x-offset");
          state = 10;
        }else{
          // Goldfball on line
          state = 12;
        }
        break;

      case 12:
        // Forward Error
        int error = target_y - center[1];
        if(abs(error) > deadband_y){
          //pose.resetPose();
          //mixer.setDesiredHeading(k*error);
          mixer.setTurnrate(0);
          mixer.setVelocity(k_y*((error > 0) ? 1 : ((error < 0) ? -1 : 0))));
          toLog("Correcting  y-offset");
          state = 10;
        }else{
          state = 13;
        }

        break;

      case 13:
        // Goldfball on line close to robot    
        mixer.setVelocity(0);
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
          mixer.setDesiredHeading(1.570796)
          toLog("Servo reached down position");
          toLog("Start Turning 90 deg");
          state = 16;
        }
        toLog("Wait for servo to reach down position");
        
        break;

      case 16:
        if(abs(pose.turned-1.570796) < 0.1){
          toLog("Finished Turn, finish Program");
          finished = true;
        }
        toLog("Wait to finish turn");
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

