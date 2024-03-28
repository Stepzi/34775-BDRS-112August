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
#include "bmission0.h"

// create class object
BMission0 mission0;


void BMission0::setup()
{ // ensure there is default values in ini-file
  if (not ini["mission0"].has("log"))
  { // no data yet, so generate some default values
    ini["mission0"]["log"] = "true";
    ini["mission0"]["run"] = "true";
    ini["mission0"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["mission0"]["print"] == "true";
  //
  if (ini["mission0"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_mission0.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission mission0 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BMission0::~BMission0()
{
  terminate();
}

void BMission0::run()
{
  if (not setupDone)
    setup();
  if (ini["mission0"]["run"] == "false")
    return;
  UTime t("now");
  bool finished = false;
  bool lost = false;
  float Turn90Deg = 3.14 / 2.0;
  float speed = 0;
  float rampSpeed = 0.6;
  int numOfDistMeas = 0;
  state = 0;
  oldstate = state;
  const int MSL = 100;
  char s[MSL];

  std::string gyro_msg;
  std::string acc_msg;
  UTime t2("now");
  //
  toLog("mission0 started");
  
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    {
      case 0: 
      
      if(t2.getTimePassed() > 0.2){
        gyro_msg = std::to_string(imu.gyro[0]) + "," + std::to_string(imu.gyro[1]) + "," + std::to_string(imu.gyro[2]) + ",";
        acc_msg = std::to_string(imu.acc[0]) + "," + std::to_string(imu.acc[1]) + "," + std::to_string(imu.acc[2]);
        toLog((gyro_msg + " " + acc_msg).c_str());
        t2.now();
        /*  
        MPU6050 GetRawData(&gyroRaw , &accelRaw ) ;   
        MPU6050 ScaleData(&gyroScal , &accelScal, &gyroRaw, &accelRaw);
        //ComplementaryFilter
        long squaresum=( long )accelScal.y∗accelScal.y+(long)accelScal.z∗accelScal.z;
        pitch += ((float)gyroScal.x) ∗ Ts;
        float pitchAcc = atan(accelScal.x/sqrt(squaresum))∗(180/3.1416); // rad to deg
        pitch = alpha∗pitch+(1.0f−alpha)∗pitchAcc;
        */
      }
      
      break;

      case 1: // Start Position, assume we are on a line but verify.
        if(medge.edgeValid && (medge.width > 0.02)) //We should be on a line 
        {
          pose.resetPose();
          toLog("Started on Line");
          toLog("Follow Line with velocity 0.2");
          mixer.setEdgeMode(false /* right */, 0.00 /* offset */);
          mixer.setVelocity(0.4);
          state = 2;
        }
        else if(medge.width < 0.01)
        {
          pose.resetPose();
          mixer.setVelocity(0.1);//Drive slowly and turn i circle
          mixer.setDesiredHeading(0.5);
        }
        else if(t.getTimePassed() > 10)
        {
          toLog("Never found Line");
          lost = true;
        }
        break;
      case 2:
        if(medge.edgeValid && (medge.width > 0.05))
        {
          toLog("Crossed first intersection");
          pose.dist = 0;
          pose.turned = 0;
          state = 3;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost Line between goal and first intersection - handle accordingly");
          lost = true;
        }
        break;      
      case 3:
        if(medge.edgeValid && pose.dist > 1)
        {
          toLog("Heading for Ramp");
          pose.dist = 0;
          pose.turned = 0;
          state = 4;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line between First intersection and heading for the ramp - handle accordingly");
          lost = true;
        }
        break;
      case 4:
        if(medge.edgeValid && abs(pose.turned) > Turn90Deg - 0.2) //Offset might be close to 90 and not exactly 90.
        {
          toLog("Going up the Ramp - Could be validated by using IMU");
          pose.dist = 0;
          pose.turned = 0;
          state = 5;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line before the ramp - handle accordingly");
          lost = true;
        }
        break;
      case 5:
        if(medge.edgeValid && medge.width > 0.05)
        {
          toLog("Crossed first intersection on Ramp towards SeeSaw");
          pose.dist = 0;
          pose.turned = 0;
          state = 7;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line beofre first intersection on the ramp - handle accordingly");
          lost = true;
        }
        break;
      case 7:
        if(medge.edgeValid && medge.width > 0.05 && pose.dist > 0.1)
        {
          toLog("Crossed second intersection on the ramp, towards stairs, reach plateu. Validate using IMU?");
          toLog("Slowing down");
          mixer.setVelocity(0.2);
          pose.dist = 0;
          pose.turned = 0;
          state = 8;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line on between the two intersections on the ramp - handle accordingly");
          lost = true;
        }
      break;
      case 8:
        if(medge.edgeValid && abs(pose.turned) > Turn90Deg - 0.2) //Offset might be close to 90 and not exactly 90.
        {
          toLog("Heading down the ramp.");
          pose.dist = 0;
          pose.turned = 0;
          state = 9;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line before going down the ramp - STOP");
          mixer.setVelocity(0.0);
          lost = true;
        }
        break;
      case 9:
        if(speed < rampSpeed)
        {
          speed = speed + 0.001;
          mixer.setVelocity(0.2 + speed);
        }
        else if(speed >= rampSpeed)
        {
          toLog("Speed up");
          state = 10;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line going down the ramp - STOP");
          mixer.setVelocity(0.0);
          lost = true;
        }
        break;
      case 10:
        if(medge.edgeValid && pose.dist > 2.0)
        {
          toLog("Got Down the Ramp, slow down");
          mixer.setVelocity(0.2);
          state = 11;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line after going down the ramp - STOP");
          mixer.setVelocity(0.0);
          lost = true;
        }
        break;
      case 11:
        //toLog(std::to_string(dist.dist[1]).c_str());
        if (medge.edgeValid && dist.dist[1] < 0.3) //A Large number will trigger on the ramp and gates
        { // something is close, assume it is the goal
          toLog("Found distance meassure");
          numOfDistMeas = numOfDistMeas + 1;
        }
        else if( dist.dist[1] > 0.3)
        {
          numOfDistMeas = 0;
        }
        else if(!medge.edgeValid)
        {
          toLog("Lost line after going down the ramp - STOP");
          mixer.setVelocity(0.0);
          lost = true;
        }

        if(numOfDistMeas >= 10) //USER MORE THAN 1 SAMPLE
        {
          state = 12;
        }
        break;
      case 12:
          toLog("Goal Found- stop in front of it");
          mixer.setVelocity(0.0);
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
    toLog("mission0 got lost - stopping");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("mission0 finished");
}


void BMission0::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BMission0::toLog(const char* message)
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
