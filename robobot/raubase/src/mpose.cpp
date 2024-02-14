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
#include <thread>
#include <math.h>
#include "sencoder.h"
#include "mpose.h"
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
#include "cmixer.h"

// create value
MPose pose;


void MPose::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("pose"))
  { // no data yet, so generate some default values
    ini["pose"]["gear"] = "19.0";
    ini["pose"]["wheelDiameter"] = "0.146";
    ini["pose"]["encTickPerRev"] = "64";
    ini["pose"]["wheelbase"] = "0.243";
    ini["pose"]["log"] = "true";
    ini["pose"]["print"] = "false";
  }
  // get values from ini-file
  gear = strtof(ini["pose"]["gear"].c_str(), nullptr);
  wheelDiameter = strtof(ini["pose"]["wheelDiameter"].c_str(), nullptr);
  encTickPerRev = strtol(ini["pose"]["encTickPerRev"].c_str(), nullptr, 10);
  wheelBase = strtof(ini["pose"]["wheelBase"].c_str(), nullptr);
  distPerTick = (wheelDiameter * M_PI) / gear / encTickPerRev;
  //
  toConsole = ini["pose"]["print"] == "true";
  if (ini["pose"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_pose.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Pose and velocity (%s)\n", fn.c_str());
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2,3 \tVelocity left, right (m/s)\n");
    fprintf(logfile, "%% 4 \tRobot velocity (m/s)\n");
    fprintf(logfile, "%% 5 \tTurnrate (rad/s)\n");
    fprintf(logfile, "%% 6 \tTurn radius (m)\n");
    fprintf(logfile, "%% 7,8 \tPosition x,y (m)\n");
    fprintf(logfile, "%% 9 \theading (rad)\n");
    fprintf(logfile, "%% 10 \tDriven distance (m) - signed\n");
    fprintf(logfile, "%% 11 \tTurned angle (rad) - signed\n");
    // and absolute pose
    fn = service.logPath + "log_pose_abs.txt";
    logAbs = fopen(fn.c_str(), "w");
    fprintf(logAbs, "%% Pose without folding and reset (%s)\n", fn.c_str());
    fprintf(logAbs, "%% 1 \tTime (sec)\n");
    fprintf(logAbs, "%% 2,3 \tPosition x,y (m)\n");
    fprintf(logAbs, "%% 4 \theading (rad)\n");
    fprintf(logAbs, "%% 5 \tDriven distance (m) - signed\n");
    fprintf(logAbs, "%% 6 \tTurned angle (rad) - signed\n");
  }
  th1 = new std::thread(runObj, this);
}


void MPose::terminate()
{ // wait for thread to finish
  if (th1 != nullptr)
  {
    th1->join();
    th1 = nullptr;
  }
}


void MPose::run()
{
//   printf("# MPose::run started\n");
  int loop = 0;
  int64_t encLast[2] = {0};
  UTime t("now"); // time of update
  UTime encTimeLast[2];
  encTimeLast[0].now();
  encTimeLast[1].now();
  float dd[2]; // wheel moved since last update
  while (not service.stop)
  {
    if (encoder.updateCnt != encoderUpdateCnt)
    {
      encoderUpdateCnt = encoder.updateCnt;
      // get new data
      t = encoder.encTime;
      int64_t enc[2] = {encoder.enc[0], encoder.enc[1]};
      // debug
//       printf("# Pose got new encoder data %d,%d, at %.3fs\n",
//              enc[0], enc[1], t.getDecSec(teensy1.justConnectedTime));
      // debug end
      if (loop < 2)
      { // first two updates take last value as current
        encLast[0] = enc[0]; // left
        encLast[1] = enc[1]; // right
      }
      float dtt = 1.0; // in seconds - for turnrate
      float dt[2];
      int64_t de[2];
      for (int i = 0; i < 2; i++)
      { // find movement in time and distance for each wheel
        dt[i] = t - encTimeLast[i]; // time
        if (dt[i] < dtt)
        { // the minimum update time (the other wheel may be stationary)
          dtt = dt[i];
        }
        // left wheel - gives wrong results on Teensy
        // so calculate folding explicitly
        de[i] = enc[i] - encLast[i];
        if (llabs(de[i]) > 1000)
        { // given up in calculating folding around MAXINT,
          // so one sample of zero change should be OK.
          de[i] = 0;
        }
        // distance traveled since last
        dd[i] = float(de[i]) * distPerTick; // encoder ticks
        if (enc[i] != encLast[i])
        { // wheel has moved since last update
          encLast[i] = enc[i];
          encTimeLast[i] = t;
          wheelVel[i] = dd[i]/dt[i];
        }
        else
        { // no tick change since last update
          // update (reduce) velocity waiting for next tick
          wheelVel[i] = copysignf(1.0, wheelVel[i]) * distPerTick/dt[i];
        }
      }
      // turned angle in radians
      // dh is positive for CCV, i.e. when right wheel (dd[1]) goes faster
      float dh = (dd[1] - dd[0])/wheelBase;
      // moved distance in meters
      float ds = (dd[0] + dd[1])/2.0;
      // update position
      // both relative (x,y,h) and absolute (x2,y2,h2)
      h += dh/2.0;
      h2 += dh/2.0;
      x += cosf(h) * ds;
      y += sinf(h) * ds;
      x2 += cosf(h2) * ds;
      y2 += sinf(h2) * ds;
      h += dh/2.0;
      h2 += dh/2.0;
      // fold angle
      if (h > M_PI)
        h -= M_PI * 2;
      else if (h < -M_PI)
        h += M_PI * 2;
      if (h2 > M_PI)
        h2 -= M_PI * 2;
      else if (h2 < -M_PI)
        h2 += M_PI * 2;
      // update traveled distance and turned angle
      dist += ds;
      dist2 += ds;
      //
      turned += dh;
      turned2 += dh;
      //
      turnrate = dh/dtt;
      robVel = ds/dtt;
      const float minTurnrate = 0.001;
      if (fabs(turnrate) > minTurnrate)
        // positive radius for positive turn-rate
        turnRadius = robVel / turnrate;
      else
        // max radius is limited to minimum about 30m (at low speed (3cm/s))
        // to avoid infinity
        turnRadius = robVel / minTurnrate * copysignf(1.0, turnrate);
      //
      poseTime = t;
      updateCnt++;
      // finished making a new pose
      toLog();
      loop++;
    }
    else
      // just wait a bit (1ms)
      usleep(1000);
  }
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}

void MPose::resetPose()
{
  x = 0.0;
  y = 0.0;
  h = 0.0;
  dist = 0.0;
  turned = 0.0;
  mixer.setDesiredHeading(0);
}

void MPose::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    { // log_pose
      fprintf(logfile, "%lu.%04ld %.4f %.4f %.4f %.5f %.3f %.3f %.3f %.4f %.3f %.4f\n", poseTime.getSec(), poseTime.getMicrosec()/100,
              wheelVel[0], wheelVel[1], robVel,
              turnrate, turnRadius,
              x, y, h, dist, turned);
    }
    if (logAbs != nullptr)
    { // log_absolute pose
      fprintf(logAbs, "%lu.%04ld %.3f %.3f %.4f %.3f %.4f\n",
              poseTime.getSec(), poseTime.getMicrosec()/100,
              x2, y2, h2, dist2, turned2);
    }
    if (toConsole)
    { // print_pose
      printf("%lu.%04ld %.4f %.4f %.4f %.5f %.3f %.3f %.3f %.4f %.3f %.4f\n", poseTime.getSec(), poseTime.getMicrosec()/100,
              wheelVel[0], wheelVel[1], robVel,
              turnrate, turnRadius,
              x, y, h, dist, turned);
    }
  }
}
