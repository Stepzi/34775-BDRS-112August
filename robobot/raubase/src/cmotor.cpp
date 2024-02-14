/*  
 * 
 * Copyright © 2022 DTU, 
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
#include "sencoder.h"
#include "cmotor.h"
#include "steensy.h"
#include "uservice.h"
#include "mpose.h"
#include "cmixer.h"

// create value
CMotor motor;


void CMotor::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("motor") or not ini["motor"].has("print_m1"))
  { // motor block is OK, but control parameters are needed too
    ini["motor"]["kp"] = "7.0"; // unit is (V per (m/sec))
    ini["motor"]["lead"] = "0 1.0"; // tau_d (sec) and alpha, tau_d = 0.0 means no function
    ini["motor"]["taui"] = "0.05"; // tau_i (sec) 0.0 is no integrator function
    ini["motor"]["maxMotV"] = "10.0"; // (volt)
    ini["motor"]["log"] = "true";
    ini["motor"]["print_m1"] = "false";
    ini["motor"]["print_m2"] = "false";
  }
  //
  // get ini-values
  kp = strtof(ini["motor"]["kp"].c_str(), nullptr);
  const char * p1 = ini["motor"]["lead"].c_str();
  // lead
  taud = strtof(p1, (char**)&p1);
  alpha = strtof(p1, (char**)&p1);
  // integrator
  taui = strtof(ini["motor"]["taui"].c_str(), nullptr);
  // output limit
  maxMotV = strtof(ini["motor"]["maxMotV"].c_str(), nullptr);
  // sample time from encoder module
  sampleTime = strtof(ini["encoder"]["rate_ms"].c_str(), nullptr) / 1000.0;
  //
  pid[0].setup(sampleTime, kp, taud, alpha, taui);
  pid[1].setup(sampleTime, kp, taud, alpha, taui);
  //
  pid[0].toConsole = ini["motor"]["print_m1"] == "true";
  pid[1].toConsole = ini["motor"]["print_m2"] == "true";
  // initialize logfile
  if (ini["motor"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_motor_0.txt";
    logfile[0] = fopen(fn.c_str(), "w");
    fn = service.logPath + "log_motor_1.txt";
    logfile[1] = fopen(fn.c_str(), "w");
    logfileLeadText(logfile[0], "left");
    pid[0].logPIDparams(logfile[0], false);
    logfileLeadText(logfile[1], "right");
    pid[1].logPIDparams(logfile[1], false);
  }
  th1 = new std::thread(runObj, this);
}

void CMotor::logfileLeadText(FILE * f, const char * side)
{
    fprintf(f, "%% Motor control (%s) logfile\n", side);
    fprintf(f, "%% 1 \tTime (sec)\n");
    fprintf(f, "%% 2 \tReference for %s motor (m/sec)\n", side);
    fprintf(f, "%% 3 \tMeasured velocity for motor (m/sec)\n");
    fprintf(f, "%% 4 \tValue after Kp (V)\n");
    fprintf(f, "%% 5 \tValue after Lead (V)\n");
    fprintf(f, "%% 6 \tIntegrator value (V)\n");
    fprintf(f, "%% 7 \tMotor voltage output (V)\n");
    fprintf(f, "%% 8 \tIs output limited (1=limited)\n");
}

void CMotor::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile[0] != nullptr)
  {
    UTime t("now");
    char d[100];
    t.getDateTimeAsString(d);
    fprintf(logfile[0], "%% ended at %lu.%4ld %s\n", t.getSec(), t.getMicrosec()/100, d);
    fprintf(logfile[1], "%% ended at %lu.%4ld %s\n", t.getSec(), t.getMicrosec()/100, d);
    fclose(logfile[0]);
    fclose(logfile[1]);
    logfile[0] = nullptr;
    logfile[1] = nullptr;
  }
}


void CMotor::run()
{
//   printf("# CMotor::run\n");
  int loop = 0;
  UTime lastPose;
  while (not service.stop)
  {
    if (false) //useTeensyControl)
    { // send new velocity ref to Teensy
      if (mixer.updateCnt != mixerUpdateCnt)
      {
        mixerUpdateCnt = mixer.updateCnt;
        const int MSL = 100;
        char s[MSL];
        float * vr = mixer.getWheelVelocityArray();
        float v = (vr[0] + vr[1])/2.0;
        float d = vr[0] - vr[1];
        snprintf(s, MSL, "rc 3 %.3f %.3f 0\n", v, d);
        teensy1.send(s, true);
      }
    }
    else if (pose.updateCnt != poseUpdateCnt)
    { // do constant rate control
      // that is every time new encoder data is available
      // new motor control values should be calculated.
      poseUpdateCnt = pose.updateCnt;
      // do velocity control.
      // got new encoder data
      float dt = lastPose - pose.poseTime;
      // desired velocity from mixer
      float * vr = mixer.getWheelVelocityArray();
      if (dt < 1.0)
      { // valid control timing
        u[0] = pid[0].pid(vr[0], pose.wheelVel[0], limited);
        u[1] = pid[1].pid(vr[1], pose.wheelVel[1], limited);
        // test for output limiting
        if (fabsf(u[0]) > maxMotV or fabsf(u[1]) > maxMotV)
        { // some speed reduction is needed
          limited = true;
          // find speed reduction factor to allow turning
          float fac;
          if (fabsf(u[0]) > fabsf(u[1]))
            fac = maxMotV/(fabsf(u[0]));
          else
            fac = maxMotV/(fabsf(u[1]));
          u[0] *= fac;
          u[1] *= fac;
        }
        else
          limited = false;
      }
      lastPose = pose.poseTime;
      // log_pose - for both motors
      pid[0].saveToLog(logfile[0], pose.poseTime);
      pid[1].saveToLog(logfile[1], pose.poseTime);
      // finished calculating motor voltage
      const int MSL = 100;
      char s[MSL];
      /// Left motor output actually inverts motor voltage.
      /// So if both are commanded with a positive voltage
      /// robot drives forward,
      /// Here the sign must therefore be changed to compensate.
      snprintf(s, MSL, "motv %.2f %.2f\n", u[0], u[1]);
      teensy1.send(s, true);
    }
    loop++;
    // sleep a little while, the sample time is
    // determined by the encoder (longer than 2ms)
    // actually determined by the Teensy, so on average
    // a constant sample rate (defined in the robot.ini file)
    usleep(2000);
  }
  // stop motors
  teensy1.send("motv 0 0\n");
}


