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
#include "cservo.h"
#include "steensy.h"
#include "uservice.h"
// create value
CServo servo;


void CServo::setup()
{ // ensure default values
  if (not ini.has("servo"))
  { // no data yet, so generate some default values
    ini["servo"]["rate_ms"] = "50";
    ini["servo"]["log"] = "true";
    ini["servo"]["print"] = "true";
  }
  // use values and subscribe to source data
  // like teensy1.send("sub pose 4\n");
  std::string s = "sub svo " + ini["servo"]["rate_ms"] + "\n";
  teensy1.send(s.c_str());
  // debug print
  toConsole = ini["servo"]["print"] == "true";
  // set servo
  if (ini["servo"]["log"] == "true")
  { // open logfile for servo data from Teensy
    std::string fn = service.logPath + "log_servo.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Servo logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2,3,4 \tservo 1: enabled, position, velocity\n");
    fprintf(logfile, "%% 5,6,7 \tservo 1: enabled, position, velocity\n");
    fprintf(logfile, "%% 8,9,10 \tservo 1: enabled, position, velocity\n");
    fprintf(logfile, "%% 11,12,13 \tservo 1: enabled, position, velocity\n");
    fprintf(logfile, "%% 14,15,16 \tservo 1: enabled, position, velocity\n");
    fn = service.logPath + "log_servo_ctrl.txt";
    logfileCtrl = fopen(fn.c_str(), "w");
    fprintf(logfileCtrl, "%% Servo commands logfile\n");
    fprintf(logfileCtrl, "%% 1 \tTime (sec)\n");
    fprintf(logfileCtrl, "%% 2 \tServo number\n");
    fprintf(logfileCtrl, "%% 3,4,5 \tEnabled, position, velocity\n");
  }
}

void CServo::setServo(int servo, bool enabled, int position, int velocity)
{
  const int MSL = 100;
  char s[MSL];
  UTime t("now");
  if (enabled)
    snprintf(s, MSL, "servo %d %d %d\n", servo, position, velocity);
  else
    snprintf(s, MSL, "servo %d 10000 0\n", servo);
  teensy1.send(s);
  if (logfileCtrl != nullptr)
  {
    fprintf(logfileCtrl, "%lu.%03ld %d %d %d\n",
            t.getSec(), t.getMilisec(),
            enabled, position, velocity);
  }
}

void CServo::terminate()
{
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
  if (logfileCtrl != nullptr)
    fclose(logfileCtrl);
}

bool CServo::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "svo ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    updTime = msgTime;
    for (int i = 0; i < 5; i++)
    {
      servo_enabled[i] = strtol(p1, (char**)&p1, 10);
      servo_position[i] = strtol(p1, (char**)&p1, 10);
      servo_velocity[i] = strtol(p1, (char**)&p1, 10);
    }
    // notify users of a new update
    updateCnt++;
    // save to log_encoder_pose
    toLog();
  }
  else
    used = false;
  return used;
}

void CServo::toLog()
{
  if (logfile != nullptr and not service.stop)
  {
    fprintf(logfile, "%lu.%03ld %d %d %d  %d %d %d  %d %d %d  %d %d %d %d %d %d\n",
            updTime.getSec(), updTime.getMilisec(),
            servo_enabled[0], servo_position[0], servo_velocity[0],
            servo_enabled[1], servo_position[1], servo_velocity[1],
            servo_enabled[2], servo_position[2], servo_velocity[2],
            servo_enabled[3], servo_position[3], servo_velocity[3],
            servo_enabled[4], servo_position[4], servo_velocity[4]
    );
  }
}

