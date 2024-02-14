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
#include "steensy.h"
#include "sstate.h"
#include "uservice.h"

// create the class with received info
SState state;


void SState::setup()
{ /// subscribe to pose information
  if (not ini.has("state"))
  { // no teensy group, so make one.
    ini["state"]["log"] = "true";
    ini["state"]["print"] = "false";
    ini["state"]["regbot_version"] = "000";
  }
  toConsole = ini["state"]["print"] == "true";
  teensy1.send("sub hbt 500\n");
  if (ini["state"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_hbt.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Heartbeat logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tRobot name index\n");
    fprintf(logfile, "%% 3 \tVersion\n");
    fprintf(logfile, "%% 4 \tState (0 = control is external to Teensy)\n");
    fprintf(logfile, "%% 5 \tBattery voltage (V)\n");
    fprintf(logfile, "%% 6 \tTeensy load (%%)\n");
    fprintf(logfile, "%% 7-8 \tMotor enabled flag (left,right) (may be 0 after overload)\n");
  }
//   printf("# SState:: setup finished\n");
}

void SState::terminate()
{
  if (logfile != nullptr)
  {
    dataLock.lock();
    fclose(logfile);
    logfile = nullptr;
    dataLock.unlock();
  }
}


bool SState::decode(const char* msg, UTime & msgTime)
{ // like: regbot:hbt 37708.7329 74 1430 5.01 0 6 1 1
  /* hbt 1 : time in seconds, updated every sample time
  *     2 : device ID (probably 1)
  *     3 : software revision number - from SVN * 10 + REV_MINOR
  *     4 : Battery voltage
  *     5 : state
  *     6 : hw type
  *     7 : load
  *     8,9 : motor enabled (left,right)
  */
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "hbt ", 4) == 0)
  { // decode pose message
    // advance to first parameter
    if (strlen(p1) > 5)
      p1 += 5;
    else
      return false;
    // get data
    dataLock.lock();
    // time in seconds from Teensy
    double tt = strtof64(p1, (char**)&p1);
    teensyTime = tt;
    int x = strtol(p1, (char**)&p1, 10); // index (robot number)
    if (x != idx)
    { // set robot number into ini-file
      idx = x;
      ini["id"]["idx"] = to_string(idx);
      // also ask for the new name
      teensy1.send("idi\n", true);
      printf("# SState::decode: asked for new name (idi -> dname)\n");
    }
    int rv = strtol(p1, (char**)&p1, 10); // index (from SVN)
    if (rv != version)
    {
      version = rv;
      ini["state"]["regbot_version"] = to_string(rv);
    }
    batteryVoltage = strtof(p1, (char**)&p1); // y
    controlState = strtol(p1, (char**)&p1, 10); // control state 0=no control, 2=user mission
    type = strtol(p1, (char**)&p1, 10); // hardware type
    load = strtol(p1, (char**)&p1, 10); // Teensy load in %
    motorEnabled[0] = strtol(p1, (char**)&p1, 10); // motor 1
    motorEnabled[1] = strtol(p1, (char**)&p1, 10); // motor 2
    //
    hbtTime = msgTime;
    // save to log if file is open
    toLog();
    dataLock.unlock();
  }
  else
    used = false;
  return used;
}


void SState::toLog()
{
  if (service.stop)
    return;
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%03ld %d %d %d %.2f %.1f %d %d\n", hbtTime.getSec(), hbtTime.getMilisec(),
            idx, version, controlState, batteryVoltage,
            load, motorEnabled[0], motorEnabled[1]);
  }
  if (toConsole)
    printf("%lu.%03ld state %d %d %d %.2f %.1f %d %d\n", hbtTime.getSec(), hbtTime.getMilisec(),
            idx, version, controlState, batteryVoltage,
            load, motorEnabled[0], motorEnabled[1]);
}

