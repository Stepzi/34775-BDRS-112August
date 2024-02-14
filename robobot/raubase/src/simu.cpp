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
#include "simu.h"
#include "steensy.h"
#include "uservice.h"
// create value
SImu imu;


void SImu::setup()
{ // ensure default values
  if (not ini.has("imu"))
  { // no data yet, so generate some default values
    ini["imu"]["rate_ms"] = "12";
    ini["imu"]["gyro_offset"] = "0 0 0";
    ini["imu"]["log"] = "true";
    ini["imu"]["print_gyro"] = "false";
    ini["imu"]["print_acc"] = "false";
  }
  // use values and subscribe to source data
  // like teensy1.send("sub pose 4\n");
  std::string s = "sub gyro0 " + ini["imu"]["rate_ms"] + "\n";
  teensy1.send(s.c_str());
  s = "sub acc0 " + ini["imu"]["rate_ms"] + "\n";
  teensy1.send(s.c_str());
  // gyro offset
  const char * p1 = ini["imu"]["gyro_offset"].c_str();
  gyroOffset[0] = strtof(p1, (char**)&p1);
  gyroOffset[1] = strtof(p1, (char**)&p1);
  gyroOffset[2] = strtof(p1, (char**)&p1);
  // send calibration values to Teensy
  const int MSL = 100;
  char ss[MSL];
  snprintf(ss, MSL, "gyrocal %g %g %g\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
  teensy1.send(ss);
  //
  toConsoleGyro = ini["imu"]["print_gyro"] == "true";
  toConsoleAcc = ini["imu"]["print_acc"] == "true";
  if (ini["imu"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_gyro.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Gyro logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2-4 \tGyro (x,y,z)\n");
    fprintf(logfile, "%% Gyro offset %g %g %g\n", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
    //
    fn = service.logPath + "log_acc.txt";
    logfileAcc = fopen(fn.c_str(), "w");
    fprintf(logfileAcc, "%% Accelerometer logfile\n");
    fprintf(logfileAcc, "%% 1 \tTime (sec)\n");
    fprintf(logfileAcc, "%% 2-4 \tAccelerometer (x,y,z)\n");
  }
}

void SImu::terminate()
{
  if (logfileAcc != nullptr)
  {
    fclose(logfileAcc);
  }
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}

bool SImu::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "acc0 ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    updTimeAcc = msgTime;
    acc[0] = strtof(p1, (char**)&p1);
    acc[1] = strtof(p1, (char**)&p1);
    acc[2] = strtof(p1, (char**)&p1);
    // notify users of a new update
    updateCnt++;
    // save to log
    toLog(true);
  }
  else if (strncmp(p1, "gyro0 ", 5) == 0)
  {
    if (strlen(p1) > 5)
      p1 += 5;
    else
      return false;
    updTime = msgTime;
    gyro[0] = strtof(p1, (char**)&p1);
    gyro[1] = strtof(p1, (char**)&p1);
    gyro[2] = strtof(p1, (char**)&p1);
    // notify users of a new update
    updateCnt++;
    // save to log
    toLog(false);
    //
    if (inCalibration)
    {
      for (int j = 0; j < 3; j++)
        calibSum[j] = gyro[j];
      calibCount++;
      if (calibCount >= calibCountMax)
      {
        for (int j = 0; j < 3; j++)
          gyroOffset[j] = calibSum[j]/calibCount;
        // implement new values
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "%g %g %g", gyroOffset[0], gyroOffset[1], gyroOffset[2]);
        ini["imu"]["gyro_offset"] = s;
        inCalibration = false;
        printf("# gyro calibration finished: %s\n", s);
      }
    }
  }
  else
    used = false;
  return used;
}

void SImu::toLog(bool accChanged)
{
  if (service.stop)
    return;
  if (accChanged)
  { // accelerometer
    if (logfileAcc != nullptr)
    {
      fprintf(logfileAcc,"%lu.%04ld %.4f %.4f %.4f\n", updTimeAcc.getSec(), updTimeAcc.getMicrosec()/100,
              acc[0], acc[1], acc[2]);
    }
    if (toConsoleAcc)
    {
      printf("%lu.%04ld %.4f %.4f %.4f\n", updTimeAcc.getSec(), updTimeAcc.getMicrosec()/100,
             acc[0], acc[1], acc[2]);
    }
  }
  else
  { // gyro data
    if (logfile != nullptr)
    {
      fprintf(logfile,"%lu.%04ld %.4f %.4f %.4f\n", updTimeAcc.getSec(), updTimeAcc.getMicrosec()/100,
              gyro[0], gyro[1], gyro[2]);
    }
    if (toConsoleGyro)
    {
      printf("%lu.%04ld %.4f %.4f %.4f\n", updTimeAcc.getSec(), updTimeAcc.getMicrosec()/100,
              gyro[0], gyro[1], gyro[2]);
    }
  }
}

void SImu::calibrateGyro()
{
  inCalibration = true;
}

