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
#include "sdist.h"
#include "steensy.h"
#include "uservice.h"
// create value
SIrDist dist;


void SIrDist::setup()
{ // ensure default values
  if (not ini.has("dist"))
  { // no data yet, so generate some default values
    ini["dist"]["rate_ms"] = "45";
    ini["dist"]["ir13cm"] = "70000 70000"; // sharp sensor calibration
    ini["dist"]["ir50cm"] = "20000 20000";
    ini["dist"]["usCalib"] = "0.00126953125"; // 5.20m / 4096 (m per LSB)
    ini["dist"]["log"] = "true"; // save to logfile
    ini["dist"]["print"] = "false"; // print to console
    ini["dist"]["sensor1"] = "sharp"; // alternatives "sharp" or "URM09"
    ini["dist"]["sensor2"] = "sharp"; // alternatives "sharp" or "URM09"
  }
  // use values and subscribe to source data
  // like teensy1.send("sub pose 4\n");
  std::string c13 = ini["dist"]["ir13cm"];
  const char * p1 = c13.c_str();
  ir13cm[0] = strtol(p1, (char**)&p1, 10);
  ir13cm[1] = strtol(p1, (char**)&p1, 10);
  std::string c50 = ini["dist"]["ir50cm"];
  p1 = c50.c_str();
  ir50cm[0] = strtol(p1, (char**)&p1, 10);
  ir50cm[1] = strtol(p1, (char**)&p1, 10);
  urm09factor = strtof(ini["dist"]["usCalib"].c_str(), nullptr);
  //
  if (ini["dist"]["sensor1"] == "sharp")
    sensortype[0] = sharp;
  else
    sensortype[0] = URM09;
  if (ini["dist"]["sensor2"] == "sharp")
    sensortype[1] = sharp;
  else
    sensortype[1] = URM09;
  // send calibration values (and turn on the sensor)
  const int MSL = 100;
  char s[MSL];
  snprintf(s, MSL, "irc %d %d %d %d 1\n", ir13cm[0], ir50cm[0], ir13cm[1], ir50cm[1]);
  teensy1.send(s);
  // subscribe to sensor data
  std::string ss = "sub ir " + ini["dist"]["rate_ms"] + "\n";
  teensy1.send(ss.c_str());
  // logfiles
  toConsole = ini["dist"]["print"] == "true";
  if (ini["dist"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_irdist.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% IR distance sensor logfile %s\n", fn.c_str());
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2,3 \tsensor 1, 2 (m)\n");
    fprintf(logfile, "%% 4,5 \tsensor AD value 1, 2 (filtered)\n");
    fprintf(logfile, "%% sensor 1 type: %s\n", ini["dist"]["sensor1"].c_str());
    fprintf(logfile, "%% sensor 2 type: %s\n", ini["dist"]["sensor2"].c_str());
    fprintf(logfile, "%% sensor 1 sharp calib: 13cm: %d, 50cm: %d\n", ir13cm[0], ir50cm[0]);
    fprintf(logfile, "%% sensor 2 sharp calib: 13cm: %d, 50cm: %d\n", ir13cm[1], ir50cm[1]);
    fprintf(logfile, "%% sensor ultrasound URM09 factor (both): %f\n", urm09factor);
  }
}

void SIrDist::terminate()
{
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}

bool SIrDist::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "ir ", 3) == 0)
  {
    if (strlen(p1) > 3)
      p1 += 3;
    else
      return false;
    updTime = msgTime;
    // get values
    dist[0] = strtof(p1, (char**)&p1); // already converted by Teensy as sharp sensor
    dist[1] = strtof(p1, (char**)&p1);
    distAD[0] = strtol(p1, (char**)&p1, 10);
    distAD[1] = strtol(p1, (char**)&p1, 10);
    // could be an URM09 sensor
    if (sensortype[0] == URM09)
      dist[0] = distAD[0] * urm09factor;
    if (sensortype[1] == URM09)
      dist[1] = distAD[1] * urm09factor;
    // notify users of a new update
    updateCnt++;
    // save to log_encoder_pose
    toLog();
    // calibration
    if (inCalibration)
    {
      if (calibSensor == 1)
        calibSum += distAD[0];
      else
        calibSum += distAD[1];
      calibCount++;
      if (calibCount >= calibCountMax)
      {
        if (calibSensor == 1)
        {
          if (calibDist == 13)
            ir13cm[0] = calibSum / calibCount;
          else
            ir50cm[0] = calibSum / calibCount;
        }
        else
        {
          if (calibDist == 13)
            ir13cm[1] = calibSum / calibCount;
          else
            ir50cm[1] = calibSum / calibCount;
        }
        // save as new value to the ini structure
        const int MSL = 100;
        char s[MSL];
        if (calibDist == 13)
        {
          snprintf(s, MSL, "%d %d", ir13cm[0], ir13cm[1]);
          ini["dist"]["ir13cm"] = s;
        }
        else
        {
          snprintf(s, MSL, "%d %d", ir50cm[0], ir50cm[1]);
          ini["dist"]["ir50cm"] = s;
        }
        //
        inCalibration = false;
        printf("# IR distance for sensor %d at %dcm finished: %s\n", calibSensor, calibDist, s);
      }
    }
  }
  else
    used = false;
  return used;
}

void SIrDist::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    {
      fprintf(logfile,"%lu.%04ld %.3f %.3f %d %d\n", updTime.getSec(), updTime.getMicrosec()/100,
              dist[0], dist[1],
              distAD[0], distAD[1]);
    }
    if (toConsole)
    {
      printf("%lu.%04ld %.3f %.3f %d %d\n", updTime.getSec(), updTime.getMicrosec()/100,
              dist[0], dist[1],
              distAD[0], distAD[1]);
    }
  }
}


void SIrDist::calibrate(int sensor, int distance_cm)
{
  calibSensor = sensor;
  calibDist = distance_cm;
  inCalibration = true;
}
