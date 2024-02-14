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
#include "medge.h"
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"

// create value
MEdge medge;


void MEdge::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("edge") or not ini["edge"].has("calibWhite"))
  { // no data yet, so generate some default values
    ini["edge"]["calibWhite"] = "1000 1000 1000 1000 1000 1000 1000 1000"; // A/D value
    ini["edge"]["calibBlack"] = "0 0 0 0 0 0 0 0"; // A/D value
    ini["edge"]["whiteThreshold"] = "700"; // of 1000
    ini["edge"]["sensorWidth"] = "0.12"; // m
    ini["edge"]["log"] = "true"; // log edge detection items
    ini["edge"]["logNorm"] = "true"; // log edge detection items
  }
  // get values from ini-file
  const char * p1 = ini["edge"]["calibWhite"].c_str();
  // white calibration value
  for (int i = 0; i < 8; i++)
    calibWhite[i] = strtol(p1, (char**)&p1,10);
  // black calibration value
  p1 = ini["edge"]["calibBlack"].c_str();
  for (int i = 0; i < 8; i++)
  {
    calibBlack[i] = strtol(p1, (char**)&p1,10);
    calibrationValid = (calibWhite[i] - calibBlack[i]) > 10;
  }
  if (not calibrationValid)
  {
    printf("# ****** MEdge::findEdge: invalid line sensor calibration values.\n");
    printf("# values white");
    for (int i = 0; i < 8; i++)
      printf(" %6d", calibWhite[i]);
    printf("\n# values black");
    for (int i = 0; i < 8; i++)
      printf(" %6d", calibBlack[i]);
    printf("\n");
  }
  // convert per-cent to per-mille
  whiteThresholdPm = strtol(ini["edge"]["whiteThreshold"].c_str(), nullptr, 10);
  sensorWidth = strtod(ini["edge"]["sensorWidth"].c_str(), nullptr);
  //
  // initiate data log for this module
  toConsole = ini["edge"]["print"] == "true";
  if (ini["edge"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_edge.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Edge sensor logfile %s\n", fn.c_str());
    // save calibration values as text
    fprintf(logfile, "%% \tCalib white");
    for (int i = 0; i < 8; i++)
      fprintf(logfile, " %d", calibWhite[i]);
    fprintf(logfile, "\n");
    //
    fprintf(logfile, "%% \tCalib black");
    for (int i = 0; i < 8; i++)
      fprintf(logfile, " %d", calibBlack[i]);
    fprintf(logfile, "\n");
    //
    fprintf(logfile, "%% \tWhite threshold (of 1000) %d \n", whiteThresholdPm);
    // and extracted values
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tEdge valid\n");
    fprintf(logfile, "%% 3 \tLeft edge position(m)\n");
    fprintf(logfile, "%% 4 \tRight edge position (m)\n");
    fprintf(logfile, "%% 5 \tLine width (m)\n");
    if (not calibrationValid)
      fprintf(logfile, "\n ### Calibration is not valid - see values above\n");
  }
  if (ini["edge"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_edge_normalized.txt";
    logfileNorm = fopen(fn.c_str(), "w");
    fprintf(logfileNorm, "%% Edge sensor logfile normalized '%s'\n", fn.c_str());
    // and extracted values
    fprintf(logfileNorm, "%% 1 \tTime (sec)\n");
    // and extracted values
    fprintf(logfileNorm, "%% 2..9 \tSensor value in 0..1000 scale for black to white\n");
    fprintf(logfileNorm, "%% 10 \tLine width (m)\n");
    if (not calibrationValid)
      fprintf(logfile, "\n ### Calibration is not valid - see log_edge.txt or robot.ini\n");
  }
  th1 = new std::thread(runObj, this);
}


void MEdge::terminate()
{ // wait for thread to finish
  if (th1 != nullptr)
    th1->join();
}

void MEdge::findEdge()
{
  if (not calibrationValid)
  { // invalid calibration values
    leftEdge = 0.0;
    rightEdge = 0.0;
//     crossingValid = false;
    edgeValid = false;
    return;
  }
  int vsum = 0;
  bool lineValid = false;
  //
  // normalize to per-mille
  // make calibrated values and scale to 1000
  for (int i = 0; i < 8; i++)
  {
    int v = sedge.edgeRaw[i] - calibBlack[i];
    v = (v * 1000) / (calibWhite[i] - calibBlack[i]);
    if (v > 1000)
      v = 1000;
    else if (v < 0)
      v = 0;
    ls[i] = v;
    vsum += v;
    // line valid detect
    if (v > whiteThresholdPm)
      lineValid = true;
  }
  //
  edgeValid = lineValid;
  //
  // edge position
  if (lineValid)
  { // calculate edge position
    // left edge
    // left-most sensors has number 0
    if (ls[0] > whiteThresholdPm)
      leftEdge = 0;
    else
    {
      for (l = 0; l < 7; l++)
      {
        if (ls[l+1] > whiteThresholdPm)
          break;
      }
      // distance to threshold
      eeL = whiteThresholdPm - ls[l];
      // change from sensor l to l+1
      ddL = ls[l+1] - ls[l];
      if (ddL > 0)
        leftEdge = l + float(eeL)/float(ddL);
    }
    //
    // right edge
    if (ls[7] > whiteThresholdPm)
      rightEdge = 7;
    else
    {
      for (r = 7; r > 0; r--)
      {
        if (ls[r-1] > whiteThresholdPm)
          break;
      }
      // distance to threshold
      eeR = whiteThresholdPm - ls[r];
      // change from sensor r to r-1
      ddR = ls[r-1] - ls[r];
      if (ddR > 0)
        rightEdge = r - float(eeR)/float(ddR);
    }
  }
  else
  { // line not valid - say (0,0)
    leftEdge = 3.5;
    rightEdge = 3.5;
  }
  //
  // scale to meters (positive is left)
  leftEdge = -((leftEdge * sensorWidth / 7.0 ) - sensorWidth/2.0);
  rightEdge = -((rightEdge * sensorWidth / 7.0 ) - sensorWidth/2.0);
  //
  width = leftEdge - rightEdge;
  // finished - log/print as needed
  toLog();
}

void MEdge::run()
{
  int loop = 0;
  while (not service.stop)
  {
    if ((sensorCalibrateWhite or sensorCalibrateBlack) and
      sedge.updateCnt > 100      )
    { // start summing calibration values
      if (sensorCalibrateCount == 0)
      { // start collect values now
        sensorCalibrateCount = sensorCalibrateSamples;
        for (int i=0; i < 8; i++)
          sensorCalibrateValue[i] = 0;
      }
    }
    if (sedge.updateCnt != lineUpdateCnt)
    { // new values are available
      updTime = sedge.updTime;
      lineUpdateCnt = sedge.updateCnt;
      loop++;
      // calculate edge position
      if (not (sensorCalibrateWhite or sensorCalibrateBlack))
      { // regular update
        findEdge();
        // inform users of update
        updateCnt++;
      }
      else if (sensorCalibrateCount > 0)
      { // calibration active
        for (int i = 0; i < 8; i++)
        { // add new value
          sensorCalibrateValue[i] += sedge.edgeRaw[i];
        }
        sensorCalibrateCount--;
        if (sensorCalibrateCount <= 0)
        { // show old calibration value
          printf("# Old calibration values:\n# white:");
          for (int i = 0; i < 8; i++)
            printf(" %5d", calibWhite[i]);
          printf("\n# black:");
          for (int i = 0; i < 8; i++)
            printf(" %5d", calibBlack[i]);
          printf("\n");
          // save new values as string for ini structure
          const int MSL = 400;
          char s[MSL];
          snprintf(s, MSL, "%d %d %d %d %d %d %d %d",
              sensorCalibrateValue[0] / sensorCalibrateSamples,
              sensorCalibrateValue[1] / sensorCalibrateSamples,
              sensorCalibrateValue[2] / sensorCalibrateSamples,
              sensorCalibrateValue[3] / sensorCalibrateSamples,
              sensorCalibrateValue[4] / sensorCalibrateSamples,
              sensorCalibrateValue[5] / sensorCalibrateSamples,
              sensorCalibrateValue[6] / sensorCalibrateSamples,
              sensorCalibrateValue[7] / sensorCalibrateSamples);
          if (sensorCalibrateWhite)
          { // save average as white value
            sensorCalibrateWhite = false;
            printf("# New calibration values:\n# white %s\n", s);
            ini["edge"]["calibWhite"] = s;
          }
          else
          { // save average as black value
            sensorCalibrateBlack = false;
            ini["edge"]["calibBlack"] = s;
            printf("# New calibration values:\n# black %s\n", s);
          }
        }
      }
    }
    else
      // no new value; just wait a bit (1000 = 1ms)
      usleep(2000);
  }
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
  if (logfileNorm != nullptr)
  {
    fclose(logfileNorm);
  }
}


void MEdge::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    { // log_line sensor detection
      fprintf(logfile, "%lu.%04ld %d %.3f %.3f %.4f\n", updTime.getSec(), updTime.getMicrosec()/100,
              edgeValid, leftEdge, rightEdge, leftEdge - rightEdge);
    }
    if (toConsole)
    { // debug print to console
      printf("%lu.%04ld %d %.4f %.4f %.4f\n", updTime.getSec(), updTime.getMicrosec()/100,
              edgeValid, leftEdge, rightEdge, leftEdge - rightEdge);
    }
    if (logfileNorm != nullptr)
    {
      fprintf(logfileNorm, "%lu.%04ld %d %d %d %d %d %d %d %d  %.4f\n",
             sedge.updTime.getSec(),
             sedge.updTime.getMicrosec()/100,
              ls[0], ls[1], ls[2], ls[3],
              ls[4], ls[5], ls[6], ls[7], leftEdge - rightEdge
      );
    }
  }
}
