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
#include "steensy.h"
#include "uservice.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"

// create value
CEdge cedge;


// Bridge class:
void CEdge::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("edge") or not ini["edge"].has("printCtrl"))
  { // motor block is OK, but control parameters are needed too
    ini["edge"]["kp"] = "40.0"; // unit is (turnrate (m/sec) per sensor error (m))
    ini["edge"]["lead"] = "0.3 0.5"; // tau_d (sec) and alpha; tau_d = 0.0 means no function
    ini["edge"]["taui"] = "0.0"; // tau_i (sec) 0.0 is no integrator function
    ini["edge"]["logCedge"] = "true"; // log from this module - but not all control parameters
    ini["edge"]["logCtrl"] = "false"; // log all control parameters
    ini["edge"]["print"] = "false";
    ini["edge"]["printCtrl"] = "false";
    ini["edge"]["maxTurnrate"] = "7.0"; // rad/sec
  }
  //
  // get values from ini-file
  float kp = strtof(ini["edge"]["kp"].c_str(), nullptr);
  const char * p1 = ini["edge"]["lead"].c_str();
  // lead
  float taud = strtof(p1, (char**)&p1);
  float alpha = strtof(p1, (char**)&p1);
  // integrator
  float taui = strtof(ini["edge"]["taui"].c_str(), nullptr);
  //
  float sampleTime = strtof(ini["edge"]["rate_ms"].c_str(), nullptr)/1000.0;
  pid.setup(sampleTime, kp, taud, alpha, taui);
  // limit turnrate
  maxTurnrate = strtof(ini["edge"]["maxTurnrate"].c_str(), nullptr);
  //
  // should debug print be enabled
  pid.toConsole = ini["edge"]["printCtrl"] == "true";
  toConsole = ini["edge"]["print"] == "true";
  //
  // initialize logfile
  if (ini["edge"]["logCtrl"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_edge_pid.txt";
    logfileCtrl = fopen(fn.c_str(), "w");
    if (logfileCtrl != nullptr)
    {
      fprintf(logfileCtrl, "%% Edge control logfile: %s\n", fn.c_str());
      pid.logPIDparams(logfileCtrl, true);
    }
    else
      printf("# cedge - Failed to create logfile at %s\n", fn.c_str());

  }
  if (ini["edge"]["logCedge"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_edge_ctrl.txt";
    logfile = fopen(fn.c_str(), "w");
    if (logfile != nullptr)
    {
      fprintf(logfile, "%% Edge logfile: %s\n", fn.c_str());
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \theading mode (edge control == 2)\n");
      fprintf(logfile, "%% 3 \tEdge 1=left, 0=right\n");
      fprintf(logfile, "%% 4 \tEdge offset (signed in m; should be less than about 0.01)\n");
      fprintf(logfile, "%% 5 \tMeasured edge value (m; positive is left)\n");
      fprintf(logfile, "%% 6 \tcontrol value (rad/sec; positive is CCV)\n");
      fprintf(logfile, "%% 7 \tlimited\n");
    }
    else
      printf("# cedge - Failed to create logfile at %s\n", fn.c_str());
  }
  th1 = new std::thread(runObj, this);
}

void CEdge::toLog()
{
  if (service.stop)
    return;
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %d %.4f %.4f %.4f %d\n",
            medge.updTime.getSec(), medge.updTime.getMicrosec()/100,
            mixer.headingMode, followLeft, followOffset, measuredValue,
            u, limited);
  }
  if (toConsole)
  { // debug print to console
    printf("%lu.%04ld %d %d %.4f %.4f %.4f %d\n",
           medge.updTime.getSec(), medge.updTime.getMicrosec()/100,
           mixer.headingMode, followLeft, followOffset, measuredValue,
           u, limited);
  }
}

void CEdge::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfileCtrl != nullptr)
    fclose(logfileCtrl);
  if (logfile != nullptr)
    fclose(logfile);
}


void CEdge::run()
{
  int loop = 0;
  bool wasEnabled = false;
  int updateCnt = medge.updateCnt;
  while (not service.stop)
  {
    if (medge.updateCnt != updateCnt)
    {
      if (mixer.headingMode == CMixer::HM_EDGE)
      { // follow edge
        if (followLeft)
          measuredValue = medge.leftEdge;
        else
          measuredValue = medge.rightEdge;
        if (medge.edgeValid)
        { // when measured are too positive, i.e. too far left
          // we should go clockwise (CV), i.e positive turn-rate.
          u = - pid.pid(followOffset, measuredValue, limited);
          if (u > maxTurnrate)
          {
            limited = true;
            u = maxTurnrate;
          }
          else if (u < -maxTurnrate)
          {
            limited = true;
            u = -maxTurnrate;
          }
          else
            limited = motor.limited;
        }
        else
        {
          u = 0.0;
          limited = motor.limited;
        }
        // finished calculating turn rate
        mixer.setInModeTurnrate(u);
        // log control values
        pid.saveToLog(logfileCtrl, medge.updTime);
        toLog();
        wasEnabled = true;
      }
      else if (wasEnabled)
      {
        wasEnabled = false;
        u = 0;
        mixer.setInModeTurnrate(u);
        pid.resetHistory();
        // log control values
        pid.saveToLog(logfileCtrl, medge.updTime);
        toLog();
      }
      loop++;
      updateCnt = medge.updateCnt;
    }
    usleep(2000);
  }
}


