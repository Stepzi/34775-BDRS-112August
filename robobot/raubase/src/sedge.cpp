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
#include "sedge.h"
#include "steensy.h"
#include "uservice.h"
// create value
SEdge sedge;


void SEdge::setup()
{ // ensure default values
  if (not ini.has("edge") or not ini["edge"].has("printRaw"))
  { // no data yet, so generate some default values
    ini["edge"]["rate_ms"] = "8";
    ini["edge"]["highPower"] = "true";
    ini["edge"]["logRaw"] = "true";
    ini["edge"]["printRaw"] = "false";
  }
  // use values and subscribe to source data
  // like teensy1.send("sub pose 4\n");
  bool high = ini["edge"]["highPower"] == "true";
  setSensor(true, high);
  //
  std::string s = "sub liv " + ini["edge"]["rate_ms"] + "\n";
  teensy1.send(s.c_str());
  //
  toConsole = ini["edge"]["printRaw"] == "true";
  // logfile
  if (ini["edge"]["logRaw"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_edge_raw.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Linesensor raw values logfile (reflectance values)\n");
    fprintf(logfile, "%% Sensor power high=%d\n", high);
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2..9 \tSensor 1..8 AD value difference (illuminated - not illuminated)\n");
  }
}

void SEdge::terminate()
{
  setSensor(false, false);
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}

bool SEdge::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "liv ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    updTime = msgTime;
//     printf("# edgeraw: %s", msg);
    for (int i = 0; i < 8; i++)
    { // get integer value (averaged over sample time)
      edgeRaw[i] = strtol(p1, (char**)&p1, 10);
    }
    // notify users of a new update
    updateCnt++;
    // save received data (if desired)
    toLog();
  }
  else if (strncmp(p1, "ls ", 3) == 0)
  { // debug for very raw values (illuminated and not illuminated values)
    // not used here
    printf("# edge AD: %s", msg);
  }
  else
    used = false;
  return used;
}

void SEdge::setSensor(bool on, bool high)
{
  const int MSL = 150;
  char s[MSL];
  // message format
  //lip p w h t xth wi s 	Set sensor basics p=on, w=white, h=high power, t=tilt comp, xth=cross_th, wi=wide, s=swap
  snprintf(s, MSL, "lip %d 0 %d 0 0 0 0\n", on, high);
  if (on)
    teensy1.send(s);
  else
    teensy1.send(s, true);
}


void SEdge::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    {
      fprintf(logfile,"%lu.%04ld %d %d %d %d %d %d %d %d\n", updTime.getSec(), updTime.getMicrosec()/100,
              edgeRaw[0],
              edgeRaw[1],
              edgeRaw[2],
              edgeRaw[3],
              edgeRaw[4],
              edgeRaw[5],
              edgeRaw[6],
              edgeRaw[7]
      );
    }
    if (toConsole)
    {
      printf("%lu.%04ld %d %d %d %d %d %d %d %d\n", updTime.getSec(), updTime.getMicrosec()/100,
              edgeRaw[0],
              edgeRaw[1],
              edgeRaw[2],
              edgeRaw[3],
              edgeRaw[4],
              edgeRaw[5],
              edgeRaw[6],
              edgeRaw[7]
      );
    }
  }
}
