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
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
// create value
SEncoder encoder;


void SEncoder::setup()
{ // ensure default values
  if (not ini.has("encoder"))
  { // no data yet, so generate some default values
    ini["encoder"]["rate_ms"] = "8";
    ini["encoder"]["log"] = "true";
    ini["encoder"]["print"] = "false";
    ini["encoder"]["encoder_reversed"] = "true";
  }
  // reset encoder and pose
  teensy1.send("enc0\n");
  // use values and subscribe to source data
  std::string s = "sub enc " + ini["encoder"]["rate_ms"] + "\n";
  teensy1.send(s.c_str());
  toConsole = ini["encoder"]["print"] == "true";
  // ensure default is true if no 'encoder_reversed' entry is available
  // Robobot motors has reversed encoders (encoder A and B is swapped)
  // this will be fixed in the Regbot firmware by this command
  encoder_reversed = true;
  if (ini["encoder"].has("encoder_reversed"))
    encoder_reversed = ini["encoder"]["encoder_reversed"] == "true";
  if (encoder_reversed)
    s = "encrev 1\n";
  else
    s = "encrev 0\n";
  teensy1.send(s.c_str());

  if (ini["encoder"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_encoder.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Encoder logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2,3 \tenc left, right\n");
    fprintf(logfile, "%% 4,5 \tencoder change left, right\n");
  }
}

void SEncoder::terminate()
{
  if (logfile != nullptr)
  {
    fclose(logfile);
  }
}

bool SEncoder::decode(const char* msg, UTime & msgTime)
{
  bool used = true;
  const char * p1 = msg;
  if (strncmp(p1, "enc ", 4) == 0)
  {
    if (strlen(p1) > 4)
      p1 += 4;
    else
      return false;
    encTime = msgTime;
    enc[0] = -strtoll(p1, (char**)&p1, 10);
    enc[1] = strtoll(p1, (char**)&p1, 10);
    // notify users of a new update
    updateCnt++;
    // save to log_encoder_pose
    toLog();
    // save new value as old value
    encLast[0] = enc[0];
    encLast[1] = enc[1];
  }
  else
    used = false;
  return used;
}

void SEncoder::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    {
      fprintf(logfile,"%lu.%04ld %lu %lu %d %d\n", encTime.getSec(), encTime.getMicrosec()/100,
              (unsigned long int)enc[0], (unsigned long int)enc[1], int(enc[0] - encLast[0]), int(enc[1] - encLast[1]));
    }
    if (toConsole)
    {
      printf("%lu.%04ld %lu %lu %d %d\n", encTime.getSec(), encTime.getMicrosec()/100,
              (unsigned long int)enc[0], (unsigned long int)enc[1], int(enc[0] - encLast[0]), int(enc[1] - encLast[1]));
    }
  }
}

