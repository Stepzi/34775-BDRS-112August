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
#include <sys/types.h>
#include "spyvision.h"
#include "steensy.h"
#include "uservice.h"

// create connection object
SPyVision pyvision;


void SPyVision::setup()
{ // ensure default values
  if (not ini.has("pyvision"))
  { // no data yet, so generate some default values
    ini["pyvision"]["host"] = "localhost";
    ini["pyvision"]["port"] = "25001";
    ini["pyvision"]["log"] = "true";
    ini["pyvision"]["print"] = "false";
    ini["pyvision"]["enabled"] = "false";
  }
  if (ini["pyvision"]["enabled"] == "true")
  {
    // connect to python server
    printf("# SPyVision:: Vision link: trying to connect to %s port %s\n",
          ini["pyvision"]["host"].c_str(), ini["pyvision"]["port"].c_str());
    sock = new USocket(ini["pyvision"]["host"].c_str(), ini["pyvision"]["port"].c_str());
    std::string c = "not connected";
    if (sock->connected)
    {
      c = "connected";
      // test message
      sock->sendCommand("aruco\n");
    }
    else
      printf("# SpyVision:: service not available\n");
    //
    // create logfile
    toConsole = ini["pyvision"]["print"] == "true";
    if (ini["pyvision"]["log"] == "true")
    { // open logfile
      std::string fn = service.logPath + "log_pyvision.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% connection to python vision - logfile\n");
      fprintf(logfile, "%% connection to %s port %s (%s)\n", ini["pyvision"]["host"].c_str(), ini["pyvision"]["port"].c_str(), c.c_str());
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tRx or Tx\n");
      fprintf(logfile, "%% 3 \tRx or Tx message count\n");
      fprintf(logfile, "%% 4 \tCommand send or string received\n");
    }
    th1 = new std::thread(runObj, this);
  }
  else
    printf("# SpyVision:: disabled in robot.ini\n");
}

void SPyVision::terminate()
{ // wait for receive thread to finish
  if (th1 != nullptr)
  {
    th1->join();
    // shutdown the connection
    if (sock->connected)
    {
      const char * q = "quit\n";
      sock->sendCommand(q);
      usleep(100);
      UTime t("now");
      toLogTx(q);
    }
    sock->terminate();
  }
  // close logfile
  if (logfile != nullptr)
  {
    fclose(logfile);
    printf("# SPyVision:: logfile closed\n");
  }
}

bool SPyVision::sendCommand(const char* command)
{
  return sock->sendCommand(command);
  toLogTx(command);
}

void SPyVision::run()
{
  printf("# SPyVision is running\n");
  while (not service.stop)
  { // wait for reply
    std::string r = sock->waitForReply(40); // ms
    if (r.length() > 1)
    { // decode the reply
      toLogRx(r.c_str());
      decodeReply(r.c_str());
    }
  }
  th1 = nullptr;
}

void SPyVision::decodeReply(const char* reply)
{
  if (strncmp(reply, "arucopos ", 8) == 0)
  {
    const char * p1 = &reply[8];
    aruco_valid = strtol(p1, (char**)&p1, 10);
    aruco_x = strtof(p1, (char**)&p1);
    aruco_y = strtof(p1, (char**)&p1);
    aruco_h = strtof(p1, (char**)&p1);
    aruco_ID = strtol(p1, (char**)&p1, 10);
  }
  else if (strncmp(reply, "golfpos ", 8) == 0)
  {

  }
}

bool SPyVision::waitForAruco(float timeoutMs)
{
  UTime t;
  t.now();
  bool updated = false;
  while (t.getTimePassed() < timeoutMs/1000.0)
  {
    if (aruco_updateCnt != aruco_updateCntLast)
    {
      updated = true;
      break;
    }
    usleep(1000);
  }
  return updated;
}

void SPyVision::toLogRx(const char * got)
{
  if (service.stop)
    return;
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04lu Rx %d %s\n",
            sock->rxTime.getSec(), sock->rxTime.getMicrosec()/100,
            sock->replyCnt,
            got);
  }
  if (toConsole)
  {
    printf("%lu.%04lu Rx %d %s\n",
           sock->rxTime.getSec(), sock->rxTime.getMicrosec()/100,
           sock->replyCnt,
           got);
  }
}

void SPyVision::toLogTx(const char * cmd)
{
  if (service.stop)
    return;
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04lu Tx %d %s\n", t.getSec(), t.getMicrosec()/100, sock->txCnt, cmd);
  }
  if (toConsole)
  {
    printf("%lu.%04lu Tx %d %s\n", t.getSec(), t.getMicrosec()/100, sock->txCnt, cmd);
  }
}
