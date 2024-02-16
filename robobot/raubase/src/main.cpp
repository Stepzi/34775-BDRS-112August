/*
 #***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jcan@dtu.dk
 #*
 #* The MIT License (MIT)  https://mit-license.org/
 #*
 #* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 #* and associated documentation files (the “Software”), to deal in the Software without restriction,
 #* including without limitation the rights to use, copy, modify, merge, publish, distribute,
 #* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 #* is furnished to do so, subject to the following conditions:
 #*
 #* The above copyright notice and this permission notice shall be included in all copies
 #* or substantial portions of the Software.
 #*
 #* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 #* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 #* PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 #* FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 #* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 #* THE SOFTWARE. */

// System libraries
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <string>
//
// include local files for data values and functions
#include "uservice.h"
#include "cmixer.h"
#include "sgpiod.h"
#include "bplan20.h"
#include "bplan21.h"
#include "bplan40.h"
#include "bplan100.h"
#include "bplan101.h"
#include "bmmission0.h"

int main (int argc, char **argv)
{ // prepare all modules and start data flow
  // but also handle command-line options
  service.setup(argc, argv);
  //
  if (not service.theEnd)
  { // all set to go
    // turn on LED on port 16
    gpio.setPin(16, 1);
    // run the planned missions
    mission0.run();
    //plan20.run();
    //plan21.run();
    //plan40.run();
    //planMission0.run();
    //plan100.run();
    plan101.run();
    //
    mixer.setVelocity(0.0);
    mixer.setTurnrate(0.0);
    sleep(1); // to allow robot to stop
    // turn off led 16
    gpio.setPin(16, 0);
  }
  // close all logfiles etc.
  service.terminate();
  return service.theEnd;
}

