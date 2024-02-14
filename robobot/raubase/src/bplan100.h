/*  
 * 
 * Copyright © 2023 DTU, Christian Andersen jcan@dtu.dk
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

// aaa
#pragma once


using namespace std;

/**
 * Class intended to accomplish a short mission,
 * e.g. one challenge or part of a challenge
 * */
class BPlan100
{
public:
  /**
   * destructor */
    ~BPlan100();
  /** setup and request data */
  void setup();
  /**
   * run this mission */
  void run();
  /**
   * terminate */
  void terminate();

private:
  /**
   * Write a timestamped message to log */
  void toLog(const char * message);
  /// added to log
  int state, oldstate;
  /// private stuff
  // debug print to console
  bool toConsole = true;
  // logfile
  FILE * logfile = nullptr;
  bool setupDone = false;
};

/**
 * Make this visible to the rest of the software */
extern BPlan100 plan100;

