/*  
 * 
 * Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk
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


#ifndef SSTATE_H
#define SSTATE_H

using namespace std;

/**
 * Class for general state of the robot
 * e.g. battery voltage */
class SState
{
public:
  /** setup and request data */
  void setup();
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(const char * msg, UTime & msgTime);
  /**
   * terminate */
  void terminate();

public:
  /// Battery voltage with a few decimals (~2 valid decimals)
  float batteryVoltage;
  /// Teensy time since start of Teensy
  double teensyTime;
  /// robot hardware index number (serial)
  int idx = 0;
  /// robot hardware version
  int version = 0;
  /// control state
  int controlState = 0;
  /// Teensy load
  float load = 0;
  /// motor enabled state
  bool motorEnabled[2];
  /// robot hardware type
  int type = 0;
  /// system time at this Teensy time
  UTime hbtTime;
  /// mutex should be used to get consistent values
  std::mutex dataLock;
private:
  void toLog();
  // logfile
  bool toConsole = false;
  FILE * logfile = nullptr;
};

/**
 * Make this visible to the rest of the software */
extern SState state;

#endif
