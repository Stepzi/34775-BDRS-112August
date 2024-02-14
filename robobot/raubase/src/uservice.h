/* #***************************************************************************
 #*   Copyright (C) 2023 by DTU
 #*   jcan@dtu.dk
 #*
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

#pragma once

#include <string>
#include <thread>
#include "utime.h"
#include "uini.h"

class UService
{
public:
    /**
     * Initialize all message data items
     * \returns true if app is to end now (error, help or calibration)
    */
    bool setup(int argc,char **argv);
    /**
     * decode messages from Teensy
     * \param msg already CRC checked text line from teensy */
    bool decode(const char * msg, UTime & msgTime);
    /**
     * decode command-line parameters */
    bool readCommandLineParameters(int argc, char ** argv);
    /**
     * Stop application - like an interrupt
     * \param who is a string to identify from where the stop order came
     * */
    void stopNow(const char * who);
    /**
     * shut down and save ini-file - but do not exit */
    void terminate();
    /**
    * thread to listen to keyboard */
    void run();
    void run2(); // and terminate request
    /**
     * Got keyboard input - e.g. enter */
    bool gotKey();
    /**
     * Return the SVN version string (version part) */
    std::string getVersionString();

public:
    // file with calibration values etc.
    std::string iniFileName = "robot.ini";
    mINI::INIFile * iniFile;
    std::string logPath = ""; // this directory
    // stop all processing
    bool stop = false;
    bool theEnd;
    bool stopNowRequest = false;
//     bool start = false;
    // keyboard input
    bool gotKeyInput;
    std::string keyString;
    bool asDaemon = false;

private:
    static void runObj(UService * obj)
    { // called, when thread is started
        // transfer to the class run() function.
        obj->run();
    }
    std::thread * th1;
    static void runObj2(UService * obj)
    { // called, when thread is started
        // transfer to the class run() function.
        obj->run2();
    }
    std::thread * th2;
    //
    bool terminating = false;
    bool setupComplete = false;
};

extern UService service;
extern mINI::INIStructure ini;
