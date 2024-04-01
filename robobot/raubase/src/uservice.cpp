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


#include <stdio.h>
#include <signal.h>
#include "CLI/CLI.hpp"
#include <filesystem>

#include "uini.h"
#include "cmotor.h"
#include "cheading.h"
#include "cmixer.h"
#include "cservo.h"
#include "cedge.h"
#include "medge.h"
#include "mpose.h"
#include "maruco.h"
#include "mgolfball.h"
#include "scam.h"
#include "sdist.h"
#include "sedge.h"
#include "sencoder.h"
#include "sgpiod.h"
#include "simu.h"
#include "sjoylogitech.h"
#include "spyvision.h"
#include "sstate.h"
#include "steensy.h"
#include "uservice.h"

#define REV "$Id: uservice.cpp 586 2024-01-24 12:42:37Z jcan $"
// define the service class
UService service;
// make a configuration structure
mINI::INIStructure ini;

void signal_callback_handler(int signum)
{ // called when pressing ctrl-C
  cout << "Caught signal " << signum << endl;
  service.terminate();
  exit(signum);
}

bool UService::setup(int argc,char **argv)
{ // Interrupt signal handler for most common signals
  signal(SIGINT, signal_callback_handler); // 2 normal ctrl-C
//   signal(SIGKILL, signal_callback_handler); // 9
  signal(SIGQUIT, signal_callback_handler); // 3
  signal(SIGHUP, signal_callback_handler); // 1
  signal(SIGPWR, signal_callback_handler); // 30
  signal(SIGTERM, signal_callback_handler); // 15 (pkill default)
  //
  bool teensyConnect = true;
  CLI::App cli{"ROBOBOT app"};
  // cli.add_option("-d,--device", service.usbDev, "USB device name for Teensy (default is /dev/ttyACM0)");
  // add reply to version request
  bool version{false};
  cli.add_flag("-v,--version", version, "Compiled SVN version (for uservice.cpp)");
  cli.add_flag("-d,--daemon", asDaemon, "Do not listen to the keyboard (daemon mode)");
  bool calibWhite{false};
  bool calibBlack{false};
  bool calibWood{false};
  cli.add_flag("-w,--white", calibWhite, "Calibrate line sensor on white surface");
  cli.add_flag("-b,--black", calibBlack, "Calibrate line sensor on black surface");
  cli.add_flag("-f,--wood", calibWood, "Calibrate line sensor on wood surface");
  // distance sensor
  int sensor = 0;
  int calibrateDistance = 0;
  cli.add_option("-s,--sensor", sensor, "Calibrate (sharp) distance sensor [1 or 2], use with '-c'");
  cli.add_option("-c,--calibrate-distance", calibrateDistance,
                 "Calibrate (sharp) distance sensor at [13 or 50] cm, use with '-s'");
  bool camCal = false;
  cli.add_flag("-m,--cam-calibrate", camCal, "Calibrate camera using checkboard images");
  bool camImg{false};
  cli.add_flag("-i,--image", camImg, "Save image from camera");
  // gyro offset
  bool calibGyro = false;
  cli.add_flag("-g,--gyro", calibGyro, "Calibrate gyro offset");
  float testSec = 0.0;
  cli.add_option("-t,--time", testSec, "Open all sensors for some time (seconds)");
  // rename feature
  int  regbotNumber{-1};
  cli.add_option("-n,--number", regbotNumber, "Set robot number to Regbot part [0..150]");
  // rename feature
  int  regbotHardware{-1};
  cli.add_option("-H,--hardware", regbotHardware, "Set robot hardware type (most likely 9)");
  // print 4x4_100 ArUco code
  int arucoID = -1;
  cli.add_option("-a,--aruco", arucoID, "Save an image with an ArUco number [0..249]");
  // Parse for command line options
  cli.allow_windows_style_options();
  theEnd = true;
  CLI11_PARSE(cli, argc, argv);
  // if we get here, then command line parameters are OK to continue
  theEnd = false;
  //
  if (version)
  {
    printf("RAUBASE SVN service version%s\n", getVersionString().c_str());
    theEnd = true;
  }
  // line sensor
  if (calibWhite)
    medge.sensorCalibrateWhite = true;
  if (calibBlack)
    medge.sensorCalibrateBlack = true;
  if (calibWood)
    medge.sensorCalibrateWood = true;
  // distance sensor
  if ((sensor == 1 or sensor == 2) and (calibrateDistance == 13 or calibrateDistance == 50))
    dist.calibrate(sensor, calibrateDistance);
  // gyro
  if (calibGyro)
    imu.calibrateGyro();
  // rename
  if (regbotNumber >= 0 and regbotNumber <= 150)
  { // save this number to the Teensy (Robobot) and exit
    teensy1.saveRegbotNumber = regbotNumber;
  }
  if (regbotHardware >= 5 and regbotHardware <= 15)
  { // save this number to the Teensy (Robobot) and exit
    teensy1.regbotHardware = regbotHardware;
  }
  //
  // create an ini-file structure
  iniFile = new mINI::INIFile(iniFileName);
  // and read the file (if any, else just create the 'ini' structure)
  iniFile->read(ini);
  // now ini structure is populated
  //
  if (not ini.has("service"))
  { // no data yet, so generate some default values
    ini["service"]["use_robot_hardware"] = "true";
    ini["service"]["logpath"] = "log_%d/";
    ini["service"]["; The '%d' will be replaced with date and timestamp (Must end with a '/')."] = "";
  }
  teensyConnect = not (camImg or camCal or ini["service"]["use_robot_hardware"] == "false");
  //
  if (arucoID >= 0)
  { // just save an image with an ArUco code
    aruco.saveCodeImage(arucoID);
    theEnd = true;
  }
  // for setup timing
  UTime t("now");
  if (not theEnd)
  { // initialize all elements
    logPath = ini["service"]["logpath"];
    int n = logPath.find("%d");
    if (n > 0)
    { // date should be added to path
      UTime t("now");
      std::string dpart = t.getForFilename();
      logPath.replace(n, 2, dpart);
    }
    std::error_code e;
    bool ok = filesystem::create_directory(logPath, e);
    if (ok)
      printf("# UService:: created directory %s\n", logPath.c_str());
    else if (n > 0)
    { // failed (probably: path exist already)
      std::perror("#*** UService:: Failed to create log path:");
    }
    if (teensyConnect)
    { // open the main data source
      printf("# UService::setup: open to Teensy\n");
      teensy1.setup();
      state.setup();
      //
      // wait for base setup to finish
      if (teensy1.teensyConnectionOpen)
      { // wait for initial setup
        usleep(10000);
        while (teensy1.getTeensyCommQueueSize() > 0 and t.getTimePassed() < 5.0)
          usleep(10000);
        if (t.getTimePassed() >= 5.0)
          printf("# UService::setup - waited %g sec for initial Teensy setup\n", t.getTimePassed());
      }
      // setup and initialize all modules
      encoder.setup();
      pose.setup();
      sedge.setup();
      servo.setup();
      imu.setup();
      motor.setup();
      gpio.setup();
    }
    else
      printf("# UService::setup: Ignoring robot hardware (Regbot and GPIO)\n");
    //
    // setup of all that do not directly interact with the robot
    medge.setup();
    cedge.setup();
    mixer.setup();
    heading.setup();
    pyvision.setup();
    dist.setup();
    joyLogi.setup();
    cam.setup();
    aruco.setup();
    golfball.setup();

    setupComplete = true;
    usleep(2000);
    //
  }
  if (not theEnd and setupComplete)
  { // run optional parts, that can be handled in one go
    theEnd = true;
    if (camImg)
      cam.saveImage();
    else if (camCal)
      cam.calibrate();
    else
      theEnd = false;
  }
  // Regbot (Teensy) need to accept settings before continue
  if (not theEnd and teensyConnect)
  { // wait for all settings to be accepted
    if (teensy1.teensyConnectionOpen)
    {
      while (teensy1.getTeensyCommQueueSize() > 0 and t.getTimePassed() < 5.0)
        usleep(10000);
      printf("# UService::setup - waited %g sec for full setup\n", t.getTimePassed());
      // decide if all setup is OK
      int retry = 0;
      int dumped = teensy1.getTeensyCommError(retry);
      if (dumped > 0 or retry > 0)
      {
        if (dumped > 0)
          printf("# UService:: ************************************************************\n");
        printf("# UService:: Teensy setup communication msg resend %d, dumped %d messages\n", retry, dumped);
        if (dumped > 0)
          printf("# UService:: ************************************************************\n");
      }
      else
        printf("# UService:: setup of all modules finished OK.\n");
      theEnd = dumped > 0 or teensy1.getTeensyCommQueueSize() > 0;
    }
    else
    {
      printf("# UService:: setup failed, no connection to Teensy - terminating.\n");
      theEnd = true;
    }
  }
  if (not theEnd)
  { // start listen to the keyboard
    th1 = new std::thread(runObj, this);
    th2 = new std::thread(runObj2, this);
  }
  // wait for optional tasks that require system to run.
  if ((calibBlack or
       calibWhite or
       calibWood or
       regbotNumber >= 0 or
       regbotHardware > 3 or
       dist.inCalibration or
       imu.inCalibration or
       testSec > 0.05) and
       not theEnd)
  { // wait until finished, then terminate
    UTime t("now");
    while (medge.sensorCalibrateBlack or
      medge.sensorCalibrateWhite or
      medge.sensorCalibrateWood or
      (teensy1.saveRegbotNumber >= 0 and teensy1.saveRegbotNumber != state.idx) or
      dist.inCalibration or
      imu.inCalibration or t.getTimePassed() < testSec)
    {
      printf("# Service is waiting for a specified action to finish\n");
      sleep(1);
    }
    theEnd = true;
  }
  return theEnd;
}

bool UService::decode(const char* msg, UTime& msgTime)
{ // decode messages from Teensy
  bool used = true;
  if      (state.decode(msg, msgTime)) {}
  else if (encoder.decode(msg, msgTime)) {}
  else if (imu.decode(msg, msgTime)) {}
  else if (servo.decode(msg, msgTime)) {}
  else if (sedge.decode(msg, msgTime)) {}
  else if (dist.decode(msg, msgTime)) {}
  //
  // add other Teensy data users here
  //
  else
    used = false;
  return used;
}

void UService::stopNow(const char * who)
{ // request a terminate and exit
  printf("# UService:: %s say stop now\n", who);
  stopNowRequest = true;
}


void UService::terminate()
{ // Terminate modules (especially threads and log files)
  if (terminating or not setupComplete)
    return;
  printf("# --------- terminating -----------\n");
  teensy1.send("stop\n");
  terminating = true;
  stop = true; // stop all threads, when finished current activity
  //
  usleep(100000);
  joyLogi.terminate();
  encoder.terminate();
  pose.terminate();
  imu.terminate();
  gpio.terminate();
  cedge.terminate();
  medge.terminate();
  sedge.terminate();
  mixer.terminate();
  motor.terminate();
  heading.terminate();
  state.terminate();
  servo.terminate();
  dist.terminate();
  // terminate sensors before Teensy
  teensy1.terminate();
  pyvision.terminate();
  cam.terminate();
  aruco.terminate();
  // service must be the last to close
  if (not ini.has("ini"))
  {
    ini["ini"]["; set 'saveConfig' to 'false' to avoid autosave"] = "";
    ini["ini"]["saveConfig"] = "true";
  }
  std::string& shouldSave = ini["ini"]["saveConfig"];
  if (shouldSave != "false")
  { // write any changes ini-file values (and structures)
    ini["ini"]["version"] = getVersionString();
    iniFile->write(ini, true);
    printf("# UService:: configuration saved to %s\n", iniFileName.c_str());
  }
}

std::string UService::getVersionString()
{
  // #define REV "$Id: uservice.cpp 586 2024-01-24 12:42:37Z jcan $"
  std::string ver = REV;
  int n1 = ver.find(' ', 10);
  int n2 = ver.rfind("Z ");
  std::string part = ver.substr(n1, n2-n1);
  return part;
}

void UService::run()
{
  gotKeyInput = false;
  while (not stop)
  {
    if (not asDaemon)
    {
      cin >> keyString;
      if (keyString == "stop")
        signal_callback_handler(-1);
      else
        gotKeyInput = true;
    }
  }
}

void UService::run2()
{
  while (not stop)
  { // e.g. using the stop switch
    if (stopNowRequest)
    {
      signal_callback_handler(-1);
    }
    usleep(50000);
  }
}

bool UService::gotKey()
{
  if (gotKeyInput)
  { // reset flag
    gotKeyInput = false;
    return true;
  }
  else
    return false;
}
