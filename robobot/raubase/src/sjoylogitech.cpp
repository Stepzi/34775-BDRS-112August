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

#include <sys/ioctl.h>
#include <signal.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include "sjoylogitech.h"
#include "uservice.h"
#include "cmixer.h"
#include "cservo.h"

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

// start is green button
#define BUTTON_GREEN   0
// red button is 1
#define BUTTON_RED   1
// blue button is 2
#define BUTTON_BLUE   2
// image is yellow button (3)
#define BUTTON_IMAGE   3
#define BUTTON_YELLOW   3
// left front button is 4
// right front button is fast
#define BUTTON_FAST    5
// goto manuel mode is all 3 upper-center buttons
#define BUTTON_BACK 6
#define BUTTON_START 7
#define BUTTON_MANUAL 8
// axis 0 is left hand left-right axis
// axis 1 is left hand up-down axis
#define AXIS_SERVO_2   1
// axis 2 is left front speeder
// axis 3 is right hand left-right
#define AXIS_TURN      3
// axis 4 is right hand up-down
#define AXIS_VEL       4
// axis 5 is right front speeder
// axis 6 id digital left-write
// axis 7 is digital up-down
// parameters for JOY control (velocity is in m/s)
// #define VELOCITY_MAX          1.5
// parameter to turn control (radians/sec)
// #define VELOCITY_TURN_MAX     0.8
// #define VELOCITY_SERVO_MAX    5.0
// factor for normal speed
// #define VELOCITY_SLOW_FACTOR  0.2

// create value
SJoyLogitech joyLogi;


void SJoyLogitech::setup()
{ // ensure default values
  if (not ini.has("Joy_Logitech"))
  { // no data yet, so generate some default values
    ini["Joy_Logitech"]["log"] = "true";
    ini["Joy_Logitech"]["print"] = "false";
    ini["Joy_Logitech"]["device"] = "/dev/input/js0";
    /// limit in m/s (velocity) and radian/sec (turnrate) and us/sec (servo rate) for max axis
    ini["Joy_Logitech"]["limit"] = "1.5 1.5 0.1";
    ini["Joy_Logitech"]["Button_fast"] = "5";
    ini["Joy_Logitech"]["axis_Vel"] = "4";
    ini["Joy_Logitech"]["axis_Turn"] = "3";
    ini["Joy_Logitech"]["slow_factor"] = "0.3";
    ini["Joy_Logitech"]["axis_Servo"] = "1";
    ini["Joy_Logitech"]["servo"] = "1"; // servo to control
    ini["Joy_Logitech"]["log_all"] = "false"; // more entries in log
  }
  // Linux device
  joyDevice = ini["Joy_Logitech"]["device"];
  buttonFast = strtol(ini["Joy_Logitech"]["Button_fast"].c_str(), nullptr, 10);
  axisVel = strtol(ini["Joy_Logitech"]["axis_Vel"].c_str(), nullptr, 10);
  if (axisVel < 0 or axisServo >= 16) axisVel = 4;
  axisTurn = strtol(ini["Joy_Logitech"]["axis_Turn"].c_str(), nullptr, 10);
  if (axisTurn < 0 or axisServo >= 16) axisTurn = 3;
  axisServo = strtol(ini["Joy_Logitech"]["axis_Servo"].c_str(), nullptr, 10);
  if (axisServo < 0 or axisServo >= 16) axisServo = 1;
  servoToControl = strtol(ini["Joy_Logitech"]["servo"].c_str(), nullptr, 10);
  if (servoToControl < 1 or servoToControl > 5)
    servoToControl = 1;
  slowFactor = strtod(ini["Joy_Logitech"]["slow_factor"].c_str(), nullptr);
  // max velocity in m/sec and rad/sec
  const char * p1 = ini["Joy_Logitech"]["limit"].c_str();
  maxVel = strtof(p1, (char**)&p1);
  maxTurn = strtof(p1, (char**)&p1);
  // convertion factors
  velScale = maxVel/32000;
  turnScale = maxTurn/32000;
  //
  joyRunning = initJoy();
  if (joyRunning)
  { // logfile
    // if joystick available, then start in manual
//     mixer.setManualOverride(true);
    // start read thread
    toConsole = ini["Joy_Logitech"]["print"] == "true";
    if (ini["Joy_Logitech"]["log"] == "true")
    { // open logfile
      std::string fn = service.logPath + "log_joy_logitech.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Logitech gamepad interface logfile\n");
      fprintf(logfile, "%% Device %s\n", joyDevice.c_str());
      fprintf(logfile, "%% Device type %s\n", deviceName.c_str());
      fprintf(logfile, "%% Button count %d\n", number_of_buttons);
      fprintf(logfile, "%% Axis count %d\n", number_of_axes);
      fprintf(logfile, "%% Button fast %d\n", buttonFast);
      fprintf(logfile, "%% Axis vel %d\n", axisVel);
      fprintf(logfile, "%% Axis turn %d\n", axisTurn);
      fprintf(logfile, "%% Axis servo %d\n", axisServo);
      fprintf(logfile, "%% Slow factor %g\n", slowFactor);
      fprintf(logfile, "%% Max velocity (m/s) %g\n", maxVel);
      fprintf(logfile, "%% Max turnrate (rad/s) %g\n", maxTurn);
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tManuel override\n");
      fprintf(logfile, "%% 3 \tLinear velocity\n");
      fprintf(logfile, "%% 4 \tTurnrate\n");
      fprintf(logfile, "%% 5 \tServo position\n");
      fprintf(logfile, "%% 6-%d \tButtons pressed\n", number_of_buttons + 5);
      fprintf(logfile, "%% %d-%d \tAxis value\n", number_of_buttons + 6, number_of_axes + number_of_buttons + 5);
    }
    // start listen thread
    th1 = new std::thread(runObj, this);
    printf("# UJoyLogitech:: joystick found (%s on %s)\n", deviceName.c_str(), joyDevice.c_str());
  }
//   else
//     printf("# UJoyLogitech:: no joystick found (%s)\n", joyDevice.c_str());
}

void SJoyLogitech::terminate()
{
  if (th1 != nullptr)
    th1->join();
  if (logfile != nullptr)
  {
    fclose(logfile);
//     printf("# SJoyLogitech:: logfile closed\n");
    logfile = nullptr;
  }
}

void SJoyLogitech::run()
{
  UTime t;
  t.now();
  sleep(3);
  bool automaticMode = true;
  bool automaticModeOld = false;
  while (not service.stop and joyRunning)
  { // handling gamepad events
    // Device is present
    bool gotEvent = getNewJsData();
    if (gotEvent)
    { //Detect manual override toggling
      if (joyValues.button[BUTTON_START] == 1)
        automaticMode = true;
      if (joyValues.button[BUTTON_BACK] == 1)
        automaticMode = false;
      //
      updTime.now();
      if (not automaticMode)
      { // we are in manual mode, so
        // generate robot control from gamepad
        joyControl();
      }
      //
      if (t.getTimePassed() > 0.01 or ini["Joy_Logitech"]["log_all"] == "true")
      { // don't save too fast
        t.now();
        toLog();
      }
    }
    else
      usleep(10000);
    // state change
    if (automaticMode != automaticModeOld)
    { // there is a change
      automaticModeOld = automaticMode;
      // Tell mixer about the change
      mixer.setManualControl(not automaticMode, 0, 0);
      printf("# SJoyLogitech:: state change (auto=%d)\n", automaticMode);
    }
  }
  if (joyRunning)
  { // close device nicely
    joyRunning = false;
    if (jDev >= 0)
      close(jDev);
    jDev = -1;
  }
//   printf("# SJoyLogitech:: device closed\n");
}

bool SJoyLogitech::initJoy()
{
  jDev = open (joyDevice.c_str(), O_RDWR | O_NONBLOCK);
  if (jDev >= 0)
  { // Joystic device found
//     int flags;
//     if (-1 == (flags = fcntl(jDev, F_GETFL, 0)))
//       flags = 0;
//     fcntl(jDev, F_SETFL, flags | O_NONBLOCK);
//     printf("Opened joystick on port: %s\n",joyDevice);
    //Query and print the joystick name
    char name[128] = "no device found";
    if (ioctl(jDev, JSIOCGNAME(sizeof(name)), name) < 0)
      strncpy(name, "Unknown", sizeof(name));
    const int MSL = 150;
    char s[MSL];
    snprintf(s, MSL, "# Joystick: %s\r\n", name);
    printf("# Joystick model: %s\r\n", name);
    //
    deviceName = name;
    ini["Joy_Logitech"]["device_type"] = deviceName;
    //Query and print number of axes and buttons
    ioctl (jDev, JSIOCGAXES, &number_of_axes);
    ioctl (jDev, JSIOCGBUTTONS, &number_of_buttons);
//     printf("Registrered %d axes and %d buttons on joystick\n",number_of_axes,number_of_buttons);
  }
  return jDev >= 0;
}

bool SJoyLogitech::getNewJsData()
{
  struct js_event jse;
  bool lostConnection = false;
  bool isOK = false;
  // read full struct or nothing
  int bytes = read(jDev, &jse, sizeof(jse));
  // detect errors
  if (bytes == -1)
  { // error - an error occurred while reading
    switch (errno)
    { // may be an error, or just nothing send (buffer full)
      case EAGAIN:
        //not all send - just continue
        usleep(100);
        break;
      default:
        perror("UJoy::getNewJsData (other error device error): ");
        lostConnection = true;
        break;
    }
  }
  if (lostConnection)
  {
    joyRunning = false;
    if (jDev >= 0)
    {
      close(jDev);
      jDev = -1;
//       sendJsMessage();
    }
    printf("UJoy::run: getNewJsData close\n");
  }
  if (joyRunning and bytes > 0)
  {
    if (bytes != sizeof(jse) and bytes > 0)
    { // size error
      printf("JOY control : Unexpected byte count from joystick:%d - continues\n", bytes);
    }
    else
    { //Proper joystick package has been received
      //Joystick package parser
      isOK = true;
      jse.type &= ~JS_EVENT_INIT; /* ignore synthetic events */
      switch(jse.type) {
        // changed axis position
        case JS_EVENT_AXIS:
          if (jse.number < 16) {
            joyValues.axes[jse.number] = jse.value;
          }
          break;
        // changed button state
        case JS_EVENT_BUTTON:
          if (jse.number < 16) {
            joyValues.button[jse.number] = jse.value;
          }
          break;
        default:
          printf("UJoy::getNewJsData : got bad data (event=%d, time=%d) - ignores\n", jse.type, jse.time);
          isOK = false;
          break;
      }
    }
  }
  return isOK;
}

void SJoyLogitech::joyControl()
{ // we are in manual override mode
  // so do what is needed
  isFast = joyValues.button[buttonFast] == 1;
  float fastScale;
  if (isFast)
    fastScale = 1.0;
  else
    fastScale = slowFactor;
  // velocity
  if (abs(joyValues.axes[axisVel]) > 500)
  { // velocity is valid
    float velVector;
    velVector = joyValues.axes[axisVel];
    velocity = -velVector * fastScale * velScale;
  }
  else
    velocity = 0.0;
  // turn
  if (abs(joyValues.axes[axisTurn]) > 500)
  { // velocity is valid
    turnVelocity = -float(joyValues.axes[axisTurn]) * turnScale;
  }
  else
    turnVelocity = 0.0;
  // implement
  mixer.setManualControl(true, velocity, turnVelocity);
  // servo
  if (abs(joyValues.axes[axisServo]) > 500)
  { // velocity is valid
    servoPosition += float(joyValues.axes[axisServo]) * servoScale;
    // control servo 1
    servo.setServo(1, true, servoPosition, 0);
  }
}

void SJoyLogitech::toLog()
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    { // save all axis and buttons
      fprintf(logfile, "%lu.%04ld %d %g %g %g ", updTime.getSec(), updTime.getMicrosec()/100,
              not mixer.autonomous(), velocity, turnVelocity, servoPosition
      );
      for (int i = 0; i < number_of_buttons; i++)
        fprintf(logfile, " %d", joyValues.button[i]);
      fprintf(logfile, " ");
      for (int i = 0; i < number_of_axes; i++)
        fprintf(logfile, " %d", joyValues.axes[i]);
      fprintf(logfile, "\n");
    }
    if (toConsole)
    { // save all axis and buttons
      fprintf(logfile, "%lu.%04ld %d %g %g %g ", updTime.getSec(), updTime.getMicrosec()/100,
              not mixer.autonomous(), velocity, turnVelocity, servoPosition
      );
      for (int i = 0; i < number_of_buttons; i++)
        fprintf(logfile, " %d", joyValues.button[i]);
      fprintf(logfile, " ");
      for (int i = 0; i < number_of_axes; i++)
        fprintf(logfile, " %5d", joyValues.axes[i]);
      fprintf(logfile, "\n");
    }
  }
}

