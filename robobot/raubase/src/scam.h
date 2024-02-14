/*
 *
 * Copyright © 2024 DTU, Christian Andersen jcan@dtu.dk
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


#pragma once

#include <unistd.h>
#include <thread>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include "utime.h"

using namespace std;

/**
 * Class for interface with vision
 * written in Python
 * The interface connects to a python socket server.
 * Requests are send to the Ptyhon server and
 * this class listens to the result */
class UCam
{
public:
  /** setup and connect to server */
  void setup();
  /**
   * Listen to socket from python vision app */
  void run();
  /**
   * terminate */
  void terminate();
  /**
   * capture and save an image */
  bool saveImage();
  /**
   * Calibrate */
  bool calibrate();
  // get the newest frame
  cv::Mat getFrameRaw();
  // get the newest frame rectified
  // using parameters in regbot.ini
  cv::Mat getFrame();
  /**
   * Camera matrix (3x3) */
  cv::Mat cameraMatrix;
  /**
   * Lens distortion coefficients (1x5) */
  cv::Mat distCoeffs;
  UTime imgTime;
  /**
   * Rotation and translation matrix (4x4) from camera-centred coordinates to robot coordinates
   * - all coordinates as is used by robots, i.e x=forward, y=left and z=up) */
  cv::Mat matCtoR;
  /**
   * Rotation matrix (3x3) from camera-centred coordinates to robot coordinates.
   * Note: include pitch angle only.
   * - all coordinates as is used by robots, i.e. x=forward, y=left and z=up) */
  cv::Mat rotCtoR;
  /**
   * Convert this position in camera coordinates to robot coordinates.
   * \param pos is in camera coordinates (x = right, y=down, z=forward)
   *          camera (0,0,0) refers to the pixel at lens center - from by the calibration
   *          x0 = at pixel where x = cameraMatrix(0,3)
   *          y0 = at pixel where y = cameraMatrix(1,3)
   * \returns position in robot coordinates using camera position and pitch
   *          robot coordinates are (x = forward, y=left, z=up). */
  cv::Vec3d getPositionInRobotCoordinates(cv::Vec3d pos);
  /**
   * Convert this orientation in Rodrigues coordinates to euler angles in robot coordinates
   * \param pos Rodrigues coordinates (vector and rotation) in camera coordinates  (x=right, y=down, z=forward)-
   * \param degrees result is in radians (default), except if 'degrees' is true.
   * \returns rotation around robot coordinate axes (right hand rules).
   *          Robot coordinates are (x = forward, y=left, z=up). */
  cv::Vec3d getOrientationInRobotEulerAngles(cv::Vec3d rodrigues, bool degrees = false);

  /**
   * from https://learnopencv.com/rotation-matrix-to-euler-angles/
   * Tests if the provided rotation matrix multiplied with itself transposed is eye(1) */
  bool isRotationMatrix(cv::Matx33d& rot);
  /**
   * from https://learnopencv.com/rotation-matrix-to-euler-angles/
   * */
   cv::Vec3d rotationMatrixToEulerAngles(cv::Matx33d &rot);


private:
  void toLog(const char * pre, const char * post = "");
  bool toConsole = false;
  FILE * logfile = nullptr;
  cv::Vec3d pos;
  double tilt;
  //
  static void runObj(UCam * obj)
  { // called, when thread is started
    // transfer to the class run() function.
    obj->run();
  }
  // camera
  cv::Mat frame;
  cv::VideoCapture cam;
  int frameCnt = 0;
  int gotFrameCnt = 0;
  bool getNewFrame = false;
  bool gotFrame = false;
  // support variables
  std::thread * th1 = nullptr;
  bool stopCam = false;
};

/**
 * Make this visible to the rest of the software */
extern UCam cam;


