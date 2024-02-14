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

#pragma once

#include <opencv2/core.hpp>
#include "utime.h"

using namespace std;

/**
 * Class with example of vision processing
 * */
class MArUco
{
public:
  /** setup and request data */
  void setup();
  /**
   * terminate */
  void terminate();
  /**
   * Find ArUco code
   * \param size is the side-size of the code.
   * \param sourcePth is a pointer to a potential source image, if
   * this pointer is a nullptr (default), then a frame is taken from camera.
   * \returns the number of codes found. */
  int findAruco(float size, cv::Mat * sourcePtr = nullptr);
  /**
   * Make an image with this ArUco ID */
  void saveCodeImage(int arucoID);

  std::vector<cv::Vec3d> arTranslate;
  std::vector<cv::Vec3d> arRotate;
  std::vector<int> arCode;

protected:
  /// PC time of last update
  UTime imgTime;
  void saveImageTimestamped(cv::Mat & img, UTime imgTime);
  void saveImageInPath(cv::Mat & img, string name);

private:
  /**
   * print to console and logfile */
  void toLog(const char * message);
  /// Debug print
  bool toConsole = false;
  /// Logfile - most details
  FILE * logfile = nullptr;
  /// save debug images
  bool debugSave = false;
};

/**
 * Make this visible to the rest of the software */
extern MArUco aruco;

