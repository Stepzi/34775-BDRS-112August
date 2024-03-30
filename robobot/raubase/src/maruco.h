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
#include <opencv2/imgproc.hpp>
#include "utime.h"
// #include "thread"


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
   * thread to do updates, when new data is available */
  // void run();
  /**
   * Find ArUco code
   * \param size is the side-size of the code.
   * \param raw if raw, distorted image should be used
   * \param sourcePth is a pointer to a potential source image, if
   * this pointer is a nullptr (default), then a frame is taken from camera.
   * \returns the number of codes found. */
  int findAruco(float size,bool raw = false, cv::Mat * sourcePtr = nullptr);
  /**
   * Make an image with this ArUco ID */
  void saveCodeImage(int arucoID);

  std::vector<cv::Vec3d> arTranslate;
  std::vector<cv::Vec3d> arRotate;
  // Detected marked IDs
  std::vector<int> arID;
  std::vector<int> IDs;

  // bool enable = false;
  // bool update = false;
  // bool use_raw = false;

  UTime fixTime;
  
  
  // Position in world frame
  cv::Vec3d pos_w;
  // Marker positions in robot frame
  std::vector<cv::Vec3d> pos_m;
  // Rotation (Euler angles) in world frame
  cv::Vec3d rot_w;
  // Marker rotations in robot frame
  std::vector<cv::Vec3d> rot_m;


  // // Pos of robot in world coordinates (X,Y,Z)
  // float pos_w[3] = {0.0,0.0,0.0};
  // // Pos of robot in world coordinates (X,Y,Z)
  // float pos_m[5][3] = {{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0},{0.0,0.0,0.0}};


protected:
  /// PC time of last update
  UTime imgTime;
  void saveImageTimestamped(cv::Mat & img, UTime imgTime);
  void saveImageInPath(cv::Mat & img, string name);


private:
  // static void runObj(MArUco * obj)
  // { // called, when thread is started
  //   // transfer to the class run() function.
  //   obj->run();
  // }
  /**
   * print to console and logfile */
  void toLog(const char * message);
  /// Debug print
  bool toConsole = false;
  /// Logfile - most details
  FILE * logfile = nullptr;
  // std::thread * th1;
  /// save debug images
  bool debugSave = false;
};

/**
 * Make this visible to the rest of the software */
extern MArUco aruco;

