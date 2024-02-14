/*  
 * 
 * Copyright © 2024 DTU,
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

/**
 * ArUco specific code is from
 * https://docs.opencv.org/3.4.20/d5/dae/tutorial_aruco_detection.html
 * */

#include <string>
#include <string.h>
#include <math.h>
#include <opencv2/aruco.hpp>
#include <filesystem>
#include "maruco.h"
#include "uservice.h"
#include "scam.h"

// create value
MArUco aruco;
namespace fs = std::filesystem;


void MArUco::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("aruco"))
  { // no data yet, so generate some default values
    ini["aruco"]["imagepath"] = "aruco";
    ini["aruco"]["save"] = "true";
    ini["aruco"]["log"] = "true";
    ini["aruco"]["print"] = "true";
  }
  // get values from ini-file
  fs::create_directory(ini["aruco"]["imagepath"]);
  //
  debugSave = ini["aruco"]["save"] == "true";
  toConsole = ini["aruco"]["print"] == "true";
  //
  if (ini["aruco"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_aruco.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Vision activity (%s)\n", fn.c_str());
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tDetected marker in this image\n");
    fprintf(logfile, "%% 3 \tDetected marker code\n");
    fprintf(logfile, "%% 4 \tMarker size (position in same units as size)\n");
    fprintf(logfile, "%% 5,6,7 \tDetected marker position in camera coordinates (x=right, y=down, z=forward)\n");
    fprintf(logfile, "%% 8,9,10 \tDetected marker orientation in Rodrigues notation (vector, rotated)\n");
  }
}


void MArUco::terminate()
{ // wait for thread to finish
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}

void MArUco::toLog(const char * message)
{
  if (not service.stop)
  {
    if (logfile != nullptr)
    { // log_pose
      fprintf(logfile, "%lu.%04ld %s\n", imgTime.getSec(), imgTime.getMicrosec()/100, message);
    }
    if (toConsole)
    { // print_pose
      printf("%lu.%04ld %s\n", imgTime.getSec(), imgTime.getMicrosec()/100, message);
    }
  }
}


int MArUco::findAruco(float size, cv::Mat * sourcePtr)
{ // taken from https://docs.opencv.org
  int count = 0;
  cv::Mat frame;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  if (sourcePtr == nullptr)
  {
    frame = cam.getFrameRaw();
    imgTime = cam.imgTime;
  }
  else
  {
    frame = *sourcePtr;
  }
  //
  // printf("# MVision::findAruco looking for ArUco of size %.3fm\n", size);
  if (frame.empty())
  {
    printf("MVision::findAruco: Failed to get an image\n");
    return 0;
  }
  cv::Mat img;
  if (debugSave)
    frame.copyTo(img);
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(frame, dictionary, markerCorners, arCode);
  count = arCode.size();
  // estimate pose of all markers
  cv::aruco::estimatePoseSingleMarkers(markerCorners, size, cam.cameraMatrix, cam.distCoeffs, arRotate, arTranslate);
  //
  if (debugSave)
  { // paint found markers in image copy 'img'.
    const int MSL = 200;
    char s[MSL];
    // draw axis for each marker
    for(int i=0; i<count; i++)
    {
      cv::aruco::drawAxis(img, cam.cameraMatrix, cam.distCoeffs, arRotate[i], arTranslate[i], 0.1);
      snprintf(s, MSL, "%d %d %g %g %g %g  %g %g %g", i, arCode[i], size,
               arTranslate[i][0], arTranslate[i][1], arTranslate[i][2],
               arRotate[i][0], arRotate[i][1], arRotate[i][2]);
      toLog(s);
    }
    saveImageTimestamped(img, imgTime);
  }
  return count;
}

void MArUco::saveImageInPath(cv::Mat& img, string name)
{ // Note, file type must be in filename
  const int MSL = 500;
  char s[MSL];
  // generate filename
  snprintf(s, MSL, "%s/%s", ini["aruco"]["imagepath"].c_str(), name.c_str());
  // save
  cv::imwrite(s, img);
  printf("# saved image to %s\n", s);
}


void MArUco::saveImageTimestamped(cv::Mat & img, UTime imgTime)
{
  const int MSL = 500;
  char s[MSL] = "aruco_";
  char * time_ptr = &s[strlen(s)];
  //
  imgTime.getForFilename(time_ptr);
  saveImageInPath(img, string(s) + ".jpg");
}

void MArUco::saveCodeImage(int arucoID)
{
  cv::Mat markerImage;
  int pixSize = 240;
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
  cv::aruco::drawMarker(dictionary, arucoID, pixSize, markerImage, 1);
  saveImageInPath(markerImage, string("marker_") + to_string(arucoID) + ".png");
}
