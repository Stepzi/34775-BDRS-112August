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
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <filesystem>

#include "mgolfball.h"
#include "uservice.h"
#include "scam.h"

// create value
Mgolfball golfball;
namespace fs = std::filesystem;


void Mgolfball::setup()
{ // ensure there is default values in ini-file
  if (not ini.has("golfball"))
  { // no data yet, so generate some default values
    ini["golfball"]["imagepath"] = "golfball";
    ini["golfball"]["save"] = "false";
    ini["golfball"]["log"] = "true";
    ini["golfball"]["print"] = "true";
  }
  // get values from ini-file
  fs::create_directory(ini["golfball"]["imagepath"]);
  //
  debugSave = ini["golfball"]["save"] == "true";
  toConsole = ini["golfball"]["print"] == "true";
  //
  if (ini["golfball"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_golfball.txt";
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


void Mgolfball::terminate()
{ // wait for thread to finish
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
  }
}

void Mgolfball::toLog(const char * message)
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

bool Mgolfball::findGolfball(std::vector<int>& pos, cv::Mat *sourcePtr)
{ // taken from https://docs.opencv.org
  // Init setup
  setup()

  // Get frame 
  cv::Mat frame;
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
  if (frame.empty())
  {
    printf("MVision::findGolfball: Failed to get an image\n");
    return 0;
  }
  cv::Mat img;
  if (debugSave)
    frame.copyTo(img);
  //=============================================

  // filter
  // blur
  // convert colors
  // apply color filter
  // detect contours
  // count == no. of detected golfball candidates
  // set bool to true if no of contours greater than 0
  // choose closest
  
  cv::Mat blurred;
  cv::GaussianBlur(frame, blurred, cv::Size(11, 11), 0);
  int width = frame.rows;
  int height = frame.cols;
  cv::Mat mask;
  cv::cvtColor(blurred, mask, cv::COLOR_BGR2HSV);
  cv::inRange(mask, cv::Scalar(10, 100, 100), cv::Scalar(20, 255, 255), mask);
  // cv::erode(mask, mask, Mat, 2);
  // cv::dilate(mask, mask, Mat, 2);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::noArray(),cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

  cv::Point2f center;
  float radius;
  if (contours.size() > 0){
        float max_area = 0;
        std::vector<cv::Point> c;
        for (int i = 0; i < contours.size(); i++){
            float area = cv::contourArea(contours[i]);
            if (area > max_area){
                max_area = area;
                c = contours[i];
            }
        }

    
    cv::minEnclosingCircle(c, center, radius);
    if (radius < 10 && radius > 65){
        debugSave = false;
        return false;
    }
      
    cv::Moments M = cv::moments(c);
    center = cv::Point2f(int(M.m10 / M.m00), int(M.m01 / M.m00));
    // center = static_cast<int>(center);
    pos[0] = int(center.x);
    pos[1] = int(center.y);
      //
    if (debugSave){ 
      const int MSL = 200;
      char s[MSL];
      // paint found golfballs in image copy 'img'.
      // Draw circle and its center
      cv::circle(img, center, static_cast<int>(radius), cv::Scalar(0,255,0), 2);
      cv::circle(img, center, 1, cv::Scalar(0, 0, 255), 2);
      snprintf(s, MSL, "center: (%d, %d), radius: %d", center[0], centert[1], radius);
      toLog(s);
      saveImageTimestamped(img, imgTime);
    }
    return true;
  }
  return false;
}

void Mgolfball::saveImageInPath(cv::Mat& img, string name)
{ // Note, file type must be in filename
  const int MSL = 500;
  char s[MSL];
  // generate filename
  snprintf(s, MSL, "%s/%s", ini["golfball"]["imagepath"].c_str(), name.c_str());
  // save
  cv::imwrite(s, img);
  printf("# saved image to %s\n", s);
}


void Mgolfball::saveImageTimestamped(cv::Mat & img, UTime imgTime)
{
  const int MSL = 500;
  char s[MSL] = "golfball_";
  char * time_ptr = &s[strlen(s)];
  //
  imgTime.getForFilename(time_ptr);
  saveImageInPath(img, string(s) + ".jpg");
}


