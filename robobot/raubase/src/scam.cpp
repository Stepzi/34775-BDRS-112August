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

#include <string>
#include <string.h>
#include <sys/types.h>
#include <filesystem>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

#include "scam.h"
#include "uservice.h"

// create connection object
UCam cam;
namespace fs = std::filesystem;

void UCam::setup()
{ // ensure default values
  if (not ini.has("camera"))
  { // no data yet, so generate some default values
    ini["camera"]["device"] = "0";
    ini["camera"]["width"] = "1280";
    ini["camera"]["height"] = "720";
    ini["camera"]["fps"] = "25";
    ini["camera"]["matrix"] = "1000 0 640 0 1000 360 0 0 1";
    ini["camera"]["distortion"] = "-0.415 0.2244 -6.875e-5 0.001279 -0.073412";
    ini["camera"]["imagepath"] = "img";
    ini["camera"]["imageName"] = "%d"; // %d means date and time
    ini["camera"]["log"] = "true";
    ini["camera"]["print"] = "false";
    ini["camera"]["enabled"] = "false";
    ini["camera"]["pos"] = "0.11 0 0.23";
    ini["camera"]["cam_tilt"] = "0.01";
  }
  if (ini["camera"]["enabled"] == "true")
  { // create directory for images
    fs::create_directory(ini["camera"]["imagepath"]);
    //
    // create log file
    toConsole = ini["camera"]["print"] == "true";
    int device = strtol(ini["camera"]["device"].c_str(), nullptr, 10);
    // Camera matrix
    const char * p1 = ini["camera"]["matrix"].c_str();
    cameraMatrix = cv::Mat(3,3, CV_64F);
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++)
        cameraMatrix.at<double>(i,j) = strtof(p1, (char**)&p1);
//     cout << "Camera matrix:\n" << cameraMatrix << "\n";
    p1 = ini["camera"]["distortion"].c_str();
    distCoeffs = cv::Mat(1,5, CV_64F);
    for (int i = 0; i < 5; i++)
      distCoeffs.at<double>(i) = strtof(p1, (char**)&p1);
//     cout << "Camera distortion:" << distCoeffs << "\n";
    // camera position and rotation
//     pos = cv::Vec3d(CV_64F);
    tilt = strtof(ini["camera"]["cam_tilt"].c_str(), nullptr);
    p1 = ini["camera"]["pos"].c_str();
    //
    for (int i = 0; i < 3; i++)
      pos[i] = strtof(p1, (char**)&p1);
    // generate transformation matrix from camera to robot
    double st = sin(tilt);
    double ct = cos(tilt);
    // generate 4x4 transformation matrix (homogene coordinates)
    matCtoR = cv::Mat::eye(4,4,CV_64F);
    matCtoR.at<double>(0,0) = ct;
    matCtoR.at<double>(0,2) = st;
    matCtoR.at<double>(0,3) = pos[0];
    matCtoR.at<double>(1,3) = pos[1];
    matCtoR.at<double>(2,0) = -st;
    matCtoR.at<double>(2,2) = ct;
    matCtoR.at<double>(2,3) = pos[2];
    // rotation only (3x3) from cam to robot
    rotCtoR = cv::Mat::eye(3,3,CV_64F);
    rotCtoR.at<double>(0,0) = ct;
    rotCtoR.at<double>(0,2) = st;
    rotCtoR.at<double>(2,0) = -st;
    rotCtoR.at<double>(2,2) = ct;
    //
    if (ini["camera"]["log"] == "true")
    { // open logfile
      std::string fn = service.logPath + "log_camera.txt";
      logfile = fopen(fn.c_str(), "w");
      fprintf(logfile, "%% Camera (not vision) - logfile\n");
      fprintf(logfile, "%% connection to camera %d\n", device);
      fprintf(logfile, "%% Image path '%s'\n", ini["camera"]["imagepath"].c_str());
      fprintf(logfile, "%% 1 \tTime (sec)\n");
      fprintf(logfile, "%% 2 \tInformation\n");
    }
    toLog("Camera matrix (from robot.ini)", ini["camera"]["matrix"].c_str());
    toLog("Distortion vector (from robot.ini)", ini["camera"]["distortion"].c_str());
    // prepare to open camera
    int apiID = cv::CAP_V4L2;  //cv::CAP_ANY;  // 0 = autodetect default API
    // open selected camera using selected API
    cam.open(device, apiID);
    // check if we succeeded
    //
    if (not cam.isOpened())
    {
      printf("# UCam - camera could not open\n");
    }
    else
    {
      uint32_t fourcc = cv::VideoWriter::fourcc('M','J','P','G');
      cam.set(cv::CAP_PROP_FOURCC, fourcc);
      // possible resolutions in JPEG coding
      // (rows x columns) 320x640 or 720x1280
      int w = strtol(ini["camera"]["width"].c_str(), nullptr, 0);
      int h = strtol(ini["camera"]["height"].c_str(), nullptr, 0);
      toLog("Width", ini["camera"]["width"].c_str());
      toLog("Width", ini["camera"]["height"].c_str());
      cam.set(cv::CAP_PROP_FRAME_HEIGHT, h);
      cam.set(cv::CAP_PROP_FRAME_WIDTH, w);
      int fps = strtol(ini["camera"]["fps"].c_str(), nullptr, 0);
      cam.set(cv::CAP_PROP_FPS, fps);
      union FourChar
      {
        uint32_t cc4;
        char ccc[4];
      } fmt;
      fmt.cc4 = cam.get(cv::CAP_PROP_FOURCC);
      const int MSL = 200;
      char s[MSL];
      snprintf(s, MSL, "# Video device %d: width=%g, height=%g, format=%c%c%c%c, FPS=%g",
             device,
             cam.get(cv::CAP_PROP_FRAME_WIDTH),
             cam.get(cv::CAP_PROP_FRAME_HEIGHT),
             fmt.ccc[0], fmt.ccc[1], fmt.ccc[2], fmt.ccc[3],
             cam.get(cv::CAP_PROP_FPS));
      printf("%s\n", s);
      toLog(s);
    }
    if (cam.isOpened())
      // start capturing images
      th1 = new std::thread(runObj, this);
  }
  else
    printf("# UCam:: disabled in robot.ini\n");
}

void UCam::terminate()
{ // wait for receive thread to finish
  if (th1 != nullptr)
  {
    th1->join();
    th1 = nullptr;
  }
  // close logfile
  if (logfile != nullptr)
  {
    fclose(logfile);
    logfile = nullptr;
    printf("# UCam:: logfile closed\n");
  }
}


void UCam::run()
{
  printf("# Camera is running (to stabilize illumination)\n");
  toLog("Camera open");
  while (not service.stop and not stopCam)
  { // wait for reply
    if (getNewFrame and not gotFrame and frameCnt > 10)
    {
      cam.read(frame);
      if (not frame.empty())
      {
        printf("# UCam::run: read frame %d/%d\n", gotFrameCnt, frameCnt);
        gotFrameCnt++;
        imgTime.now();
        getNewFrame = false;
        gotFrame = true;
      }
    }
    else
    { // just mark as used to keep the buffer empty
      cam.grab();
    }
    frameCnt++;
//    if (frameCnt % 100 == 3)
//      printf("# cam got frame %d/%d\n", gotFrameCnt, frameCnt);
  }
  th1 = nullptr;
  cam.release();
  printf("# UCam::run: camera released\n");
}


cv::Mat UCam::getFrameRaw()
{ // request new frame
  if (not cam.isOpened())
  {
    printf("# camera not open\n");
    return frame;
  }
//   printf("Asking for a frame\n");
  getNewFrame = true;
  // allow timeout, 1 second from now
  UTime t;
  t.now();
  while (not gotFrame and t.getTimePassed() < 5.0)
  { // wait for frame (or timeout of 1 second)
    usleep(3000);
  }
  if (gotFrame)
    ; // printf("# Got an image frame\n");
  else
    printf("# failed to get an image frame\n");
  // mark finished with frame
  gotFrame = false;
  return frame;
}


bool UCam::saveImage()
{
  if (not cam.isOpened())
  {
    printf("# camera not open\n");
    return false;
  }
  toLog("Save image");
  cv::Mat rgb = getFrameRaw();
  if (not rgb.empty())
  {
    printf("# ready to save\n");
    const int MSL = 500;
    char sfn[MSL];
    const char * sfn_ptr = sfn;
    char s[MSL];
    auto p = ini["camera"]["imageName"].find('%');
    if (p != std::string::npos)
    { // make timestamped image filename
      imgTime.getForFilename(sfn);
      printf("# found '%%' in ini[camera][imageName]\n");
    }
    else
    { // use specified filename
      printf("# no '%%' in ini[camera][imageName]\n");
      sfn_ptr = ini["camera"]["imageName"].c_str();
    }
    // generate filename
    snprintf(s, MSL, "%s/img_raw_%s.jpg", ini["camera"]["imagepath"].c_str(), sfn_ptr);
    // save
    cv::imwrite(s, rgb);
    printf("# saved image to %s\n", s);
    // save also rectified image
    cv::Mat rec;
    cv::undistort(rgb, rec, cameraMatrix, distCoeffs);
    // generate filename
    snprintf(s, MSL, "%s/img_rec_%s.jpg", ini["camera"]["imagepath"].c_str(), sfn_ptr);
    cv::imwrite(s, rec);
    printf("# saved image to %s\n", s);

  }
  else
  {
    printf("UCam:: could not get a frame\n");
  }
  return not rgb.empty();
}


bool UCam::calibrate()
{
  //
  // code from https://learnopencv.com/camera-calibration-using-opencv/
  //
  printf("# ready to calibrate (stopping camera)\n");
  stopCam = true;
  toLog("Start calibrate");
  //
  // Defining the dimensions of checkerboard
  int CHECKERBOARD[2]{6,9};

  // Creating vector to store vectors of 3D points for each checkerboard image
  std::vector<std::vector<cv::Point3f> > objpoints;

  // Creating vector to store vectors of 2D points for each checkerboard image
  std::vector<std::vector<cv::Point2f> > imgpoints;

  // Defining the world coordinates for 3D points
  std::vector<cv::Point3f> objp;
  for(int i{0}; i<CHECKERBOARD[1]; i++)
  {
    for(int j{0}; j<CHECKERBOARD[0]; j++)
      objp.push_back(cv::Point3f(j,i,0));
  }


  // Extracting path of individual image stored in a given directory
  std::vector<cv::String> images;
  // Path of the folder containing images with checkerboard
  std::string path = ini["camera"]["imagepath"] + "/img_raw_*.jpg";

  cv::glob(path, images);

  std::vector<cv::String> okImages;
  cv::Mat frame, gray;
  // vector to store the pixel coordinates of detected checker board corners
  std::vector<cv::Point2f> corner_pts;
  bool success;
  int j = 0;
  // Looping over all the images in the directory
  for(int i = 0; i < (int)images.size(); i++)
  {
    frame = cv::imread(images[i]);
    cv::cvtColor(frame,gray,cv::COLOR_BGR2GRAY);

    // Finding checker board corners
    // If desired number of corners are found in the image then success = true
    success = cv::findChessboardCorners(gray,cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

    /*
      * If desired number of corner are detected,
      * we refine the pixel coordinates and display
      * them on the images of checker board
      */
    if(success)
    {
      cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

      // refining pixel coordinates for given 2d points.
      cv::cornerSubPix(gray,corner_pts,cv::Size(11,11), cv::Size(-1,-1),criteria);

      // Displaying the detected corner points on the checker board
      cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0],CHECKERBOARD[1]), corner_pts,success);

      objpoints.push_back(objp);
      imgpoints.push_back(corner_pts);
      printf("# %2d succes    %s\n", j++, images[i].c_str());
      okImages.push_back(images[i]);
    }
    else
      printf("#   no corners %s\n", images[i].c_str());

//     cv::imshow("Image",frame);
//     cv::waitKey(0);
  }
  // if needed
  cv::destroyAllWindows();
  // estimated camera pose (Translate and Rotate)
//   cv::Mat R,T;
  if (j > 0)
  {
    std::vector<cv::Mat> rvecs, tvecs;
    /*
      * Performing camera calibration by
      * passing the value of known 3D points (objpoints)
      * and corresponding pixel coordinates of the
      * detected corners (imgpoints)
      */
    cv::calibrateCamera(objpoints, imgpoints,cv::Size(gray.rows,gray.cols),cameraMatrix,distCoeffs,rvecs,tvecs);
    // show results
    for (int i = 0; i < cameraMatrix.rows; i++)
    {
      printf("# Camera matrix %d: %7.1f %7.1f %7.1f\n", i,
            cameraMatrix.at<double>(i,0),
            cameraMatrix.at<double>(i,1),
            cameraMatrix.at<double>(i,2)
      );
    }
    // copy to ini-file
    const int MSL = 200;
    char s[MSL];
    snprintf(s, MSL, "%7.1f%7.1f%7.1f %7.1f%7.1f%7.1f %7.1f%7.1f%7.1f",
            cameraMatrix.at<double>(0,0),
            cameraMatrix.at<double>(0,1),
            cameraMatrix.at<double>(0,2),
            cameraMatrix.at<double>(1,0),
            cameraMatrix.at<double>(1,1),
            cameraMatrix.at<double>(1,2),
            cameraMatrix.at<double>(2,0),
            cameraMatrix.at<double>(2,1),
            cameraMatrix.at<double>(2,2));
    ini["camera"]["matrix"] = s;
    toLog("Camera matrix", s);
    // also lens distortion
    snprintf(s, MSL, "%g %g %g %g %g",
            distCoeffs.at<double>(0,0),
            distCoeffs.at<double>(0,1),
            distCoeffs.at<double>(0,2),
            distCoeffs.at<double>(0,3),
            distCoeffs.at<double>(0,4));
    ini["camera"]["distortion"] = s;
    toLog("Distortion vector", s);

    // Show distortion in screen
    const char * kx[] = {"k1","k2","p1","p2","k3"};
    for (int j = 0; j < distCoeffs.cols; j++)
    {
      printf("# Distortion %s: %g\n", kx[j], distCoeffs.at<double>(j));
    }

  // show using stream (less structured)
  //   std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
  //   std::cout << "distCoeffs : " << distCoeffs << std::endl;
  //   std::cout << "cam pose Rotation vectors : " << R << std::endl;
  //   std::cout << "cam pose Translation vectors : " << T << std::endl;

    // calculate pixel error for these images
    vector<float> imgErr;
    vector<cv::Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    imgErr.resize(objpoints.size());
    for(size_t i = 0; i < objpoints.size(); ++i )
    {
      cv::projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
      err = cv::norm(imgpoints[i], imagePoints2, cv::NORM_L2);
      size_t n = objpoints[i].size();
      imgErr[i] = (float) std::sqrt(err*err/n);
      totalErr        += err*err;
      totalPoints     += n;
    }
    snprintf(s, MSL, "# Average pixel error is %.2f", sqrt(totalErr/totalPoints));
    printf("%s\n", s);
    toLog(s);
    for (int i = 0; i < (int)imgErr.size(); i++)
    {
      snprintf(s, MSL, "# Image %d error %.2f pixels", i, imgErr[i]);
      printf("%s\n", s);
      toLog(s, okImages[i].c_str());
    }
  }
  else
    printf("# No usable images were found (in %s/%s/img_raw_*.jpg)\n", fs::current_path().c_str(), ini["camera"]["imagepath"].c_str());

  return j > 0;
}



void UCam::toLog(const char * pre, const char * post)
{
  if (service.stop)
    return;
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04lu %s %s\n",
            t.getSec(), t.getMicrosec()/100, pre, post);
  }
  if (toConsole)
  {
    printf("%lu.%04lu %s %s\n",
            t.getSec(), t.getMicrosec()/100, pre, post);
  }
}

cv::Mat UCam::getFrame()
{
  cv::Mat raw;
  cv::Mat rectified;
  raw = getFrameRaw();
  cv::undistort(raw, rectified, cameraMatrix, distCoeffs);
  // cv::imshow("Rectified image",rectified);
  // cv::waitKey(0);
  return rectified;
}

// Checks if a matrix is a valid rotation matrix.
bool UCam::isRotationMatrix(cv::Matx33d &rot)
{
  cv::Mat Rt;
  cv::transpose(rot, Rt);
  cv::Mat shouldBeIdentity = Rt * rot;
  cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
  return  cv::norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
cv::Vec3d UCam::rotationMatrixToEulerAngles(cv::Matx33d &rot)
{
  if (not isRotationMatrix(rot))
    printf("Given rotation matrix is not a rotation matrix\n");
  //
  float sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
  bool singular = sy < 1e-6;
  float x, y, z;
  if (!singular)
  {
    x = atan2(rot(2,1) , rot(2,2));
    y = atan2(-rot(2,0), sy);
    z = atan2(rot(1,0), rot(0,0));
  }
  else
  {
    x = atan2(-rot(1,2), rot(1,1));
    y = atan2(-rot(2,0), sy);
    z = 0;
  }
  return cv::Vec3d(x, y, z);
}


cv::Vec3d UCam::getOrientationInRobotEulerAngles(cv::Vec3d rodrigues, bool degrees)
{
  cv::Vec3d rh;
  rh[0] =  rodrigues[2]; // robot x is forward, i.e. image z (z is distance away from cam)
  rh[1] =  -rodrigues[0]; // robot y is left, i.e. image -x (image x is right)
  rh[2] =  -rodrigues[1]; // robot z is up, i.e image -y (image y is down)
  // convert to robot coordinates
  cv::Mat rr = rotCtoR * rh;
  // rotation is in Rodrigues coordinates (vector and rotation around this vector)
  cv::Matx33d mrr;
  cv::Rodrigues(rr, mrr);
  cv::Vec3f re = rotationMatrixToEulerAngles(mrr);
//   printf("# Euler angles in robot coordinates %dx%d (x,y,z) = (%g %g %g)\n",
//          re.rows, re.cols,
//          re[0], // roll
//          re[1], // pitch
//          re[2]); //yaw
  // make angles more useful
  // facing robot is angle (0,0,0)
  re[0] *= -1.0; // for some reason
  re[0] += M_PI;
  if (re[0] > M_PI)
    re[0] -= 2 * M_PI;
  re[1] *= -1.0; // for some reason
  re[2] += M_PI;
  if (re[2] > M_PI)
    re[2] -= 2 * M_PI;
  if (degrees)
    re *= 180.0/M_PI;
  return re;
}


cv::Vec3d UCam::getPositionInRobotCoordinates(cv::Vec3d pos)
{
  cv::Vec4d ph;
  ph[0] =  pos[2];  // robot x is forward, i.e. image z (z is distance away from cam)
  ph[1] = -pos[0]; // robot y is left, i.e. image -x (image x is right)
  ph[2] = -pos[1]; // robot z is up, i.e image -y (image y is down)
  ph[3] = 1.0;
  // convert to robot coordinates
  cv::Mat pr = matCtoR * ph;
  // reformat to result vector
  cv::Vec3d result(pr.at<double>(0), pr.at<double>(1), pr.at<double>(2));
  //
  if (false)
    printf("# pos in robot coordinates %dx%d (x,y,z,1) = (%g %g %g)\n",
           pr.rows, pr.cols, result[0], result[1], result[2]);
  return result;
}
