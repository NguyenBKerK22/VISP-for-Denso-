/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 *   tests the control law
 *   eye-in-hand control
 *   velocity computed in the articular frame
 */
/*!
  \example servoViper850FourPoints2DArtVelocityLs_des.cpp

  \brief Example of eye-in-hand control law. We control here a real robot, the
  Viper S850 robot (arm with 6 degrees of freedom). The velocities resulting
  from visual servo are here joint velocities. Visual features are the image
  coordinates of 4 points. The target is made of 4 dots arranged as a 10cm by
  10cm square.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h> // Debug trace
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpJSON.h>
#include <visp3/blob/vpDot2.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/robot/vpRobotDenso.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <visp3/core/vpImageConvert.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <visp3/core/vpTrackingException.h>
#include <fcntl.h>
// =============================================================
// ViSP example: Eye-in-hand visual servoing với robot Denso
// + Camera track 1 điểm (dot)
// + Điều khiển robot bằng visual servo (joint velocity)
// + Điều khiển gripper qua UART (JSON)
// =============================================================
using json = nlohmann::json;
// =======================
// StereoSGBM Parameters
// =======================

cv::Mat disparity_visual;
cv::Mat disp32f;
int minDisparity = 0;
int numDisparities = 10; // sẽ nhân 16
int blockSize = 10;

int uniquenessRatio = 0;
int speckleWindowSize = 10;
int speckleRange = 100;
int disp12MaxDiff = 25;

int flag = false;
cv::Ptr<cv::StereoSGBM> stereo;
std::vector<float> z_vec = { 400, 500, 600,
    700, 800, 900, 1000, 1100
};
std::vector<float> coeff_vec;
float M = 42886.78906;
int x_old = -1, y_old = -1;
void mouseClickHandle(int action, int x, int y, int flags, void *userdata)
{
  if (action == cv::EVENT_LBUTTONDOWN) {
    if (x != x_old || y != y_old) {
      std::cout << "Click at " << x <<" " << y <<std::endl;
      x_old = x;
      y_old = y;
      cv::Mat *disp = static_cast<cv::Mat *>(userdata);
      float d = disp->at<float>(y, x);
      if (d > 0.0f) {
        std::cout << "Z depth = " << M * 1.0 / d << std::endl;
      }
      else {
        std::cout << "Invalid disparity" << std::endl;
      }
    }
  }
}


static void on_numDisparities(int, void *)
{
  int actual_numDisparities = numDisparities * 16;

  if (actual_numDisparities < 16)
    actual_numDisparities = 16;

  stereo->setNumDisparities(actual_numDisparities);

  std::cout << "numDisparities = "
    << actual_numDisparities << std::endl;
}
static void on_blockSize(int, void *)
{
  int actual_blockSize = blockSize;

  if (actual_blockSize % 2 == 0)
    actual_blockSize++;

  if (actual_blockSize < 3)
    actual_blockSize = 3;

  stereo->setBlockSize(actual_blockSize);

  int P1 = 8 * 3 * actual_blockSize * actual_blockSize;
  int P2 = 32 * 3 * actual_blockSize * actual_blockSize;

  stereo->setP1(P1);
  stereo->setP2(P2);

  std::cout << "blockSize = "
    << actual_blockSize << std::endl;
}
static void on_minDisparity(int, void *)
{
  stereo->setMinDisparity(minDisparity);

  std::cout << "minDisparity = "
    << minDisparity << std::endl;
}
static void on_uniquenessRatio(int, void *)
{
  stereo->setUniquenessRatio(uniquenessRatio);
}
static void on_speckleWindowSize(int, void *)
{
  stereo->setSpeckleWindowSize(
      speckleWindowSize > 0 ?
      speckleWindowSize * 10 : 0);
}
static void on_speckleRange(int, void *)
{
  stereo->setSpeckleRange(speckleRange);
}
static void on_disp12MaxDiff(int, void *)
{
  stereo->setDisp12MaxDiff(disp12MaxDiff);
}
enum ROBOT_STATE
{
  PREINIT,     // Đưa robot về pose ban đầu
  INIT,        // Khởi tạo gripper + vision
  JOINT,       // Visual servoing
  APPROACH,    // Chờ tín hiệu tiếp cận
  GRIPPER,     // Đóng gripper
  CLASSIFIED,   // Hoàn thành chu trình
  NEXT_STEP
};
enum GripperState
{
  CLOSED,
  OPENED,
  INIT_GRIPPER
};
void gripperFlush(serialib *gripper)
{
  char c;
  // Đọc hết những gì còn trong buffer
  while (gripper->readChar(&c, 1) == 1) {
    // bỏ qua dữ liệu
  }
}
bool gripperSendCommand(serialib *gripper, std::string command)
{
  return gripper->writeBytes(
            reinterpret_cast<const uint8_t *>(command.data()),
            command.size()
  );

}
bool gripperReceiveBuffer(serialib *gripper, char *buffer)
{
  int idx = 0;

  bool received_left = false;
  bool received_right = false;

  char c;
  int ret;

  timeOut timer;
  timer.initTimer();

  // Đọc từng byte cho tới khi nhận đủ JSON + '\r'
  while (true) {
    ret = gripper->readChar(&c, 10);

    if (ret == 1) {
        // Debug (có thể bật nếu cần)
        // std::cout << c;

      if ((c == '{')) {
        received_left = !received_left;
      }

      if (c == '}') {
        received_right = !received_right;
      }
      if ((c == '\r') && received_left && received_right) {
        buffer[idx] = '\0';
        break;
      }

      if ((idx <  255) && received_left) {
        buffer[idx++] = c;
      }
    }
    if (timer.elapsedTime_ms() > 1500) {
      return false;
    }
  }
  if (received_left && received_right) {
    return true;
  }
  return false;
}
bool gripperStatusRequest(serialib *gripper)
{
  json object = { {"T", 105} };
  std::string command = object.dump() + "\r\n";

  return gripperSendCommand(gripper, command);
}
bool gripperOpen(serialib *gripper)
{
  char buffer[256];
  json object = {
                  {"T", 121},
                  {"acc", 20.0},
                  {"angle", 1.7486619853687655}, // Góc mở
                  {"spd", 500.0}
  };

  // Tạo chuỗi command
  std::string command = object.dump() + "\r\n";

  gripperSendCommand(gripper, command);

  gripperStatusRequest(gripper);

  if (gripperReceiveBuffer(gripper, buffer)) {
    try {
      nlohmann::json object = nlohmann::json::parse(buffer);

      int T = object.value("T", -1);
      int load = object.value("load", -1);

      if (T == 1051 && load >= 50) {
        return true;
      }
    }
    catch (const nlohmann::json::parse_error &e) {
      return false;
    }
  }
  return false;
}
bool gripperClose(serialib *gripper)
{
  char buffer[256];
  json object = {
                  {"T", 121},
                  {"acc", 20.0},
                  {"angle", 3.1447332076700687}, // Góc mở
                  {"spd", 500.0}
  };

  // Tạo chuỗi command trực tiếp
  std::string command = object.dump() + "\r\n";
  gripperSendCommand(gripper, command);
  gripperStatusRequest(gripper);
  gripperReceiveBuffer(gripper, buffer);

  if (gripperReceiveBuffer(gripper, buffer)) {
    try {
      nlohmann::json object = nlohmann::json::parse(buffer);

      int T = object.value("T", -1);
      int load = object.value("load", -1);

      if (T == 1051 && load <= -100) {
        return true;
      }
    }
    catch (const nlohmann::json::parse_error &e) {
      return false;
    }
  }
  return false;
}
bool loadIntrinsicParameters(const std::string &filename, vpCameraParameters &cam)
{
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return false;
  }

  std::string line;

  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  double k1 = 0.0;
  // ===== Read "intrinsic:" =====
  while (std::getline(file, line)) {

    if (line.find("intrinsic:") != std::string::npos) {

        // Row 1
      std::getline(file, line);
      std::stringstream ss1(line);

      double temp;
      ss1 >> fx >> temp >> cx;

      // Row 2
      std::getline(file, line);
      std::stringstream ss2(line);

      ss2 >> temp >> fy >> cy;

      // Row 3 (ignored)
      std::getline(file, line);

      break;
    }
  }

  // ===== Read "distortion:" =====
  while (std::getline(file, line)) {

    if (line.find("distortion:") != std::string::npos) {

      std::getline(file, line);

      std::stringstream ss(line);

      ss >> k1;

      break;
    }
  }

  file.close();

  // ===== OpenCV -> ViSP =====
  double kud = k1;
  double kdu = -k1;

  cam.initPersProjWithDistortion(
      fx,
      fy,
      cx,
      cy,
      kud,
      kdu
  );


  return true;
}
bool detectCylinderWithAI(const vpImage<unsigned char> &I, vpImagePoint &detected_center, const std::string &config_path = "ai_module/config.json", const vpImagePoint *hint = nullptr)
{
  // Convert grayscale vpImage to BGR cv::Mat — model expects 3-channel input
  cv::Mat gray_mat, bgr_mat;
  vpImageConvert::convert(I, gray_mat);
  cv::cvtColor(gray_mat, bgr_mat, cv::COLOR_GRAY2BGR);
  if (!cv::imwrite("/tmp/visp_ai_frame.jpg", bgr_mat)) {
    std::cerr << "[AI] Failed to save frame to /tmp/visp_ai_frame.jpg" << std::endl;
    return false;
  }

  // Read python_bin from config.json (nlohmann::json already available via vpJSON.h)
  std::string python_bin;
  try {
    std::ifstream cfg_file(config_path);
    if (!cfg_file.is_open()) {
      std::cerr << "[AI] Cannot open config: " << config_path << std::endl;
      return false;
    }
    nlohmann::json cfg = nlohmann::json::parse(cfg_file, nullptr, true, true);
    python_bin = cfg.value("python_bin", "/usr/bin/python3");
  }
  catch (const std::exception &e) {
    std::cerr << "[AI] Config parse error: " << e.what() << std::endl;
    return false;
  }

  // Resolve detect_cylinder.py path from the same directory as config.json
  std::string script_dir;
  auto slash_pos = config_path.rfind('/');
  if (slash_pos != std::string::npos)
    script_dir = config_path.substr(0, slash_pos);
  else
    script_dir = ".";
  std::string cmd = python_bin + " " + script_dir + "/detect_cylinder.py /tmp/visp_ai_frame.jpg";
  if (hint != nullptr)
    cmd += " --hint " + std::to_string(hint->get_u()) + " " + std::to_string(hint->get_v());

  FILE *pipe = popen(cmd.c_str(), "r");
  if (!pipe) {
    std::cerr << "[AI] popen failed: " << cmd << std::endl;
    return false;
  }

  // 3-second timeout via POSIX select()
  int fd = fileno(pipe);
  fd_set rfds;
  struct timeval tv;
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
  tv.tv_sec = 3;
  tv.tv_usec = 0;
  if (select(fd + 1, &rfds, nullptr, nullptr, &tv) <= 0) {
    std::cerr << "[AI] Subprocess timeout (3 s)" << std::endl;
    pclose(pipe);
    return false;
  }

  char line[512] = {};
  if (fgets(line, sizeof(line), pipe) == nullptr) {
    pclose(pipe);
    return false;
  }
  pclose(pipe);

  std::string result(line);
  if (result.rfind("SUCCESS", 0) == 0) {
    double u = 0, v = 0, conf = 0, x = 0, y = 0, w = 0, h = 0;
    if (std::sscanf(line, "SUCCESS %lf %lf %lf %lf %lf %lf %lf",
                    &u, &v, &conf, &x, &y, &w, &h) == 7) {
      detected_center = vpImagePoint(v, u);
      return true;
    }
    std::cerr << "[AI] Malformed SUCCESS line: " << result;
    return false;
  }
  if (result.rfind("FAILURE", 0) == 0)
    std::cerr << "[AI] " << result;
  return false;
}
bool loadExtrinsicParameters(const std::string &filename, vpHomogeneousMatrix &M)
{
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Cannot open file: "
      << filename << std::endl;
    return false;
  }

  std::string line;

  vpRotationMatrix R;
  vpTranslationVector T;

  // =========================
  // Read rotation matrix
  // =========================

  while (std::getline(file, line)) {
    if (line.find("R:") != std::string::npos) {
      for (int i = 0; i < 3; i++) {
        std::getline(file, line);

        std::stringstream ss(line);

        ss >> R[i][0]
          >> R[i][1]
          >> R[i][2];
      }

      break;
    }
  }

  // =========================
  // Read translation vector
  // =========================

  while (std::getline(file, line)) {
    if (line.find("T:") != std::string::npos) {
      for (int i = 0; i < 3; i++) {
        std::getline(file, line);

        std::stringstream ss(line);

        ss >> T[i];
      }

      break;
    }
  }

  file.close();

  // =========================
  // Create homogeneous matrix
  // =========================

  M.buildFrom(T, R);
  std::cout << M.t() << std::endl;
  return true;
}
bool loadCameraParameters(std::string camera_left_folder, std::string camera_right_folder, std::string camera_extrinsic_folder, vpHomogeneousMatrix &cRightMcLeft, vpCameraParameters &cam_left, vpCameraParameters &cam_right)
{
  if (!loadIntrinsicParameters(
    camera_left_folder,
    cam_left)) {
    return false;
  }

  if (!loadIntrinsicParameters(
    camera_right_folder,
    cam_right)) {
    return false;
  }
  cam_left.printParameters();
  cam_right.printParameters();

  return loadExtrinsicParameters(camera_extrinsic_folder, cRightMcLeft);
}
bool loadOpenCVIntrinsics(
    const std::string &filename,
    cv::Mat &cameraMatrix,
    cv::Mat &disCoeff)
{
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "Cannot open file: "
      << filename << std::endl;
    return false;
  }

  std::string line;

  // =========================
  // Initialize matrices
  // =========================

  cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

  // OpenCV distortion:
  // [k1 k2 p1 p2 k3]
  disCoeff = cv::Mat::zeros(1, 5, CV_64F);

  // =========================
  // Read intrinsic matrix
  // =========================

  while (std::getline(file, line)) {
    if (line.find("intrinsic:") != std::string::npos) {
      for (int i = 0; i < 3; i++) {
        std::getline(file, line);

        std::stringstream ss(line);

        for (int j = 0; j < 3; j++) {
          ss >> cameraMatrix.at<double>(i, j);
        }
      }

      break;
    }
  }

  // =========================
  // Read distortion coefficients
  // =========================

  while (std::getline(file, line)) {
    if (line.find("distortion:") != std::string::npos) {
      std::getline(file, line);

      std::stringstream ss(line);

      double val;
      int idx = 0;

      while (ss >> val) {
        if (idx < 5) {
          disCoeff.at<double>(0, idx) = val;
        }

        idx++;
      }

      break;
    }
  }

  file.close();

  return true;
}
bool cameraInitialize(cv::VideoCapture &cap)
{
  if (!cap.isOpened()) {            // check if we succeeded
    std::cout << "Failed to open the camera" << std::endl;
    return false;
  }
  // Use MJPEG
  cap.set(
      cv::CAP_PROP_FOURCC,
      cv::VideoWriter::fourcc('M', 'J', 'P', 'G')
  );

  // Resolution
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

  // FPS
  cap.set(cv::CAP_PROP_FPS, 30);
  return true;
}
int main(int argc, char **argv)
{
  // ========================
  // Cấu hình hệ thống
  // ========================
  int count = 0;
  int opt_device = 2;
  bool opt_display = true;
  std::string opt_camera_name = "Camera";
  std::string opt_eMc_filename = "rc5_ePc.yaml";
  std::string opt_camera_folder = ".";
  GripperState gripperStatus = INIT_GRIPPER;

  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--no-display") {
      opt_display = false;
    }
    else if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--eMc" && i + 1 < argc) {
      opt_eMc_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--intrinsic-folder" && i + 1 < argc) {
      opt_camera_folder = std::string(argv[++i]);
    }
    else {
      std::cout << "\nERROR" << std::endl
        << std::string(argv[i]) << " command line option is not supported." << std::endl
        << "Use " << std::string(argv[0]) << " --help" << std::endl
        << std::endl;
      return EXIT_FAILURE;
    }
  }
  // // Init Gripper
  // serialib *gripper = new serialib;
  // gripper->openDevice("/dev/ttyACM0", 115200);

  // Init Camera
  int i = 0;
  cv::Mat frame;
  cv::VideoCapture cap(opt_device, cv::CAP_V4L2); // open the default camera
  if (!cameraInitialize(cap)) {
    return EXIT_FAILURE;
  }

  vpHomogeneousMatrix cLeftMcRight;
  vpCameraParameters cam_left, cam_right;
  std::string camera_right_folder = opt_camera_folder + "camera1_intrinsics.dat";
  std::string camera_left_folder = opt_camera_folder + "camera0_intrinsics.dat";
  std::string camera_extrinsic_folder = opt_camera_folder + "camera1_rot_trans.dat";
  if (!loadCameraParameters(camera_left_folder, camera_right_folder, camera_extrinsic_folder, cLeftMcRight, cam_left, cam_right)) {
    return EXIT_FAILURE;
  }

  cv::Mat cameraMatrixLeft, cameraMatrixRight;
  cv::Mat disCoeffLeft, disCoeffRight;
  loadOpenCVIntrinsics(camera_left_folder, cameraMatrixLeft, disCoeffLeft);
  loadOpenCVIntrinsics(camera_right_folder, cameraMatrixRight, disCoeffRight);


  vpRotationMatrix R_visp;
  vpTranslationVector T_visp;

  cLeftMcRight.extract(R_visp);
  cLeftMcRight.extract(T_visp);

  cv::Mat R_mat = cv::Mat::eye(3, 3, CV_64F);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R_mat.at<double>(i, j) = R_visp[i][j];
    }
  }

  cv::Mat T_mat = cv::Mat::zeros(3, 1, CV_64F);
  for (int i = 0; i < 3; i++) {
    T_mat.at<double>(i, 0) = T_visp[i] * 10;
  }
  cv::namedWindow("Left Rectified");
  cv::setMouseCallback("Left Rectified", mouseClickHandle, &disp32f);
  // Initialize variables to store the maps for stereo rectification
  cv::Size imgSize(640, 480);
  cv::Mat R1, R2, P1, P2, Q;
  cv::Rect validPixROI1, validPixROI2;
  cv::Mat map1x, map1y;
  cv::Mat map2x, map2y;
  cv::stereoRectify(cameraMatrixLeft, disCoeffLeft,
                    cameraMatrixRight, disCoeffRight,
                    imgSize, R_mat, T_mat,
                    R1, R2, P1, P2, Q,
                    cv::CALIB_ZERO_DISPARITY,
                    0,
                    imgSize,
                    &validPixROI1, &validPixROI2);
  cv::initUndistortRectifyMap(cameraMatrixLeft, disCoeffLeft,
                              R1, P1, imgSize,
                              CV_32FC1, map2x, map2y);

  cv::initUndistortRectifyMap(cameraMatrixRight, disCoeffRight,
                            R2, P2, imgSize,
                            CV_32FC1, map1x, map1y);
  std::cout << "\n================ CAMERA LEFT =================\n";
  std::cout << "Camera Matrix Left:\n"
    << cameraMatrixLeft << std::endl;

  std::cout << "\nDistortion Coeff Left:\n"
    << disCoeffLeft << std::endl;

  std::cout << "\n================ CAMERA RIGHT =================\n";
  std::cout << "Camera Matrix Right:\n"
    << cameraMatrixRight << std::endl;

  std::cout << "\nDistortion Coeff Right:\n"
    << disCoeffRight << std::endl;

  std::cout << "\n================ EXTRINSIC =================\n";
  std::cout << "R_mat:\n"
    << R_mat << std::endl;

  std::cout << "\nT_mat:\n"
    << T_mat << std::endl;

  std::cout << "\n================ RECTIFICATION =================\n";
  std::cout << "R1:\n"
    << R1 << std::endl;

  std::cout << "\nR2:\n"
    << R2 << std::endl;

  std::cout << "\nP1:\n"
    << P1 << std::endl;

  std::cout << "\nP2:\n"
    << P2 << std::endl;

  std::cout << "\nQ:\n"
    << Q << std::endl;

  std::cout << "\nROI1: "
    << validPixROI1.x << ", "
    << validPixROI1.y << ", "
    << validPixROI1.width << ", "
    << validPixROI1.height
    << std::endl;

  std::cout << "ROI2: "
    << validPixROI2.x << ", "
    << validPixROI2.y << ", "
    << validPixROI2.width << ", "
    << validPixROI2.height
    << std::endl;
// Creating a named window to be linked to the trackbars
// =======================
    // Create StereoSGBM
    // =======================

  int actual_numDisparities = numDisparities * 16;
  int actual_blockSize = blockSize;


  stereo = cv::StereoSGBM::create(
      minDisparity,
      actual_numDisparities,
      actual_blockSize,
      8 * 3 * actual_blockSize * actual_blockSize,
      32 * 3 * actual_blockSize * actual_blockSize,
      disp12MaxDiff,
      31,
      uniquenessRatio,
      speckleWindowSize * 10,
      speckleRange,
      cv::StereoSGBM::MODE_SGBM_3WAY
  );

  // =======================
  // Trackbars
  // =======================

  cv::namedWindow("disparity", cv::WINDOW_NORMAL);

  cv::createTrackbar(
      "numDisparities x16",
      "disparity",
      &numDisparities,
      20,
      on_numDisparities);

  cv::createTrackbar(
      "blockSize",
      "disparity",
      &blockSize,
      21,
      on_blockSize);

  cv::createTrackbar(
      "minDisparity",
      "disparity",
      &minDisparity,
      100,
      on_minDisparity);

  cv::createTrackbar(
      "uniquenessRatio",
      "disparity",
      &uniquenessRatio,
      50,
      on_uniquenessRatio);

  cv::createTrackbar(
      "speckleWindow",
      "disparity",
      &speckleWindowSize,
      50,
      on_speckleWindowSize);

  cv::createTrackbar(
      "speckleRange",
      "disparity",
      &speckleRange,
      100,
      on_speckleRange);

  cv::createTrackbar(
      "disp12MaxDiff",
      "disparity",
      &disp12MaxDiff,
      25,
      on_disp12MaxDiff);


  cv::Mat disp, disparity;
  // vpPoseVector e_P_c;
  // if (!opt_eMc_filename.empty()) {
  //   e_P_c.loadYAML(opt_eMc_filename, e_P_c);
  // }
  // else {
  //   return EXIT_FAILURE;
  // }
  // vpHomogeneousMatrix e_M_c(e_P_c);

  // ========================
  // Display
  // ========================
  vpImage<unsigned char> I;
  vpImage<unsigned char> I_left;
  vpImage<unsigned char> I_right;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display_left;
  std::shared_ptr<vpDisplay> display_right;
#else
  vpDisplay *display_left = nullptr;
  vpDisplay *display_right = nullptr;
#endif

  while ((i++ < 60) && !cap.read(frame)) {
  } // warm up camera by skiping unread frames

  cap.read(frame);
  cv::flip(frame, frame, -1);
  vpImageConvert::convert(frame, I);

  display_left = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");

  vpDisplay::display(I);

  vpDisplay::flush(I);

//  // ========================
//  // Khởi tạo robot Denso
//  // ========================
//   vpRobotDenso6577 robot;
//   robot.init(); // param: redefine tool and camera extrinsic parameters for eMC
//   robot.set_eMc(e_M_c);

//   vpImagePoint ai_hint;

//   vpTRACE("sets the current position of the visual feature ");
//   vpFeaturePoint p;

//   vpTRACE("sets the desired position of the visual feature ");
//   vpFeaturePoint pd;
//   pd.buildFrom(0, 0, 1);
//   // ========================
//   // Visual servo task
//   // ========================
//   vpServo task;
//   vpVelocityTwistMatrix cVe;
//   vpMatrix eJe;
//   vpDot2 dot;
//   vpImagePoint cog_left, cog_right;
//   // ========================
//   // Biến điều khiển
//   // ========================
//   vpColVector q_cur(6), q_new(6);
//   const uint8_t *converged = (const uint8_t *)"OKE\r";
//   int state = PREINIT;
//   bool gripper_init = false;
//   bool pose_init = false;
//   bool sendClassified = false;
//   bool flushedGripper = false;
//   bool init_feature = false;
//   vpChrono chrono, chrene;
//   bool left_clicked = false;
//   bool right_clicked = false;
// Khởi tạo cấu trúc dữ liệu cho Callback chuột

  for (;;) {
    cap.read(frame);

    if (frame.empty())
      continue;

    cv::flip(frame, frame, -1);

    // =======================
    // Split stereo frame
    // =======================

    int single_width = frame.cols / 2;
    int height = frame.rows;

    cv::Mat frame_right =
      frame(cv::Rect(
        0,
        0,
        single_width,
        height));

    cv::Mat frame_left =
      frame(cv::Rect(
        single_width,
        0,
        single_width,
        height));

    cv::Mat gray_left;
    cv::Mat gray_right;

    cv::cvtColor(
        frame_left,
        gray_left,
        cv::COLOR_BGR2GRAY);

    cv::cvtColor(
        frame_right,
        gray_right,
        cv::COLOR_BGR2GRAY);

    // =======================
    // Rectification
    // =======================

    cv::Mat Left_nice;
    cv::Mat Right_nice;

    cv::remap(
        gray_left,
        Left_nice,
        map1x,
        map1y,
        cv::INTER_LINEAR);

    cv::remap(
        gray_right,
        Right_nice,
        map2x,
        map2y,
        cv::INTER_LINEAR);

    // =======================
    // Rectification debug
    // =======================

    cv::Mat rect_debug;

    cv::hconcat(
        Left_nice,
        Right_nice,
        rect_debug);

    for (int y = 0; y < rect_debug.rows; y += 40) {
      cv::line(
          rect_debug,
          cv::Point(0, y),
          cv::Point(rect_debug.cols, y),
          cv::Scalar(255),
          1);
    }

    cv::imshow(
        "Rectified Debug",
        rect_debug);

    // =======================
    // Update StereoSGBM
    // =======================

    int actual_numDisparities =
      std::max(
          16,
          numDisparities * 16);

    int actual_blockSize =
      std::max(
          3,
          blockSize);

    if (actual_blockSize % 2 == 0)
      actual_blockSize++;

    stereo->setMinDisparity(
        minDisparity);

    stereo->setNumDisparities(
        actual_numDisparities);

    stereo->setBlockSize(
        actual_blockSize);

    stereo->setP1(
        8 * 3 *
        actual_blockSize *
        actual_blockSize);

    stereo->setP2(
        32 * 3 *
        actual_blockSize *
        actual_blockSize);

    stereo->setDisp12MaxDiff(
        disp12MaxDiff);

    stereo->setUniquenessRatio(
        uniquenessRatio);

    stereo->setSpeckleWindowSize(
        speckleWindowSize * 10);

    stereo->setSpeckleRange(
        speckleRange);

    // =======================
    // Compute disparity
    // =======================

    cv::Mat disp16s;

    stereo->compute(
        Left_nice,
        Right_nice,
        disp16s);

    // =======================
    // Convert to float
    // =======================


    disp16s.convertTo(
        disp32f,
        CV_32F,
        1.0 / 16.0);

    // =======================
    // Invalid disparity mask
    // =======================

    cv::Mat validMask =
      disp32f > minDisparity;


  // =======================
  // Median filter
  // =======================

    cv::Mat disp_filtered;

    cv::medianBlur(
        disp32f,
        disp_filtered,
        5);

    // =======================
    // Zero invalid pixels
    // =======================

    disp_filtered.setTo(
        0,
        ~validMask);

    // =======================
    // Normalize
    // =======================

    cv::normalize(
        disp_filtered,
        disparity_visual,
        0,
        255,
        cv::NORM_MINMAX,
        CV_8U);

    // =======================
    // Color map
    // =======================

    cv::Mat disparity_color;

    cv::applyColorMap(
        disparity_visual,
        disparity_color,
        cv::COLORMAP_JET);

    // =======================
    // Display
    // =======================

    cv::imshow(
        "Left Rectified",
        Left_nice);

    cv::imshow(
        "Right Rectified",
        Right_nice);

    cv::imshow(
        "Disparity Gray",
        disparity_visual);

    // if (flag) {
    //   cv::Mat Z_mat(z_vec.size(), 1, CV_32F, z_vec.data());
    //   cv::Mat coeff(z_vec.size(), 2, CV_32F, coeff_vec.data());

    //   cv::Mat sol(2, 1, CV_32F);
    //   cv::solve(coeff, Z_mat, sol, cv::DECOMP_QR);

    //   M = sol.at<float>(0, 0);
    //   std::cout << "M: " << M << std::endl;
    //   std::cout << "Z_mat =" << std::endl;
    //   std::cout << Z_mat << std::endl;

    //   std::cout << "coeff =" << std::endl;
    //   std::cout << coeff << std::endl;
    // }
    if (cv::waitKey(1) == 27)
      break;
  // if (state == PREINIT) {
  //   q_new[0] = 0;
  //   q_new[1] = 0;
  //   q_new[2] = 90;
  //   q_new[3] = 0;
  //   q_new[4] = 90;
  //   q_new[5] = 0;

  //   robot.sendPosition(q_new.data);
  //   task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  //   task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

  //   vpTRACE("Set the position of the end-effector frame in the camera frame");

  //   robot.get_cVe(cVe);
  //   task.set_cVe(cVe);
  //   vpTRACE("Set the Jacobian (expressed in the end-effector frame)");

  //   robot.get_eJe(eJe);
  //   task.set_eJe(eJe);

  //   vpTRACE("\t set the gain");
  //   task.setLambda(0.4);

  //   vpTRACE("Display task information ");
  //   task.print();

  //   robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  //   state = INIT;
  // }
  // else if (state == INIT) {
  //   if (!gripper_init) {
  //     gripper_init = gripperOpen(gripper);
  //     if (gripper_init) {
  //       gripperStatus = OPENED;
  //     }
  //   }
  //   if (!pose_init) {
  //     robot.getPosition(vpRobot::ARTICULAR_FRAME, q_cur);
  //     q_cur.rad2deg();
  //     bool reached =
  //       std::abs(q_cur[0] - q_new[0])   < 0.01 &&
  //       std::abs(q_cur[1] - q_new[1])   < 0.01 &&
  //       std::abs(q_cur[2] - q_new[2])  < 0.01 &&
  //       std::abs(q_cur[3] - q_new[3])   < 0.01 &&
  //       std::abs(q_cur[4] - q_new[4])  < 0.01 &&
  //       std::abs(q_cur[5] - q_new[5])   < 0.01;
  //     if (reached) {
  //       try {
  //         // INIT: no hint — pick highest-confidence cylinder
  //         vpImagePoint ai_hint;
  //         if (detectCylinderWithAI(I, ai_hint)) {
  //           std::cout << "[AI] Cylinder detected at: " << ai_hint << std::endl;
  //           dot.initTracking(I, ai_hint);  // automatic init at AI-detected centre
  //         }
  //         else {
  //           std::cout << "[AI] Detection failed. Falling back to manual click." << std::endl;
  //           dot.initTracking(I);           // original manual-click fallback
  //         }
  //         cog = dot.getCog();

  //         vpDisplay::displayCross(I, cog, 10, vpColor::blue);
  //         vpDisplay::flush(I);

  //         vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure

  //         p.set_Z(1);

  //         if (!init_feature) {
  //           task.addFeature(p, pd);
  //           init_feature = true;
  //         }
  //         task.print();

  //         pose_init = true;

  //       }
  //       catch (const vpTrackingException &e) {
  //       }
  //     }
  //   }
  //   if (pose_init && gripper_init) {
  //     chrono.start(true);
  //     chrene.start(true);
  //     state = JOINT;
  //     q_cur = q_new.deg2rad();
  //   }
  // }
  // else if (state == JOINT) {
  //   robot.getPosition(vpRobot::ARTICULAR_FRAME, q_cur);
  //   bool reached =
  //     std::abs(q_cur[0] - q_new[0])   < 0.01 &&
  //     std::abs(q_cur[1] - q_new[1])   < 0.01 &&
  //     std::abs(q_cur[2] - q_new[2])  < 0.01 &&
  //     std::abs(q_cur[3] - q_new[3])   < 0.01 &&
  //     std::abs(q_cur[4] - q_new[4])  < 0.01 &&
  //     std::abs(q_cur[5] - q_new[5])   < 0.01;

  //   if (reached) {
  //     try {
  //       // JOINT: pass &ai_hint so Python picks the cylinder closest to
  //       // the last known position, not the highest-confidence one.
  //       // This prevents the servo from jumping to a different cylinder
  //       // when scores fluctuate as the robot moves.
  //       // vpImagePoint ai_hint;
  //       // if (detectCylinderWithAI(I, ai_hint)) {
  //       //   std::cout << "[AI] Cylinder detected at: " << ai_hint << std::endl;
  //       //   dot.initTracking(I, ai_hint);  // automatic init at AI-detected centre
  //       // }
  //       // else {
  //       //   std::cout << "[AI] Detection failed. Falling back to manual click." << std::endl;
  //       //   dot.initTracking(I);           // original manual-click fallback
  //       // }
  //       // cog = dot.getCog();
  //       dot.track(I);
  //       cog = dot.getCog();
  //     }
  //     catch (const vpTrackingException &e) {
  //       // sendClassified = false;
  //       // pose_init = false;
  //       // gripper_init = false;
  //       // task.kill();
  //       // state = PREINIT;
  //       // continue;
  //       vpImagePoint ai_hint;
  //       if (detectCylinderWithAI(I, ai_hint)) {
  //         std::cout << "[AI] Cylinder detected at: " << ai_hint << std::endl;
  //         dot.initTracking(I, ai_hint);  // automatic init at AI-detected centre
  //       }
  //       else {
  //         std::cout << "[AI] Detection failed. Falling back to manual click." << std::endl;
  //         dot.initTracking(I);           // original manual-click fallback
  //         continue;
  //       }
  //       cog = dot.getCog();

  //       vpDisplay::displayCross(I, cog, 10, vpColor::blue);
  //       vpDisplay::flush(I);

  //       vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure
  //     }
  //     // Display a green cross at the center of gravity position in the image
  //     // vpDisplay::displayCross(I, ai_hint, 10, vpColor::green);
  //     // vpFeatureBuilder::create(p, cam, ai_hint); // retrieve x,y and Z of the vpPoint structure
  //     vpDisplay::displayCross(I, cog, 10, vpColor::green);

  //     vpFeatureBuilder::create(p, cam, dot);
  //     robot.get_eJe(eJe);
  //     task.set_eJe(eJe);


  //     vpColVector v;
  //     vpColVector vel_max(6);
  //     double delta_t = 0.5; // 10 ms
  //     v = task.computeControlLaw();

  //     vpServoDisplay::display(task, cam, I);
  //   // if (getMaxRotationVelocity() == getMaxRotationVelocityJoint6()) {
  //     if (std::fabs(robot.getMaxRotationVelocity() - robot.getMaxRotationVelocityJoint6()) < std::numeric_limits<double>::epsilon()) {
  //       for (unsigned int i = 0; i < 6; i++)
  //         vel_max[i] = robot.getMaxRotationVelocity();
  //     }
  //     else {
  //       for (unsigned int i = 0; i < 5; i++)
  //         vel_max[i] = robot.getMaxRotationVelocity();
  //       vel_max[5] = robot.getMaxRotationVelocityJoint6();
  //     }
  //     v = vpRobot::saturateVelocities(v, vel_max, true);
  //     robot.getPosition(vpRobot::ARTICULAR_FRAME, q_cur);
  //     q_new = q_cur + v * delta_t;
  //     robot.setPosition(vpRobot::ARTICULAR_FRAME, q_new);

  //     if (abs(task.getError()[0]) < 5e-3 && abs(task.getError()[1]) < 5e-3) {
  //       chrene.stop();
  //       std::cout << " TIME CONVERGED:" << chrene.getDurationMs() << std::endl;
  //       task.print();
  //       robot.uartSend(converged, 4);
  //       state = APPROACH;
  //     }
  //     task.print();
  //   }
  //   vpDisplay::flush(I);
  // }
  // else if (state == APPROACH) {
  //   robot.getPosition(vpRobot::ARTICULAR_FRAME, q_cur);
  //   q_cur.rad2deg();
  //   if (q_cur.data[0] == -1 && q_cur.data[1] == -1 && q_cur.data[2] == -1 && q_cur.data[3] == -1 && q_cur.data[4] == -1 && q_cur.data[5] == -1) {
  //     state = GRIPPER;
  //   }
  // }
  // else if (state == GRIPPER) {
  //   if (gripperClose(gripper)) {
  //     state = CLASSIFIED;
  //   }
  // }
  // else if (state == CLASSIFIED) {
  //   // gripperClose(gripper);
  //   // SEND oke to RC5 controller
  //   //
  //   // robot.uartSend(converged, 4);
  //   if (!sendClassified) {
  //     q_new[0] = 53.35;
  //     q_new[1] = 25.15;
  //     q_new[2] = 91.58;
  //     q_new[3] = 0;
  //     q_new[4] = 63.27;
  //     q_new[5] = 53.35;

  //     robot.sendPosition(q_new.data);
  //     sendClassified = true;
  //   }
  //   // gripperClose(gripper);
  //   robot.flush();
  //   robot.getPosition(vpRobot::ARTICULAR_FRAME, q_cur);
  //   q_cur.rad2deg();
  //   bool reached =
  //     std::abs(q_cur[0] - q_new[0])   < 0.01 &&
  //     std::abs(q_cur[1] - q_new[1])   < 0.01 &&
  //     std::abs(q_cur[2] - q_new[2])  < 0.01 &&
  //     std::abs(q_cur[3] - q_new[3])   < 0.01 &&
  //     std::abs(q_cur[4] - q_new[4])  < 0.01 &&
  //     std::abs(q_cur[5] - q_new[5])   < 0.01;

  //   if (reached) {
  //     gripperFlush(gripper);
  //     gripperOpen(gripper);
  //     vpTime::wait(500);
  //     sendClassified = false;
  //     pose_init = false;
  //     gripper_init = false;
  //     std::cout << "###########################################################################" <<std::endl;
  //     chrono.stop();
  //     std::cout << "COMPLETE ONE " << chrono.getDurationMs() << std::endl;
  //     state = PREINIT;
  //   }
  // }
    vpDisplay::flush(I);
  }
  return EXIT_SUCCESS;
}
