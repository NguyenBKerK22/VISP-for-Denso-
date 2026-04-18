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
 * Universal Robots (UR5 or UR10) data acquisition to prepare hand-eye calibration.
 */

//! \example visp-acquire-universal-robots-calib-data.cpp
#include <iostream>
#include <termios.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/robot/vpRobotDenso.h>
#include <visp3/io/vpImageStorageWorker.h>

#if defined(VISP_HAVE_V4L2) && \
    defined(VISP_HAVE_DISPLAY) && defined(VISP_HAVE_PUGIXML) && \
    defined(VISP_HAVE_MODULE_GUI) && defined(VISP_HAVE_MODULE_ROBOT) && defined(VISP_HAVE_MODULE_SENSOR) // optional
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

int openSerial(const std::string &port)
{
  int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    perror("open");
    return -1;
  }

  struct termios tty;
  tcgetattr(fd, &tty);

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;        // 8-bit
  tty.c_cflag &= ~PARENB;    // no parity
  tty.c_cflag &= ~CSTOPB;    // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;   // no flow control

  tty.c_lflag = 0;           // raw mode
  tty.c_oflag = 0;
  tty.c_iflag = 0;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10; // timeout = 1s

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

// Gửi lệnh
void sendCommand(int fd, const std::string &cmd)
{
  std::string data = cmd + "\r";
  write(fd, data.c_str(), data.size());
}

// Đọc phản hồi
std::string readResponse(int fd)
{
  char buffer[256];
  std::string result;

  while (true) {
    int n = read(fd, buffer, sizeof(buffer));
    if (n > 0) {
      result.append(buffer, n);
      if (result.find('\r') != std::string::npos) break;
    }
    else {
      break;
    }
  }
  return result;
}

// Parse string → vector double
std::vector<double> parseJoints(const std::string &data)
{
  std::vector<double> joints;
  std::stringstream ss(data);
  std::string item;

  while (std::getline(ss, item, ',')) {
    try {
      joints.push_back(std::stod(item));
    }
    catch (...) { }
  }
  return joints;
}
// vpPoseVector generateHemispherePoses(double R,
//                                           int n_theta,
//                                           int n_phi, double x_offset = 0.5, double y_offset = 0.5, double z_offset = 0.5)
// {
//   vpPoseVector poses;

//   for (int i = 0; i < n_theta; i++) {
//     double theta = (M_PI / 2.0) * i / (n_theta - 1);  // 0 → pi/2
//     for (int j = 0; j < n_phi; j++) {
//       double phi = (2 * M_PI) * j / n_phi;  // 0 → 2pi

//       // ===== Position =====
//       double x = R * sin(theta) * cos(phi) + x_offset; // offset x by x_offset to avoid singularity at origin
//       double y = R * sin(theta) * sin(phi) + y_offset; // offset y by y_offset to avoid singularity at origin
//       double z = R * cos(theta) + z_offset; // offset z by z_offset to avoid singularity at origin

//       // ===== Orientation: look at origin =====
//       vpColVector z_axis(3);
//       z_axis[0] = -x;
//       z_axis[1] = -y;
//       z_axis[2] = -z;
//       z_axis.normalize();

//       // giả sử up vector là (0,0,1)
//       vpColVector up(3);
//       up[0] = 0; up[1] = 0; up[2] = 1;

//       // x_axis = up × z_axis
//       vpColVector x_axis = vpColVector::crossProd(up, z_axis);
//       x_axis.normalize();

//       // y_axis = z × x
//       vpColVector y_axis = vpColVector::crossProd(z_axis, x_axis);

//       // ===== Rotation matrix =====
//       vpRotationMatrix Rmat;
//       for (int k = 0; k < 3; k++) {
//         Rmat[k][0] = x_axis[k];
//         Rmat[k][1] = y_axis[k];
//         Rmat[k][2] = z_axis[k];
//       }

//       // ===== Convert to rx, ry, rz =====
//       vpRxyzVector rxyz(Rmat);

//       Pose p;
//       p.x = x;
//       p.y = y;
//       p.z = z;
//       p.rx = rxyz[0];
//       p.ry = rxyz[1];
//       p.rz = rxyz[2];

//       poses.push_back(p);
//     }
//   }
//   return poses;
// }
void usage(const char **argv, int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--device <number>] [--output-folder <name>] [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  --device <number>" << std::endl
    << "    Video device number." << std::endl
    << "    Default: 0" << std::endl
    << std::endl
    << "  --output-folder <name>" << std::endl
    << "    Acquired data output folder." << std::endl
    << "    Default: ./" << std::endl
    << std::endl
    << "  --help, -h  Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char **argv)
{
#if defined(ENABLE_VISP_NAMESPACE)
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  vpDisplay *pdisp = nullptr;
#endif
  try {
    bool opt_display = true;
    int opt_device = 0;
    std::string opt_output_folder = ".";
    std::string file_pose = "";
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device" && i + 1 < argc) {
        opt_device = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--output-folder" && i + 1 < argc) {
        opt_output_folder = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--pose" && i + 1 < argc) {
        file_pose = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    // Create output folder if necessary
    if (!vpIoTools::checkDirectory(opt_output_folder)) {
      std::cout << "Create output directory: " << opt_output_folder << std::endl;
      vpIoTools::makeDirectory(opt_output_folder);
    }

    // vpImage<unsigned char> I;

    // vpRobotUniversalRobots robot; Robot from Universal Robots (UR5 or UR10)

    // vpRealSense2 g;
    // rs2::config config;
    // config.disable_stream(RS2_STREAM_DEPTH);
    // config.disable_stream(RS2_STREAM_INFRARED);
    // config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    // g.open(config);
    // g.acquire(I);


    cv::VideoCapture cap(opt_device, cv::CAP_V4L2); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    int i = 0;
    while ((i++ < 20) && !cap.read(frame)) {
    } // warm up camera by skiping unread frames

    std::cout << "Image size : " << frame.rows << " " << frame.cols << std::endl;
#ifdef USE_COLOR
    vpImage<vpRGBa> I; // To acquire color images
#else
    vpImage<unsigned char> I; // To acquire gray images
#endif
    vpImageConvert::convert(frame, I);


#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::shared_ptr<vpDisplay> pdisp = vpDisplayFactory::createDisplay(I, 10, 10, "Color image");
#else
    pdisp = vpDisplayFactory::allocateDisplay(I, 10, 10, "Color image");
#endif
    vpRobotDenso6577 robot;
    unsigned cpt = 0;
    double d2r = M_PI / 180.0;
    robot.init();
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    while (true) {
      bool end = false;
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);
      vpDisplay::display(I);
      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;

      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt++;
          double *q = new double[6];

          vpPoseVector rPe;
          rPe.buildFrom(q[0], q[1], q[2], q[3], q[4], q[5]);
          std::stringstream ss_img, ss_pos;
          ss_img << opt_output_folder + "/ur_image-" << cpt << ".png";
          ss_pos << opt_output_folder + "/ur_pose_rPe_" << cpt << ".yaml";
          std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
          vpImageIo::write(I, ss_img.str());
          rPe.saveYAML(ss_pos.str(), rPe);
          end = true;
          delete q;
        }
      }
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cerr << "ViSP exception " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (pdisp != nullptr) {
    delete pdisp;
  }
#endif
  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_MODULE_GUI)
  std::cout << "visp_gui module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_MODULE_SENSOR)
  std::cout << "visp_sensor module is not available?" << std::endl;
#endif
#if !defined(VISP_HAVE_UR_RTDE)
  std::cout << "ViSP is not build with libur_rtde 3rd party used to control a robot from Universal Robots..."
    << std::endl;
#endif
#if !defined(VISP_HAVE_PUGIXML)
  std::cout << "Enable pugyxml built-in usage." << std::endl;
#endif

  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake"
    << " and build ViSP again." << std::endl;
  return EXIT_SUCCESS;
}
#endif
