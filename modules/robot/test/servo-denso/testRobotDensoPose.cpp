/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test for Viper650 6 dof robot.
 */

/*!
  \example testRobotViper850Pose.cpp

  Example of robot pose usage.

  Show how to compute rMo = rMc * cMo with cMo obtained by pose computation
  and rMc from the robot position.

*/

#include <iostream>
#include <visp3/blob/vpDot.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotDenso.h>
#include <visp3/vision/vpPose.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <opencv2/videoio.hpp>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpXmlParserCamera.h>
int main(int argc, const char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    int opt_device = 0;
    bool opt_display = true;
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device" && i + 1 < argc) {
        opt_device = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--no-display" && i + 1 < argc) {
        opt_display = false;
      }
      else {
        std::cout << "Usage: " << argv[0] << " [--device <number>] [--no-display]" << std::endl;
        return EXIT_FAILURE;
      }
    }
//#######################################################################
// Initialize camera and display
    vpImage<unsigned char> I;
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
//#######################################################################
//#######################################################################
// Initialize display and camera parameters
    vpImageConvert::convert(frame, I);
    vpDisplayOpenCV *d = nullptr;
    if (opt_display) {
      d = new vpDisplayOpenCV(I);
    }
    std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");
//#######################################################################

    vpDisplay::display(I);
    vpDisplay::flush(I);
    // Define a squared target
    // The target is made of 4 planar points (square dim = 0.077m)
    double sdim = 0.077; // square width and height
    vpPoint target[4];
    // Set the point world coordinates (x,y,z) in the object frame
    // o ----> x
    // |
    // |
    // \/
    // y
    target[0].setWorldCoordinates(-sdim / 2., -sdim / 2., 0);
    target[1].setWorldCoordinates(sdim / 2., -sdim / 2., 0);
    target[2].setWorldCoordinates(sdim / 2., sdim / 2., 0);
    target[3].setWorldCoordinates(-sdim / 2., sdim / 2., 0);

    // Image processing to extract the 2D coordinates in sub-pixels of the 4
    // points from the image acquired by the camera
    // Creation of 4 trackers
    vpDot dot[4];
    vpImagePoint cog;
    for (int i = 0; i < 4; i++) {
      dot[i].setGraphics(true); // to display the tracking results
      std::cout << "Click on dot " << i << std::endl;
      dot[i].initTracking(I);
      // The tracker computes the sub-pixels coordinates in the image
      // i ----> u
      // |
      // |
      // \/
      // v
      std::cout << "  Coordinates: " << dot[i].getCog() << std::endl;
      // Flush the tracking results in the viewer
      vpDisplay::flush(I);
    }

    // Create an intrinsic camera parameters structure
    std::string opt_camera_name = "Camera";
    std::string opt_intrinsic_file = "test.xml";
    vpCameraParameters cam;
    vpXmlParserCamera parser;
    if (!vpIoTools::checkFilename(opt_intrinsic_file)) {
      std::cout << "Camera parameters file " << opt_intrinsic_file << " doesn't exist." << std::endl;
      std::cout << "Use --help option to see how to set its location..." << std::endl;
      return EXIT_FAILURE;
    }
    if (parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) !=
      vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Unable to parse parameters with distortion for camera \"" << opt_camera_name << "\" from "
        << opt_intrinsic_file << " file" << std::endl;
      std::cout << "Attempt to find parameters without distortion" << std::endl;

      if (parser.parse(cam, opt_intrinsic_file, opt_camera_name,
                       vpCameraParameters::perspectiveProjWithoutDistortion) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Unable to parse parameters without distortion for camera \"" << opt_camera_name << "\" from "
          << opt_intrinsic_file << " file" << std::endl;
        return EXIT_FAILURE;
      }
    }

    // Create a robot access
    vpRobotDenso6577 robot;

    // Create a string filename for the extrinsic camera parameters
    std::string filename = "./visp-ws/visp-build/apps/calibration/hand-eye/rc5_ePc.yaml";
    // Load the end-effector to camera frame transformation obtained
    // using a camera intrinsic model with distortion

    // Get the intrinsic camera parameters associated to the image
    // Using the camera parameters, compute the perspective projection
    // (transform the dot sub-pixel coordinates into coordinates in the camera
    // frame in meter)
    for (int i = 0; i < 4; i++) {
      double x = 0, y = 0; // coordinates of the dots in the camera frame
      // c ----> x
      // |
      // |
      // \/
      // y
      // pixel to meter conversion
      cog = dot[i].getCog();
      vpPixelMeterConversion::convertPoint(cam, cog, x, y);
      target[i].set_x(x);
      target[i].set_y(y);
    }

    // From now, in target[i], we have the 3D coordinates of a point in the
    // object frame, and their correspondences in the camera frame. We can now
    // compute the pose cMo between the camera and the object.
    vpPose pose;
    // Add the 4 points to compute the pose
    for (int i = 0; i < 4; i++) {
      pose.addPoint(target[i]);
    }
    // Create an homogeneous matrix for the camera to object transformation
    // computed just bellow
    vpHomogeneousMatrix cMo;
    vpRotationMatrix R;
    vpRxyzVector r;
    // Compute the pose: initialisation is done by Dementhon or Lagrange method, and the
    // final pose is computed by the more accurate Virtual Visual Servoing method.
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo);

    std::cout << "Pose cMo: " << std::endl << cMo;
    cMo.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
      << std::endl
      << std::endl;

// Get the robot position in the reference frame
    vpHomogeneousMatrix rMc;
    vpColVector p; // position x,y,z,rx,ry,rz
    robot.getPosition(vpRobot::REFERENCE_FRAME, p);
    std::cout << "Robot pose in reference frame: " << p << std::endl;
    vpTranslationVector t;
    t[0] = p[0];
    t[1] = p[1];
    t[2] = p[2];
    r[0] = p[3];
    r[1] = p[4];
    r[2] = p[5];
    R.buildFrom(r);
    rMc.buildFrom(t, R);
    std::cout << "Pose rMc: " << std::endl << rMc;
    rMc.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
      << std::endl
      << std::endl;

    robot.getPosition(vpRobot::ARTICULAR_FRAME, p);
    std::cout << "Robot pose in articular: " << p << std::endl;

    robot.get_fMc(p, rMc);
    std::cout << "Pose rMc from MGD: " << std::endl << rMc;
    rMc.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
      << std::endl
      << std::endl;

    vpHomogeneousMatrix rMo;
    rMo = rMc * cMo;
    std::cout << "Pose rMo = rMc * cMo: " << std::endl << rMo;
    rMo.extract(R);
    r.buildFrom(R);
    std::cout << "  rotation: " << vpMath::deg(r[0]) << " " << vpMath::deg(r[1]) << " " << vpMath::deg(r[2]) << " deg"
      << std::endl
      << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
