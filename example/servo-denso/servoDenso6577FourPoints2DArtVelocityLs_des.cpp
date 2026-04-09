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

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

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
#include <visp3/core/vpImageConvert.h>

int main()
{
// Parameter definition
  int opt_device = 2;
  bool opt_display = true;
  std::string opt_camera_name = "Camera";
  std::string opt_intrinsic_file = "camera.xml";
  std::string opt_eMc_filename = "rc5_ePc.yaml";
// Initialize camera and display
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

// Get camera intrinsics
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

// Get camera extrinsics
  vpPoseVector e_P_c;
  if (!opt_eMc_filename.empty()) {
    e_P_c.loadYAML(opt_eMc_filename, e_P_c);
  }
  else {
    std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << std::endl;
  }
  vpHomogeneousMatrix e_M_c(e_P_c);
  std::cout << "e_M_c:\n" << e_M_c << std::endl;
// Initialize display and camera parameters
  vpImage<unsigned char> I;

  vpImageConvert::convert(frame, I);
  vpDisplayOpenCV *d = nullptr;
  if (opt_display) {
    d = new vpDisplayOpenCV(I);
  }
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");

  vpDisplay::display(I);
  vpDisplay::flush(I);

  vpRobotDenso6577 robot;
  robot.init(); // param: redefine tool and camera extrinsic parameters for eMC
  robot.set_eMc(e_M_c);

  vpDot2 dot;
  std::cout << "Click on a dot..." << std::endl;
  dot.initTracking(I);
  vpImagePoint cog = dot.getCog();
  vpDisplay::displayCross(I, cog, 10, vpColor::blue);
  vpDisplay::flush(I);

  vpTRACE("sets the current position of the visual feature ");
  vpFeaturePoint p;
  vpFeatureBuilder::create(p, cam, dot); // retrieve x,y and Z of the vpPoint structure

  p.set_Z(1);
  vpTRACE("sets the desired position of the visual feature ");
  vpFeaturePoint pd;
  pd.buildFrom(0, 0, 1);

  vpServo task;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);

  vpTRACE("Set the position of the end-effector frame in the camera frame");

  vpVelocityTwistMatrix cVe;
  robot.get_cVe(cVe);
  std::cout << cVe << std::endl;
  task.set_cVe(cVe);

  //    vpDisplay::getClick(I) ;
  vpTRACE("Set the Jacobian (expressed in the end-effector frame)");
  vpMatrix eJe;
  robot.get_eJe(eJe);
  task.set_eJe(eJe);

  vpTRACE("\t we want to see a point on a point..");
  std::cout << std::endl;
  task.addFeature(p, pd);

  vpTRACE("\t set the gain");
  task.setLambda(0.8);

  vpTRACE("Display task information ");
  task.print();

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  vpColVector q;
  std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
  for (;;) {
    cap >> frame; // get a new frame from camera
    // Convert the image in ViSP format and display it
    vpImageConvert::convert(frame, I);

    vpDisplay::display(I);

    // Achieve the tracking of the dot in the image
    dot.track(I);
    cog = dot.getCog();

    // Display a green cross at the center of gravity position in the image
    vpDisplay::displayCross(I, cog, 10, vpColor::green);

    // Update the point feature from the dot location
    vpFeatureBuilder::create(p, cam, dot);

    // Get the jacobian of the robot
    robot.get_eJe(eJe);
    // Update this jacobian in the task structure. It will be used to
    // compute the velocity skew (as an articular velocity) qdot = -lambda *
    // L^+ * cVe * eJe * (s-s*)
    task.set_eJe(eJe);

    //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

    vpColVector v;
    // Compute the visual servoing skew vector
    v = task.computeControlLaw();

    // Display the current and desired feature points in the image display
    vpServoDisplay::display(task, cam, I);

    // Apply the computed joint velocities to the robot
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

    // Get the measured joint positions of the robot
    robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
    // Save measured joint positions of the robot in the log file
    // - q[0], q[1], q[2] correspond to measured joint translation
    //   positions in m
    // - q[3], q[4], q[5] correspond to measured joint rotation
    //   positions in rad

    // Save feature error (s-s*) for the feature point. For this feature
    // point, we have 2 errors (along x and y axis).  This error is
    // expressed in meters in the camera frame

    vpDisplay::flush(I);

    // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() <<
    // std::endl;
    vpTime::wait(200); // ~100 Hz (mượt hơn)
  }

  std::cout << "Display task information: " << std::endl;
  task.print();
  return EXIT_SUCCESS;
}
