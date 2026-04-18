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
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>

<<<<<<< HEAD

#define HAVE_OPENCV_FEATURES
int main()
=======
int main(int argc, char *argv[])
>>>>>>> 2c3823b (Best calibration result - move oke)
{
// Parameter definition
  int opt_device = 2;
  bool opt_display = true;
  std::string opt_camera_name = "Camera";
  std::string opt_intrinsic_file = "camera.xml";
  std::string opt_eMc_filename = "rc5_ePc.yaml";
// Initialize camera and display
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device" && i + 1 < argc) {
      opt_device = std::atoi(argv[++i]);
    }
  }
  cv::VideoCapture cap(opt_device, cv::CAP_V4L2); // open the default camera
  cv::Mat frame;
  try {
    /* code */
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    int i = 0;
    while ((i++ < 20) && !cap.read(frame)) {
    } // warm up camera by skiping unread frames
    std::cout << "Image size : " << frame.rows << " " << frame.cols << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  // Get camera intrinsics
  vpCameraParameters cam;
  vpXmlParserCamera parser;
  try {
    /* code */
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

  }
  catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  // Initialize display
  vpImage<unsigned char> I;
  vpImageConvert::convert(frame, I);
  std::shared_ptr<vpDisplay> display = nullptr;
  vpDisplayOpenCV *d = nullptr;
  try {
    /* code */
    if (opt_display) {
      d = new vpDisplayOpenCV(I);
    }
    display = vpDisplayFactory::createDisplay(I, 10, 10, "Current image");
    vpDisplay::display(I);
    vpDisplay::flush(I);

  }
  catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  // Initialize tracker
  vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
  std::string videoname = "teabox.mp4";
  std::string objectname = vpIoTools::getNameWE(videoname);
  bool usexml = false;
  try {
    if (!usexml) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
      me.setThreshold(20);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      tracker.setCameraParameters(cam);
      tracker.setAngleAppear(vpMath::rad(70));
      tracker.setAngleDisappear(vpMath::rad(80));
      tracker.setNearClippingDistance(0.1);
      tracker.setFarClippingDistance(100.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    }
    tracker.setOgreVisibilityTest(false);
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);
    tracker.initClick(I, objectname + ".init", true);
    tracker.track(I);
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
  }
  //! [Keypoint selection]
#if ((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_XFEATURES2D)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES))
  std::string detectorName = "SIFT";
  std::string extractorName = "SIFT";
  std::string matcherName = "BruteForce";
  std::string configurationFile = "detection-config-SIFT.xml";
#elif ((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_FEATURES2D)) || ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_FEATURES))
  std::string detectorName = "FAST";
  std::string extractorName = "ORB";
  std::string matcherName = "BruteForce-Hamming";
  std::string configurationFile = "detection-config.xml";
#endif

      //! [Keypoint selection]
    //! [Keypoint declaration]
  vpKeyPoint keypoint_learning;
  //! [Keypoint declaration]

  //! [Keypoint code config]
  keypoint_learning.setDetector(detectorName);
  keypoint_learning.setExtractor(extractorName);
  keypoint_learning.setMatcher(matcherName);
  //! [Keypoint code config]

  //! [Keypoints reference detection]
  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
  //! [Keypoints reference detection]

  //! [Keypoints selection on faces]
  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
  polygons = pair.first;
  roisPt = pair.second;

  vpHomogeneousMatrix cdMo; // desired pose of the camera in the object frame
  vpHomogeneousMatrix cMo; // current pose of the camera in the object frame

  std::vector<cv::Point3f> points3f;
  tracker.getPose(cMo);
  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
  //! [Keypoints selection on faces]

  //! [Keypoints build reference]
  keypoint_learning.buildReference(I, trainKeyPoints, points3f);
  //! [Keypoints build reference]

  //! [Save learning data]
  keypoint_learning.saveLearningData("teabox_learning_data.bin", true);
  //! [Save learning data]

  //! [Display reference keypoints]

  //! [Init keypoint detection]
  vpKeyPoint keypoint_detection;
  if (usexml) {
    keypoint_detection.loadConfigFile(configurationFile);
  }
  else {
    keypoint_detection.setDetector(detectorName);
    keypoint_detection.setExtractor(extractorName);
    keypoint_detection.setMatcher(matcherName);
    keypoint_detection.setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);
    keypoint_detection.setMatchingRatioThreshold(0.8);
    keypoint_detection.setUseRansacVVS(true);
    keypoint_detection.setUseRansacConsensusPercentage(true);
    keypoint_detection.setRansacConsensusPercentage(20.0);
    keypoint_detection.setRansacIteration(200);
    keypoint_detection.setRansacThreshold(0.005);
  }
  //! [Init keypoint detection]

  //! [Load teabox learning data]
  keypoint_detection.loadLearningData("teabox_learning_data.bin", true);

  vpPoseVector e_P_c;
  if (!opt_eMc_filename.empty()) {
    e_P_c.loadYAML(opt_eMc_filename, e_P_c);
  }
  else {
    std::cout << "Warning, opt_eMc_filename is empty! Use hard coded values." << std::endl;
  }
  vpHomogeneousMatrix e_M_c(e_P_c);

  vpRobotDenso6577 robot;
  robot.init(); // param: redefine tool and camera extrinsic parameters for eMC
  robot.set_eMc(e_M_c);

  vpDisplay::flush(I);

  vpFeatureTranslation s_translation; // Hiện tại
  vpFeatureTranslation sd_translation; // Mong muốn

  vpFeatureThetaU s_theta; // Hiện tại (Rotation Current to Desired)
  vpFeatureThetaU sd_theta; // Mong muốn (mặc định sẽ là 0)

  // --- Thiết lập vị trí đích (cdMo) ---
  // Giả sử bạn muốn camera cách vật thể 0.5m theo trục Z và nhìn thẳng vào nó

  // Xây dựng các đặc trưng mong muốn từ cdMo
  sd_translation.buildFrom(cdMo);
  sd_theta.buildFrom(cdMo); // s* của ThetaU thường là ma trận đơn vị (sai lệch bằng 0)
  std::cout << "hehe" << std::endl;
  vpServo task;
  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE);
    // Creation of the current feature s that correspond to the rotation
  // in angle/axis parametrization between the current camera frame
  // and the desired camera frame

  task.setLambda(0.5);

  // Thêm các đặc trưng vào task
  task.addFeature(s_translation, sd_translation);
  task.addFeature(s_theta, sd_theta);



  vpTRACE("Display task information ");
  task.print();

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  vpColVector q;
  std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;

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

<<<<<<< HEAD
  double error;
  std::cout << "\nHit CTRL-C to stop the loop...\n" << std::endl;
=======
  vpTRACE("\t we want to see a point on a point..");
  std::cout << std::endl;
  task.addFeature(p, pd);

  vpTRACE("\t set the gain");
  task.setLambda(0.8);

  vpTRACE("Display task information ");
  task.print();
  std::cout << "px:" << p.get_x() << ", py:" << p.get_y() << ", pz:" << p.get_Z() << std::endl;
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  vpColVector q;
  vpHomogeneousMatrix fMe;
  std::cout << "\nHit CTRL-C to stop the loop...\n" << std::flush;
>>>>>>> 2c3823b (Best calibration result - move oke)
  for (;;) {
    cap >> frame; // get a new frame from camera
    // Convert the image in ViSP format and display it
    vpImageConvert::convert(frame, I);

    vpDisplay::display(I);

    // Achieve the tracking of the dot in the image
    if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
      try {
          // 1. Cập nhật Tracker với Pose từ Keypoint
        tracker.setPose(I, cMo);

        // 2. Thực hiện tracking dựa trên cạnh (Edge-based tracking)
        // Bước này cực kỳ quan trọng để tăng độ chính xác trước khi Servo
        tracker.track(I);
        tracker.getPose(cMo); // Lấy Pose đã được tối ưu

        // 3. Hiển thị mô hình để debug
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);

        // 4. Cập nhật đặc trưng Visual Servoing (s)
        s_translation.buildFrom(cMo);
        s_theta.buildFrom(cMo);

        // 5. Tính toán ma trận Jacobian và Vận tốc
        // Lấy ma trận vận tốc từ Camera sang End-effector (nếu robot di động)
        // vphmcl::get_cVe(robot, cVe);

        vpMatrix eJe;
        robot.get_eJe(eJe); // Lấy Jacobian của robot
        task.set_eJe(eJe);

        // Tính toán luật điều khiển (Control Law)
        // v = -lambda * L^+ * (s - s*)
        vpColVector qdot = task.computeControlLaw();

        // 7. Gửi lệnh điều khiển đến robot
        robot.setVelocity(vpRobot::ARTICULAR_FRAME, qdot);

<<<<<<< HEAD
      }
      catch (const vpException &e) {
        std::cout << "Tracking lost or Matrix error: " << e.getMessage() << std::endl;
        robot.stopMotion(); // Dừng robot nếu mất dấu
      }
    }
    else {
      robot.stopMotion(); // Dừng robot nếu Keypoint không khớp
    }
  // robot.get_eJe(eJe);
  //   // Update this jacobian in the task structure. It will be used to
  //   // compute the velocity skew (as an articular velocity) qdot = -lambda *
  //   // L^+ * cVe * eJe * (s-s*)
  // task.set_eJe(eJe);

  // //  std::cout << (vpMatrix)cVe*eJe << std::endl ;

  // vpColVector v;
  // // Compute the visual servoing skew vector
  // v = task.computeControlLaw();

  // // Display the current and desired feature points in the image display
  // vpServoDisplay::display(task, cam, I);

  // // Apply the computed joint velocities to the robot
  // robot.setVelocity(vpRobot::ARTICULAR_FRAME, v);

  // // Get the measured joint positions of the robot
  // robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
  // // Save measured joint positions of the robot in the log file
  // // - q[0], q[1], q[2] correspond to measured joint translation
  // //   positions in m
  // // - q[3], q[4], q[5] correspond to measured joint rotation
  // //   positions in rad

  // // Save feature error (s-s*) for the feature point. For this feature
  // // point, we have 2 errors (along x and y axis).  This error is
  // // expressed in meters in the camera frame

=======
    // Get the measured joint positions of the robot
    robot.getPosition(vpRobot::ARTICULAR_FRAME, q);
    robot.get_fMe(q, fMe);
    // Save measured joint positions of the robot in the log file
    // - q[0], q[1], q[2] correspond to measured joint translation
    //   positions in m
    // - q[3], q[4], q[5] correspond to measured joint rotation
    //   positions in rad

    // Save feature error (s-s*) for the feature point. For this feature
    // point, we have 2 errors (along x and y axis).  This error is
    // expressed in meters in the camera frame
    task.print();
>>>>>>> 2c3823b (Best calibration result - move oke)
    vpDisplay::flush(I);
    // // std::cout << "|| s - s* || = "  << ( task.getError() ).sumSquare() <<
    // // std::endl;
    // vpTime::wait(200); // ~100 Hz (mượt hơn)
  }

  // std::cout << "Display task information: " << std::endl;
  // task.print();
  return EXIT_SUCCESS;
}
