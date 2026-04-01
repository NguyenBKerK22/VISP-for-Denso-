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
 * Interface for the Irisa's Viper S850 robot.
 */

#include <visp3/core/vpConfig.h>


#include <signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotDenso.h>

BEGIN_VISP_NAMESPACE
/* ---------------------------------------------------------------------- */
/* --- STATIC ----------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

bool vpRobotDenso6577::m_robotAlreadyCreated = false;

/*!

  Default positioning velocity in percentage of the maximum
  velocity. This value is set to 15. The member function
  setPositioningVelocity() allows to change this value.

*/
const double vpRobotDenso6577::m_defaultPositioningVelocity = 15.0;

/* ---------------------------------------------------------------------- */
/* --- EMERGENCY STOP --------------------------------------------------- */
/* ---------------------------------------------------------------------- */
bool parseData(char *buffer, double *q, int func)
{
   // 2. Parse dữ liệu: "x,x,x,x,x,x"
  char *token = strtok(buffer, ",");
  int i = 0;
  switch (func) {
  case 0:
    std::cout << "Received position: " << buffer << std::endl;
    while (token != NULL) {
      if (i >= 6) return true;

      q[i++] = atof(token);
      token = strtok(NULL, ",");
    }
    break;
  case 1:
    std::cout << "Received joint positions: " << buffer << std::endl;
    while (token != NULL) {
      if (i >= 6) return false;

      q[i++] = atof(token);
      token = strtok(NULL, ",");
    }
    std::cout << "Number of joints read: " << i << std::endl;
    break;
  default:
    std::cerr << "Unknown function type for parsing data" << std::endl;
    return false;
  }
  return true;
}
int vpRobotDenso6577::uartSend(const uint8_t *data, size_t len)
{
  return this->serial.writeBytes(data, len);
}
bool vpRobotDenso6577::readUARTJoint(double *q, int timeout_ms, int func)
{
  char buffer[128];
  int idx = 0;

  char c;
  int ret;

  timeOut timer;
  timer.initTimer();

  // 1. Đọc từng byte cho tới '\r'
  while (true) {
      // đọc 1 byte (timeout nhỏ để loop)
    ret = serial.readChar(&c, 10);

    if (ret == 1)  // đọc được 1 byte
    {
      if (c == '\r') {
        buffer[idx] = '\0'; // kết thúc chuỗi
        break;
      }

      if (idx < (int)sizeof(buffer) - 1) {
        buffer[idx++] = c;
      }
      else {
          // overflow buffer
        return false;
      }
    }

    // kiểm tra timeout tổng
    if (timer.elapsedTime_ms() > (unsigned long)timeout_ms) {
      return false;
    }
  }
  return parseData(buffer, q, func);
}
bool vpRobotDenso6577::getJointPosition(double *q, int func)
{
  const char *buffer = nullptr;
  if (func == 1) {
    buffer = "GET_JOINT\r";
  }
  else {
    buffer = "GET_POS\r";
  }
  int ret_val = this->uartSend((const uint8_t *)buffer, strlen(buffer));
  if (ret_val < 0) {
    std::cerr << "Error sending command to get joint position" << std::endl;
    return false;
  }
  bool success = readUARTJoint(q);
  if (!success) {
    std::cerr << "Error reading joint position from UART" << std::endl;
    return false;
  }
  return true;
}
bool vpRobotDenso6577::sendPosition(double *q)
{
  char buffer[256];
  int positionRounded[6];
  for (int i = 0; i < 6; i++) {
    positionRounded[i] = (int)round(q[i] * 100);
  }
  snprintf(buffer, sizeof(buffer), "%d,%d,%d,%d,%d,%d\r", positionRounded[0], positionRounded[1], positionRounded[2], positionRounded[3], positionRounded[4], positionRounded[5]);
  std::cout << "Sending position command: " << buffer  << std::endl;
  int ret_val = this->uartSend((const uint8_t *)buffer, strlen(buffer));
  if (ret_val < 0) {
    std::cerr << "Error sending command to set joint position" << std::endl;
    return false;
  }
  return true;
}
/*!

  Emergency stops the robot if the program is interrupted by a SIGINT
  (CTRL C), SIGSEGV (segmentation fault), SIGBUS (bus error), SIGKILL
  or SIGQUIT signal.

*/
void emergencyStopViper850(int signo)
{
  std::cout << "Stop the Viper850 application by signal (" << signo << "): " << static_cast<char>(7);
  switch (signo) {
  case SIGINT:
    std::cout << "SIGINT (stop by ^C) " << std::endl;
    break;
  case SIGBUS:
    std::cout << "SIGBUS (stop due to a bus error) " << std::endl;
    break;
  case SIGSEGV:
    std::cout << "SIGSEGV (stop due to a segmentation fault) " << std::endl;
    break;
  case SIGKILL:
    std::cout << "SIGKILL (stop by CTRL \\) " << std::endl;
    break;
  case SIGQUIT:
    std::cout << "SIGQUIT " << std::endl;
    break;
  default:
    std::cout << signo << std::endl;
  }
  // std::cout << "Emergency stop called\n";
  //  PrimitiveESTOP_Viper850();
  // PrimitiveSTOP_Viper850();
  std::cout << "Robot was stopped\n";

  // Free allocated resources
  //  ShutDownConnection(); // Some times cannot exit here when Ctrl-C

  fprintf(stdout, "Application ");
  fflush(stdout);
  kill(getpid(), SIGKILL);
  std::exit(EXIT_FAILURE);
}

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  The only available constructor.

  This constructor calls init() to initialise the connection with the
  MotionBox or low level controller, send the default \f$^e{\bf
  M}_c\f$ homogeneous matrix and power on the robot.

  It also set the robot state to vpRobot::STATE_STOP.

  To set the extrinsic camera parameters related to the \f$^e{\bf
  M}_c\f$ matrix obtained with a camera perspective projection model
  including the distortion, use the code below:

  \code
  #include <visp3/core/vpCameraParameters.h>
  #include <visp3/core/vpImage.h>
  #include <visp3/robot/vpRobotDenso6577.h>
  #include <visp3/sensor/vp1394TwoGrabber.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Set the extrinsic camera parameters obtained with a perspective
    // projection model including a distortion parameter
    robot.init(vpDenso6577::TOOL_MARLIN_F033C_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
    \endcode

    Now, you can get the intrinsic camera parameters associated to an
    image acquired by the camera attached to the robot, with:

    \code
    vpImage<unsigned char> I(480, 640);

    // Get an image from the camera attached to the robot
  #ifdef VISP_HAVE_DC1394
    vp1394TwoGrabber g;
    g.acquire(I);
  #endif
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);
    // In cam, you get the intrinsic parameters of the projection model
    // with distortion.
  #endif
  }
  \endcode

  \sa vpCameraParameters, init(vpDenso6577::vpToolType,
  vpCameraParameters::vpCameraParametersProjType)

*/
vpRobotDenso6577::vpRobotDenso6577(bool verbose) : vpDenso6577(), vpRobot()
{

  /*
    #define  SIGHUP  1  // hangup
    #define  SIGINT  2  // interrupt (rubout)
    #define  SIGQUIT  3  // quit (ASCII FS)
    #define  SIGILL  4  // illegal instruction (not reset when caught)
    #define  SIGTRAP  5  // trace trap (not reset when caught)
    #define  SIGIOT  6  // IOT instruction
    #define  SIGABRT 6  // used by abort, replace SIGIOT in the future
    #define  SIGEMT  7  // EMT instruction
    #define  SIGFPE  8  // floating point exception
    #define  SIGKILL  9  // kill (cannot be caught or ignored)
    #define  SIGBUS  10  // bus error
    #define  SIGSEGV  11  // segmentation violation
    #define  SIGSYS  12  // bad argument to system call
    #define  SIGPIPE  13  // write on a pipe with no one to read it
    #define  SIGALRM  14  // alarm clock
    #define  SIGTERM  15  // software termination signal from kill
  */

  // signal(SIGINT, emergencyStopViper850);
  // signal(SIGBUS, emergencyStopViper850);
  // signal(SIGSEGV, emergencyStopViper850);
  // signal(SIGKILL, emergencyStopViper850);
  // signal(SIGQUIT, emergencyStopViper850);

  // setVerbose(verbose);
  // if (verbose_)
  //   std::cout << "Open communication with MotionBlox.\n";
  try {
    this->init();
    this->setRobotState(vpRobot::STATE_STOP);
  }
  catch (...) {
 //  vpERROR_TRACE("Error caught") ;
    throw;
  }
  m_positioningVelocity = m_defaultPositioningVelocity;

  maxRotationVelocity_joint6 = maxRotationVelocity;

  vpRobotDenso6577::m_robotAlreadyCreated = true;

  return;
}

/* ------------------------------------------------------------------------ */
/* --- INITIALISATION ----------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Initialise the connection with the MotionBox or low level
  controller, send the default eMc homogeneous matrix, power on the
  robot and wait 1 sec before returning to be sure the initialisation
  is done.

  \warning This method sets the camera extrinsic parameters (matrix
  eMc) to the one obtained by calibration with a camera projection
  model without distortion by calling
  init(vpDenso6577::defaultCameraRobot). If you want to set the extrinsic
  camera parameters to those obtained with a camera perspective model
  including the distortion you have to call the
  init(vpDenso6577::vpDenso6577CameraRobotType,
  vpCameraParameters::vpCameraParametersProjType) method. If you want to set
  custom extrinsic camera parameters you have to call the
  init(vpDenso6577::vpToolType, const vpHomogeneousMatrix&) method.

  \sa vpCameraParameters, init(vpDenso6577::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpDenso6577::vpToolType, const vpHomogeneousMatrix&),
  init(vpDenso6577::vpToolType, const std::string&)

*/
void vpRobotDenso6577::init(void)
{
  // Initialise private variables used to compute the measured velocities
  m_q_prev_getvel.resize(6);
  m_q_prev_getvel = 0;
  m_time_prev_getvel = 0;
  m_first_time_getvel = true;

  // Initialise private variables used to compute the measured displacement
  m_q_prev_getdis.resize(6);
  m_q_prev_getdis = 0;
  m_first_time_getdis = true;
  // Update the eMc matrix in the low level controller
  init(vpDenso6577::defaultTool);
  this->serial.openDevice("/dev/ttyUSB0", 115200);
  // get real joint min/max from the MotionBlox
  // Test if an error occurs
  return;
}

/*!

  Initialize the robot kinematics with the extrinsic calibration
  parameters associated to a specific camera (set the eMc homogeneous
  parameters in the low level controller) and also get the joint
  limits from the low-level controller.

  The eMc parameters depend on the camera and the projection model in use.

  \param tool : Tool to use.

  \param projModel : Projection model associated to the camera.

  To set the extrinsic camera parameters related to the \f$^e{\bf
  M}_c\f$ matrix obtained with a camera perspective projection model
  including the distortion, use the code below:

  \code
  #include <visp3/core/vpCameraParameters.h>
  #include <visp3/core/vpImage.h>
  #include <visp3/robot/vpRobotDenso6577.h>
  #include <visp3/sensor/vp1394TwoGrabber.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Set the extrinsic camera parameters obtained with a perspective
    // projection model including a distortion parameter
    robot.init(vpDenso6577::TOOL_MARLIN_F033C_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
    \endcode

    Now, you can get the intrinsic camera parameters associated to an
    image acquired by the camera attached to the robot, with:

    \code
    vpImage<unsigned char> I(480, 640);

    // Get an image from the camera attached to the robot
  #ifdef VISP_HAVE_DC1394
    vp1394TwoGrabber g;
    g.acquire(I);
  #endif
    vpCameraParameters cam;
    robot.getCameraParameters(cam, I);
    // In cam, you get the intrinsic parameters of the projection model
    // with distortion.
  #endif
  }
  \endcode

  \sa vpCameraParameters,
  init(vpDenso6577::vpToolType, const vpHomogeneousMatrix&),
  init(vpDenso6577::vpToolType, const std::string&)
*/
void vpRobotDenso6577::init(vpDenso6577::vpToolType tool, vpCameraParameters::vpCameraParametersProjType projModel)
{
  // Read the robot constants from files
  // - joint [min,max], coupl_56, long_56
  // - camera extrinsic parameters relative to eMc
  vpDenso6577::init(tool, projModel);

  // // Set the camera constant (eMc pose) in the MotionBlox
  // double eMc_pose[6];
  // for (unsigned int i = 0; i < 3; i++) {
  //   eMc_pose[i] = etc[i];     // translation in meters
  //   eMc_pose[i + 3] = erc[i]; // rotation in rad
  // }
  this->serial.openDevice("/dev/ttyUSB0", 115200);
  // Update the eMc pose in the low level controller
  // // Try(PrimitiveCONST_Viper850(eMc_pose));

  return;
}

/*!

  Initialize the robot kinematics (set the eMc homogeneous
  parameters in the low level controller) from a file and
  also get the joint limits from the low-level controller.

  \param tool : Tool to use.

  \param filename : Path of the configuration file containing the
  transformation between the end-effector frame and the tool frame.

  To set the transformation parameters related to the \f$^e{\bf
  M}_c\f$ matrix, use the code below:

  \code
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Set the transformation between the end-effector frame
    // and the tool frame from a file
    std::string filename("./EffectorToolTransformation.cnf");

    robot.init(vpDenso6577::TOOL_CUSTOM, filename);
  #endif
  }
  \endcode

  The configuration file should have the form below:

  \code
  # Start with any number of consecutive lines
  # beginning with the symbol '#'
  #
  # The 3 following lines contain the name of the camera,
  # the rotation parameters of the geometric transformation
  # using the Euler angles in degrees with convention XYZ and
  # the translation parameters expressed in meters
  CAMERA CameraName
  eMc_ROT_XYZ 10.0 -90.0 20.0
  eMc_TRANS_XYZ  0.05 0.01 0.06
  \endcode

  \sa init(), init(vpDenso6577::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpDenso6577::vpToolType, const vpHomogeneousMatrix&)
*/
void vpRobotDenso6577::init(vpDenso6577::vpToolType tool, const std::string &filename)
{
  vpDenso6577::init(tool, filename);

  // Set the camera constant (eMc pose) in the MotionBlox
  // double eMc_pose[6];
  // for (unsigned int i = 0; i < 3; i++) {
  //   eMc_pose[i] = etc[i];     // translation in meters
  //   eMc_pose[i + 3] = erc[i]; // rotation in rad
  // }
  // Update the eMc pose in the low level controller
  // // Try(PrimitiveCONST_Viper850(eMc_pose));
  this->serial.openDevice("/dev/ttyUSB0", 115200);
  return;
}

/*!

  Initialize the robot kinematics with user defined parameters
  (set the eMc homogeneous parameters in the low level controller)
  and also get the joint limits from the low-level controller.

  \param tool : Tool to use.

  \param eMc_ : Transformation between the end-effector frame
  and the tool frame.

  To set the transformation parameters related to the \f$^e{\bf
  M}_c\f$ matrix, use the code below:

  \code
  #include <visp3/core/vpHomogeneousMatrix.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Set the transformation between the end-effector frame
    // and the tool frame.
    vpHomogeneousMatrix eMc(0.001, 0.0, 0.1, 0.0, 0.0, M_PI/2);

    robot.init(vpDenso6577::TOOL_CUSTOM, eMc);
  #endif
  }
  \endcode

  \sa vpCameraParameters, init(), init(vpDenso6577::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpDenso6577::vpToolType, const std::string&)
*/
void vpRobotDenso6577::init(vpDenso6577::vpToolType tool, const vpHomogeneousMatrix &eMc_)
{
  vpDenso6577::init(tool, eMc_);

  //   for (unsigned int i=0; i < njoint; i++) {
  //     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i],
  //     joint_max[i]);
  //   }

  // Set the camera constant (eMc pose) in the MotionBlox
  // double eMc_pose[6];
  // for (unsigned int i = 0; i < 3; i++) {
  //   eMc_pose[i] = etc[i];     // translation in meters
  //   eMc_pose[i + 3] = erc[i]; // rotation in rad
  // }
  // Update the eMc pose in the low level controller
  // // Try(PrimitiveCONST_Viper850(eMc_pose));
  this->serial.openDevice("/dev/ttyUSB0", 115200);
  return;
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame in the low level controller.

  \warning This function overwrite the transformation parameters that were
  potentially set using one of the init functions

  \param eMc_ : Transformation between the end-effector frame
  and the tool frame.
*/
void vpRobotDenso6577::set_eMc(const vpHomogeneousMatrix &eMc_)
{
  this->vpDenso6577::set_eMc(eMc_);

  // Set the camera constant (eMc pose) in the MotionBlox
  // double eMc_pose[6];
  // for (unsigned int i = 0; i < 3; i++) {
  //   eMc_pose[i] = etc[i];     // translation in meters
  //   eMc_pose[i + 3] = erc[i]; // rotation in rad
  // }
  // Update the eMc pose in the low level controller
  // // Try(PrimitiveCONST_Viper850(eMc_pose));

  return;
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame in the low level controller.

  \warning This function overwrite the transformation parameters that were
  potentially set using one of the init functions.

  \param etc_ : Translation between the end-effector frame and the tool frame.
  \param erc_ : Rotation between the end-effector frame and the tool frame
  using the Euler angles in radians with the XYZ convention.
*/
void vpRobotDenso6577::set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_)
{
  this->vpDenso6577::set_eMc(etc_, erc_);


  // Set the camera constant (eMc pose) in the MotionBlox
  // double eMc_pose[6];
  // for (unsigned int i = 0; i < 3; i++) {
  //   eMc_pose[i] = etc[i];     // translation in meters
  //   eMc_pose[i + 3] = erc[i]; // rotation in rad
  // }
  // Update the eMc pose in the low level controller
  // // Try(PrimitiveCONST_Viper850(eMc_pose));


  return;
}

/* ------------------------------------------------------------------------ */
/* --- DESTRUCTOR --------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Destructor.

  Free allocated resources.
*/
vpRobotDenso6577::~vpRobotDenso6577(void)
{
#if defined(USE_ATI_DAQ) && defined(VISP_HAVE_COMEDI)
  m_ati.close();
#endif
  setRobotState(vpRobot::STATE_STOP);
  // Look if the power is on or off

  //   if (HIPowerStatus == 1) {
  //     fprintf(stdout, "Power OFF the robot\n");
  //     fflush(stdout);

  //     // Try( PrimitivePOWEROFF_Viper850() );
  //   }

  vpRobotDenso6577::m_robotAlreadyCreated = false;

  return;
}

/*!

Change the robot state.

\param newState : New requested robot state.
*/
vpRobot::vpRobotStateType vpRobotDenso6577::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      vpTime::sleepMs(100); // needed to ensure velocity task ends up on low level
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
    }
    else {
   // std::cout << "Change the control mode from stop to position
   // control.\n";
    }
    this->powerOn();
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    this->powerOn();
    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

/* ------------------------------------------------------------------------ */
/* --- STOP --------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Stop the robot and set the robot state to vpRobot::STATE_STOP.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.
*/
void vpRobotDenso6577::stopMotion(void)
{
  if (getRobotState() != vpRobot::STATE_VELOCITY_CONTROL) return;
  setRobotState(vpRobot::STATE_STOP);
}

/*!

  Power on the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot power on.

  \sa powerOff(), getPowerState()
*/
void vpRobotDenso6577::powerOn(void)
{
  return;
}

/*!

  Power off the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.

  \sa powerOn(), getPowerState()
*/
void vpRobotDenso6577::powerOff(void)
{
  return;
}

/*!

  Get the robot power state indication if power is on or off.

  \return true if power is on. false if power is off.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error.

  \sa powerOn(), powerOff()
*/
bool vpRobotDenso6577::getPowerState(void) const
{
  return true;
}

/*!

  Get the twist transformation \f$^c{\bf V}_e\f$ from camera frame to
  end-effector frame.  This transformation allows to compute a
  velocity expressed in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void vpRobotDenso6577::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  vpDenso6577::get_cMe(cMe);

  cVe.buildFrom(cMe);
}

/*!

  Get the geometric transformation \f$^c{\bf M}_e\f$ between the
  camera frame and the end-effector frame. This transformation is
  constant and correspond to the extrinsic camera parameters estimated
  by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void vpRobotDenso6577::get_cMe(vpHomogeneousMatrix &cMe) const { vpDenso6577::get_cMe(cMe); }

/*!

  Get the robot jacobian expressed in the end-effector frame.

  To compute \f$^e{\bf J}_e\f$, we communicate with the low level
  controller to get the joint position of the robot.

  \param eJe : Robot jacobian \f$^e{\bf J}_e\f$ expressed in the
  end-effector frame.

*/
void vpRobotDenso6577::get_eJe(vpMatrix &eJe)
{

  double position[6];
  // double timestamp;


  //// Try(PrimitiveACQ_POS_J_Viper850(position, &timestamp));
  // TODO: get the joint position from the low level controller
  getJointPosition(position);
  vpColVector q(6);
  for (unsigned int i = 0; i < njoint; i++)
    q[i] = vpMath::rad(position[i]);
  try {
    vpDenso6577::get_eJe(q, eJe);
  }
  catch (...) {
    vpERROR_TRACE("catch exception ");
    throw;
  }
}
/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame.

  To compute \f$^f{\bf J}_e\f$, we communicate with the low level
  controller to get the joint position of the robot.

  \param fJe : Robot jacobian \f$^f{\bf J}_e\f$ expressed in the
  reference frame.
*/

void vpRobotDenso6577::get_fJe(vpMatrix &fJe)
{

  double position[6];
  // double timestamp;
  // // Try(PrimitiveACQ_POS_Viper850(position, &timestamp));
  // TODO: get the joint position from the low level controller
  getJointPosition(position);
  vpColVector q(6);
  for (unsigned int i = 0; i < njoint; i++)
    q[i] = position[i];

  try {
    vpDenso6577::get_fJe(q, fJe);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Set the maximal velocity percentage to use for a position control.

  The default positioning velocity is defined by
  vpRobotDenso6577::m_defaultPositioningVelocity. This method allows to
  change this default positioning velocity

  \param velocity : Percentage of the maximal velocity. Values should
  be in ]0:100].

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpColVector position(6);
    position = 0; // position in rad

    vpRobotDenso6577 robot;

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    // Set the max velocity to 20%
    robot.setPositioningVelocity(20);

    // Moves the robot to the joint position [0,0,0,0,0,0]
    robot.setPosition(vpRobot::ARTICULAR_FRAME, position);
  #endif
  }
  \endcode

  \sa getPositioningVelocity()
*/
void vpRobotDenso6577::setPositioningVelocity(double velocity) { m_positioningVelocity = velocity; }

/*!
  Get the maximal velocity percentage used for a position control.

  \sa setPositioningVelocity()
*/
double vpRobotDenso6577::getPositioningVelocity(void) const { return m_positioningVelocity; }

/*!

  Move to an absolute position with a given percent of max velocity.
  The percent of max velocity is to set with setPositioningVelocity().
  The position to reach can be specified in joint coordinates, in the
  camera frame or in the reference frame.

  \warning This method is blocking. It returns only when the position
  is reached by the robot.

  \param position : A six dimension vector corresponding to the
  position to reach. All the positions are expressed in meters for the
  translations and radians for the rotations. If the position is out
  of range, an exception is provided.

  \param frame : Frame in which the position is expressed.

  - In the joint space, positions are the six joint positions.

  - In the camera and the reference frame, positions are respectively
  X,Y,Z translations and 3 rotations around the X, Y and Z
  axis. Rotations are represented by a vpRxyzVector.

  - Mixt frame is not implemented. By mixt frame we mean, translations
  expressed in the reference frame, and rotations in the camera
  frame.

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME
  and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpColVector position(6);
    // Set positions in the camera frame
    position[0] = 0.1;    // x axis, in meter
    position[1] = 0.2;    // y axis, in meter
    position[2] = 0.3;    // z axis, in meter
    position[3] = M_PI/8; // rotation around x axis, in rad
    position[4] = M_PI/4; // rotation around y axis, in rad
    position[5] = M_PI;   // rotation around z axis, in rad

    vpRobotDenso6577 robot;

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    // Set the max velocity to 20%
    robot.setPositioningVelocity(20);

    // Moves the robot in the camera frame
    robot.setPosition(vpRobot::CAMERA_FRAME, position);
  #endif
  }
  \endcode

  To catch the exception if the position is out of range, modify the code
like:

  \code
  try {
    robot.setPosition(vpRobot::CAMERA_FRAME, position);
  }
  catch (vpRobotException &e) {
    if (e.getCode() == vpRobotException::positionOutOfRangeError) {
    std::cout << "The position is out of range" << std::endl;
  }
  \endcode

*/
void vpRobotDenso6577::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    vpERROR_TRACE("Robot was not in position-based control\n"
                  "Modification of the robot state");
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  vpColVector destination(njoint);
  // double timestamp;


  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    vpColVector q(njoint);
    //// Try(PrimitiveACQ_POS_Viper850(q.data, &timestamp));
    // TODO: get the joint position from the low level controller
    // Convert degrees into rad
    getJointPosition(q.data);
    q.deg2rad();

    // Get fMc from the inverse kinematics
    vpHomogeneousMatrix fMc;
    vpDenso6577::get_fMc(q, fMc);

    // Set cMc from the input position
    vpTranslationVector txyz;
    vpRxyzVector rxyz;
    for (unsigned int i = 0; i < 3; i++) {
      txyz[i] = position[i];
      rxyz[i] = position[i + 3];
    }

    // Compute cMc2
    vpRotationMatrix cRc2(rxyz);
    vpHomogeneousMatrix cMc2(txyz, cRc2);

    // Compute the new position to reach: fMc*cMc2
    vpHomogeneousMatrix fMc2 = fMc * cMc2;

    // Compute the corresponding joint position from the inverse kinematics
    unsigned int solution = this->getInverseKinematics(fMc2, q);
    if (solution) { // Position is reachable
      destination = q;
      // convert rad to deg requested for the low level controller
      destination.rad2deg();
      // // Try(PrimitiveMOVE_J_Viper850(destination.data, m_positioningVelocity));
      // // Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));
      // TODO: move the robot to the destination joint position
      sendPosition(destination.data);
    }
    else {
   // Cartesian position is out of range
      vpERROR_TRACE("Position out of range.");
      throw vpRobotException(vpRobotException::positionOutOfRangeError, "Position out of range.");
    }

    break;
  }
  case vpRobot::ARTICULAR_FRAME: {
    destination = position;
    // convert rad to deg requested for the low level controller
    destination.rad2deg();

    // std::cout << "Joint destination (deg): " << destination.t() <<
    // std::endl;
    // // Try(PrimitiveMOVE_J_Viper850(destination.data, m_positioningVelocity));
    // // Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));
    // TODO: move the robot to the destination joint position
    sendPosition(destination.data);
    std::cout << "Move to joint position: " << destination.t() << std::endl;
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    // Convert angles from Rxyz representation to Rzyz representation
    vpRxyzVector rxyz(position[3], position[4], position[5]);
    vpRotationMatrix R(rxyz);
    vpRzyzVector rzyz(R);

    for (unsigned int i = 0; i < 3; i++) {
      destination[i] = position[i];
      destination[i + 3] = vpMath::deg(rzyz[i]); // convert also angles in deg
    }

    // std::cout << "Base frame destination Rzyz (deg): " << destination.t()
    // << std::endl;
    // // Try(PrimitiveMOVE_C_Viper850(destination.data, configuration, m_positioningVelocity));
    // // Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));
    // TODO: move the robot to the destination joint position
    sendPosition(destination.data);
    break;
  }
  default: {
    throw vpRobotException(vpRobotException::lowLevelError, "vpRobot::MIXT_FRAME and vpRobot::END_EFFECTOR_FRAME not implemented.");
  }
  }
  // if (// TryStt == InvalidPosition || // TryStt == -1023)
  //   std::cout << " : Position out of range.\n";
  // else if (// TryStt == -3019) {
  //   if (frame == vpRobot::ARTICULAR_FRAME)
  //     std::cout << " : Joint position out of range.\n";
  //   else
  //     std::cout << " : Cartesian position leads to a joint position out of "
  //     "range.\n";
  // }
  // else if (// TryStt < 0)
  //   std::cout << " : Unknown error (see Fabien).\n";
  // else if (error == -1)
  //   std::cout << "Position out of range.\n";

  // if (// TryStt < 0 || error < 0) {
  //   vpERROR_TRACE("Positioning error.");
  //   throw vpRobotException(vpRobotException::positionOutOfRangeError, "Position out of range.");
  // }

  return;
}

/*!
  Move to an absolute position with a given percent of max velocity.
  The percent of max velocity is to set with setPositioningVelocity().
  The position to reach can be specified in joint coordinates, in the
  camera frame or in the reference frame.

  This method owerloads setPosition(const
  vpRobot::vpControlFrameType, const vpColVector &).

  \warning This method is blocking. It returns only when the position is reached by the robot.

  All the positions are expressed in meters for the translations and radians for the rotations.

  \param pos1 : First coordinate of the position to reach.
  \param pos2 : Second coordinate of the position to reach.
  \param pos3 : Third coordinate of the position to reach.
  \param pos4 : Fourth coordinate of the position to reach.
  \param pos5 : Fifth coordinate of the position to reach.
  \param pos6 : Sixth coordinate of the position to reach.

  \param frame : Frame in which the position is expressed.

  - In the joint space, positions are respectively X (pos1), Y (pos2), Z (pos3), A (pos4), B (pos5), C (pos6),
    with X,Y,Z the translation positions in meters, and A,B,C the rotations in radians of the end-effector.

  - In the camera and the reference frame, rotations [pos4, pos5, pos6] are represented by a vpRxyzVector with
    values in radians.

  - Mixt frame is not implemented. By mixt frame we mean, translations expressed in the reference frame,
    and rotations in the camera frame.

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested position is out of range.

  \code
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    // Set positions in the camera frame
    double pos1 = 0.1;    // x axis, in meter
    double pos2 = 0.2;    // y axis, in meter
    double pos3 = 0.3;    // z axis, in meter
    double pos4 = M_PI/8; // rotation around x axis, in rad
    double pos5 = M_PI/4; // rotation around y axis, in rad
    double pos6 = M_PI;   // rotation around z axis, in rad

    vpRobotDenso6577 robot;

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    // Set the max velocity to 20%
    robot.setPositioningVelocity(20);

    // Moves the robot in the camera frame
    robot.setPosition(vpRobot::CAMERA_FRAME, pos1, pos2, pos3, pos4, pos5, pos6);
  #endif
  }
  \endcode

  \sa setPosition()
*/
void vpRobotDenso6577::setPosition(const vpRobot::vpControlFrameType frame, double pos1, double pos2, double pos3,
                                  double pos4, double pos5, double pos6)
{
  try {
    vpColVector position(6);
    position[0] = pos1;
    position[1] = pos2;
    position[2] = pos3;
    position[3] = pos4;
    position[4] = pos5;
    position[5] = pos6;

    setPosition(frame, position);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Move to an absolute joint position with a given percent of max
  velocity. The robot state is set to position control.  The percent
  of max velocity is to set with setPositioningVelocity(). The
  position to reach is defined in the position file.

  \param filename : Name of the position file to read. The
  readPosFile() documentation shows a typical content of such a
  position file.

  This method has the same behavior than the sample code given below;
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpColVector q;
    vpRobotDenso6577 robot;

    robot.readPosFile("MyPositionFilename.pos", q);
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
  #endif
  }
  \endcode

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME
  and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \sa setPositioningVelocity()

*/
void vpRobotDenso6577::setPosition(const std::string &filename)
{
  vpColVector q;
  bool ret;

  ret = this->readPosFile(filename, q);

  if (ret == false) {
    vpERROR_TRACE("Bad position in \"%s\"", filename.c_str());
    throw vpRobotException(vpRobotException::lowLevelError, "Bad position in filename.");
  }
  this->setRobotState(vpRobot::STATE_POSITION_CONTROL);
  this->setPosition(vpRobot::ARTICULAR_FRAME, q);
}

/*!

  Get the current position of the robot.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesian frame,
  - joint (articular) coordinates of each axes
  - in a reference or fixed cartesian frame attached to the robot base
  - in a mixt cartesian frame (translation in reference
  frame, and rotation in camera frame)

  \param position : Measured position of the robot:
  - in camera cartesian frame, a 6 dimension vector, set to 0.

  - in articular, a 6 dimension vector corresponding to the joint
  position of each dof in radians.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector). The code
  below show how to convert this position into a vpHomogeneousMatrix:

  \param timestamp : Time in second since last robot power on.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpHomogeneousMatrix.h>
  #include <visp3/core/vpRotationMatrix.h>
  #include <visp3/core/vpRxyzVector.h>
  #include <visp3/core/vpTranslationVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    vpColVector position;
    double timestamp;
    robot.getPosition(vpRobot::REFERENCE_FRAME, position, timestamp);

    vpTranslationVector ftc; // reference frame to camera frame translations
    vpRxyzVector frc; // reference frame to camera frame rotations

    // Update the transformation between reference frame and camera frame
    for (unsigned int i=0; i < 3; i++) {
      ftc[i] = position[i];   // tx, ty, tz
      frc[i] = position[i+3]; // ry, ry, rz
    }

    // Create a rotation matrix from the Rxyz rotation angles
    vpRotationMatrix fRc(frc); // reference frame to camera frame rotation matrix

    // Create the camera to fix frame transformation in terms of a
    // homogeneous matrix
    vpHomogeneousMatrix fMc(ftc, fRc);
  #endif
  }
  \endcode

  \exception vpRobotException::lowLevelError : If the position cannot
  be get from the low level controller.

  \sa setPosition(const vpRobot::vpControlFrameType frame, const
  vpColVector & r)

*/
void vpRobotDenso6577::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp)
{



  position.resize(6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    position = 0;
    return;
  }
  case vpRobot::ARTICULAR_FRAME: {
    // // Try(PrimitiveACQ_POS_J_Viper850(position.data, &timestamp));
    // TODO: get the joint position from the low level controller
    // vpCTRACE << "Get joint position (deg)" << position.t() << std::endl;
    getJointPosition(position.data, 1);
    position.deg2rad();

    return;
  }
  case vpRobot::REFERENCE_FRAME: {
    vpColVector q(njoint);
    getJointPosition(q.data);
    // // Try(PrimitiveACQ_POS_J_Viper850(q.data, &timestamp));
    // TODO: get the joint position from the low level controller
    // Compute fMc
    vpHomogeneousMatrix fMc = vpDenso6577::get_fMc(q);

    // From fMc extract the pose
    vpRotationMatrix fRc;
    fMc.extract(fRc);
    vpRxyzVector rxyz;
    rxyz.buildFrom(fRc);

    for (unsigned int i = 0; i < 3; i++) {
      position[i] = fMc[i][3];   // translation x,y,z
      position[i + 3] = rxyz[i]; // Euler rotation x,y,z
    }

    break;
  }
  default: {
    throw vpRobotException(vpRobotException::lowLevelError, "vpRobot::MIXT_FRAME and vpRobot::END_EFFECTOR_FRAME not implemented.");
  }
  }
  return;
}

/*!
  Returns the robot controller current time (in second) since last robot power
  on.
*/
double vpRobotDenso6577::getTime() const
{
  // PrimitiveACQ_TIME_Viper850(&timestamp);
  return -1.0; // TODO: get the time from the low level controller
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpColVector
  &, double &).

  The difference is here that the timestamp is not used.

*/
void vpRobotDenso6577::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  double timestamp;
  getPosition(frame, position, timestamp);
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpColVector
  &, double &).

  The difference is here that the position is returned using a \f$ \theta {\bf
  u}\f$ representation.

*/
void vpRobotDenso6577::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp)
{
  vpColVector posRxyz;
  // recupere  position en Rxyz
  this->getPosition(frame, posRxyz, timestamp);
  vpRxyzVector RxyzVect;
  for (unsigned int j = 0; j < 3; j++)
    RxyzVect[j] = posRxyz[j + 3];
  // recupere le vecteur thetaU correspondant
  vpThetaUVector RtuVect(RxyzVect);

  // remplit le vpPoseVector avec translation et rotation ThetaU
  for (unsigned int j = 0; j < 3; j++) {
    position[j] = posRxyz[j];
    position[j + 3] = RtuVect[j];
  }
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector
  &, double &).

  The difference is here that the timestamp is not used.

*/
void vpRobotDenso6577::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position)
{
  vpColVector posRxyz;
  double timestamp;
  // recupere  position en Rxyz
  this->getPosition(frame, posRxyz, timestamp);
  vpRxyzVector RxyzVect;
  for (unsigned int j = 0; j < 3; j++)
    RxyzVect[j] = posRxyz[j + 3];
  // recupere le vecteur thetaU correspondant
  vpThetaUVector RtuVect(RxyzVect);

  // remplit le vpPoseVector avec translation et rotation ThetaU
  for (unsigned int j = 0; j < 3; j++) {
    position[j] = posRxyz[j];
    position[j + 3] = RtuVect[j];
  }
}

/*!
  Apply a velocity to the robot.

  \param frame : Control frame in which the velocity is expressed. Velocities
  could be expressed in articular, camera frame, reference frame or mixt
frame.

  \param vel : Velocity vector. Translation velocities are expressed
  in m/s while rotation velocities in rad/s. The size of this vector
  is always 6.

  - In articular, \f$ vel = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
  \dot{q}_5, \dot{q}_6]^t \f$ correspond to joint velocities in rad/s.

  - In camera frame, \f$ vel = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c}
  \omega_x, ^{c} \omega_y, ^{c} \omega_z]^t \f$ is a velocity twist vector
expressed in the camera frame, with translations velocities \f$ ^{c} v_x, ^{c}
v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
\omega_y, ^{c} \omega_z \f$ in rad/s.

  - In reference frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{r}
  \omega_x, ^{r} \omega_y, ^{r} \omega_z]^t \f$ is a velocity twist vector
expressed in the reference frame, with translations velocities \f$ ^{c} v_x,
^{c} v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
\omega_y, ^{c} \omega_z \f$ in rad/s.

  - In mixt frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{c} \omega_x,
  ^{c} \omega_y, ^{c} \omega_z]^t \f$ is a velocity twist vector where,
translations \f$ ^{r} v_x, ^{r} v_y, ^{r} v_z \f$ are expressed in the
reference frame in m/s and rotations \f$ ^{c} \omega_x, ^{c} \omega_y, ^{c}
\omega_z \f$ in the camera frame in rad/s.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \warning Velocities could be saturated if one of them exceed the
  maximal authorized speed (see vpRobot::maxTranslationVelocity and
  vpRobot::maxRotationVelocity). To change these values use
  setMaxTranslationVelocity() and setMaxRotationVelocity().

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMath.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    vpColVector qvel(6);
    // Set a joint velocity
    qvel[0] = 0.1;             // Joint 1 velocity in rad/s
    qvel[1] = vpMath::rad(15); // Joint 2 velocity in rad/s
    qvel[2] = 0;               // Joint 3 velocity in rad/s
    qvel[3] = M_PI/8;          // Joint 4 velocity in rad/s
    qvel[4] = 0;               // Joint 5 velocity in rad/s
    qvel[5] = 0;               // Joint 6 velocity in rad/s

    // Initialize the controller to velocity control
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    while (1) {
      // Apply a velocity in the joint space
      robot.setVelocity(vpRobot::ARTICULAR_FRAME, qvel);

      // Compute new velocities qvel...
    }

    // Stop the robot
    robot.setRobotState(vpRobot::STATE_STOP);
  #endif
  }
  \endcode
*/
void vpRobotDenso6577::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "use setRobotState(vpRobot::STATE_POSITION_CONTROL ) first) ");
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot "
                           "use setRobotState(vpRobot::STATE_POSITION_CONTROL ) first) ");
  }

  vpColVector vel_sat(6);

  // Velocity saturation
  switch (frame) {
  // saturation in cartesian space
  case vpRobot::CAMERA_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    break;
  }
  // saturation in joint space
  case vpRobot::ARTICULAR_FRAME: {
    vpColVector vel_max(6);

    // if (getMaxRotationVelocity() == getMaxRotationVelocityJoint6()) {
    if (std::fabs(getMaxRotationVelocity() - getMaxRotationVelocityJoint6()) < std::numeric_limits<double>::epsilon()) {

      for (unsigned int i = 0; i < 6; i++)
        vel_max[i] = getMaxRotationVelocity();
    }
    else {
      for (unsigned int i = 0; i < 5; i++)
        vel_max[i] = getMaxRotationVelocity();
      vel_max[5] = getMaxRotationVelocityJoint6();
    }

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
  }
  }



  double delta_t = 0.10; // 10 ms
  switch (frame) {

  case vpRobot::CAMERA_FRAME: {
    // Convert velocity camera -> end-effector
    vpHomogeneousMatrix cMe;
    this->get_cMe(cMe);
    vpVelocityTwistMatrix eVc = cMe.inverse(); // eVc = inverse(cVe)
    vpColVector v_e = eVc * vel_sat;

    // Exponential map
    vpHomogeneousMatrix eMe = vpExponentialMap::direct(v_e, delta_t);

    // Current pose in base frame
    vpPoseVector position;
    getPosition(vpRobot::REFERENCE_FRAME, position);
    vpHomogeneousMatrix fMe(position);
    // Update pose
    vpHomogeneousMatrix fMe_new = fMe * eMe;


    // Send position
    setPosition(vpRobot::REFERENCE_FRAME, vpPoseVector(fMe_new));
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME: {
    // Velocity already in end-effector frame
    vpColVector v_e = vel_sat;

    vpHomogeneousMatrix eMe = vpExponentialMap::direct(v_e, delta_t);

    vpPoseVector position;
    getPosition(vpRobot::REFERENCE_FRAME, position);
    vpHomogeneousMatrix fMe = vpHomogeneousMatrix(position);

    vpHomogeneousMatrix fMe_new = fMe * eMe;

    // Send position
    setPosition(vpRobot::REFERENCE_FRAME, vpPoseVector(fMe_new));
    break;
  }

  case vpRobot::REFERENCE_FRAME: {
    // Velocity in base frame
    vpColVector v_f = vel_sat;

    // Convert base velocity -> end-effector frame
    vpPoseVector position;
    getPosition(vpRobot::REFERENCE_FRAME, position);
    vpHomogeneousMatrix fMe(position);
    vpVelocityTwistMatrix eVf(fMe.inverse());
    vpColVector v_e = eVf * v_f;

    vpHomogeneousMatrix eMe = vpExponentialMap::direct(v_e, delta_t);

    vpHomogeneousMatrix fMe_new = fMe * eMe;

    setPosition(vpRobot::REFERENCE_FRAME, vpPoseVector(fMe_new));
    break;
  }

  case vpRobot::ARTICULAR_FRAME: {
    // Joint velocity → joint position
    vpColVector q;
    getPosition(vpRobot::ARTICULAR_FRAME, q);
    std::cout << "Current joint position: " << q.t() << std::endl;
    vpColVector q_new = q + vel_sat * delta_t;
    std::cout << "Set velocity in joint space, new joint position: " << q_new.t() << std::endl;
    setPosition(vpRobot::ARTICULAR_FRAME, q_new);
    break;
  }

  case vpRobot::MIXT_FRAME: {
    // Not implemented
    break;
  }

  default: {
    vpERROR_TRACE("Error in spec of vpRobot.");
    return;
  }
  }
  return;
}

/* ------------------------------------------------------------------------ */
/* --- GET ---------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Get the robot velocities.

  \param frame : Frame in which velocities are measured.

  \param velocity : Measured velocities. Translations are expressed in m/s
  and rotations in rad/s.

  \param timestamp : Time in second since last robot power on.

  \warning In camera frame, reference frame and mixt frame, the representation
  of the rotation is \f$ \theta {\bf u}\f$. In that cases, \f$velocity = [\dot
  x, \dot y, \dot z, \dot {\theta u}_x, \dot {\theta u}_y, \dot {\theta
  u}_z]\f$.

  \warning The first time this method is called, \e velocity is set to 0. The
  first call is used to intialise the velocity computation for the next call.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    // Set requested joint velocities
    vpColVector q_dot(6);
    q_dot[0] = 0.1;    // Joint 1 velocity in rad/s
    q_dot[1] = 0.2;    // Joint 2 velocity in rad/s
    q_dot[2] = 0.3;    // Joint 3 velocity in rad/s
    q_dot[3] = M_PI/8; // Joint 4 velocity in rad/s
    q_dot[4] = M_PI/4; // Joint 5 velocity in rad/s
    q_dot[5] = M_PI/16;// Joint 6 velocity in rad/s

    vpRobotDenso6577 robot;

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // Moves the joint in velocity
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);

    // Initialisation of the velocity measurement
    vpColVector q_dot_mes; // Measured velocities
    robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes); // q_dot_mes =0
    // q_dot_mes is resized to 6, the number of joint

    while (1) {
      robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes);
      vpTime::wait(40); // wait 40 ms
      // here q_dot_mes is equal to [0.1, 0.2, 0.3, M_PI/8, M_PI/4, M_PI/16]
    }
  #endif
  }
  \endcode
*/
void vpRobotDenso6577::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp)
{
  velocity.resize(6);
  velocity = 0;

  vpColVector q_cur(6);
  vpHomogeneousMatrix fMe_cur, fMc_cur;
  vpHomogeneousMatrix eMe, cMc; // camera displacement
  double time_cur;



  // Get the current joint position
  // // Try(PrimitiveACQ_POS_J_Viper850(q_cur.data, &timestamp));
  // TODO: get the joint position from the low level controller
  getJointPosition(q_cur.data);
  time_cur = timestamp;
  q_cur.deg2rad();

  // Get the end-effector pose from the direct kinematics
  vpDenso6577::get_fMe(q_cur, fMe_cur);
  // Get the camera pose from the direct kinematics
  vpDenso6577::get_fMc(q_cur, fMc_cur);

  if (!m_first_time_getvel) {

    switch (frame) {
    case vpRobot::END_EFFECTOR_FRAME: {
      // Compute the displacement of the end-effector since the previous call
      eMe = m_fMe_prev_getvel.inverse() * fMe_cur;

      // Compute the velocity of the end-effector from this displacement
      velocity = vpExponentialMap::inverse(eMe, time_cur - m_time_prev_getvel);

      break;
    }

    case vpRobot::CAMERA_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = m_fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      velocity = vpExponentialMap::inverse(cMc, time_cur - m_time_prev_getvel);

      break;
    }

    case vpRobot::ARTICULAR_FRAME: {
      velocity = (q_cur - m_q_prev_getvel) / (time_cur - m_time_prev_getvel);
      break;
    }

    case vpRobot::REFERENCE_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = m_fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      vpColVector v;
      v = vpExponentialMap::inverse(cMc, time_cur - m_time_prev_getvel);

      // Express this velocity in the reference frame
      vpVelocityTwistMatrix fVc(fMc_cur);
      velocity = fVc * v;

      break;
    }

    case vpRobot::MIXT_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = m_fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the ThetaU representation for the rotation
      vpRotationMatrix cRc;
      cMc.extract(cRc);
      vpThetaUVector thetaU;
      thetaU.buildFrom(cRc);

      for (unsigned int i = 0; i < 3; i++) {
        // Compute the translation displacement in the reference frame
        velocity[i] = m_fMc_prev_getvel[i][3] - fMc_cur[i][3];
        // Update the rotation displacement in the camera frame
        velocity[i + 3] = thetaU[i];
      }

      // Compute the velocity
      velocity /= (time_cur - m_time_prev_getvel);
      break;
    }
    default: {
      throw(vpException(vpException::functionNotImplementedError,
                        "vpRobotDenso6577::getVelocity() not implemented in end-effector"));
    }
    }
  }
  else {
    m_first_time_getvel = false;
  }

  // Memorize the end-effector pose for the next call
  m_fMe_prev_getvel = fMe_cur;
  // Memorize the camera pose for the next call
  m_fMc_prev_getvel = fMc_cur;

  // Memorize the joint position for the next call
  m_q_prev_getvel = q_cur;

  // Memorize the time associated to the joint position for the next call
  m_time_prev_getvel = time_cur;

}

/*!

  Get robot velocities.

  The behavior is the same than getVelocity(const vpRobot::vpControlFrameType,
  vpColVector &, double &) except that the timestamp is not returned.

  */
void vpRobotDenso6577::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity)
{
  double timestamp;
  getVelocity(frame, velocity, timestamp);
}

/*!

  Get the robot velocities.

  \param frame : Frame in which velocities are measured.

  \param timestamp : Time in second since last robot power on.

  \return Measured velocities. Translations are expressed in m/s
  and rotations in rad/s.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    // Set requested joint velocities
    vpColVector q_dot(6);
    q_dot[0] = 0.1;    // Joint 1 velocity in rad/s
    q_dot[1] = 0.2;    // Joint 2 velocity in rad/s
    q_dot[2] = 0.3;    // Joint 3 velocity in rad/s
    q_dot[3] = M_PI/8; // Joint 4 velocity in rad/s
    q_dot[4] = M_PI/4; // Joint 5 velocity in rad/s
    q_dot[5] = M_PI/16;// Joint 6 velocity in rad/s

    vpRobotDenso6577 robot;

    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

    // Moves the joint in velocity
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);

    // Initialisation of the velocity measurement
    vpColVector q_dot_mes; // Measured velocities
    robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes); // q_dot_mes =0
    // q_dot_mes is resized to 6, the number of joint

    double timestamp;
    while (1) {
      q_dot_mes = robot.getVelocity(vpRobot::ARTICULAR_FRAME, timestamp);
      vpTime::wait(40); // wait 40 ms
      // here q_dot_mes is equal to [0.1, 0.2, 0.3, M_PI/8, M_PI/4, M_PI/16]
    }
  #endif
  }
  \endcode
*/
vpColVector vpRobotDenso6577::getVelocity(vpRobot::vpControlFrameType frame, double &timestamp)
{
  vpColVector velocity;
  getVelocity(frame, velocity, timestamp);

  return velocity;
}

/*!

  Get robot velocities.

  The behavior is the same than getVelocity(const vpRobot::vpControlFrameType,
  double &) except that the timestamp is not returned.

  */
vpColVector vpRobotDenso6577::getVelocity(vpRobot::vpControlFrameType frame)
{
  vpColVector velocity;
  double timestamp;
  getVelocity(frame, velocity, timestamp);

  return velocity;
}

/*!

  Read joint positions in a specific Viper850 position file.

  This position file has to start with a header. The six joint positions
  are given after the "R:" keyword. The first 3 values correspond to the
  joint translations X,Y,Z expressed in meters. The 3 last values
  correspond to the joint rotations A,B,C expressed in degres to be more
  representative for the user. Theses values are then converted in
  radians in \e q. The character "#" starting a line indicates a
  comment.

  A typical content of such a file is given below:

  \code
  #Viper - Position - Version 1.0
  # file: "myposition.pos "
  #
  # R: A B C D E F
  # Joint position in degrees
  #

  R: 0.1 0.3 -0.25 -80.5 80 0
  \endcode

  \param filename : Name of the position file to read.

  \param q : The six joint positions. Values are expressed in radians.

  \return true if the positions were successfully readen in the file. false, if
  an error occurs.

  The code below shows how to read a position from a file and move the robot to
  this position.
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Enable the position control of the robot
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    // Get the current robot joint positions
    vpColVector q;        // Current joint position
    robot.getPosition(vpRobot::ARTICULAR_FRAME, q);

    // Save this position in a file named "current.pos"
    robot.savePosFile("current.pos", q);

    // Get the position from a file and move to the registered position
    robot.readPosFile("current.pos", q); // Set the joint position from the file

    robot.setPositioningVelocity(5); // Positioning velocity set to 5%
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
  #endif
  }
  \endcode

  \sa savePosFile()
*/

bool vpRobotDenso6577::readPosFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#Viper850 - Position");
  bool pos_found = false;
  int lineNum = 0;

  q.resize(njoint);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Viper850 position file
        std::cout << "Error: this position file " << filename << " is not for Viper850 robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < njoint + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < njoint + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < njoint; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  q.deg2rad();

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Viper850 robot in " << filename << std::endl;
    return false;
  }

  return true;
}

/*!

  Save joint (articular) positions in a specific Viper850 position file.

  This position file starts with a header on the first line. After
  convertion of the rotations in degrees, the joint position \e q is
  written on a line starting with the keyword "R: ". See readPosFile()
  documentation for an example of such a file.

  \param filename : Name of the position file to create.

  \param q : The six joint positions to save in the
  filename. Values are expressed in radians.

  \warning All the six joint rotations written in the file are converted
  in degrees to be more representative for the user.

  \return true if the positions were successfully saved in the file. false, if
  an error occurs.

  \sa readPosFile()
*/

bool vpRobotDenso6577::savePosFile(const std::string &filename, const vpColVector &q)
{

  FILE *fd;
  fd = fopen(filename.c_str(), "w");
  if (fd == nullptr)
    return false;

  fprintf(fd, "\
#Viper850 - Position - Version 1.00\n\
#\n\
# R: A B C D E F\n\
# Joint position in degrees\n\
#\n\
#\n\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf\n", vpMath::deg(q[0]), vpMath::deg(q[1]), vpMath::deg(q[2]),
          vpMath::deg(q[3]), vpMath::deg(q[4]), vpMath::deg(q[5]));

  fclose(fd);
  return (true);
}

/*!

  Moves the robot to the joint position specified in the filename. The
  positioning velocity is set to 10% of the robot maximal velocity.

  \param filename : File containing a joint position.

  \sa readPosFile

*/
void vpRobotDenso6577::move(const std::string &filename)
{
  vpColVector q;

  try {
    this->readPosFile(filename, q);
    this->setRobotState(vpRobot::STATE_POSITION_CONTROL);
    this->setPositioningVelocity(10);
    this->setPosition(vpRobot::ARTICULAR_FRAME, q);
  }
  catch (...) {
    throw;
  }
}

/*!

  Get the robot displacement since the last call of this method.

  \warning This functionnality is not implemented for the moment in the
  cartesian space. It is only available in the joint space
  (vpRobot::ARTICULAR_FRAME).

  \param frame : The frame in which the measured displacement is expressed.

  \param displacement : The measured displacement since the last call
  of this method. The dimension of \e displacement is always
  6. Translations are expressed in meters, rotations in radians.

  In camera or reference frame, rotations are expressed with the
  Euler Rxyz representation.

*/
void vpRobotDenso6577::getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement)
{
  displacement.resize(6);
  displacement = 0;

  double q[6];
  vpColVector q_cur(6);


  // Get the current joint position
  // // Try(PrimitiveACQ_POS_Viper850(q, &timestamp));
  // TODO: get the joint position from the low level controller
  getJointPosition(q);
  for (unsigned int i = 0; i < njoint; i++) {
    q_cur[i] = q[i];
  }

  if (!m_first_time_getdis) {
    switch (frame) {
    case vpRobot::CAMERA_FRAME: {
      std::cout << "getDisplacement() CAMERA_FRAME not implemented\n";
      return;
    }

    case vpRobot::ARTICULAR_FRAME: {
      displacement = q_cur - m_q_prev_getdis;
      break;
    }

    case vpRobot::REFERENCE_FRAME: {
      std::cout << "getDisplacement() REFERENCE_FRAME not implemented\n";
      return;
    }

    case vpRobot::MIXT_FRAME: {
      std::cout << "getDisplacement() MIXT_FRAME not implemented\n";
      return;
    }
    case vpRobot::END_EFFECTOR_FRAME: {
      std::cout << "getDisplacement() END_EFFECTOR_FRAME not implemented\n";
      return;
    }
    }
  }
  else {
    m_first_time_getdis = false;
  }

  // Memorize the joint position for the next call
  m_q_prev_getdis = q_cur;

}

/*!

  Bias the force/torque sensor.

  \sa unbiasForceTorqueSensor(), getForceTorque()

*/
void vpRobotDenso6577::biasForceTorqueSensor()
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  m_ati.bias();
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. // Try sudo "
                    "apt-get install libcomedi-dev"));
#endif
#else // Use serial link


  // Try(PrimitiveTFS_BIAS_Viper850());

  // Wait 500 ms to be sure the next measures take into account the bias
  vpTime::wait(500);

  ();
  if (// TryStt < 0) {
    vpERROR_TRACE("Cannot bias the force/torque sensor.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot bias the force/torque sensor.");
}
#endif
}

/*!

  Unbias the force/torque sensor.

  \sa biasForceTorqueSensor(), getForceTorque()

*/
void vpRobotDenso6577::unbiasForceTorqueSensor()
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  m_ati.unbias();
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. // Try sudo "
                    "apt-get install libcomedi-dev"));
#endif
#else // Use serial link
// Not implemented
#endif
}

/*!

  Get the rough force/torque sensor measures.

  \param H: [Fx, Fy, Fz, Tx, Ty, Tz] Forces/torques measured by the sensor.

  The code below shows how to get the force/torque measures after a sensor
bias.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpTime.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpColVector  H; // force/torque measures [Fx, Fy, Fz, Tx, Ty, Tz]

    vpRobotDenso6577 robot;

    // Bias the force/torque sensor
    robot.biasForceTorqueSensor();

    for (unsigned int i=0; i< 10; i++) {
      robot.getForceTorque(H) ;
      std::cout << "Measured force/torque: " << H.t() << std::endl;
      vpTime::wait(5);
    }
  #endif
  }
  \endcode

  \exception vpRobotException::lowLevelError : If the force/torque measures
  cannot be get from the low level controller.

  \sa biasForceTorqueSensor(), unbiasForceTorqueSensor()

*/
void vpRobotDenso6577::getForceTorque(vpColVector &H) const
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  H = m_ati.getForceTorque();
#else
  (void)H;
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. // Try sudo "
                    "apt-get install libcomedi-dev"));
#endif
#else // Use serial link


  H.resize(6);

  // Try(PrimitiveTFS_ACQ_Viper850(H.data));

  ();
  if (// TryStt < 0) {
    vpERROR_TRACE("Cannot get the force/torque measures.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get force/torque measures.");
}
#endif
}

/*!

  Get the rough force/torque sensor measures.

  \return [Fx, Fy, Fz, Tx, Ty, Tz] Forces/torques measured by the sensor.

  The code below shows how to get the force/torque measures after a sensor
bias.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpTime.h>
  #include <visp3/robot/vpRobotDenso6577.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #ifdef VISP_HAVE_VIPER850
    vpRobotDenso6577 robot;

    // Bias the force/torque sensor
    robot.biasForceTorqueSensor();

    for (unsigned int i=0; i< 10; i++) {
      vpColVector H = robot.getForceTorque(); // force/torque measures [Fx, Fy, Fz, Tx, Ty, Tz]
      std::cout << "Measured force/torque: " << H.t() << std::endl;
      vpTime::wait(5);
    }
  #endif
  }
  \endcode

  \exception vpRobotException::lowLevelError : If the force/torque measures
  cannot be get from the low level controller.

  \sa biasForceTorqueSensor(), unbiasForceTorqueSensor()

*/
vpColVector vpRobotDenso6577::getForceTorque() const
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  vpColVector H = m_ati.getForceTorque();
  return H;
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. // Try sudo "
                    "apt-get install libcomedi-dev"));
#endif
#else // Use serial link


  vpColVector H(6);

  // Try(PrimitiveTFS_ACQ_Viper850(H.data));
  return H;

  ();
  if (// TryStt < 0) {
    vpERROR_TRACE("Cannot get the force/torque measures.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get force/torque measures.");
}
return H; // Here to avoid a warning, but should never be called
#endif
}

/*!

  Open the pneumatic two fingers gripper.

  \sa closeGripper()
*/
void vpRobotDenso6577::openGripper()
{

  // Try(PrimitivePneumaticGripper_Viper850(1));
  std::cout << "Open the pneumatic gripper..." << std::endl;
  // ();
  // if (// TryStt < 0) {
  //   throw vpRobotException(vpRobotException::lowLevelError, "Cannot open the gripper.");
  // }
}

/*!

  Close the pneumatic two fingers gripper.

  \sa openGripper()

*/
void vpRobotDenso6577::closeGripper() const
{

  // Try(PrimitivePneumaticGripper_Viper850(0));
  std::cout << "Close the pneumatic gripper..." << std::endl;
  // ();
  // if (// TryStt < 0) {
  //   throw vpRobotException(vpRobotException::lowLevelError, "Cannot close the gripper.");
  // }
}

/*!
  Enable the joint limits on axis number 6. This is the default.

  \sa disbleJoint6Limits()
*/
void vpRobotDenso6577::enableJoint6Limits() const
{

  // Try(PrimitiveREMOVE_JOINT6_LIMITS_Viper850(0));
  std::cout << "Enable joint limits on axis 6..." << std::endl;
  // ();
  // if (// TryStt < 0) {
  //   vpERROR_TRACE("Cannot enable joint limits on axis 6");
  //   throw vpRobotException(vpRobotException::lowLevelError, "Cannot enable joint limits on axis 6.");
  // }
}

/*!
  \warning Each call to this function should be done carefully.

  Disable the joint limits on axis number 6. When joint 6 is outside the
  limits, a call to this function allows to bring the robot to a position
  inside the limits. Don't forget then to call enableJoint6Limits() to reduce
  the working space for joint 6.

  \sa enableJoint6Limits()
*/
void vpRobotDenso6577::disableJoint6Limits() const
{

  // Try(PrimitiveREMOVE_JOINT6_LIMITS_Viper850(1));
  std::cout << "Warning: Disable joint limits on axis 6..." << std::endl;
  // ();
  // if (// TryStt < 0) {
  //   vpERROR_TRACE("Cannot disable joint limits on axis 6");
  //   throw vpRobotException(vpRobotException::lowLevelError, "Cannot disable joint limits on axis 6.");
  // }
}

/*!

  Set the maximal rotation velocity that can be sent to the robot  during a
  velocity control.

  \param w_max : Maximum rotation velocity expressed in rad/s.
*/

void vpRobotDenso6577::setMaxRotationVelocity(double w_max)
{
  vpRobot::setMaxRotationVelocity(w_max);
  setMaxRotationVelocityJoint6(w_max);

  return;
}

/*!

  Set the maximal rotation velocity on joint 6 that is used only during
  velocity joint control.

  This function affects only the velocities that are sent as joint velocities.

  \code
  vpRobotDenso6577 robot;
  robot.setMaxRotationVelocity( vpMath::rad(20) );
  robot.setMaxRotationVelocityJoint6( vpMath::rad(50) );

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
  robot.setVelocity(ARTICULAR_FRAME, v);
  \endcode


  \param w6_max : Maximum rotation velocity expressed in rad/s on joint 6.
*/

void vpRobotDenso6577::setMaxRotationVelocityJoint6(double w6_max)
{
  maxRotationVelocity_joint6 = w6_max;
  return;
}

/*!

  Get the maximal rotation velocity on joint 6 that is used only during
  velocity joint control.

  \return Maximum rotation velocity on joint 6 expressed in rad/s.
*/
double vpRobotDenso6577::getMaxRotationVelocityJoint6() const { return maxRotationVelocity_joint6; }
END_VISP_NAMESPACE

// Work around to avoid warning: libvisp_robot.a(vpRobotDenso6577.cpp.o) has
// no symbols
void dummy_vpRobotDenso6577() {}
