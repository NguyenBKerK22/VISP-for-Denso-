#ifndef vpRobotDenso_H
#define vpRobotDenso_H

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <stdio.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpDenso6577.h>
#include <visp3/io/vpUART.h>
// low level controller api

// If USE_ATI_DAQ defined, use DAQ board instead of serial connexion to
// acquire data using comedi
#define USE_ATI_DAQ

#ifdef USE_ATI_DAQ
#include <visp3/sensor/vpForceTorqueAtiSensor.h>
#endif

BEGIN_VISP_NAMESPACE

class VISP_EXPORT vpRobotDenso6577 : public vpDenso6577, public vpRobot
{
public: /* Constantes */
  /*! \enum vpControlModeType Control mode. */
  typedef enum
  {
    AUTO,   //!< Automatic control mode (default).
    MANUAL, //!< Manual control mode activated when the dead man switch is in
            //!< use.
    ESTOP   //!< Emergency stop activated.
  } vpControlModeType;

  /* Max velocity used during robot control in position.
   * this value could be changed using setPositioningVelocity().
   */
  static const double m_defaultPositioningVelocity; // = 20.0;

private: /* Not allowed functions. */
  /*!
    Copy constructor not allowed.
   */
  vpRobotDenso6577(const vpRobotDenso6577 &robot);

private: /* Attributs prives. */
  /** \brief Vrai ssi aucun objet de la classe vpRobotDenso6577 n'existe.
   *
   * Il ne peut exister simultanement qu'un seul objet de la classe
   * vpRobotDenso6577, car il correspond a un seul robot AFMA6. Creer
   * simultanement deux objets peut engendrer des conflits. Le constructeur
   * lance une erreur si le champ n'est pas FAUX puis positionne le champ
   * a VRAI. Seul le destructeur repositionne le champ a FAUX, ce qui
   * alors la creation d'un nouvel objet.
   */
  static bool m_robotAlreadyCreated;
  serialib serial;
  double m_positioningVelocity;

  // Variables used to compute the measured velocities (see getVelocity() )
  vpColVector m_q_prev_getvel;
  vpHomogeneousMatrix m_fMc_prev_getvel;
  vpHomogeneousMatrix m_fMe_prev_getvel;
  double m_time_prev_getvel;
  bool m_first_time_getvel;

  // Variables used to compute the measured displacement (see
  // getDisplacement() )
  vpColVector m_q_prev_getdis;
  bool m_first_time_getdis;
  vpControlModeType m_controlMode;

#if defined(USE_ATI_DAQ) && defined(VISP_HAVE_COMEDI)
  vpForceTorqueAtiSensor m_ati;
#endif

public: /* Methode publiques */
  VP_EXPLICIT vpRobotDenso6577(bool verbose = true);
  virtual ~vpRobotDenso6577(void);

  // Force/Torque control
  void biasForceTorqueSensor();

  void closeGripper() const;

  void disableJoint6Limits() const;
  void enableJoint6Limits() const;

  void getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement);
  /*!
    \return The control mode indicating if the robot is in automatic,
    manual (usage of the dead man switch) or emergnecy stop mode.
  */
  vpControlModeType getControlMode() const { return m_controlMode; }

  void getForceTorque(vpColVector &H) const;
  vpColVector getForceTorque() const;

  double getMaxRotationVelocityJoint6() const;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position) VP_OVERRIDE;
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position);
  void getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp);

  double getPositioningVelocity(void) const;
  bool getPowerState() const;

  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity);
  void getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp);

  vpColVector getVelocity(const vpRobot::vpControlFrameType frame);
  vpColVector getVelocity(const vpRobot::vpControlFrameType frame, double &timestamp);

  double getTime() const;

  void get_cMe(vpHomogeneousMatrix &cMe) const;
  void get_cVe(vpVelocityTwistMatrix &cVe) const;
  void get_eJe(vpMatrix &eJe) VP_OVERRIDE;
  void get_fJe(vpMatrix &fJe) VP_OVERRIDE;

  void init(void);
  void
    init(vpDenso6577::vpToolType tool,
         vpCameraParameters::vpCameraParametersProjType projModel = vpCameraParameters::perspectiveProjWithoutDistortion);
  void init(vpDenso6577::vpToolType tool, const std::string &filename);
  void init(vpDenso6577::vpToolType tool, const vpHomogeneousMatrix &eMc_);

  void move(const std::string &filename);

  void openGripper();

  void powerOn();
  void powerOff();

  static bool readPosFile(const std::string &filename, vpColVector &q);
  static bool savePosFile(const std::string &filename, const vpColVector &q);

  void set_eMc(const vpHomogeneousMatrix &eMc_);
  void set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_);

  void setMaxRotationVelocity(double w_max);
  void setMaxRotationVelocityJoint6(double w6_max);

  // Position control
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position) VP_OVERRIDE;
  void setPosition(const vpRobot::vpControlFrameType frame, double pos1, double pos2, double pos3, double pos4,
                   double pos5, double pos6);
  void setPosition(const std::string &filename);
  void setPositioningVelocity(double velocity);

  // State
  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  // Velocity control
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &velocity) VP_OVERRIDE;

  void stopMotion();
  void unbiasForceTorqueSensor();

  int uartSend(const uint8_t *data, size_t len);
  bool readUARTJoint(double *q, int timeout_ms = 200, int func = 0);
  bool getJointPosition(double *q, int func = 0);
  bool sendPosition(double *q);

private:
  double maxRotationVelocity_joint6;
};
END_VISP_NAMESPACE
#endif
