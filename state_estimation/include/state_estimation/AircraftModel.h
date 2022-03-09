#ifndef AIRCRAFT_MODEL_H
#define AIRCRAFT_MODEL_H

#include <TooN/TooN.h>
#include <state_estimation/AircraftMeasurements.h>
#include <ros/ros.h>

class AircraftState
{
public:
  AircraftState();

  void reset();
  void printState();
  void printCov();
  void setCovariance(double position_cov, double rotation_cov, double velocity_cov);
  void setCovariance(double sigma);                                                   
  
#ifdef CALIBRATION
  static const int STATE_DIMENSION = 24;
#else
  static const int STATE_DIMENSION = 18;
#endif

  TooN::Vector<3> position;   // world frame
  TooN::SO3<> rotation;       // world frame to body frame, ie transforms vectors from body frame to world frame
  TooN::Vector<3> velocity;   // world frame
  TooN::Vector<3> accel_bias;
  TooN::Vector<3> gyro_bias;
  TooN::SO3<> rot_r;          // rotation of relative pose world frame in inertial frame
#ifdef CALIBRATION
  // Relative pose sensor offsets from imu
  TooN::Vector<3> pos_c; 
  TooN::SO3<> rot_c;
#endif

  TooN::Matrix<STATE_DIMENSION> covariance;
  
  // These are saved directly from the IMU input, not an estimated state.
  // However, some measurement functions might need them.
  TooN::Vector<3> last_accel;
  TooN::Vector<3> last_gyro;
  
  ros::Time stamp;
};


class AircraftInput
{
public:
  AircraftInput();
  
  TooN::Vector<3> accel;
  TooN::Vector<3> gyro;
  double accel_noise;
  double gyro_noise;
  double accel_bias_noise;
  double gyro_bias_noise;
  
  double rot_r_noise;
#ifdef CALIBRATION
  double pos_c_noise; 
  double rot_c_noise;
#endif
  
  TooN::Vector<3> gravity;
  double dt;
  
  ros::Time stamp;
};

class AircraftModel 
{
public:

  TooN::Matrix<AircraftState::STATE_DIMENSION> jacobian;
  TooN::Matrix<AircraftState::STATE_DIMENSION> noise;
  
  AircraftModel();
  
  // Jacobian has pos, rot, vel, angularVel in this order
  const TooN::Matrix<AircraftState::STATE_DIMENSION> & getJacobian(const AircraftState& state, const AircraftInput& input, const double dt);
  void updateState( AircraftState & state, const AircraftInput& input, const double dt );
  const TooN::Matrix<AircraftState::STATE_DIMENSION> & getNoiseCovariance(const AircraftInput& input, const double dt );
  void updateFromMeasurement( AircraftState & state, const TooN::Vector<AircraftState::STATE_DIMENSION> & innovation );
  
  ros::Publisher accel_pub;
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv;
  
  bool fix_xy_output;
};



#endif
