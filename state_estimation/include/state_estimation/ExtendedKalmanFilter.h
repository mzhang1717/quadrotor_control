#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <TooN/TooN.h>
#include <state_estimation/kalmanfilter.h>
#include <state_estimation/FilterBase.h>
#include <state_estimation/AircraftModel.h>

#include <boost/circular_buffer.hpp>
#include <boost/any.hpp>

class ExtendedKalmanFilter : public FilterBase
{
  public:
    ExtendedKalmanFilter();
    
  private:
    virtual void imuCallbackInternal(TooN::Vector<3> accel_meas, double accel_cov, double accel_bias_cov, TooN::Vector<3> gyro_meas, double gyro_cov, double gyro_bias_cov, double dt, TooN::Vector<3> gravity, ros::Time stamp);
    virtual void rotCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp);
    virtual void yawCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp);
    virtual void relPoseCallbackInternal(TooN::Vector<3> rel_pos, TooN::SO3<> rel_rot, TooN::Matrix<6> rel_cov, ros::Time stamp);
    virtual void bodyXYVelCallbackInternal(TooN::Vector<2> vel, TooN::Matrix<2> cov, ros::Time stamp);
    virtual void groundDistanceCallbackInternal(double distance, double cov, ros::Time stamp);
    
    virtual void fillStateMessage(state_estimation::AircraftStateMsg &state_msg);
    
    int saveMeas(boost::any meas, ros::Time stamp);
    void filterFromBuffer(int startIdx);
    
    tag::KalmanFilter<AircraftState, AircraftModel> filter;
    
    tf::TransformListener tf_listener;
    
    boost::circular_buffer<AircraftState> cbStates;
    boost::circular_buffer<boost::any> cbMeasurements;  //includes imu data
    
    double rot_r_noise;
#ifdef CALIBRATION
	double pos_c_noise;
	double rot_c_noise;
#endif
    
    WorldRotation<AircraftState> rot_meas_template;
    WorldYaw<AircraftState> yaw_meas_template;
    RelativePose<AircraftState> rel_pose_meas_template;
    BodyXYVelocity<AircraftState> body_vel_meas_template;
    GroundPlaneDistance<AircraftState> ground_distance_meas_template;
    
};

#endif
