//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Revised by Mingfeng Zhang, 2014
// Build a cascaded PID controller and refine its parameters to enable smooth & stable hovering,
// take-off, landing, moving, and anti-wind manoeuvres
//=========================================================================================

#ifndef QUADROTOR_PID_CONTROLLER_H_
#define QUADROTOR_PID_CONTROLLER_H_

#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Quaternion.hh>
#include <gazebo/sdf/sdf.hh>
#include <gazebo/common/Exception.hh>

#include <gazebo_quadrotor/QuadrotorState.h>
#include <gazebo_quadrotor/QuadrotorCommand.h>
#include <gazebo_quadrotor/QuadrotorInput.h>

#include <control_toolbox/pid.h>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <gazebo_quadrotor/QuadrotorPIDControllerConfig.h>

namespace gazebo
{
  
/// Get a Vector3 parameter from the SDF node, given a default value and if we should complain if the parameter is not defined
inline math::Vector3 getParamVector3(sdf::ElementPtr _sdf, std::string name, math::Vector3 defaultVal, bool fatalIfNotDefined)
{
  if (!_sdf->HasElement(name))
  {
    if(fatalIfNotDefined)
    {
      std::stringstream ss;
      ss<<"GazeboROSQuadrotorPID plugin missing <"<<name<<">, cannot continue";
      ROS_FATAL_STREAM(ss.str());
      gzthrow(ss.str());;
      return math::Vector3();
    }
    else
    {
      ROS_WARN_STREAM("GazeboROSQuadrotorPID plugin missing <"<<name<<">, using default value: "<<defaultVal);
      return defaultVal;
    }
  }
  else
    return _sdf->GetElement(name)->GetValueVector3();
}

/** @brief The PID controller for roll, pitch, yaw */
class QuadrotorPIDController
{
  public: 
    QuadrotorPIDController();
    ~QuadrotorPIDController();
    
    /** @brief Load PID parameters from an SDF node in a world file */
    void Load(sdf::ElementPtr _sdf);
    
    /// Initialize the controllers and the reconfigure server
    void Init(const ros::NodeHandle& nh);
    
    /// Reset the controllers
    void Reset();
    
    /** @brief Update all the PID controllers based on user input
     *  @param state The current state of the quadrotor
     *  @param input The latest input message received
     *  @param dt The time step to take
     *  @return The four calculated quadrotor commands */
    QuadrotorCommand Update(const QuadrotorState &state, const QuadrotorInput &input, double dt);
    
    static const unsigned int ROLL = 0;
    static const unsigned int PITCH = 1;
    static const unsigned int YAW = 2;
    
  private: 
  
    /// Apply the stored gains to the controllers
    void SetGains();
  
    /// Calculate the error between actual and target values, with 2pi check for angular values
    double CalcError(double actual, double target, bool angle = false);
    
    /// Callback for dynamic reconfigure to set gains
    void ReconfigureCallback(gazebo_quadrotor::QuadrotorPIDControllerConfig &config, uint32_t level);
  
    std::vector<control_toolbox::Pid> controllers;   ///< The pid controllers that will be doing the low level stuff (derivative, integration, summation etc)
      
    // The stored gains
    math::Vector3 rollGains;
    math::Vector3 pitchGains;
    math::Vector3 yawGains;
    
    dynamic_reconfigure::Server<gazebo_quadrotor::QuadrotorPIDControllerConfig>* reconfigureServer;
  
};

} // end namespace

#endif
