//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Revised by Mingfeng Zhang, 2014
// Build a cascaded PID controller and refine its parameters to enable smooth & stable hovering,
// take-off, landing, moving, and anti-wind manoeuvres
//=========================================================================================

#include <gazebo_quadrotor/QuadrotorPIDController.h>

using namespace gazebo;

// Constructor
QuadrotorPIDController::QuadrotorPIDController()
: controllers(3)
, reconfigureServer(NULL)
{
  
}

// Destructor
QuadrotorPIDController::~QuadrotorPIDController()
{
  if(reconfigureServer)
    delete reconfigureServer;
}

// Callback for dynamic reconfigure
void QuadrotorPIDController::ReconfigureCallback(gazebo_quadrotor::QuadrotorPIDControllerConfig &config, uint32_t level)
{
  static bool first = true;
  
  // On the very first callback we'll set the config values to the values we got from parsing the world file
  // Subsequently the reconfigure values will be applied as expected
  if(first)
  {
    first = false;
    
    config.roll_p = this->rollGains.x;
    config.roll_i = this->rollGains.y;
    config.roll_d = this->rollGains.z;
    
    config.pitch_p = this->pitchGains.x;
    config.pitch_i = this->pitchGains.y;
    config.pitch_d = this->pitchGains.z;
    
    config.yaw_p = this->yawGains.x;
    config.yaw_i = this->yawGains.y;
    config.yaw_d = this->yawGains.z;
  }
  else
  {
    this->rollGains.x = config.roll_p;
    this->rollGains.y = config.roll_i;
    this->rollGains.z = config.roll_d;
    
    this->pitchGains.x = config.pitch_p;
    this->pitchGains.y = config.pitch_i;
    this->pitchGains.z = config.pitch_d;
    
    this->yawGains.x = config.yaw_p;
    this->yawGains.y = config.yaw_i;
    this->yawGains.z = config.yaw_d;
    
    SetGains();
  }
  
}

// Load data from sdf node
void QuadrotorPIDController::Load(sdf::ElementPtr _sdf)
{
  this->rollGains = getParamVector3(_sdf, "rollGains", math::Vector3(0.1,0,0), false);
  this->pitchGains = getParamVector3(_sdf, "pitchGains", math::Vector3(0.1,0,0), false);
  this->yawGains = getParamVector3(_sdf, "yawGains", math::Vector3(0.1,0,0), false);
}

// Initialize reconfigure server and set the gains
void QuadrotorPIDController::Init(const ros::NodeHandle& nh)
{
  reconfigureServer = new dynamic_reconfigure::Server<gazebo_quadrotor::QuadrotorPIDControllerConfig>(nh);
  reconfigureServer->setCallback(boost::bind(&QuadrotorPIDController::ReconfigureCallback, this, _1, _2));
  
  SetGains();
}


// Set the gains to the currently stored values
void QuadrotorPIDController::SetGains()
{
  this->controllers[ROLL].initPid(this->rollGains.x, this->rollGains.y, this->rollGains.z, 1000, -1000);  
  this->controllers[PITCH].initPid(this->pitchGains.x, this->pitchGains.y, this->pitchGains.z, 1000, -1000); 
  this->controllers[YAW].initPid(this->yawGains.x, this->yawGains.y, this->yawGains.z, 1000, -1000);
}

// Reset all the controllers
void QuadrotorPIDController::Reset()
{
  for(unsigned int i=0; i<controllers.size(); i++)
    controllers[i].reset();
  
}

// Update all the PID controllers based on user input
QuadrotorCommand QuadrotorPIDController::Update(const QuadrotorState &state, const QuadrotorInput &input, double dt)
{
  // Calculate desired setpoints
  double roll_des = input.roll * input.rollMax;
  double pitch_des = input.pitch * input.pitchMax;
  double yaw_rate_des = -1 * input.yawRate * input.yawRateMax;
  
  ros::Duration step(dt);
  
  QuadrotorCommand command;
  
  command.U1 = input.throttle * input.throttleMax; 
  command.U2 = controllers[ROLL].updatePid(-1*CalcError(state.roll, roll_des, true), step);
  command.U3 = controllers[PITCH].updatePid(-1*CalcError(state.pitch, pitch_des, true), step);
  command.U4 = controllers[YAW].updatePid(-1*CalcError(state.yawRate, yaw_rate_des, false), step);
  
  return command;
}

// Calculate the error between actual and target values, with 2pi check for angular values
double QuadrotorPIDController::CalcError(double actual, double target, bool angle)
{
  double err = target - actual;
 
  if(angle)
  {
    while (err > M_PI)
      err -= 2*M_PI;
      
    while(err < -M_PI)
      err += 2*M_PI;
  }
  
  return err;
}
