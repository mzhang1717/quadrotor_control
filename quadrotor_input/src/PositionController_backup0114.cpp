#include <quadrotor_input/PositionController.h>
#include <quadrotor_msgs/PositionDebug.h>
#include <tf/transform_datatypes.h>
//#include <TooN/TooN.h>
//#include <TooN/Cholesky.h>

void constrainVector3(geometry_msgs::Vector3& val, const geometry_msgs::Vector3& min, const geometry_msgs::Vector3& max)
{
  if(val.x > max.x)
    val.x = max.x;
  if(val.x < min.x)
    val.x = min.x;
    
  if(val.y > max.y)
    val.y = max.y;
  if(val.y < min.y)
    val.y = min.y;
    
  if(val.z > max.z)
    val.z = max.z;
  if(val.z < min.z)
    val.z = min.z;
}

void constrainVector3(geometry_msgs::Vector3& val, double min, double max)
{
  if(val.x > max)
    val.x = max;
  if(val.x < min)
    val.x = min;
    
  if(val.y > max)
    val.y = max;
  if(val.y < min)
    val.y = min;
    
  if(val.z > max)
    val.z = max;
  if(val.z < min)
    val.z = min;
}

geometry_msgs::Vector3 operator*(geometry_msgs::Vector3 vec, double val)
{
  vec.x *= val;
  vec.y *= val;
  vec.z *= val;
  return vec;
}

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2)
{
  vec1.x -= vec2.x;
  vec1.y -= vec2.y;
  vec1.z -= vec2.z;
  return vec1;
}

geometry_msgs::Point vector3ToPoint(geometry_msgs::Vector3 vec)
{
  geometry_msgs::Point p;
  p.x = vec.x;
  p.y = vec.y;
  p.z = vec.z;
  return p;
}

geometry_msgs::Vector3 pointToVector3(geometry_msgs::Point p)
{
  geometry_msgs::Vector3 vec;
  vec.x = p.x;
  vec.y = p.y;
  vec.z = p.z;
  return vec;
}

std::ostream& operator<< (std::ostream& stream, const geometry_msgs::Vector3& vec)
{
  stream<<"["<<vec.x<<", "<<vec.y<<", "<<vec.z<<"]";
  return stream;
}

// expressed a point (pointInWorld) that is defined in world frame in the body frame  defined by (pose)
geometry_msgs::Point worldToPose(geometry_msgs::Pose pose, geometry_msgs::Point pointInWorld, bool rotOnly = false)
{
  tf::Pose tfPose;
  tf::poseMsgToTF(pose, tfPose);
  
  if(rotOnly)
    tfPose.setOrigin(tf::Point(0,0,0));
  
  tf::Point tfPointInWorld;
  tf::pointMsgToTF(pointInWorld, tfPointInWorld);
  
  tf::Point tfPointInBody = tfPose.inverse() * tfPointInWorld;
  
  geometry_msgs::Point pointInBody;
  tf::pointTFToMsg(tfPointInBody, pointInBody);
  return pointInBody;
}

PositionController::PositionController()
: errorHistory(MEAS_BUFFER_SIZE)
, voltageHistory(MEAS_BUFFER_SIZE)
{
  double nominalPoseRate;
  if(!nh_priv.getParam("nominal_pose_rate", nominalPoseRate))
  {
    ROS_FATAL("Need to set nominal_pose_rate param!");
    ros::shutdown();
    return;
  }
  
  double nominalVelocityRate;
  if(!nh_priv.getParam("nominal_velocity_rate", nominalVelocityRate))
  {
    ROS_FATAL("Need to set nominal_velocity_rate param!");
    ros::shutdown();
    return;
  }
  
  double nominalBatteryRate;
  if(!nh_priv.getParam("nominal_battery_rate", nominalBatteryRate))
  {
    ROS_FATAL("Need to set nominal_battery_rate param!");
    ros::shutdown();
    return;
  }
  
  positionTimeBuffer = ros::Duration((1/nominalPoseRate));
  batteryTimeBuffer = ros::Duration((1/nominalBatteryRate));
  velocityTimeBuffer = ros::Duration((1/nominalVelocityRate));
  
  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose",10, &PositionController::PoseCallback, this);
  velocitySub = nh.subscribe<geometry_msgs::TwistStamped>("velocity",10, &PositionController::VelocityCallback, this);
  batterySub = nh.subscribe<quadrotor_msgs::BatteryStatus>("battery",10, &PositionController::BatteryCallback,this);
  
  // Read gains and other variables
  double ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min;
  double ctrl_z_p, ctrl_z_i, ctrl_z_d, ctrl_z_i_max, ctrl_z_i_min;
  
  // Read parameters from launch file (position_controller_x8.launch)
  // Note that these parameters are defined within this node's namespace (see the tag <node ... /node>)
  // Therefore, nh_priv is used here because it is defined to read private names (see its initilization in ControllerBase::ControllerBase() with '~') 
  nh_priv.param<double>("ctrl_xy_p", ctrl_xy_p, 0);
  nh_priv.param<double>("ctrl_xy_i", ctrl_xy_i, 0);
  nh_priv.param<double>("ctrl_xy_d", ctrl_xy_d, 0);
  nh_priv.param<double>("ctrl_xy_i_max", ctrl_xy_i_max, 0);
  nh_priv.param<double>("ctrl_xy_i_min", ctrl_xy_i_min, 0);
  nh_priv.param<double>("pid_xy_limit", pidLimit.x, 0);  pidLimit.y = pidLimit.x;
  nh_priv.param<double>("cmd_xy_max", cmdMax.x, 0);  cmdMax.y = cmdMax.x;
  nh_priv.param<double>("cmd_xy_min", cmdMin.x, 0);   cmdMin.y = cmdMin.x;
  
  nh_priv.param<double>("ctrl_z_p", ctrl_z_p, 0);
  nh_priv.param<double>("ctrl_z_i", ctrl_z_i, 0);
  nh_priv.param<double>("ctrl_z_d", ctrl_z_d, 0);
  nh_priv.param<double>("ctrl_z_i_max", ctrl_z_i_max, 0);
  nh_priv.param<double>("ctrl_z_i_min", ctrl_z_i_min, 0);
  nh_priv.param<double>("pid_z_limit", pidLimit.z, 0);
  nh_priv.param<double>("cmd_z_max", cmdMax.z, 0);
  nh_priv.param<double>("cmd_z_min", cmdMin.z, 0);
  
  nh_priv.param<double>("base_throttle_start", baseThrottleSetpointStart, 0);
  nh_priv.param<double>("base_throttle_end", baseThrottleSetpointEnd, 0);
  nh_priv.param<double>("voltage_start", voltageSetpointStart, 0);
  nh_priv.param<double>("voltage_end", voltageSetpointEnd, 0);
  nh_priv.param<double>("mass", mass, 1.0);
  
  if(baseThrottleSetpointStart > baseThrottleSetpointEnd)
  {
    ROS_FATAL_STREAM("Base throttle start ("<<baseThrottleSetpointStart<<") greater than end ("<<baseThrottleSetpointEnd<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  if(voltageSetpointStart < voltageSetpointEnd)
  {
    ROS_FATAL_STREAM("Voltage start ("<<voltageSetpointStart<<") less than end ("<<voltageSetpointEnd<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  if(cmdMax.x < cmdMin.x)
  {
    ROS_FATAL_STREAM("Max x command ("<<cmdMax.x<<") less than min x command ("<<cmdMin.x<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  if(cmdMax.y < cmdMin.y)
  {
    ROS_FATAL_STREAM("Max y command ("<<cmdMax.y<<") less than min y command ("<<cmdMin.y<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  if(cmdMax.z < cmdMin.z)
  {
    ROS_FATAL_STREAM("Max z command ("<<cmdMax.z<<") less than min z command ("<<cmdMin.z<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  
  
  xCtrl.setGains(ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min);
  yCtrl.setGains(ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min);
  zCtrl.setGains(ctrl_z_p, ctrl_z_i, ctrl_z_d, ctrl_z_i_max, ctrl_z_i_min);  
  
  positionDebugPub = nh_priv.advertise<quadrotor_msgs::PositionDebug>("position_debug", 1);
  reconfigureServer.setCallback(boost::bind(&PositionController::ReconfigureCallback, this, _1, _2));
  
  // IMPORTANT! If we don't define which channels we're using then ControllerBase won't let us send anything on those channels
  controllerChannels.insert(THROTTLE);
  controllerChannels.insert(ROLL);
  controllerChannels.insert(PITCH);
  
  // Initialize some values
  ros::Time nowTime = ros::Time::now();
  lastPositionTime = nowTime;
  lastBatteryTime = nowTime;
  lastVelocityTime = nowTime;
  
  // Not needed since geometry_msgs::Point and geometry_msgs::Vector3 initialize to zeros
  //currentHeight = 0;
  //currentVelocity = 0;
  
  xyActive = true;
}

void PositionController::ReconfigureCallback(quadrotor_input::PositionControllerConfig &config, uint32_t level)
{
  static bool first = true;
  
  // On first call write the values we got from the parameter server to config
  if(first)
  {
    first = false;
    
    xCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    // Don't need this because xCtrl and yCtrl have same gains
    //yCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    zCtrl.getGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
    
    config.base_throttle_start = baseThrottleSetpointStart;
    config.base_throttle_end = baseThrottleSetpointEnd;
    config.voltage_start = voltageSetpointStart;
    config.voltage_end = voltageSetpointEnd;
    config.mass = mass;
    config.pid_xy_limit = pidLimit.x;
    //config.pid_yy_limit = pidLimit.y;
    config.pid_z_limit = pidLimit.z;
    config.cmd_xy_max = cmdMax.x;
    //config.cmd_xy_max = cmdMax.y;
    config.cmd_z_max = cmdMax.z;
    config.cmd_xy_min = cmdMin.x;
    //config.cmd_xy_min = cmdMin.y;
    config.cmd_z_min = cmdMin.z;
    
    config.xy_active = xyActive;
  }
  else  // Afterwards treat config calls as normal ie apply them directly
  {
    xCtrl.setGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    yCtrl.setGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    zCtrl.setGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
   
    if(config.base_throttle_start > config.base_throttle_end)
    {
      config.base_throttle_start = config.base_throttle_end;
    }
    
    if(config.voltage_start < config.voltage_end)
    {
      config.voltage_start = config.voltage_end;
    }
    
    if(config.cmd_xy_max < config.cmd_xy_min)
    {
      config.cmd_xy_max = config.cmd_xy_min;
    }
    
    if(config.cmd_z_max < config.cmd_z_min)
    {
      config.cmd_z_max = config.cmd_z_min;
    }
    
    baseThrottleSetpointStart = config.base_throttle_start;
    baseThrottleSetpointEnd = config.base_throttle_end;
    voltageSetpointStart = config.voltage_start;
    voltageSetpointEnd = config.voltage_end;
    mass = config.mass;
    pidLimit.x = config.pid_xy_limit;
    pidLimit.y = config.pid_xy_limit;
    pidLimit.z = config.pid_z_limit;
    cmdMax.x = config.cmd_xy_max;
    cmdMax.y = config.cmd_xy_max;
    cmdMax.z = config.cmd_z_max;
    cmdMin.x = config.cmd_xy_min;
    cmdMin.y = config.cmd_xy_min;
    cmdMin.z = config.cmd_z_min;
    
    xyActive = config.xy_active;
  }
  
  boost::mutex::scoped_lock lock(commandMutex);
  currentPath.poses.resize(1);
  currentPath.poses[0].pose.position.x = config.x_des;
  currentPath.poses[0].pose.position.y = config.y_des;
  currentPath.poses[0].pose.position.z = config.z_des;
  currentPath.poses[0].header.stamp = ros::Time::now();
  
  std::cout<<">>>>>>>>>>>>>>>>>> Setting desired position: "<<config.x_des<<", "<<config.y_des<<", "<<config.z_des<<std::endl;
  xCtrl.reset();
  yCtrl.reset();
  zCtrl.reset();
}

geometry_msgs::Point PositionController::GetDesiredPosition()
{
  boost::mutex::scoped_lock lock(commandMutex);
  ros::Time nowTime = ros::Time::now();
  
  ROS_ASSERT(currentPath.poses.size() > 0);
  
  /////////////////////////////////////////////////////
  //*****   commented by Mingfeng, Jan 13, 2014   ***//
  // After the modification, it only returns the final destination point
  /////////////////////////////////////////////////////
  
  // First remove all old poses from the path
  //if(currentPath.poses.size() > 1)
  //{
    //std::vector<geometry_msgs::PoseStamped>::iterator pose_it = currentPath.poses.begin();
    //while(pose_it->header.stamp <= nowTime && pose_it != currentPath.poses.end())
      //++pose_it;
      
    //ROS_ASSERT(pose_it != currentPath.poses.begin());  // this means we haven't even started the first pose yet, but we should be processing first pose immediately
    //--pose_it; // back up one step before deleting
    //if(pose_it != currentPath.poses.begin())
      //currentPath.poses.erase(currentPath.poses.begin(), pose_it);
  //}
  
  // Now, the pose at index 0 contains the pose we are working on
  return currentPath.poses[currentPath.poses.size()-1].pose.position;
}

void PositionController::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  lastPositionTime = poseMsg->header.stamp;
  if(errorHistory.size() > 0 && lastPositionTime == errorHistory.back().first)  // same timestamp as what we already have, don't use (happens if Gazebo is running slower than control loop)
    return;
    
  currentPose = poseMsg->pose;
  //currentPosition = poseMsg->pose.position;
  desiredPosition = GetDesiredPosition();
  
  geometry_msgs::Vector3 err = pointToVector3( worldToPose(currentPose, desiredPosition) );
  errorHistory.push_back(std::make_pair(lastPositionTime, err));
}

void PositionController::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
{
  // Current velocity needs to be in body frame because it's used as input to the PID controller's D term
  currentVelocity = pointToVector3( worldToPose(currentPose, vector3ToPoint(velocityMsg->twist.linear), true) );
  //std::cout<<"Velocity world: "<<velocityMsg->twist.linear<<"  body: "<<currentVelocity<<std::endl;
  lastVelocityTime = velocityMsg->header.stamp;
}

void PositionController::BatteryCallback(const quadrotor_msgs::BatteryStatus::ConstPtr& batteryMsg)
{
  voltageHistory.push_back(batteryMsg->voltage);
  //currentBatteryVoltage = batteryMsg->voltage;
  lastBatteryTime = batteryMsg->header.stamp;
  //std::cout<<"Got battery, time: "<<lastBatteryTime<<std::endl;
}

geometry_msgs::Vector3 PositionController::CalcErrorDerivative()
{
  // If we haven't received a velocity message in a while, 
  bool discreteDeriv = (ros::Time::now() - lastVelocityTime > velocityTimeBuffer); 
  
  // The components of this will be zero by default
  geometry_msgs::Vector3 dError_dt;
  
  if(discreteDeriv)
  {
    std::cout<<"Haven't received velocity message, using discrete derivative"<<std::endl;
    int historySize = errorHistory.size();
      
    if(historySize >= 2)
    {
      ros::Duration dt = errorHistory[historySize-1].first - errorHistory[historySize-2].first;
      
      if(dt != ros::Duration(0))
      {
        geometry_msgs::Vector3 dError = errorHistory[historySize-1].second - errorHistory[historySize-2].second;
        dError_dt = dError * (1/dt.toSec());
      }
    }
  }
  else
  {
    dError_dt = currentVelocity * -1;
  }
  
  //std::cout<<"dError_dt: "<<dError_dt.x<<", "<<dError_dt.y<<", "<<dError_dt.z<<std::endl;
  
  return dError_dt;
}

double PositionController::CalcBatteryVoltage()
{
  if(voltageHistory.size() == 0)
    return 0;
  
  double voltage = 0;
  for(unsigned i=0; i < voltageHistory.size(); ++i)
  {
    voltage += voltageHistory[i];
  }
  
  return voltage / voltageHistory.size();
}

double PositionController::CalcBaseThrottle(double voltage)
{
  double baseThrottle;
  
  if(voltage > voltageSetpointStart)
    baseThrottle = baseThrottleSetpointStart;
  else if(voltage < voltageSetpointEnd)
    baseThrottle = baseThrottleSetpointEnd;
  else
  {
    double slope = (baseThrottleSetpointStart - baseThrottleSetpointEnd)/(voltageSetpointStart - voltageSetpointEnd);
    double intercept = baseThrottleSetpointEnd - slope*voltageSetpointEnd;
    
    //std::cout<<"slope: "<<slope<<" intercept: "<<intercept<<std::endl;
    baseThrottle = slope*voltage + intercept;
  }
  //std::cout<<"baseThrottle: "<<baseThrottle<<std::endl;
  return baseThrottle;
}

double PositionController::CalcScale(double baseThrottle)
{
  if(baseThrottle <= baseThrottleSetpointStart)
    return 1.0;
  else
    return baseThrottle/baseThrottleSetpointStart;
  //return baseThrottle/(mass*9.81);
}

bool PositionController::Init()
{
  xCtrl.reset();
  yCtrl.reset();
  zCtrl.reset();
  lastControlTime = ros::Time::now();
  return true;
}

// Main computation function
void PositionController::Compute(quadrotor_msgs::RadioControl &rcMsg, quadrotor_msgs::ControllerError& errorMsg)
{
  // Check if we have some new gains to apply
  boost::mutex::scoped_lock lock(commandMutex);
  if(newGains.size() > 0) // yes, new gains
  {
    // either P,I,D or P,I,D,Imax,Imin for xy and z
    // XY comes before Z, so if you want to specify just P,I,D then first 3 terms are XY ctrl gains, last 3 terms are 
    // Z ctrl gains. If you want to specify P,I,D,Imax,Imin, then first 5 terms are XY ctrl params, last
    // 5 terms are Z ctrl params
    ROS_ASSERT(newGains.size() == 6 || newGains.size() == 10);  
  
    if(newGains.size() == 6)
    {
      double curr_xy_p, curr_xy_i, curr_xy_d, curr_xy_imax, curr_xy_imin;
      double curr_z_p, curr_z_i, curr_z_d, curr_z_imax, curr_z_imin;
      
      xCtrl.getGains(curr_xy_p, curr_xy_i, curr_xy_d, curr_xy_imax, curr_xy_imin);
      zCtrl.getGains(curr_z_p, curr_z_i, curr_z_d, curr_z_imax, curr_z_imin);
    
      xCtrl.setGains(newGains[0], newGains[1], newGains[2], curr_xy_imax, curr_xy_imin);
      yCtrl.setGains(newGains[0], newGains[1], newGains[2], curr_xy_imax, curr_xy_imin);
      zCtrl.setGains(newGains[3], newGains[4], newGains[5], curr_z_imax, curr_z_imin);
    }
    else
    {
      xCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      yCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      zCtrl.setGains(newGains[5], newGains[6], newGains[7], newGains[8], newGains[9]);
    }
      
    // IMPORTANT: clear this otherwise we'll try to set gains gain next time
    newGains.clear();
  }
  lock.unlock();
  
  ros::Time nowTime = ros::Time::now();
  
  if(nowTime - lastPositionTime > positionTimeBuffer)  // too long since last position message
  {
    ROS_ERROR("Last position time too far in the past!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    return;
  }
  
  if(nowTime - lastBatteryTime > batteryTimeBuffer)  // too long since last battery message
  {
    ROS_ERROR("Last battery time too far in the past!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    return;
  }
  
  if(errorHistory.size() == 0)
  {
    ROS_WARN("Error history size is zero!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    return;
  }
  
  ros::Duration step = nowTime - lastControlTime;
  lastControlTime = nowTime;
  
  double currentBatteryVoltage = CalcBatteryVoltage();
  geometry_msgs::Vector3 error = errorHistory.back().second;
  geometry_msgs::Vector3 errorDerivative = CalcErrorDerivative();
  
  double baseThrottle = CalcBaseThrottle(currentBatteryVoltage);
  double scale = CalcScale(baseThrottle);
  
  geometry_msgs::Vector3 pidVal;
  
  pidVal.x = xCtrl.updatePid(-1*error.x, -1*errorDerivative.x, step) * scale;
  pidVal.y = yCtrl.updatePid(-1*error.y, -1*errorDerivative.y, step) * scale;
  pidVal.z = zCtrl.updatePid(-1*error.z, -1*errorDerivative.z, step) * scale;
  
  geometry_msgs::Vector3 pidValLimited = pidVal;
    
  // Apply limit to pid output
  constrainVector3(pidValLimited, pidLimit*-1, pidLimit);

  geometry_msgs::Vector3 commandOut = pidValLimited;
  commandOut.z += baseThrottle;
  
  // Constrain comand out to be between cmdMin and cmdMax
  constrainVector3(commandOut, cmdMin, cmdMax);
  
  // Constrain command out to be between -1.0 and 1.0
  constrainVector3(commandOut, -1.0, 1.0); 

  // Aircraft coordinate system is North-West-Up (X axis out of nose, Y axis to the left, Z axis up)
  rcMsg.roll = -1.0 * commandOut.y;  // positive roll moves aircraft to the right
  rcMsg.pitch = commandOut.x;  // positive pitch moves aircraft forward
  rcMsg.throttle = commandOut.z;
  
  if(!xyActive)  // If xy control not active set roll and pitch to invalid values
  {
    rcMsg.pitch= -10;
    rcMsg.roll = -10;
  }
  
  errorMsg.error = error;
  errorMsg.derivative = errorDerivative;
  
  // Publish some debug info
  quadrotor_msgs::PositionDebug positionDebugMsg;
  xCtrl.getCurrentPIDErrors(&positionDebugMsg.p_error.x, &positionDebugMsg.i_error.x, &positionDebugMsg.d_error.x);
  yCtrl.getCurrentPIDErrors(&positionDebugMsg.p_error.y, &positionDebugMsg.i_error.y, &positionDebugMsg.d_error.y);
  zCtrl.getCurrentPIDErrors(&positionDebugMsg.p_error.z, &positionDebugMsg.i_error.z, &positionDebugMsg.d_error.z);
  
  // This is necessary because the pid controller's internal error states are the opposite of ours
  positionDebugMsg.p_error = positionDebugMsg.p_error * -1;
  positionDebugMsg.i_error = positionDebugMsg.i_error * -1;
  positionDebugMsg.d_error = positionDebugMsg.d_error * -1;
  
  positionDebugMsg.cmd.x = pidVal.x;
  positionDebugMsg.cmd_limited.x = pidValLimited.x;
  positionDebugMsg.cmd.y = pidVal.y;
  positionDebugMsg.cmd_limited.y = pidValLimited.y;
  positionDebugMsg.cmd.z = pidVal.z;
  positionDebugMsg.cmd_limited.z = pidValLimited.z;
  
  positionDebugMsg.position = currentPose.position; //currentPosition;
  positionDebugMsg.desired = desiredPosition;
  
  positionDebugMsg.base_throttle = baseThrottle;
  positionDebugMsg.throttle = rcMsg.throttle;
  positionDebugMsg.roll = rcMsg.roll;
  positionDebugMsg.pitch = rcMsg.pitch;
  positionDebugMsg.scale = scale;
  positionDebugMsg.voltage = currentBatteryVoltage;  
  
  positionDebugPub.publish(positionDebugMsg);
}
