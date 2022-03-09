#include <quadrotor_input/AltitudeController.h>
#include <quadrotor_msgs/AltitudeDebug.h>
//#include <TooN/TooN.h>
//#include <TooN/Cholesky.h>

AltitudeController::AltitudeController()
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
  
  heightTimeBuffer = ros::Duration((1/nominalPoseRate));
  batteryTimeBuffer = ros::Duration((1/nominalBatteryRate));
  velocityTimeBuffer = ros::Duration((1/nominalVelocityRate));
  
  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose",10, &AltitudeController::PoseCallback, this);
  velocitySub = nh.subscribe<geometry_msgs::TwistStamped>("velocity",10, &AltitudeController::VelocityCallback, this);
  batterySub = nh.subscribe<quadrotor_msgs::BatteryStatus>("battery",10, &AltitudeController::BatteryCallback,this);
  
  // Read gains and other variables
  double ctrl_p, ctrl_i, ctrl_d, ctrl_i_max, ctrl_i_min;
  
  nh_priv.param<double>("ctrl_p", ctrl_p, 0);
  nh_priv.param<double>("ctrl_i", ctrl_i, 0);
  nh_priv.param<double>("ctrl_d", ctrl_d, 0);
  nh_priv.param<double>("ctrl_i_max", ctrl_i_max, 0);
  nh_priv.param<double>("ctrl_i_min", ctrl_i_min, 0);
  
  nh_priv.param<double>("base_throttle_start", baseThrottleSetpointStart, 0);
  nh_priv.param<double>("base_throttle_end", baseThrottleSetpointEnd, 0);
  nh_priv.param<double>("voltage_start", voltageSetpointStart, 0);
  nh_priv.param<double>("voltage_end", voltageSetpointEnd, 0);
  nh_priv.param<double>("mass", mass, 1.0);
  nh_priv.param<double>("pid_limit", pidLimit, 0);
  nh_priv.param<double>("cmd_max", cmdMax, 0);
  nh_priv.param<double>("cmd_min", cmdMin, 0);
  
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
  
  if(cmdMax < cmdMin)
  {
    ROS_FATAL_STREAM("Max command ("<<cmdMax<<") less than min command ("<<cmdMin<<"), should be the other way around!");
    ros::shutdown();
    return;
  }
  
  zCtrl.setGains(ctrl_p, ctrl_i, ctrl_d, ctrl_i_max, ctrl_i_min);
  
  altitudeDebugPub = nh_priv.advertise<quadrotor_msgs::AltitudeDebug>("altitude_debug", 1);
  reconfigureServer.setCallback(boost::bind(&AltitudeController::ReconfigureCallback, this, _1, _2));
  
  // IMPORTANT! If we don't define which channels we're using then ControllerBase won't let us send anything on those channels
  controllerChannels.insert(THROTTLE);
  
  // Initialize some values
  ros::Time nowTime = ros::Time::now();
  lastHeightTime = nowTime;
  lastBatteryTime = nowTime;
  lastVelocityTime = nowTime;
  
  currentHeight = 0;
  currentVelocity = 0;
}

void AltitudeController::ReconfigureCallback(quadrotor_input::AltitudeControllerConfig &config, uint32_t level)
{
  static bool first = true;
  
  // On first call write the values we got from the parameter server to config
  if(first)
  {
    first = false;
    
    zCtrl.getGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
    config.base_throttle_start = baseThrottleSetpointStart;
    config.base_throttle_end = baseThrottleSetpointEnd;
    config.voltage_start = voltageSetpointStart;
    config.voltage_end = voltageSetpointEnd;
    config.mass = mass;
    config.pid_limit = pidLimit;
    config.cmd_max = cmdMax;
    config.cmd_min = cmdMin;
  }
  else  // Afterwards treat config calls as normal ie apply them directly
  {
    zCtrl.setGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
    
    if(config.base_throttle_start > config.base_throttle_end)
    {
      config.base_throttle_start = config.base_throttle_end;
    }
    
    if(config.voltage_start < config.voltage_end)
    {
      config.voltage_start = config.voltage_end;
    }
    
    if(config.cmd_max < config.cmd_min)
    {
      config.cmd_max = config.cmd_min;
    }
    
    baseThrottleSetpointStart = config.base_throttle_start;
    baseThrottleSetpointEnd = config.base_throttle_end;
    voltageSetpointStart = config.voltage_start;
    voltageSetpointEnd = config.voltage_end;
    mass = config.mass;
    pidLimit = config.pid_limit;
    cmdMax = config.cmd_max;
    cmdMin = config.cmd_min;
  }
  
  boost::mutex::scoped_lock lock(commandMutex);
  currentPath.poses.resize(1);
  currentPath.poses[0].pose.position.z = config.z_des;
  currentPath.poses[0].header.stamp = ros::Time::now();
  
  std::cout<<">>>>>>>>>>>>>>>>>> Setting desired height: "<<config.z_des<<std::endl;
  zCtrl.reset();
}

double AltitudeController::GetDesiredHeight()
{
  boost::mutex::scoped_lock lock(commandMutex);
  ros::Time nowTime = ros::Time::now();
  
  ROS_ASSERT(currentPath.poses.size() > 0);
  
  // First remove all old poses from the path
  if(currentPath.poses.size() > 1)
  {
    std::vector<geometry_msgs::PoseStamped>::iterator pose_it = currentPath.poses.begin();
    while(pose_it->header.stamp <= nowTime && pose_it != currentPath.poses.end())
      ++pose_it;
      
    ROS_ASSERT(pose_it != currentPath.poses.begin());  // this means we haven't even started the first pose yet, but we should be processing first pose immediately
    --pose_it; // back up one step before deleting
    if(pose_it != currentPath.poses.begin())
      currentPath.poses.erase(currentPath.poses.begin(), pose_it);
  }
  
  // Now, the pose at index 0 contains the pose we are working on
  return currentPath.poses[0].pose.position.z;
}

void AltitudeController::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  lastHeightTime = poseMsg->header.stamp;
  if(errorHistory.size() > 0 && lastHeightTime == errorHistory.back().first)  // same timestamp as what we already have, don't use (happens if Gazebo is running slower than control loop)
    return;
  
  currentHeight = poseMsg->pose.position.z;
  desiredHeight = GetDesiredHeight();
  
  double err = desiredHeight - currentHeight;
  
  //std::cout<<"Pushing "<<err<<" at time "<<lastHeightTime<<" in buffer"<<std::endl;
  errorHistory.push_back(std::make_pair(lastHeightTime, err));
}

void AltitudeController::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
{
  currentVelocity = velocityMsg->twist.linear.z;
  lastVelocityTime = velocityMsg->header.stamp;
  //std::cout<<"Got velocity of "<<currentVelocity<<", time: "<<lastVelocityTime<<std::endl;
}

void AltitudeController::BatteryCallback(const quadrotor_msgs::BatteryStatus::ConstPtr& batteryMsg)
{
  voltageHistory.push_back(batteryMsg->voltage);
  //currentBatteryVoltage = batteryMsg->voltage;
  lastBatteryTime = batteryMsg->header.stamp;
  //std::cout<<"Got battery, time: "<<lastBatteryTime<<std::endl;
}

double AltitudeController::CalcErrorDerivative()
{
  // If we haven't received a velocity message in a while, 
  bool discreteDeriv = (ros::Time::now() - lastVelocityTime > velocityTimeBuffer); 
  
  if(discreteDeriv)
  {
    std::cout<<"Haven't received velocity message, using discrete derivative"<<std::endl;
    int historySize = errorHistory.size();
      
    if(historySize < 2)
      return 0;
      
    ros::Duration dt = errorHistory[historySize-1].first - errorHistory[historySize-2].first;
    
    if(dt == ros::Duration(0))
        return 0;
    
    double dError = errorHistory[historySize-1].second - errorHistory[historySize-2].second;
    
    std::cout<<"dError: "<<dError<<" dt: "<<dt<<std::endl;
    
    return dError/dt.toSec();
  }
  else
  {
    return -1*currentVelocity;
  }
  /*
  std::cout<<"Error history size: "<<errorHistory.size()<<std::endl;
  
  if(errorHistory.size() == 0)
    return 0;
  
  TooN::Matrix<MEAS_BUFFER_SIZE, 2> X;
  TooN::Vector<MEAS_BUFFER_SIZE> y;
  
  double firstTime = errorHistory[0].first.toSec();
  
  for(unsigned i=0; i < errorHistory.size(); ++i)
  {
    double t = errorHistory[i].first.toSec() - firstTime;
    double err = errorHistory[i].second;
    X(i,0) = t;
    X(i,1) = 1;
    y[i] = err;
  }
  
  std::cout<<"X: "<<std::endl<<X<<std::endl;
  std::cout<<"y: "<<y<<std::endl;
  
  TooN::Matrix<2> A = X.T() * X;
  TooN::Vector<2> b = X.T() * y;
  TooN::Cholesky<2> chol(A);
  
  TooN::Vector<2> beta = chol.backsub(b);
  std::cout<<"beta: "<<beta<<std::endl;
  return beta[0];
  */
}

double AltitudeController::CalcBatteryVoltage()
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

double AltitudeController::CalcBaseThrottle(double voltage)
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
    
    std::cout<<"slope: "<<slope<<" intercept: "<<intercept<<std::endl;
    baseThrottle = slope*voltage + intercept;
  }
  std::cout<<"baseThrottle: "<<baseThrottle<<std::endl;
  return baseThrottle;
}

double AltitudeController::CalcScale(double baseThrottle)
{
  if(baseThrottle <= baseThrottleSetpointStart)
    return 1.0;
  else
    return baseThrottle/baseThrottleSetpointStart;
  //return baseThrottle/(mass*9.81);
}

bool AltitudeController::Init()
{
  zCtrl.reset();
  lastControlTime = ros::Time::now();
  return true;
}

// Main computation function
void AltitudeController::Compute(quadrotor_msgs::RadioControl& rcMsg, quadrotor_msgs::ControllerError& errorMsg)
{
  // Check if we have some new gains to apply
  boost::mutex::scoped_lock lock(commandMutex);
  if(newGains.size() > 0) // yes, new gains
  {
    ROS_ASSERT(newGains.size() == 3 || newGains.size() == 5);  // either P,I,D or P,I,D,Imax,Imin
    
    double currP, currI, currD, currImax, currImin;
    zCtrl.getGains(currP, currI, currD, currImax, currImin);
    
    if(newGains.size() == 3)
      zCtrl.setGains(newGains[0], newGains[1], newGains[2], currImax, currImin);
    else
      zCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      
    newGains.clear();
  }
  lock.unlock();
  
  ros::Time nowTime = ros::Time::now();
  
  if(nowTime - lastHeightTime > heightTimeBuffer)  // too long since last height message
  {
    ROS_ERROR("Last height time too far in the past!");
    ROS_ERROR_STREAM("Last height time: "<<lastHeightTime<<"  now time: "<<nowTime<<"  buffer: "<<heightTimeBuffer);
    rcMsg.throttle = -10;   // set to invalid
    return;
  }
  
  if(nowTime - lastBatteryTime > batteryTimeBuffer)  // too long since last battery message
  {
    ROS_ERROR("Last battery time too far in the past!");
    rcMsg.throttle = -10;   // set to invalid
    return;
  }
  
  if(errorHistory.size() == 0)
  {
    ROS_WARN("Error history size is zero!");
    rcMsg.throttle = -10;   // set to invalid
    return;
  }
  
  ros::Duration step = nowTime - lastControlTime;
  lastControlTime = nowTime;
  
  double currentBatteryVoltage = CalcBatteryVoltage();
  double error = errorHistory.back().second;
  double errorDerivative = CalcErrorDerivative();
  
  double baseThrottle = CalcBaseThrottle(currentBatteryVoltage);
  double scale = CalcScale(baseThrottle);
  
  double pidVal = zCtrl.updatePid(-1*error, -1*errorDerivative, step) * scale;
  double pidValLimited = pidVal;
    
  // Apply limit to pid output
  if(pidValLimited > pidLimit)
    pidValLimited = pidLimit;
  else if(pidValLimited< -1*pidLimit)
    pidValLimited = -1*pidLimit;
      
  //std::cout<<"desired: "<<desiredHeight<<" current: "<<currentHeight;
  //std::cout<<"  PID val: "<<pidVal<<" Limited: "<<pidValLimited<<std::endl;
    
  rcMsg.throttle = baseThrottle + pidValLimited;
  
  if(rcMsg.throttle > cmdMax)
    rcMsg.throttle = cmdMax;
  else if(rcMsg.throttle < cmdMin)
    rcMsg.throttle = cmdMin;
    
  if(rcMsg.throttle > 1.0)
    rcMsg.throttle = 1.0;
  else if(rcMsg.throttle < -1.0)
    rcMsg.throttle = -1.0;
    
  errorMsg.error.z = error;
  errorMsg.derivative.z = errorDerivative;
    
  // Publish some debug info
  quadrotor_msgs::AltitudeDebug altitudeDebugMsg;
  zCtrl.getCurrentPIDErrors(&altitudeDebugMsg.p_error, &altitudeDebugMsg.i_error, &altitudeDebugMsg.d_error);
  
  // This is necessary because the pid controller's internal error states are the opposite of ours
  altitudeDebugMsg.p_error *= -1;
  altitudeDebugMsg.i_error *= -1;
  altitudeDebugMsg.d_error *= -1;
  
  altitudeDebugMsg.cmd = pidVal;
  altitudeDebugMsg.cmd_limited = pidValLimited;
  altitudeDebugMsg.z = currentHeight;
  altitudeDebugMsg.z_des = desiredHeight;
  altitudeDebugMsg.base_throttle = baseThrottle;
  altitudeDebugMsg.throttle = rcMsg.throttle;
  altitudeDebugMsg.scale = scale;
  altitudeDebugMsg.voltage = currentBatteryVoltage;
  
  altitudeDebugPub.publish(altitudeDebugMsg);
}
