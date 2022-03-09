#include <quadrotor_input/PositionController.h>
#include <quadrotor_msgs/PositionDebug.h>
#include <quadrotor_msgs/ControlWarn.h>
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

geometry_msgs::Vector3 QuatToEulerRPY(tf::Quaternion q) 
{
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  geometry_msgs::Vector3 EulerRPY;
  
  m.getRPY(roll, pitch, yaw);
  
  EulerRPY.x = roll;
  EulerRPY.y = pitch;
  EulerRPY.z= yaw;
  
  return EulerRPY;
}

geometry_msgs::Quaternion EulerRPYToQuat(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    
    geometry_msgs::Quaternion quat;
    tf::quaternionTFToMsg(q, quat);
    
    return quat;
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
  
  poseSub = nh.subscribe<geometry_msgs::PoseStamped>("pose", 10, &PositionController::PoseCallback, this);
  velocitySub = nh.subscribe<geometry_msgs::TwistStamped>("velocity", 10, &PositionController::VelocityCallback, this);
  batterySub = nh.subscribe<quadrotor_msgs::BatteryStatus>("battery",10, &PositionController::BatteryCallback,this);
  
  // Read gains and other variables
  double ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min;
  double ctrl_z_p, ctrl_z_i, ctrl_z_d, ctrl_z_i_max, ctrl_z_i_min;
  
  // newly added, Jan. 15, 2014
  double ctrl_vxy_p, ctrl_vxy_i, ctrl_vxy_d, ctrl_vxy_i_max, ctrl_vxy_i_min;
  double ctrl_vz_p, ctrl_vz_i, ctrl_vz_d, ctrl_vz_i_max, ctrl_vz_i_min;
  
  // newly added, Jan. 19, 2014
  double ctrl_yaw_p, ctrl_yaw_i, ctrl_yaw_d, ctrl_yaw_i_max, ctrl_yaw_i_min;
  
  // Read parameters from launch file (position_controller_x8.launch)
  // Note that these parameters are defined within this node's namespace (see the tag <node ... /node>)
  // Therefore, nh_priv is used here because it is defined to read private names (see its initilization in ControllerBase::ControllerBase() with '~') 
  
  nh_priv.param<double>("pid_xy_limit", pidLimit.x, 0);  pidLimit.y = pidLimit.x;
  nh_priv.param<double>("cmd_xy_max", cmdMax.x, 0);  cmdMax.y = cmdMax.x;
  nh_priv.param<double>("cmd_xy_min", cmdMin.x, 0);   cmdMin.y = cmdMin.x;
  
  nh_priv.param<double>("pid_z_limit", pidLimit.z, 0);
  nh_priv.param<double>("cmd_z_max", cmdMax.z, 0);
  nh_priv.param<double>("cmd_z_min", cmdMin.z, 0);
  
  nh_priv.param<double>("ctrl_xy_p", ctrl_xy_p, 0);
  nh_priv.param<double>("ctrl_xy_i", ctrl_xy_i, 0);
  nh_priv.param<double>("ctrl_xy_d", ctrl_xy_d, 0);
  nh_priv.param<double>("ctrl_xy_i_max", ctrl_xy_i_max, 0);
  nh_priv.param<double>("ctrl_xy_i_min", ctrl_xy_i_min, 0);
  
  nh_priv.param<double>("ctrl_z_p", ctrl_z_p, 0);
  nh_priv.param<double>("ctrl_z_i", ctrl_z_i, 0);
  nh_priv.param<double>("ctrl_z_d", ctrl_z_d, 0);
  nh_priv.param<double>("ctrl_z_i_max", ctrl_z_i_max, 0);
  nh_priv.param<double>("ctrl_z_i_min", ctrl_z_i_min, 0);
  
  nh_priv.param<double>("base_throttle_start", baseThrottleSetpointStart, 0);
  nh_priv.param<double>("base_throttle_end", baseThrottleSetpointEnd, 0);
  nh_priv.param<double>("voltage_start", voltageSetpointStart, 0);
  nh_priv.param<double>("voltage_end", voltageSetpointEnd, 0);
  nh_priv.param<double>("mass", mass, 1.0);

  // newly added gains anf parameters, Jan. 15, 2014
  nh_priv.param<double>("ctrl_vxy_p", ctrl_vxy_p, 0);
  nh_priv.param<double>("ctrl_vxy_i", ctrl_vxy_i, 0);
  nh_priv.param<double>("ctrl_vxy_d", ctrl_vxy_d, 0);
  nh_priv.param<double>("ctrl_vxy_i_max", ctrl_vxy_i_max, 0);
  nh_priv.param<double>("ctrl_vxy_i_min", ctrl_vxy_i_min, 0);
  
  nh_priv.param<double>("pid_vxy_limit", pidLimit_v.x, 0);  pidLimit_v.y = pidLimit_v.x;
  nh_priv.param<double>("cmd_vxy_max", cmdMax_v.x, 0);  cmdMax_v.y = cmdMax_v.x;
  nh_priv.param<double>("cmd_vxy_min", cmdMin_v.x, 0);  cmdMin_v.y = cmdMin_v.x;
  
  nh_priv.param<double>("ctrl_vz_p", ctrl_vz_p, 0);
  nh_priv.param<double>("ctrl_vz_i", ctrl_vz_i, 0);
  nh_priv.param<double>("ctrl_vz_d", ctrl_vz_d, 0);
  nh_priv.param<double>("ctrl_vz_i_max", ctrl_vz_i_max, 0);
  nh_priv.param<double>("ctrl_vz_i_min", ctrl_vz_i_min, 0);
  
  nh_priv.param<double>("pid_vz_limit", pidLimit_v.z, 0);
  nh_priv.param<double>("cmd_vz_max", cmdMax_v.z, 0);
  nh_priv.param<double>("cmd_vz_min", cmdMin_v.z, 0);
  
  // newly added gains anf parameters, Jan. 19, 2014
  nh_priv.param<double>("ctrl_yaw_p", ctrl_yaw_p, 0);
  nh_priv.param<double>("ctrl_yaw_i", ctrl_yaw_i, 0);
  nh_priv.param<double>("ctrl_yaw_d", ctrl_yaw_d, 0);
  nh_priv.param<double>("ctrl_yaw_i_max", ctrl_yaw_i_max, 0);
  nh_priv.param<double>("ctrl_yaw_i_min", ctrl_yaw_i_min, 0);
  
  nh_priv.param<double>("pid_yaw_limit", pidLimit_yaw, 0); 
  nh_priv.param<double>("cmd_yaw_max", cmdMax_yaw, 0);  
  nh_priv.param<double>("cmd_yaw_min", cmdMin_yaw, 0); 
  
  nh_priv.param<double>("tol_position", tolPosition, 0.0025);  
  nh_priv.param<double>("tol_yaw", tolYaw, 0.005);   
  
  nh_priv.param<double>("xyspeedLimit", xyspeedLimit, 1.0);   
  nh_priv.param<double>("zspeedLimit", zspeedLimit, 0.4);  
  
  nh_priv.param<double>("ascendingSpeedMax", ascendingSpeedMax, 0.2);  
  nh_priv.param<double>("descendingSpeedMax", descendingSpeedMax, 0.12);  
  nh_priv.param<double>("touchdownSpeedMax", touchdownSpeedMax, 0.04);  
  
  nh_priv.param<double>("pitchLimit", pitchLimit, 0.1);  
  nh_priv.param<double>("rollLimit", rollLimit, 0.1);  
  nh_priv.param<double>("yawrateLimit", yawrateLimit, 0.3); 
  nh_priv.param<double>("throttleHighLimit", throttleHighLimit, 0.8);  
  nh_priv.param<double>("throttleLowLimit", throttleLowLimit, 0.0); 
  
  nh_priv.param<double>("scaleRP", scaleRP, 34.0); 
  
  convertRP = 180/M_PI/34.0;
  
  
  ///////TAU Controller/////////////////////////
  //nh_priv.param<double>("tau_p", tau_p, 0.5);
  //nh_priv.param<double>("tau_i", tau_i, 0.3);
  //nh_priv.param<double>("tau_d", tau_d, 0.0);
  //nh_priv.param<double>("tau_f", tau_f, 5.0);
  //nh_priv.param<double>("tau_k", tau_k, 0.4);
  ////////////////////
  
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
  
  
  pCtrl.setGains(ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min);
  
  //xCtrl.setGains(ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min);
  //yCtrl.setGains(ctrl_xy_p, ctrl_xy_i, ctrl_xy_d, ctrl_xy_i_max, ctrl_xy_i_min);
  zCtrl.setGains(ctrl_z_p, ctrl_z_i, ctrl_z_d, ctrl_z_i_max, ctrl_z_i_min);  
  
  // newly added, Jan. 15, 2014
  vxCtrl.setGains(ctrl_vxy_p, ctrl_vxy_i, ctrl_vxy_d, ctrl_vxy_i_max, ctrl_vxy_i_min);
  vyCtrl.setGains(ctrl_vxy_p, ctrl_vxy_i, ctrl_vxy_d, ctrl_vxy_i_max, ctrl_vxy_i_min);
  vzCtrl.setGains(ctrl_vz_p, ctrl_vz_i, ctrl_vz_d, ctrl_vz_i_max, ctrl_vz_i_min);  
  
  // newly added, Jan. 19, 2014
  yawCtrl.setGains(ctrl_yaw_p, ctrl_yaw_i, ctrl_yaw_d, ctrl_yaw_i_max, ctrl_yaw_i_min);  
  //rCtrl.setGains(ctrl_r_p, ctrl_r_i, ctrl_r_d, ctrl_r_i_max, ctrl_r_i_min); 
  
  
  ////TAU-Controller///////////////////////////////////////////////
  //tauCtrl.setGains(tau_p, tau_i, tau_d, 1000, -1000);
  //tau_ref = 0;
  //////////////////////////////////////////////////
  
  
  
  // IMPORTANT! If we don't define which channels we're using then ControllerBase won't let us send anything on those channels
  controllerChannels.insert(THROTTLE);
  controllerChannels.insert(ROLL);
  controllerChannels.insert(PITCH);
  
  //newly added, Jan. 19, 2014
  controllerChannels.insert(YAW); // To enable autonomous control of yaw channel
  
  // Initialize some values
  ros::Time nowTime = ros::Time::now();
  lastPositionTime = nowTime;
  lastBatteryTime = nowTime;
  lastVelocityTime = nowTime;
  
  // Not needed since geometry_msgs::Point and geometry_msgs::Vector3 initialize to zeros
  //currentHeight = 0;
  //currentVelocity = 0;
  
  xyActive = true;
  
  //std::cout<<"In Constructor: xyActive = " << xyActive  << std::endl;
  
  positionDebugPub = nh_priv.advertise<quadrotor_msgs::PositionDebug>("position_debug", 1);
  controlWarnPub = nh_priv.advertise<quadrotor_msgs::ControlWarn>("control_warn", 1);
  
  wpIndexPub = nh_priv.advertise<std_msgs::Int16>("wp_index", 1);
  
  reconfigureServer.setCallback(boost::bind(&PositionController::ReconfigureCallback, this, _1, _2));
  
}

void PositionController::ReconfigureCallback(quadrotor_input::PositionControllerConfig &config, uint32_t level)
{
  static bool first = true;
  
  // On first call write the values we got from the parameter server to config
  if(first)
  {
    first = false;
    
    pCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    
    //xCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    // Don't need this because xCtrl and yCtrl have same gains
    //yCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    zCtrl.getGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
    
    // newly added, Jan. 15, 2014
    vxCtrl.getGains(config.vxy_p, config.vxy_i, config.vxy_d, config.vxy_i_max, config.vxy_i_min);
    // Don't need this because xCtrl and yCtrl have same gains
    //yCtrl.getGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    vzCtrl.getGains(config.vz_p, config.vz_i, config.vz_d, config.vz_i_max, config.vz_i_min);
    
    // Need to modify PositionControllerConfig to include the following parameters in the configuration file
    yawCtrl.getGains(config.yaw_p, config.yaw_i, config.yaw_d, config.yaw_i_max, config.yaw_i_min);  
    
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
    
    // newly added, Jan. 15, 2014
    config.pid_vxy_limit = pidLimit_v.x;
    //config.pid_yy_limit = pidLimit_v.y;
    config.pid_vz_limit = pidLimit_v.z;
    config.cmd_vxy_max = cmdMax_v.x;
    //config.cmd_xy_max = cmdMax_v.y;
    config.cmd_vz_max = cmdMax_v.z;
    config.cmd_vxy_min = cmdMin_v.x;
    //config.cmd_xy_min = cmdMin_v.y;
    config.cmd_vz_min = cmdMin_v.z;
    
    config.pid_yaw_limit = pidLimit_yaw;
    config.cmd_yaw_max = cmdMax_yaw;
    config.cmd_yaw_min =  cmdMin_yaw;
    
    config.xy_active = xyActive;
    //std::cout<<"In first reconfig: xyActive = " << xyActive  << std::endl;
    //std::cout<<"In first reconfig: config.xy_active = " << config.xy_active  << std::endl;
    
    config.x_des = -1.7;
    config.y_des = 0.13;
    config.z_des = 1.5;
    config.yaw_des = 0.0;
    
    config.xyspeedLimit = xyspeedLimit;
    config.zspeedLimit = zspeedLimit;
    config.ascendingSpeedMax = ascendingSpeedMax;  
    config.descendingSpeedMax =  descendingSpeedMax;  
    config.touchdownSpeedMax =  touchdownSpeedMax; 
    config.pitchLimit =  pitchLimit;
    config.rollLimit = rollLimit;
    config.yawrateLimit = yawrateLimit ;
    config.throttleHighLimit = throttleHighLimit;
    config.throttleLowLimit = throttleLowLimit;
    
    
  }
  else  // Afterwards treat config calls as normal ie apply them directly
  {
    pCtrl.setGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);  
    
    //xCtrl.setGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    //yCtrl.setGains(config.xy_p, config.xy_i, config.xy_d, config.xy_i_max, config.xy_i_min);
    zCtrl.setGains(config.z_p, config.z_i, config.z_d, config.z_i_max, config.z_i_min);
    
    vxCtrl.setGains(config.vxy_p, config.vxy_i, config.vxy_d, config.vxy_i_max, config.vxy_i_min);
    vyCtrl.setGains(config.vxy_p, config.vxy_i, config.vxy_d, config.vxy_i_max, config.vxy_i_min);
    vzCtrl.setGains(config.vz_p, config.vz_i, config.vz_d, config.vz_i_max, config.vz_i_min);
    
    // Need to modify PositionControllerConfig to include the following parameters in the configuration file
    yawCtrl.setGains(config.yaw_p, config.yaw_i, config.yaw_d, config.yaw_i_max, config.yaw_i_min);  
    
   
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
    
    pidLimit_v.x = config.pid_vxy_limit;
    pidLimit_v.y = config.pid_vxy_limit;
    pidLimit_v.z = config.pid_vz_limit;
    cmdMax_v.x = config.cmd_vxy_max;
    cmdMax_v.y = config.cmd_vxy_max;
    cmdMax_v.z = config.cmd_vz_max;
    cmdMin_v.x = config.cmd_vxy_min;
    cmdMin_v.y = config.cmd_vxy_min;
    cmdMin_v.z = config.cmd_vz_min;
    
    pidLimit_yaw = config.pid_yaw_limit;
    cmdMax_yaw = config.cmd_yaw_max;
    cmdMin_yaw = config.cmd_yaw_min;
    
    xyActive = config.xy_active;
    
    
  xyspeedLimit = config.xyspeedLimit;
  zspeedLimit = config.zspeedLimit;
  ascendingSpeedMax = config.ascendingSpeedMax;  
  descendingSpeedMax = config.descendingSpeedMax;  
  touchdownSpeedMax = config.touchdownSpeedMax; 
  pitchLimit = config.pitchLimit;
  rollLimit = config.rollLimit;
  yawrateLimit = config.yawrateLimit;
  throttleHighLimit = config.throttleHighLimit;
  throttleLowLimit = config.throttleLowLimit;
    
  }
  
  boost::mutex::scoped_lock lock(commandMutex);
  currentPath.poses.resize(1);
  currentPath.poses[0].pose.position.x = config.x_des;
  currentPath.poses[0].pose.position.y = config.y_des;
  currentPath.poses[0].pose.position.z = config.z_des;
  
  double yaw_des = config.yaw_des/180.0*M_PI;
  
  geometry_msgs::Quaternion quat = EulerRPYToQuat(0.0, 0.0, yaw_des); 
  
  //currentPath.poses[0].pose.position.x = desiredPosition.x;
  //currentPath.poses[0].pose.position.y = desiredPosition.y;
  //currentPath.poses[0].pose.position.z = desiredPosition.z;
  
  currentPath.poses[0].pose.orientation = quat;

  currentPath.poses[0].header.stamp = ros::Time::now();
  
  //std::cout<<">>>>>>>>>>>>>>>>>> Setting desired position: "<<config.x_des<<", "<<config.y_des<<", "<<config.z_des<<std::endl;
  //xCtrl.reset();
  //yCtrl.reset();
  
  zCtrl.reset();
  vzCtrl.reset();
  
  pCtrl.reset();
  vxCtrl.reset();
  vyCtrl.reset();
 
  yawCtrl.reset();
}

geometry_msgs::Pose PositionController::GetDesiredPose()
{
  boost::mutex::scoped_lock lock(commandMutex);
  
  ros::Time nowTime = ros::Time::now();
  
  ROS_ASSERT(currentPath.poses.size() > 0);
  
  geometry_msgs::Pose currentWaypointPose = currentPath.poses[waypointIndex].pose;
  
  geometry_msgs::Vector3 errPosition = pointToVector3(currentWaypointPose.position) - pointToVector3(currentPose.position); // position difference between desired waypoint and vehicle position in world frame
  double disPosition = errPosition.x * errPosition.x + errPosition.y * errPosition.y + errPosition.z * errPosition.z; // square of position error
  
  tf::Quaternion q_1;
  tf::quaternionMsgToTF(currentWaypointPose.orientation, q_1);
  tf::Quaternion q_2;
  tf::quaternionMsgToTF(currentPose.orientation, q_2);
  
  geometry_msgs::Vector3 errEuler = QuatToEulerRPY(q_1) - QuatToEulerRPY(q_2);
  double errYaw = errEuler.z; // yaw error
  
  // When the current desired waypoint and yaw angle are acheived and the current waypoint is NOT final destination, set the next waypoint as the desired one
  if (disPosition <= tolPosition && fabs(errYaw) <= tolYaw && (waypointIndex < (currentPath.poses.size() - 1)))
  {
	waypointIndex = waypointIndex + 1;
  }
  
  std_msgs::Int16 msg;
  msg.data = waypointIndex + 1;
  wpIndexPub.publish(msg);

  return currentPath.poses[waypointIndex].pose;
}

void PositionController::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& poseMsg)
{
  lastPositionTime = poseMsg->header.stamp;
  if(errorHistory.size() > 0 && lastPositionTime == errorHistory.back().first)  // same timestamp as what we already have, don't use (happens if Gazebo is running slower than control loop)
    return;
    
  currentPose = poseMsg->pose; // current pose of the vehicle
}

void PositionController::VelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& velocityMsg)
{
  // current velocity in world frame
  currentVelocity_world = velocityMsg->twist.linear;
  
  // Current velocity in body frame 
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

void PositionController::CalcErrorHistory()
{
// Convert Quaternion to Euler (RPY)
  tf::Quaternion q(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w);
  
  currentEulerRPY = QuatToEulerRPY(q);
  
  if (currentPath.poses.size() > 0)
  {
      geometry_msgs::Pose desiredPose = GetDesiredPose();
      
      desiredPosition = desiredPose.position;
      //desiredOrientation = desiredPose.orientation;
      
      tf::Quaternion q_d;
      tf::quaternionMsgToTF(desiredPose.orientation, q_d);
      
      geometry_msgs::Vector3 desiredEuler = QuatToEulerRPY(q_d);
      
      desiredYaw = desiredEuler.z;
      
      //geometry_msgs::Vector3 err = pointToVector3( worldToPose(currentPose, desiredPosition) ); // position difference in body frame???
      
      geometry_msgs::Vector3 err = pointToVector3(desiredPosition) - pointToVector3(currentPose.position); // position difference in world frame???
      errorHistory.push_back(std::make_pair(lastPositionTime, err));
  }    
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
  //xCtrl.reset();
  //yCtrl.reset();
  zCtrl.reset();
  
  // newly added, Jan. 15th, 2014
  vxCtrl.reset();
  vyCtrl.reset();
  vzCtrl.reset();
  
  // Jan. 27, 2014
  yawCtrl.reset();
  pCtrl.reset();
  
  //////TAU Controller//////////
  //tauCtrl.reset();
  ////////////////
  
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
      
      pCtrl.getGains(curr_xy_p, curr_xy_i, curr_xy_d, curr_xy_imax, curr_xy_imin);
       
      //xCtrl.getGains(curr_xy_p, curr_xy_i, curr_xy_d, curr_xy_imax, curr_xy_imin);
      zCtrl.getGains(curr_z_p, curr_z_i, curr_z_d, curr_z_imax, curr_z_imin);
    
      pCtrl.setGains(newGains[0], newGains[1], newGains[2], curr_xy_imax, curr_xy_imin);
      //xCtrl.setGains(newGains[0], newGains[1], newGains[2], curr_xy_imax, curr_xy_imin);
      //yCtrl.setGains(newGains[0], newGains[1], newGains[2], curr_xy_imax, curr_xy_imin);
      zCtrl.setGains(newGains[3], newGains[4], newGains[5], curr_z_imax, curr_z_imin);
    }
    else
    {
      pCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      //xCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      //yCtrl.setGains(newGains[0], newGains[1], newGains[2], newGains[3], newGains[4]);
      zCtrl.setGains(newGains[5], newGains[6], newGains[7], newGains[8], newGains[9]);
    }
      
    // IMPORTANT: clear this otherwise we'll try to set gains gain next time
    newGains.clear();
  }
  lock.unlock();
  
  ros::Time nowTime = ros::Time::now();
  
  quadrotor_msgs::ControlWarn controlWarnMsg;
  
  if(nowTime - lastPositionTime > positionTimeBuffer)  // too long since last position message
  {
    ROS_ERROR("Last position time too far in the past!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    
    controlWarnMsg.warnName = "PositionDelay";
    controlWarnMsg.header.stamp = ros::Time::now();
    controlWarnMsg.warnInfo = nowTime.toSec() -  lastPositionTime.toSec();
    controlWarnPub.publish(controlWarnMsg);
    return;
  }
  
  if(nowTime - lastBatteryTime > batteryTimeBuffer)  // too long since last battery message
  {
    ROS_ERROR("Last battery time too far in the past!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    
    controlWarnMsg.warnName = "BatteryDelay";
    controlWarnMsg.header.stamp = ros::Time::now();
    controlWarnMsg.warnInfo = nowTime.toSec() - lastBatteryTime.toSec();
    controlWarnPub.publish(controlWarnMsg);
    
    return;
  }
  
  //calculate the history of control errors
  CalcErrorHistory();
  
  if(errorHistory.size() == 0)
  {
    ROS_WARN("Error history size is zero!");
    rcMsg.throttle = -10;   // set to invalid
    rcMsg.roll = -10;
    rcMsg.pitch = -10;
    
    controlWarnMsg.warnName = "ZeroErrorSize";
    controlWarnMsg.header.stamp = ros::Time::now();
    controlWarnMsg.warnInfo = 0.0;
    controlWarnPub.publish(controlWarnMsg);
    
    return;
  }
  
  controlWarnMsg.warnName = "NA";
  controlWarnMsg.header.stamp = ros::Time::now();
  controlWarnMsg.warnInfo = 0.0;
  controlWarnPub.publish(controlWarnMsg);
  
  
  
  ros::Duration step = nowTime - lastControlTime;
  lastControlTime = nowTime;
  
  double currentBatteryVoltage = CalcBatteryVoltage();
  
  geometry_msgs::Vector3 error = errorHistory.back().second; // Position error
  geometry_msgs::Vector3 errorDerivative = CalcErrorDerivative(); // Derivative of position error
  
  double baseThrottle = CalcBaseThrottle(currentBatteryVoltage);
  double scale = CalcScale(baseThrottle);
  
  // Horizontal distance controller, Jan 27, 2014
  double xyDistance = sqrt(error.x * error.x + error.y * error.y);
  double pidVal_xy = pCtrl.updatePid(-1.0 * xyDistance, step);
  double desiredxySpeed = std::min(xyspeedLimit, std::max(-1.0 * xyspeedLimit, pidVal_xy));
  
  // Horizontal velocity (V_x & V_y) controller
  //double desiredForwardSpeed = 0.0; //desiredxySpeed * cos(eta_xy);
  //double desiredLateralSpeed = 0.0; //desiredxySpeed * sin(eta_xy);
  
  //if (xyDistance != 0.0) 
  //{
        //double eta_xy = atan2(error.y, error.x);
        //desiredForwardSpeed = desiredxySpeed * cos(eta_xy);
        //desiredLateralSpeed = desiredxySpeed * sin(eta_xy);
  //}
  
  double desiredForwardSpeed = 0.0; //desiredxySpeed * cos(eta_xy);
  double desiredLateralSpeed = 0.0; //desiredxySpeed * sin(eta_xy);
  
  if (xyDistance > 0.01)  //(error.x != 0.0 || error.y != 0.0)
  {
        desiredForwardSpeed = desiredxySpeed * error.x / xyDistance;
        desiredLateralSpeed = desiredxySpeed * error.y / xyDistance;
  }
  
  //***************************************************************
  //pidVal.x = xCtrl.updatePid(-1*error.x, -1*errorDerivative.x, step);
  //double desiredForwardSpeed = std::min(1.0, std::max(-1.0, pidVal.x));
  //**************************************************************
  double forwardSpeedError = desiredForwardSpeed - currentVelocity_world.x;
  double desiredForwardAcceleration = vxCtrl.updatePid(-1.0 * forwardSpeedError, step);// * scale;
  //double desiredForwardAcceleration = vxCtrl.updatePid(-1.0 * error.x, step);// * scale;
  
  //****************************************************************//
  //pidVal.y = yCtrl.updatePid(-1*error.y, -1*errorDerivative.y, step);
  //double desiredLateralSpeed = std::min(1.0, std::max(-1.0, pidVal.y));
  //****************************************************************8//
  double laterSpeedError = desiredLateralSpeed- currentVelocity_world.y;
  double desiredLateralAcceleration = vyCtrl.updatePid(-1.0 * laterSpeedError, step);// * scale;
  //double desiredLateralAcceleration = vyCtrl.updatePid(-1.0 * error.y, step);// * scale;
  
  double currentYaw = currentEulerRPY.z;

  double desiredPitch = ((-1.0) * cos(currentYaw) * desiredForwardAcceleration - sin(currentYaw) * desiredLateralAcceleration) / 9.81; 
  double desiredRoll = ((-1.0) * sin(currentYaw) * desiredForwardAcceleration + cos(currentYaw) * desiredLateralAcceleration) / 9.81;
  
  // Pitch and roll commands
  double desiredPitch_limited = std::min(pitchLimit, std::max(-1.0 * pitchLimit, desiredPitch))*convertRP;
  double desiredRoll_limited  = std::min(rollLimit, std::max(-1.0 * rollLimit, desiredRoll))*convertRP;
  
  // Vertical height and speed controllers
  double pidVal_z = zCtrl.updatePid(-1.0 * error.z, -1.0 * errorDerivative.z, step);
  
  double upSpeed = fabs(zspeedLimit);
  double downSpeed = -1.0 * fabs(zspeedLimit);
  
  // when close to ground, reduce the vertical speed
  if (fabs(error.z) < 1.4)
  {
        upSpeed = fabs(ascendingSpeedMax);
        downSpeed = -1.0 * fabs(descendingSpeedMax);
        
        if (fabs(error.z) <= 0.4) // further reduce descending speed right before touchdown
        {
            downSpeed = -1.0 * fabs(touchdownSpeedMax);
        }
    }

  double desiredVerticalSpeed = std::min(upSpeed, std::max(downSpeed, pidVal_z));
  
  double verticalSpeedError = desiredVerticalSpeed - currentVelocity_world.z;
  double desiredVerticalAcceleration = vzCtrl.updatePid(-1.0 * verticalSpeedError, step); // * scale;
  
  
  /////////////TAU-Controller/////////////////////////

  //double desiredAltitude = desiredPosition.z;
  //double currentAltitude =  currentPose.position.z;
  //double currentVerticalSpeed = currentVelocity_world.z;
  
  //double saturation = 1.0;
  
  //double temp_p;
  //double temp_i;
  //double temp_d;
  //double temp_i_max;
  //double temp_i_min;
  //static bool init_time = true;
  
  //if (desiredAltitude == 0.05)
  //{   
      
      //if (init_time == true)
      //{
          //t_i = nowTime.toSec();
          //init_time = false;
      //}
      
      //if (init_time == false)
      //{
          ////sets time_elapsed to zero when landing is executed
          //time_elapsed = nowTime.toSec() - t_i;
          
          ////calls Tau_Controller and returns to desired vertical acceleration                                                       //baseThrottle offset
          //desiredVerticalAcceleration = Tau_Controller(time_elapsed, tau_f, currentAltitude - 0.05, currentVerticalSpeed, tau_k, step);

          ////saturates the maximum and miminum acceleration (must change based on agility requirements)
          //if (desiredVerticalAcceleration > saturation)
          //{
              //desiredVerticalAcceleration = saturation;
          //}
          
          //if (desiredVerticalAcceleration < -saturation + 0.6)
          //{
              //desiredVerticalAcceleration = -saturation + 0.6;
          //}
      //}
   //}
  
  //////////////////End of TAU////////////////////////////
  
  
  
  // Throttle command
  double desiredThrottle = (desiredVerticalAcceleration * baseThrottle * mass / 9.81 + baseThrottle)/cos(currentEulerRPY.x)/cos(currentEulerRPY.y);
  double desiredThrottle_limited = std::min(throttleHighLimit, std::max(throttleLowLimit, desiredThrottle));
  
  // Yaw angle controller
  double yawError = desiredYaw - currentYaw;
  if(yawError > M_PI)
      yawError = yawError - 2.0 * M_PI;
  else if(yawError < -1.0*M_PI)
      yawError = yawError + 2.0 * M_PI;

  double desiredYawRate = yawCtrl.updatePid(-1.0 * yawError, step) ;//* scale;
  
  // Yaw rate command
  double desiredYawRate_limited = std::min(yawrateLimit, std::max(-1.0 * yawrateLimit, desiredYawRate));
  
  // Generate control commands
  rcMsg.roll = -1.0 * desiredRoll_limited;  // positive roll moves aircraft to the right
  rcMsg.pitch = -1.0 * desiredPitch_limited;  // positive pitch moves aircraft forward
  rcMsg.throttle = desiredThrottle_limited;
  rcMsg.yaw = -1.0 * desiredYawRate_limited; //  rcMsg.yaw is actually yaw rate command
  
  
  //std::cout<<"In compute: xyActive = " << xyActive  << std::endl;
  if(!xyActive)  // If xy control not active set roll and pitch to invalid values
  {
    rcMsg.pitch= -10;
    rcMsg.roll = -10;
  }

  errorMsg.error = error;
  errorMsg.derivative = errorDerivative;

  // Publish debugging information
  quadrotor_msgs::PositionDebug positionDebugMsg;
  
  pCtrl.getCurrentPIDErrors(&positionDebugMsg.xy_error.x, &positionDebugMsg.xy_error.y, &positionDebugMsg.xy_error.z);
  vxCtrl.getCurrentPIDErrors(&positionDebugMsg.vx_error.x, &positionDebugMsg.vx_error.y, &positionDebugMsg.vx_error.z);
  vyCtrl.getCurrentPIDErrors(&positionDebugMsg.vy_error.x, &positionDebugMsg.vy_error.y, &positionDebugMsg.vy_error.z);
  
  zCtrl.getCurrentPIDErrors(&positionDebugMsg.z_error.x, &positionDebugMsg.z_error.y, &positionDebugMsg.z_error.z);
  vzCtrl.getCurrentPIDErrors(&positionDebugMsg.vz_error.x, &positionDebugMsg.vz_error.y, &positionDebugMsg.vz_error.z);
  
  yawCtrl.getCurrentPIDErrors(&positionDebugMsg.yaw_error.x, &positionDebugMsg.yaw_error.y, &positionDebugMsg.yaw_error.z);
  
  // This is necessary because the pid controller's internal error states are the opposite of ours
  positionDebugMsg.xy_error = positionDebugMsg.xy_error * -1.0;
  positionDebugMsg.vx_error = positionDebugMsg.vx_error * -1.0;
  positionDebugMsg.vy_error = positionDebugMsg.vy_error * -1.0;
  positionDebugMsg.z_error = positionDebugMsg.z_error * -1.0;
  positionDebugMsg.vz_error = positionDebugMsg.vz_error * -1.0;
  positionDebugMsg.yaw_error = positionDebugMsg.yaw_error * -1.0;
  
  positionDebugMsg.desired_xyz_yaw.x = desiredPosition.x; 
  positionDebugMsg.desired_xyz_yaw.y = desiredPosition.y;
  positionDebugMsg.desired_xyz_yaw.z = desiredPosition.z;
  positionDebugMsg.desired_xyz_yaw.w = desiredYaw;
  
  positionDebugMsg.current_xyz_yaw.x = currentPose.position.x;
  positionDebugMsg.current_xyz_yaw.y = currentPose.position.y; 
  positionDebugMsg.current_xyz_yaw.z = currentPose.position.z;
  positionDebugMsg.current_xyz_yaw.w = currentYaw; //currentPosition;
 
  positionDebugMsg.cmd.x = -1.0 * desiredRoll;     //roll cmd
  positionDebugMsg.cmd_limited.x = -1.0 * desiredRoll_limited;
  positionDebugMsg.cmd.y = -1.0 * desiredPitch;            //pitch cmd
  positionDebugMsg.cmd_limited.y = -1.0 * desiredPitch_limited;
  positionDebugMsg.cmd.z = desiredThrottle;            //throttle cmd
  positionDebugMsg.cmd_limited.z = desiredThrottle_limited;
  positionDebugMsg.cmd.w = -1.0 * desiredYawRate; //yaw_rate cmd
  positionDebugMsg.cmd_limited.w = -1.0 * desiredYawRate_limited;

  positionDebugMsg.desired_velocity.x = desiredForwardSpeed;
  positionDebugMsg.desired_velocity.y = desiredLateralSpeed;
  positionDebugMsg.desired_velocity.z = desiredVerticalSpeed;
  
  positionDebugMsg.desired_acceleration.x = desiredForwardAcceleration;
  positionDebugMsg.desired_acceleration.y = desiredLateralAcceleration;
  positionDebugMsg.desired_acceleration.z = desiredVerticalAcceleration;
  
  positionDebugMsg.base_throttle = baseThrottle;
  positionDebugMsg.scale =  scale;
  positionDebugMsg.voltage = currentBatteryVoltage;  
  
  ////// TAU /////////
  //tauCtrl.getGains(temp_p, temp_i, temp_d, temp_i_max, temp_i_min);
  
  //positionDebugMsg.base_throttle = tau;
  //positionDebugMsg.voltage = tau_ref; 
  //positionDebugMsg.scale = tau_f;
  
  //// TAU END ///////// 
  
  positionDebugPub.publish(positionDebugMsg);
}


/////////////TAU CONTROLLER FUNCTION//////////////////

//double PositionController::Tau_Controller(double t, double t_f, double altitude, double speed, double k, ros::Duration step)
//{
    
    //// GENERATE TAU REFERENCE

    //if (t < 0.2)
    //{
        //tau_ref = -80;
    //}
    //else if (t < t_f)
    //{
        //tau_ref = k/2*(t - pow(t_f, 2)/t);
    //}
    //else
    //{	
        //tau_ref = 0;
    //}

    ////ESTIMATE TAU

    //if (speed != 0)
    //{
        //tau = altitude/speed;
    //}
    //else 
    //{
        //tau = 0;
    //}   
    
    ////CALCULATE ERROR
    
    //if (tau != 0)
    //{
        //error = tau_ref/tau - 1; 
    //}
    //else
    //{
        //error = tau_ref - tau;
    //}
    
    //return tauCtrl.updatePid(-1.0*error, step);
//}


