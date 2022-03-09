#include <quadrotor_input/ControllerBase.h>

ControllerBase::ControllerBase()
: nh_priv("~")//initialized private
{
  waypointIndex = 0;
    
  if(!nh_priv.getParam("rate", rate))
  {
    ROS_FATAL("ControllerBase needs a rate parameter to set the loop rate");
    ros::shutdown();
    return;
  }
  
  commandService = nh_priv.advertiseService("command", &ControllerBase::CommandServiceCallback, this);
  radioControlPub = nh_priv.advertise<quadrotor_msgs::RadioControl>("radio_control", 1);
  errorPub = nh_priv.advertise<quadrotor_msgs::ControllerError>("error", 1);
  controllerStatusPub = nh_priv.advertise<std_msgs::Bool>("controller_status",1);
  
  status = INACTIVE;
  controller_status = false;
}

// Main run loop lives here
void ControllerBase::Run()
{
  if(controllerChannels.empty())
  {
    ROS_FATAL("ControllerBase: No controller channels added by child class, exiting.");
    ros::shutdown();
    return;
  }
  
  ros::Rate loopRate(rate);
  
  while(ros::ok())
  {
	
	
    if(status == ACTIVE)
    {
      quadrotor_msgs::RadioControl rcMsg;
      quadrotor_msgs::ControllerError errorMsg;
      
      Compute(rcMsg, errorMsg);
      
      // Set all uncontrolled channels to an illegal value
      // This way RadioControlMixer will discard them and they won't
      // overwrite the manual channels coming from the joystick
      
      if(!controllerChannels.count(THROTTLE))
        rcMsg.throttle = -10;
      
      if(!controllerChannels.count(ROLL))
        rcMsg.roll = -10;
        
      if(!controllerChannels.count(PITCH))
        rcMsg.pitch = -10;
        
      if(!controllerChannels.count(YAW))
        rcMsg.yaw = -10;
      
      rcMsg.header.stamp = ros::Time::now();
      radioControlPub.publish(rcMsg);
      errorPub.publish(errorMsg);
      
      std_msgs::Bool constatusMSG;
      
      constatusMSG.data = true;
      controllerStatusPub.publish(constatusMSG);
    }
    else{
		std_msgs::Bool constatusMSG;
        constatusMSG.data = false;
		controllerStatusPub.publish(constatusMSG);
	}
    
    ros::spinOnce();
    loopRate.sleep();
  }
  
}

bool ControllerBase::CommandServiceCallback(quadrotor_input::CommandController::Request &req, quadrotor_input::CommandController::Response &res)
{
  waypointIndex = 0;

  if(req.running == true)
  {
    
      
    if(status == ACTIVE) // we are already active
    {
      ROS_WARN("ControllerBase: Got request to go active but we are already active!");
      
      boost::mutex::scoped_lock lock(commandMutex);
      SetCurrentPath(req.path);
      newGains = req.gains;
      bool bI = Init();
      status = ACTIVE;
      res.success = true;
    }
    else  // we were inactive, switch to active
    {
      if(Init())
      {
        ROS_WARN("ControllerBase: Controller turned active");
        
        boost::mutex::scoped_lock lock(commandMutex);
        SetCurrentPath(req.path);
        newGains = req.gains;
        
        status = ACTIVE;
        res.success = true;
      }
      else
      {
        ROS_ERROR("ControllerBase: Got request to go active but couldn't initialize!");
        status = INACTIVE;
        res.success = false;
      }
    }
  }
  else if(req.running == false)
  {
    if(status == INACTIVE) // we are already inactive
    {
      ROS_WARN("ControllerBase: Got request to go inactive but we are already inactive!");
      res.success = true;
    }
    else  // we were active, switch to inactive
    {
      if(Fini())
      {
        ROS_WARN("ControllerBase: Controller turned inactive");
        status = INACTIVE;
        res.success = true;
      }
      else
      {
        ROS_ERROR("ControllerBase: Got request to go inactive but couldn't finish!");
        status = ACTIVE;
        res.success = false;
      }
    }
  }
  
  return true;
}

void ControllerBase::SetCurrentPath(const nav_msgs::Path& path)
{
  ros::Time startTime = ros::Time::now();
  currentPath = path;
  for(unsigned i=0; i < currentPath.poses.size(); ++i)
  {
    ros::Duration offset(currentPath.poses[i].header.stamp.sec, currentPath.poses[i].header.stamp.nsec);
    currentPath.poses[i].header.stamp = startTime + offset;
  }
  
  std::cout<<"Set current path, end position: "<<currentPath.poses.rbegin()->pose.position.x<<
             "  "<<currentPath.poses.rbegin()->pose.position.y<<"  "<<currentPath.poses.rbegin()->pose.position.z<<std::endl;
}
