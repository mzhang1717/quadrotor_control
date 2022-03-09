#include <quadrotor_input/JoystickConverter.h>

JoystickConverter::JoystickConverter()
: nh_priv("~")
, throttlePolyCoeffs(4)
{
  bool paramSuccess = true;
  
  paramSuccess &= nh_priv.getParam("axis_throttle", axisThrottle);     // With DX6i, trainer cable: 0, simstick: 4
  paramSuccess &= nh_priv.getParam("axis_roll", axisRoll);             // With DX6i, trainer cable: 1, simstick: 3
  paramSuccess &= nh_priv.getParam("axis_pitch", axisPitch);           // With DX6i, trainer cable: 2, simstick: 5
  paramSuccess &= nh_priv.getParam("axis_yaw", axisYaw);               // With DX6i, trainer cable: 5, simstick: 2
  paramSuccess &= nh_priv.getParam("axis_flap", axisFlap);             // With DX6i, trainer cable: 4, simstick: 0
  paramSuccess &= nh_priv.getParam("axis_gear", axisGear);             // With DX6i, trainer cable: 3, simstick: 1
  
  paramSuccess &= nh_priv.getParam("c0", throttlePolyCoeffs[0]);
  paramSuccess &= nh_priv.getParam("c1", throttlePolyCoeffs[1]);
  paramSuccess &= nh_priv.getParam("c2", throttlePolyCoeffs[2]);
  paramSuccess &= nh_priv.getParam("c3", throttlePolyCoeffs[3]);
  
  if(!paramSuccess)
  {
    ROS_FATAL("JoystickConverter needs the following parameters: axis_throttle, axis_roll, axis_pitch, axis_yaw, axis_flap, axis_gear, c0, c1, c2, c3");
    ROS_FATAL("One of these was missing, can't continue");
    ros::shutdown();
    return;
  }
  
  bool flipThrottle, flipRoll, flipPitch, flipYaw, flipFlap, flipGear;
  
  nh_priv.param<bool>("flip_throttle", flipThrottle, false);
  nh_priv.param<bool>("flip_roll", flipRoll, false);
  nh_priv.param<bool>("flip_pitch", flipPitch, false);
  nh_priv.param<bool>("flip_yaw", flipYaw, false);
  nh_priv.param<bool>("flip_flap", flipFlap, false);
  nh_priv.param<bool>("flip_gear", flipGear, false);
  
  signThrottle = flipThrottle ? -1 : 1;
  signRoll = flipRoll ? -1 : 1;
  signPitch = flipPitch ? -1 : 1;
  signYaw = flipYaw ? -1 : 1;
  signFlap = flipFlap ? -1 : 1;
  signGear = flipGear ? -1 : 1;
  
  radioControlPub = nh_priv.advertise<quadrotor_msgs::RadioControl>("radio_control", 1);
  joySub = nh.subscribe<joy::Joy>("joy", 10, &JoystickConverter::joyCallback, this);
  
  //reconfigureServer.setCallback(boost::bind(&JoystickConverter::reconfigureCallback, this, _1, _2));
}

quadrotor_msgs::RadioControl JoystickConverter::joyToRC(const joy::Joy& joyMsg)
{
  quadrotor_msgs::RadioControl rcMsg;
  
  rcMsg.throttle = scaleThrottle(signThrottle * joyMsg.axes[axisThrottle]);
  rcMsg.roll = signRoll * joyMsg.axes[axisRoll];
  rcMsg.pitch = signPitch * joyMsg.axes[axisPitch];
  rcMsg.yaw = signYaw * joyMsg.axes[axisYaw];
  rcMsg.flap = (int)round(signFlap * joyMsg.axes[axisFlap]);
  rcMsg.gear = (int)round(signGear * joyMsg.axes[axisGear]);

  return rcMsg;
}

double JoystickConverter::scaleThrottle(double throttle)
{
  // Polynomials are stored with the coefficient of zero in the first spot
	double val=0;
	for(int i=throttlePolyCoeffs.size()-1; i > 0; i--)
	{
		val += throttlePolyCoeffs[i];	
		val *= throttle;
	}

	val += throttlePolyCoeffs[0];
	return val;
}


void JoystickConverter::joyCallback(const joy::Joy::ConstPtr& joy)
{
  radioControlPub.publish(joyToRC(*joy));
}
/*
void JoystickConverter::reconfigureCallback(quadrotor_input::JoystickConverterConfig &config, uint32_t level)
{
  if(config.throttleSensitiveStartOutput > config.throttleMaxOutput)  // sensitive start cannot be greater than max throttle
    config.throttleSensitiveStartOutput = config.throttleMaxOutput;
  
  throttleSensitiveStartStick = config.throttleSensitiveStartStick;
  
  // Calculate from basic equation for lines
  insensitiveIntercept = config.throttleSensitiveStartOutput / (1 + throttleSensitiveStartStick);
  insensitiveSlope = insensitiveIntercept;
  
  sensitiveSlope = (config.throttleMaxOutput - config.throttleSensitiveStartOutput) / (1 - throttleSensitiveStartStick);
  sensitiveIntercept = config.throttleSensitiveStartOutput - sensitiveSlope*throttleSensitiveStartStick;
  
  if(config.startRotors)
  {
    if(gearStatus == ACTIVE)
    {
      ROS_ERROR("Gear status is active, turn gear off before starting motors from GUI");
    }
    else if(radioControlMsg.flap != 0) 
    {
      ROS_ERROR("Put the flap switch to zero before starting motors from GUI");
    }
    else
    {
      startingRotors = true;
      rotorStartTime = ros::Time::now();
    }
    
    // Reset checkmark button in GUI
    config.startRotors = false;
  }
  
}
*/
