//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Revised by Mingfeng Zhang, 2014
//=========================================================================================

#include <gazebo_quadrotor/GazeboDraganflyX8.h>

#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/State.h>
#include <quadrotor_msgs/StateDebug.h>
#include <quadrotor_msgs/BatteryStatus.h>
#include <quadrotor_msgs/HeightStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboDraganflyX8::GazeboDraganflyX8()
{

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboDraganflyX8::~GazeboDraganflyX8()
{
  event::Events::DisconnectWorldUpdateStart(this->updateConnection);
  this->queue.clear();
  this->queue.disable();
  this->rosnode->shutdown();
  this->callbackQueueThread.join();
  delete this->rosnode;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboDraganflyX8::Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->myModel = _parent;
  
  // load parameters
  this->robotNamespace = "/";
  if (_sdf->HasElement("robotNamespace"))
    this->robotNamespace = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("inputTopicName"))
  {
    ROS_FATAL("GazeboDraganflyX8 plugin missing <inputTopicName>, cannot proceed");
    return;
  }
  else
    this->inputTopicName = _sdf->GetElement("inputTopicName")->GetValueString();
    
  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL("GazeboDraganflyX8 plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->bodyName = _sdf->GetElement("bodyName")->GetValueString();
    
  this->thrustFactorTop = getParamDouble(_sdf, "thrustFactorTop", 3.721e-5, true);
  this->thrustFactorBottom = getParamDouble(_sdf, "thrustFactorBottom", 5.513e-5, true);
  this->dragFactorTop = getParamDouble(_sdf, "dragFactorTop", -2.2056e-6, true);
  this->dragFactorBottom = getParamDouble(_sdf, "dragFactorBottom", 8.3546e-7, true); // sign of the drag factors change to its original since originally they were regressed in the frame where y to right and z goes down while here in the simulation y to the left and z goes up.
  this->dragCoeff = getParamDouble(_sdf, "dragCoeff", 0.006, false); 
  this->JrTop = getParamDouble(_sdf, "JrTop", 1.39e-4, true);
  this->JrBottom = getParamDouble(_sdf, "JrBottom", 1.29e-4, true);
  this->rRotor = getParamDouble(_sdf, "rRotor", 0.2025, true);
  this->L = getParamDouble(_sdf, "L", 0.3305, true);
  this->maxOmega = getParamDouble(_sdf, "maxOmega", 1000, true);
  this->input.rollMax = getParamDouble(_sdf, "maxRoll", 1.0, false);
  this->input.pitchMax = getParamDouble(_sdf, "maxPitch", 1.0, false);
  this->input.yawRateMax = getParamDouble(_sdf, "maxYawRate", 1.0, false); 
  this->batteryVoltage = getParamDouble(_sdf, "batteryVoltage", 15.0, false); 
  this->batteryCurrent = getParamDouble(_sdf, "batteryCurrent", 20.0, false); 
  
  // Now we need to load the PID node manually
  sdf::ElementPtr pidElem = _sdf->GetElement("PID");
  if(!pidElem)
  {
    ROS_FATAL("GazeboDraganflyX8 plugin is missing <PID>, cannot continue");  
    return;
  }
    
  // Load the pid controller from sdf node
  this->attitudeController.Load(pidElem);
  
}

void GazeboDraganflyX8::Init()
{
  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode = new ros::NodeHandle(this->robotNamespace);
  this->rosnode->setCallbackQueue(&this->queue);
  
  this->myWorld = this->myModel->GetWorld();

  // assert that the body by bodyName exists
  this->myBody = this->myModel->GetLink(this->bodyName);
  if (!this->myBody)
  {
    ROS_FATAL_STREAM("GazeboDraganflyX8 plugin error: bodyName: "<<this->bodyName<<" does not exist");
    return;
  }
  
  this->Ixx = this->myBody->GetInertial()->GetIXX();
  this->Iyy = this->myBody->GetInertial()->GetIYY();
  this->Izz = this->myBody->GetInertial()->GetIZZ();
  
  this->maxOmegaPow2 = this->maxOmega * this->maxOmega;   // Maximum rotor speed squared
  this->input.throttleMax = (this->thrustFactorTop + this->thrustFactorBottom) * 4 * this->maxOmegaPow2;  // Newtons
  
  math::Vector3 gravity = this->myWorld->GetPhysicsEngine()->GetGravity();
  this->hoverThrust = fabs(this->myBody->GetInertial()->GetMass() * gravity.z);  // used only for message output not altitude control
  
  // Subscribe to radio control inputs  
  this->inputSubscriber = this->rosnode->subscribe<quadrotor_msgs::DraganflyRadioControl>(this->inputTopicName,1, boost::bind( &GazeboDraganflyX8::UpdateInput,this,_1));
  
  // Advertise the publishers
  this->batteryPub = this->rosnode->advertise<quadrotor_msgs::BatteryStatus>("output/battery",1);
  this->forcePub = this->rosnode->advertise<geometry_msgs::WrenchStamped>("force",1);
  this->statePub = this->rosnode->advertise<quadrotor_msgs::State>("state",1);
  this->posePub = this->rosnode->advertise<geometry_msgs::PoseStamped>("output/pose",1);
  this->velocityPub = this->rosnode->advertise<geometry_msgs::TwistStamped>("output/velocity",1);
  this->stateDebugPub = this->rosnode->advertise<quadrotor_msgs::StateDebug>("state_debug",1);
  this->heightPub = this->rosnode->advertise<quadrotor_msgs::HeightStamped>("output/height",1);
  
  // Initialize the PID controllers
  this->attitudeController.Init(*(this->rosnode));
  
  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(boost::bind(&GazeboDraganflyX8::Update, this));
  
  // Start custom Callback Queue
  this->callbackQueueThread = boost::thread( boost::bind( &GazeboDraganflyX8::QueueThread,this ) );
  
  this->lastTime = this->myWorld->GetSimTime();
}

////////////////////////////////////////////////////////////////////////////////
// Called by gazebo to update the controller
void GazeboDraganflyX8::Update()
{
  // Update the current aircraft state
  math::Pose pose = this->myBody->GetWorldPose();
  math::Quaternion rot = pose.rot;
  
  this->state.roll = rot.GetRoll();
  this->state.pitch = rot.GetPitch();
  this->state.yaw = rot.GetYaw();
  
  // In body frame
  math::Vector3 angularVel = rot.GetInverse().RotateVector( this->myBody->GetWorldAngularVel() );
  this->state.rollRate = angularVel.x + tan(this->state.pitch) * (angularVel.y * sin(this->state.roll) + angularVel.z * cos(this->state.roll));
  this->state.pitchRate = angularVel.y * cos(this->state.roll) - angularVel.z * sin(this->state.roll);
  this->state.yawRate = (angularVel.y * sin(this->state.roll) + angularVel.z * cos(this->state.roll)) / cos(this->state.pitch);
  
  // In world frame
  math::Vector3 vel = this->myBody->GetWorldLinearVel();
  this->state.xRate = vel.x;
  this->state.yRate = vel.y;
  this->state.zRate = vel.z;
  
  // In world frame
  this->state.x = pose.pos.x;
  this->state.y = pose.pos.y;
  this->state.z = pose.pos.z;
  
  // This will be used to step the PID controller forward
  common::Time nowTime = this->myWorld->GetSimTime();
  common::Time stepTime = nowTime  - this->lastTime;
  this->lastTime = nowTime;
  
  // Now that we have updated our states, update the control commands based on the state and RC input
  this->command = this->attitudeController.Update(this->state, this->input, stepTime.Double());
 
 // Update the rotor speeds, which are stored in this->state
  this->UpdateOmegas(this->state, this->command);
  
  // Update the ground effect factor (this comment is added by Mingfeng, Jan. 9, 2014)
  double k_ge = 1 - (this->rRotor) * (this->rRotor) / 16 / this->state.z / this->state.z;
  
  // Calculate actual force/torque inputs based on rotor speeds (this->state) and ground effect
  double U1 = k_ge * (this->thrustFactorTop * (this->state.Om1pow2 + this->state.Om2pow2 + this->state.Om3pow2 + this->state.Om4pow2) +
                      this->thrustFactorBottom * (this->state.Om4pow2 + this->state.Om5pow2 + this->state.Om6pow2 + this->state.Om7pow2));
  double U2 = k_ge * (this->thrustFactorTop * (-1 * this->state.Om2pow2 + this->state.Om4pow2) + 
                      this->thrustFactorBottom * (-1 * this->state.Om6pow2 + this->state.Om8pow2));
  double U3 = k_ge * (this->thrustFactorTop * (-1 * this->state.Om1pow2 + this->state.Om3pow2) +
                      this->thrustFactorBottom * (-1 * this->state.Om5pow2 + this->state.Om7pow2));
  double U4 = this->dragFactorTop * (this->state.Om1pow2 + this->state.Om2pow2 + this->state.Om3pow2 + this->state.Om4pow2) +
              this->dragFactorBottom * (this->state.Om4pow2 + this->state.Om5pow2 + this->state.Om6pow2 + this->state.Om7pow2);  
                 
  math::Vector3 groundVel = this->myBody->GetWorldLinearVel();  // ground speed
  math::Vector3 airVel = groundVel;  // airspeed, this is where we'd subtract wind velocity if we had it
  
  // Start of calculations for forces and torques, standard quadrotor equations
  double angMom_res = -1 * this->JrTop * (this->state.Om1 + this->state.Om2 + this->state.Om3 + this->state.Om4) + 
                          this->JrBottom  * (this->state.Om5 + this->state.Om6 + this->state.Om7 + this->state.Om8);  // Residual angular momentum of rotors

  double cos_roll = cos(this->state.roll);
  double cos_pitch = cos(this->state.pitch);
  double cos_yaw = cos(this->state.yaw);
  
  double sin_roll = sin(this->state.roll);
  double sin_pitch = sin(this->state.pitch);
  double sin_yaw = sin(this->state.yaw);
  
  double fz = cos_pitch * cos_roll * U1;
  double fx = (cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw) * U1;
  double fy = (cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw) * U1;

  double torqueRoll = (this->Iyy - this->Izz) * angularVel.y * angularVel.z + this->L * U2 - angularVel.y * angMom_res; 
  double torquePitch =  (this->Izz - this->Ixx) * angularVel.x * angularVel.z - this->L * U3 + angularVel.x * angMom_res; 
  double torqueYaw = (this->Ixx - this->Iyy) * angularVel.x * angularVel.y + U4; 
  
  math::Vector3 force(fx, fy, fz);
  math::Vector3 torque(torqueRoll, torquePitch, torqueYaw);
  
  // Force of air due to drag
  math::Vector3 airForce =  airVel * airVel.GetLength() * (-1) * this->dragCoeff;
  
  // Total force to apply
  math::Vector3 totalForce = force + airForce;
  
  this->myBody->AddForce(totalForce);  // absolute coords
  this->myBody->AddRelativeTorque(torque);  // relative coords
  
  // Start of sending messages
  
  quadrotor_msgs::BatteryStatus batteryMsg;
  batteryMsg.header.stamp.sec = nowTime.sec;
  batteryMsg.header.stamp.nsec = nowTime.nsec;
  batteryMsg.voltage = this->batteryVoltage;
  batteryMsg.current = this->batteryCurrent;

  batteryPub.publish(batteryMsg);
  
  
  quadrotor_msgs::StateDebug stateDebugMsg;
  stateDebugMsg.attitude.x = this->state.roll;
  stateDebugMsg.attitude.y = this->state.pitch;
  stateDebugMsg.attitude.z = this->state.yawRate;
  stateDebugMsg.attitude_desired.x = this->input.roll * this->input.rollMax;
  stateDebugMsg.attitude_desired.y = this->input.pitch * this->input.pitchMax;
  stateDebugMsg.attitude_desired.z = this->input.yawRate * this->input.yawRateMax;
  
  stateDebugPub.publish(stateDebugMsg);
  
  // Next few lines get the force and torque from the world, put it in body coordinate frame,
  // and publishes it as a wrench message
  //math::Vector3 worldForceBody = rot.GetInverse().RotateVector(this->myBody->GetWorldForce());
  //math::Vector3 worldTorqueBody = rot.GetInverse().RotateVector(this->myBody->GetWorldTorque());
  math::Vector3 worldForceBody = totalForce;
  math::Vector3 worldTorqueBody = torque;
  
  geometry_msgs::WrenchStamped wrenchMsg;
  
  wrenchMsg.header.frame_id = "world";
  wrenchMsg.header.stamp.sec = nowTime.sec;
  wrenchMsg.header.stamp.nsec = nowTime.nsec;
  wrenchMsg.wrench.force.x    = worldForceBody.x;
  wrenchMsg.wrench.force.y    = worldForceBody.y;
  wrenchMsg.wrench.force.z    = worldForceBody.z;
  wrenchMsg.wrench.torque.x   = worldTorqueBody.x;
  wrenchMsg.wrench.torque.y   = worldTorqueBody.y;
  wrenchMsg.wrench.torque.z   = worldTorqueBody.z;

  this->forcePub.publish(wrenchMsg);
  
  quadrotor_msgs::State stateMsg;
  stateMsg.pose.position.x = this->state.x;
  stateMsg.pose.position.y = this->state.y;
  stateMsg.pose.position.z = this->state.z;
  stateMsg.pose.orientation.x = rot.x;
  stateMsg.pose.orientation.y = rot.y;
  stateMsg.pose.orientation.z = rot.z;
  stateMsg.pose.orientation.w = rot.w;
  stateMsg.velocity.x = this->state.xRate;
  stateMsg.velocity.y = this->state.yRate;
  stateMsg.velocity.z = this->state.zRate;
  stateMsg.base_throttle = this->hoverThrust / input.throttleMax;
  this->statePub.publish(stateMsg);
  
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.pose = stateMsg.pose;
  poseMsg.header.stamp.sec = nowTime.sec;
  poseMsg.header.stamp.nsec = nowTime.nsec;
  this->posePub.publish(poseMsg);
  
  geometry_msgs::TwistStamped velocityMsg;
  velocityMsg.twist.linear = stateMsg.velocity;
  velocityMsg.twist.angular.x = this->state.rollRate;
  velocityMsg.twist.angular.y = this->state.pitchRate;
  velocityMsg.twist.angular.z = this->state.yawRate;
  velocityMsg.header.stamp.sec = nowTime.sec;
  velocityMsg.header.stamp.nsec = nowTime.nsec;
  this->velocityPub.publish(velocityMsg);
  
  quadrotor_msgs::HeightStamped heightMsg;
  heightMsg.header.stamp = poseMsg.header.stamp;
  heightMsg.height = this->state.z;
  this->heightPub.publish(heightMsg);

}  

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboDraganflyX8::Reset()
{
  this->attitudeController.Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Custom callback queue thread
void GazeboDraganflyX8::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Callback when an input message is received
void GazeboDraganflyX8::UpdateInput(const quadrotor_msgs::DraganflyRadioControl::ConstPtr& inputMsg)
{
  this->input.roll = inputMsg->roll / 511.0;
  this->input.pitch = inputMsg->pitch / 511.0;
  this->input.yawRate = inputMsg->yaw / 511.0;
  this->input.throttle = inputMsg->throttle / 511.0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the rotor speeds given the commands
void GazeboDraganflyX8::UpdateOmegas(QuadrotorState &st, const QuadrotorCommand &cmd)
{
  // Based on the quadrotor equations
  st.Om1pow2 = cmd.U1 * this->dragFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) + 
               cmd.U3 * this->thrustFactorTop / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) - 
               cmd.U4 * this->thrustFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om2pow2 = cmd.U1 * this->dragFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) - 
               cmd.U2 * this->thrustFactorTop / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) - 
               cmd.U4 * this->thrustFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om3pow2 = cmd.U1 * this->dragFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) - 
               cmd.U3 * this->thrustFactorTop / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) - 
               cmd.U4 * this->thrustFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om4pow2 = cmd.U1 * this->dragFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) + 
               cmd.U2 * this->thrustFactorTop / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) - 
               cmd.U4 * this->thrustFactorBottom / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om5pow2 = -1 * cmd.U1 * this->dragFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) + 
               cmd.U3 * this->thrustFactorBottom / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) + 
               cmd.U4 * this->thrustFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om6pow2 = -1 * cmd.U1 * this->dragFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) - 
               cmd.U2 * this->thrustFactorBottom / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) + 
               cmd.U4 * this->thrustFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om7pow2 = -1 * cmd.U1 * this->dragFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) - 
               cmd.U3 * this->thrustFactorBottom / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) + 
               cmd.U4 * this->thrustFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  st.Om8pow2 = -1 * cmd.U1 * this->dragFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop) + 
               cmd.U2 * this->thrustFactorBottom / 2 / (this->thrustFactorTop * this->thrustFactorTop + this->thrustFactorBottom * this->thrustFactorBottom) + 
               cmd.U4 * this->thrustFactorTop / 4 / (this->thrustFactorTop * this->dragFactorBottom - this->thrustFactorBottom * this->dragFactorTop);
  
  // Enforce min and max bounds
  // Maybe minimum speed should be higher than 0?
	if(st.Om1pow2 < 0)
		st.Om1pow2 = 0;
	else if(st.Om1pow2 > this->maxOmegaPow2)
		st.Om1pow2 = this->maxOmegaPow2;

	if(st.Om2pow2 < 0)
		st.Om2pow2 = 0;
	else if(st.Om2pow2 > this->maxOmegaPow2)
		st.Om2pow2 = this->maxOmegaPow2;

	if(st.Om3pow2 < 0)
		st.Om3pow2 = 0;
	else if(st.Om3pow2 > this->maxOmegaPow2)
		st.Om3pow2 = this->maxOmegaPow2;

	if(st.Om4pow2 < 0)
		st.Om4pow2 = 0;
	else if(st.Om4pow2 > this->maxOmegaPow2)
		st.Om4pow2 = this->maxOmegaPow2;
	
	if(st.Om5pow2 < 0)
		st.Om5pow2 = 0;
	else if(st.Om5pow2 > this->maxOmegaPow2)
		st.Om5pow2 = this->maxOmegaPow2;

	if(st.Om6pow2 < 0)
		st.Om6pow2 = 0;
	else if(st.Om6pow2 > this->maxOmegaPow2)
		st.Om6pow2 = this->maxOmegaPow2;

	if(st.Om7pow2 < 0)
		st.Om7pow2 = 0;
	else if(st.Om7pow2 > this->maxOmegaPow2)
		st.Om7pow2 = this->maxOmegaPow2;

	if(st.Om8pow2 < 0)
		st.Om8pow2 = 0;
	else if(st.Om8pow2 > this->maxOmegaPow2)
		st.Om8pow2 = this->maxOmegaPow2;
    
  st.Om1 = sqrt( st.Om1pow2 );
	st.Om2 = sqrt( st.Om2pow2 );
	st.Om3 = sqrt( st.Om3pow2 );
	st.Om4 = sqrt( st.Om4pow2 );
	st.Om5 = sqrt( st.Om5pow2 );
	st.Om6 = sqrt( st.Om6pow2 );
	st.Om7 = sqrt( st.Om7pow2 );
	st.Om8 = sqrt( st.Om8pow2 );  
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboDraganflyX8)
} // end namespace gazebo
