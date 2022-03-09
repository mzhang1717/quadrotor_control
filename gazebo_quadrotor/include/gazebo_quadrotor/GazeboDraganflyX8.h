//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#ifndef GAZEBO_DRAGANFLY_X8_H
#define GAZEBO_DRAGANFLY_X8_H

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/TransportTypes.hh>

#include <quadrotor_msgs/DraganflyRadioControl.h>

#include <gazebo_quadrotor/QuadrotorState.h>
#include <gazebo_quadrotor/QuadrotorCommand.h>
#include <gazebo_quadrotor/QuadrotorPIDController.h>

namespace gazebo
{
  
/// Get a double parameter from the SDF node, given a default value and if we should complain if the parameter is not defined
inline double getParamDouble(sdf::ElementPtr _sdf, std::string name, double defaultVal, bool fatalIfNotDefined)
{
  if (!_sdf->HasElement(name))
  {
    if(fatalIfNotDefined)
    {
      std::stringstream ss;
      ss<<"GazeboDraganflyX8 plugin missing <"<<name<<">, cannot continue";
      ROS_FATAL_STREAM(ss.str());
      gzthrow(ss.str());
      return 0;
    }
    else
    {
      ROS_WARN_STREAM("GazeboDraganflyX8 plugin missing <"<<name<<">, using default value: "<<defaultVal);
      return defaultVal;
    }
  }
  else
    return _sdf->GetElement(name)->GetValueDouble();
}
  

/** @brief The controller that is called by gazebo.
 * 
 *  Loads parameters from the world file, receives radio control input, calls the PID controller, applies forces/torques */
class GazeboDraganflyX8 : public ModelPlugin
{
  public:
    GazeboDraganflyX8();
    virtual ~GazeboDraganflyX8();

  protected:
  
    /// \brief Load the controller
    /// \param _parent The parent model
    /// \param _sdf The sdf node that will be scanned for parameters
    virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

    /// \brief Initialize the controller
    virtual void Init();
    
    /// \brief Reset the controller
    virtual void Reset();
    
  private:
    
    /// \brief Callback to update the internal input message based on a received message
    void UpdateInput(const quadrotor_msgs::DraganflyRadioControl::ConstPtr& inputMsg);
    
    /** @brief Update the propeller speeds based on the input commands */
    void UpdateOmegas(QuadrotorState &st, const QuadrotorCommand &cmd);
    
    /// \brief Main update function, called every world update iteration
    void Update();
    
    /** @brief This calls callbacks in the custom queue repeatedly, spawned in its own thread */
    void QueueThread();

    
    physics::WorldPtr myWorld;  ///< The parent World
    physics::ModelPtr myModel;  ///< The parent model
    physics::LinkPtr myBody; ///< The body that will be manipulated by this plugin
    
    QuadrotorPIDController attitudeController;   ///< Does the actual PID control

    // Variables to hold the values of parameters
    std::string inputTopicName;
    std::string bodyName;
    std::string robotNamespace;
    double thrustFactorTop;
    double thrustFactorBottom;
    double dragFactorTop;
    double dragFactorBottom;
    double dragCoeff;
    double JrTop;
    double JrBottom;
    double rRotor;
    double L;
    double maxOmega;
    
    double batteryVoltage;
    double batteryCurrent;
    
    // Inertial values
    double Ixx;
    double Iyy;
    double Izz;
    
    double  maxOmegaPow2;   ///< The maximum rotor speed squared
    double hoverThrust;     ///< Base thrust (N) to keep vehicle aloft
   
    ros::NodeHandle* rosnode;  ///< A pointer to the ROS node.  A node will be instantiated if it does not exist.
    ros::Subscriber inputSubscriber;  ///< Subscribes to incoming DraganflyRadioControl messages
    ros::Publisher batteryPub;
    ros::Publisher forcePub;  ///< Publishes force/torque messages
    ros::Publisher statePub;  ///< Publishes state messages
    ros::Publisher posePub;   ///< Publishes pose messages
    ros::Publisher velocityPub;  ///<Publishes twist messages
    ros::Publisher stateDebugPub;   ///< Publishes state debug messages
    ros::Publisher heightPub;
    
    ros::CallbackQueue queue; ///< Custom callback queue
    boost::thread callbackQueueThread;   ///< The thread where callbacks are called

    common::Time lastTime;    ///< For controller step
    
    QuadrotorCommand command;    ///< The current four commands
    QuadrotorState state;        ///< The current quadrotor state
    QuadrotorInput input;        ///< The current inputs from the RC controller and max values for roll,pitch,yaw,throttle in appropriate units
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
  
};

}
#endif
