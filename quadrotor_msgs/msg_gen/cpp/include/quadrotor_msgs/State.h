/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/State.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_STATE_H
#define QUADROTOR_MSGS_MESSAGE_STATE_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct State_ {
  typedef State_<ContainerAllocator> Type;

  State_()
  : pose()
  , velocity()
  , base_throttle(0.0)
  {
  }

  State_(const ContainerAllocator& _alloc)
  : pose(_alloc)
  , velocity(_alloc)
  , base_throttle(0.0)
  {
  }

  typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
   ::geometry_msgs::Pose_<ContainerAllocator>  pose;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  velocity;

  typedef float _base_throttle_type;
  float base_throttle;


  typedef boost::shared_ptr< ::quadrotor_msgs::State_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::State_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct State
typedef  ::quadrotor_msgs::State_<std::allocator<void> > State;

typedef boost::shared_ptr< ::quadrotor_msgs::State> StatePtr;
typedef boost::shared_ptr< ::quadrotor_msgs::State const> StateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::State_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::State_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::State_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::State_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "71e4dee1ab3c8b1fb6bbd4a948424df2";
  }

  static const char* value(const  ::quadrotor_msgs::State_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x71e4dee1ab3c8b1fULL;
  static const uint64_t static_value2 = 0xb6bbd4a948424df2ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/State";
  }

  static const char* value(const  ::quadrotor_msgs::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::State_<ContainerAllocator> > {
  static const char* value() 
  {
    return "geometry_msgs/Pose pose\n\
geometry_msgs/Vector3 velocity\n\
float32 base_throttle\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const  ::quadrotor_msgs::State_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::quadrotor_msgs::State_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::State_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.pose);
    stream.next(m.velocity);
    stream.next(m.base_throttle);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct State_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::State_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::State_<ContainerAllocator> & v) 
  {
    s << indent << "pose: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "base_throttle: ";
    Printer<float>::stream(s, indent + "  ", v.base_throttle);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_STATE_H

