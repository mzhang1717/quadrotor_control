/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/PositionDebug.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_POSITIONDEBUG_H
#define QUADROTOR_MSGS_MESSAGE_POSITIONDEBUG_H
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

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Quaternion.h"

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct PositionDebug_ {
  typedef PositionDebug_<ContainerAllocator> Type;

  PositionDebug_()
  : base_throttle(0.0)
  , scale(0.0)
  , voltage(0.0)
  , xy_error()
  , vx_error()
  , vy_error()
  , z_error()
  , vz_error()
  , yaw_error()
  , desired_xyz_yaw()
  , current_xyz_yaw()
  , desired_velocity()
  , desired_acceleration()
  , cmd()
  , cmd_limited()
  {
  }

  PositionDebug_(const ContainerAllocator& _alloc)
  : base_throttle(0.0)
  , scale(0.0)
  , voltage(0.0)
  , xy_error(_alloc)
  , vx_error(_alloc)
  , vy_error(_alloc)
  , z_error(_alloc)
  , vz_error(_alloc)
  , yaw_error(_alloc)
  , desired_xyz_yaw(_alloc)
  , current_xyz_yaw(_alloc)
  , desired_velocity(_alloc)
  , desired_acceleration(_alloc)
  , cmd(_alloc)
  , cmd_limited(_alloc)
  {
  }

  typedef double _base_throttle_type;
  double base_throttle;

  typedef double _scale_type;
  double scale;

  typedef double _voltage_type;
  double voltage;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _xy_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  xy_error;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vx_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  vx_error;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vy_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  vy_error;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _z_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  z_error;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _vz_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  vz_error;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _yaw_error_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  yaw_error;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _desired_xyz_yaw_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  desired_xyz_yaw;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _current_xyz_yaw_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  current_xyz_yaw;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _desired_velocity_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  desired_velocity;

  typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _desired_acceleration_type;
   ::geometry_msgs::Vector3_<ContainerAllocator>  desired_acceleration;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _cmd_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  cmd;

  typedef  ::geometry_msgs::Quaternion_<ContainerAllocator>  _cmd_limited_type;
   ::geometry_msgs::Quaternion_<ContainerAllocator>  cmd_limited;


  typedef boost::shared_ptr< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::PositionDebug_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PositionDebug
typedef  ::quadrotor_msgs::PositionDebug_<std::allocator<void> > PositionDebug;

typedef boost::shared_ptr< ::quadrotor_msgs::PositionDebug> PositionDebugPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::PositionDebug const> PositionDebugConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::PositionDebug_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PositionDebug_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "93d1333b166d8ea6c1d79f7209332822";
  }

  static const char* value(const  ::quadrotor_msgs::PositionDebug_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x93d1333b166d8ea6ULL;
  static const uint64_t static_value2 = 0xc1d79f7209332822ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/PositionDebug";
  }

  static const char* value(const  ::quadrotor_msgs::PositionDebug_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 base_throttle\n\
float64 scale\n\
float64 voltage\n\
geometry_msgs/Vector3 xy_error\n\
geometry_msgs/Vector3 vx_error\n\
geometry_msgs/Vector3 vy_error\n\
geometry_msgs/Vector3 z_error\n\
geometry_msgs/Vector3 vz_error\n\
geometry_msgs/Vector3 yaw_error\n\
geometry_msgs/Quaternion desired_xyz_yaw\n\
geometry_msgs/Quaternion current_xyz_yaw\n\
geometry_msgs/Vector3 desired_velocity\n\
geometry_msgs/Vector3 desired_acceleration\n\
geometry_msgs/Quaternion cmd\n\
geometry_msgs/Quaternion cmd_limited\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::quadrotor_msgs::PositionDebug_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.base_throttle);
    stream.next(m.scale);
    stream.next(m.voltage);
    stream.next(m.xy_error);
    stream.next(m.vx_error);
    stream.next(m.vy_error);
    stream.next(m.z_error);
    stream.next(m.vz_error);
    stream.next(m.yaw_error);
    stream.next(m.desired_xyz_yaw);
    stream.next(m.current_xyz_yaw);
    stream.next(m.desired_velocity);
    stream.next(m.desired_acceleration);
    stream.next(m.cmd);
    stream.next(m.cmd_limited);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PositionDebug_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::PositionDebug_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::PositionDebug_<ContainerAllocator> & v) 
  {
    s << indent << "base_throttle: ";
    Printer<double>::stream(s, indent + "  ", v.base_throttle);
    s << indent << "scale: ";
    Printer<double>::stream(s, indent + "  ", v.scale);
    s << indent << "voltage: ";
    Printer<double>::stream(s, indent + "  ", v.voltage);
    s << indent << "xy_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.xy_error);
    s << indent << "vx_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vx_error);
    s << indent << "vy_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vy_error);
    s << indent << "z_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.z_error);
    s << indent << "vz_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.vz_error);
    s << indent << "yaw_error: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.yaw_error);
    s << indent << "desired_xyz_yaw: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.desired_xyz_yaw);
    s << indent << "current_xyz_yaw: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.current_xyz_yaw);
    s << indent << "desired_velocity: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.desired_velocity);
    s << indent << "desired_acceleration: ";
s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.desired_acceleration);
    s << indent << "cmd: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd);
    s << indent << "cmd_limited: ";
s << std::endl;
    Printer< ::geometry_msgs::Quaternion_<ContainerAllocator> >::stream(s, indent + "  ", v.cmd_limited);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_POSITIONDEBUG_H
