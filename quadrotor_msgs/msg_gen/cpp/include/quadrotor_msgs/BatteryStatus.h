/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/BatteryStatus.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_BATTERYSTATUS_H
#define QUADROTOR_MSGS_MESSAGE_BATTERYSTATUS_H
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

#include "std_msgs/Header.h"

namespace quadrotor_msgs
{
template <class ContainerAllocator>
struct BatteryStatus_ {
  typedef BatteryStatus_<ContainerAllocator> Type;

  BatteryStatus_()
  : header()
  , voltage(0.0)
  , current(0.0)
  {
  }

  BatteryStatus_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , voltage(0.0)
  , current(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _voltage_type;
  float voltage;

  typedef float _current_type;
  float current;


  typedef boost::shared_ptr< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct BatteryStatus
typedef  ::quadrotor_msgs::BatteryStatus_<std::allocator<void> > BatteryStatus;

typedef boost::shared_ptr< ::quadrotor_msgs::BatteryStatus> BatteryStatusPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::BatteryStatus const> BatteryStatusConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "75c8e4b7132acff3b679451f5604f145";
  }

  static const char* value(const  ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x75c8e4b7132acff3ULL;
  static const uint64_t static_value2 = 0xb679451f5604f145ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/BatteryStatus";
  }

  static const char* value(const  ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 voltage\n\
float32 current\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.voltage);
    stream.next(m.current);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct BatteryStatus_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::BatteryStatus_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "voltage: ";
    Printer<float>::stream(s, indent + "  ", v.voltage);
    s << indent << "current: ";
    Printer<float>::stream(s, indent + "  ", v.current);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_BATTERYSTATUS_H
