/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/PressureAltitude.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_PRESSUREALTITUDE_H
#define QUADROTOR_MSGS_MESSAGE_PRESSUREALTITUDE_H
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
struct PressureAltitude_ {
  typedef PressureAltitude_<ContainerAllocator> Type;

  PressureAltitude_()
  : header()
  , altitude(0.0)
  , pressure(0.0)
  , ADC(0)
  , error(0.0)
  , vel(0.0)
  , correction(0)
  {
  }

  PressureAltitude_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , altitude(0.0)
  , pressure(0.0)
  , ADC(0)
  , error(0.0)
  , vel(0.0)
  , correction(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef float _altitude_type;
  float altitude;

  typedef float _pressure_type;
  float pressure;

  typedef int32_t _ADC_type;
  int32_t ADC;

  typedef float _error_type;
  float error;

  typedef float _vel_type;
  float vel;

  typedef int16_t _correction_type;
  int16_t correction;


  typedef boost::shared_ptr< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PressureAltitude
typedef  ::quadrotor_msgs::PressureAltitude_<std::allocator<void> > PressureAltitude;

typedef boost::shared_ptr< ::quadrotor_msgs::PressureAltitude> PressureAltitudePtr;
typedef boost::shared_ptr< ::quadrotor_msgs::PressureAltitude const> PressureAltitudeConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8b5fd5f2e7ab353ddae702e5d39677d8";
  }

  static const char* value(const  ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8b5fd5f2e7ab353dULL;
  static const uint64_t static_value2 = 0xdae702e5d39677d8ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/PressureAltitude";
  }

  static const char* value(const  ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
float32 altitude\n\
float32 pressure\n\
int32 ADC\n\
float32 error\n\
float32 vel\n\
int16 correction\n\
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

  static const char* value(const  ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.altitude);
    stream.next(m.pressure);
    stream.next(m.ADC);
    stream.next(m.error);
    stream.next(m.vel);
    stream.next(m.correction);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PressureAltitude_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::PressureAltitude_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "altitude: ";
    Printer<float>::stream(s, indent + "  ", v.altitude);
    s << indent << "pressure: ";
    Printer<float>::stream(s, indent + "  ", v.pressure);
    s << indent << "ADC: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ADC);
    s << indent << "error: ";
    Printer<float>::stream(s, indent + "  ", v.error);
    s << indent << "vel: ";
    Printer<float>::stream(s, indent + "  ", v.vel);
    s << indent << "correction: ";
    Printer<int16_t>::stream(s, indent + "  ", v.correction);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_PRESSUREALTITUDE_H

