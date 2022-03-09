/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/HeightWithVarianceStamped.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_HEIGHTWITHVARIANCESTAMPED_H
#define QUADROTOR_MSGS_MESSAGE_HEIGHTWITHVARIANCESTAMPED_H
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
struct HeightWithVarianceStamped_ {
  typedef HeightWithVarianceStamped_<ContainerAllocator> Type;

  HeightWithVarianceStamped_()
  : header()
  , height(0.0)
  , variance(0.0)
  {
  }

  HeightWithVarianceStamped_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , height(0.0)
  , variance(0.0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _height_type;
  double height;

  typedef double _variance_type;
  double variance;


  typedef boost::shared_ptr< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct HeightWithVarianceStamped
typedef  ::quadrotor_msgs::HeightWithVarianceStamped_<std::allocator<void> > HeightWithVarianceStamped;

typedef boost::shared_ptr< ::quadrotor_msgs::HeightWithVarianceStamped> HeightWithVarianceStampedPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::HeightWithVarianceStamped const> HeightWithVarianceStampedConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e8bd1477ab0ef2652e4a3929da4c5973";
  }

  static const char* value(const  ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe8bd1477ab0ef265ULL;
  static const uint64_t static_value2 = 0x2e4a3929da4c5973ULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/HeightWithVarianceStamped";
  }

  static const char* value(const  ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > {
  static const char* value() 
  {
    return "std_msgs/Header header\n\
float64 height\n\
float64 variance\n\
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

  static const char* value(const  ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.height);
    stream.next(m.variance);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct HeightWithVarianceStamped_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::HeightWithVarianceStamped_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "height: ";
    Printer<double>::stream(s, indent + "  ", v.height);
    s << indent << "variance: ";
    Printer<double>::stream(s, indent + "  ", v.variance);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_HEIGHTWITHVARIANCESTAMPED_H
