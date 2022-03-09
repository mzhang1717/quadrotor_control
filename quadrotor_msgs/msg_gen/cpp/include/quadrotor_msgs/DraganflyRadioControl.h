/* Auto-generated by genmsg_cpp for file /home/mingfeng/ros_workspace/quadrotor_msgs/msg/DraganflyRadioControl.msg */
#ifndef QUADROTOR_MSGS_MESSAGE_DRAGANFLYRADIOCONTROL_H
#define QUADROTOR_MSGS_MESSAGE_DRAGANFLYRADIOCONTROL_H
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
struct DraganflyRadioControl_ {
  typedef DraganflyRadioControl_<ContainerAllocator> Type;

  DraganflyRadioControl_()
  : header()
  , throttle(0)
  , roll(0)
  , pitch(0)
  , yaw(0)
  , shutter(0)
  , ascent(0)
  , zoom(0)
  , tilt(0)
  , hold(0)
  {
  }

  DraganflyRadioControl_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , throttle(0)
  , roll(0)
  , pitch(0)
  , yaw(0)
  , shutter(0)
  , ascent(0)
  , zoom(0)
  , tilt(0)
  , hold(0)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef int16_t _throttle_type;
  int16_t throttle;

  typedef int16_t _roll_type;
  int16_t roll;

  typedef int16_t _pitch_type;
  int16_t pitch;

  typedef int16_t _yaw_type;
  int16_t yaw;

  typedef int16_t _shutter_type;
  int16_t shutter;

  typedef int16_t _ascent_type;
  int16_t ascent;

  typedef int16_t _zoom_type;
  int16_t zoom;

  typedef int16_t _tilt_type;
  int16_t tilt;

  typedef int16_t _hold_type;
  int16_t hold;


  typedef boost::shared_ptr< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DraganflyRadioControl
typedef  ::quadrotor_msgs::DraganflyRadioControl_<std::allocator<void> > DraganflyRadioControl;

typedef boost::shared_ptr< ::quadrotor_msgs::DraganflyRadioControl> DraganflyRadioControlPtr;
typedef boost::shared_ptr< ::quadrotor_msgs::DraganflyRadioControl const> DraganflyRadioControlConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace quadrotor_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "79d4bc54ef71d2c3d5fd7f5ce72a368a";
  }

  static const char* value(const  ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x79d4bc54ef71d2c3ULL;
  static const uint64_t static_value2 = 0xd5fd7f5ce72a368aULL;
};

template<class ContainerAllocator>
struct DataType< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "quadrotor_msgs/DraganflyRadioControl";
  }

  static const char* value(const  ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > {
  static const char* value() 
  {
    return "std_msgs/Header header\n\
int16 throttle\n\
int16 roll\n\
int16 pitch\n\
int16 yaw\n\
int16 shutter\n\
int16 ascent\n\
int16 zoom\n\
int16 tilt\n\
int16 hold\n\
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

  static const char* value(const  ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.throttle);
    stream.next(m.roll);
    stream.next(m.pitch);
    stream.next(m.yaw);
    stream.next(m.shutter);
    stream.next(m.ascent);
    stream.next(m.zoom);
    stream.next(m.tilt);
    stream.next(m.hold);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DraganflyRadioControl_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::quadrotor_msgs::DraganflyRadioControl_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "throttle: ";
    Printer<int16_t>::stream(s, indent + "  ", v.throttle);
    s << indent << "roll: ";
    Printer<int16_t>::stream(s, indent + "  ", v.roll);
    s << indent << "pitch: ";
    Printer<int16_t>::stream(s, indent + "  ", v.pitch);
    s << indent << "yaw: ";
    Printer<int16_t>::stream(s, indent + "  ", v.yaw);
    s << indent << "shutter: ";
    Printer<int16_t>::stream(s, indent + "  ", v.shutter);
    s << indent << "ascent: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ascent);
    s << indent << "zoom: ";
    Printer<int16_t>::stream(s, indent + "  ", v.zoom);
    s << indent << "tilt: ";
    Printer<int16_t>::stream(s, indent + "  ", v.tilt);
    s << indent << "hold: ";
    Printer<int16_t>::stream(s, indent + "  ", v.hold);
  }
};


} // namespace message_operations
} // namespace ros

#endif // QUADROTOR_MSGS_MESSAGE_DRAGANFLYRADIOCONTROL_H

