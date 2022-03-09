#ifndef YAML_PARSER_H
#define YAML_PARSER_H

#include <state_estimation/Utility.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <TooN/TooN.h>
#include <TooN/so3.h>
#include <tf/transform_datatypes.h>

namespace yaml{
  
/// Reads a Point message from a YAML node
void operator >> (const YAML::Node& node,  TooN::Vector<3> &p) 
{
  node[0] >> p[0];
  node[1] >> p[1];
  node[2] >> p[2];
}

/// Reads a Quaternion message from a YAML node
void operator >> (const YAML::Node& node,  TooN::SO3<>& r) 
{
  double x,y,z,w;
  
  node[0] >> x;
  node[1] >> y;
  node[2] >> z;
  node[3] >> w;
  
  tf::Quaternion q(x,y,z,w);
  r = util::QuatToSO3(q);
}

/// Writes a Point message to a YAML node 
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Point& v)
{
  out << YAML::Flow;
  out << YAML::BeginSeq;
  out << v.x << v.y << v.z;
  out << YAML::EndSeq;
  
  return out;
}

/// Writes a Quaternion message to a YAML node
YAML::Emitter& operator << (YAML::Emitter& out, const geometry_msgs::Quaternion& q)
{
  out << YAML::Flow;
  out << YAML::BeginSeq;
  out << q.x << q.y << q.z << q.w;
  out << YAML::EndSeq;
  
  return out;
}

/// Writes a Point to a YAML node 
YAML::Emitter& operator << (YAML::Emitter& out, const tf::Point& v)
{
  out << YAML::Flow;
  out << YAML::BeginSeq;
  out << v.x() << v.y() << v.z();
  out << YAML::EndSeq;
  
  return out;
}

/// Writes a Quaternion to a YAML node
YAML::Emitter& operator << (YAML::Emitter& out, const tf::Quaternion& q)
{
  out << YAML::Flow;
  out << YAML::BeginSeq;
  out << q.x() << q.y() << q.z() << q.w();
  out << YAML::EndSeq;
  
  return out;
}

inline void writePose(std::ostream &out, tf::Transform transform)
{
  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "position" << YAML::Value << transform.getOrigin();
  emitter << YAML::Key << "orientation" << YAML::Value << transform.getRotation();
  emitter << YAML::EndMap;
  
  out << emitter.c_str();
}

inline void writePose(std::ostream &out, geometry_msgs::Pose pose)
{
  YAML::Emitter emitter;

  emitter << YAML::BeginMap;
  emitter << YAML::Key << "position" << YAML::Value << pose.position;
  emitter << YAML::Key << "orientation" << YAML::Value << pose.orientation;
  emitter << YAML::EndMap;
  
  out << emitter.c_str();
}

inline void loadPose(std::string filename, TooN::Vector<3> &pos, TooN::SO3<> &rot)
{
  std::ifstream ifs(filename.c_str());
  
  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser) 
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    doc["position"] >> pos;
    doc["orientation"] >> rot;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

inline void loadPosition(std::string filename, TooN::Vector<3> &pos)
{
  std::ifstream ifs(filename.c_str());
  
  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser) 
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    doc["position"] >> pos;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

inline void loadOrientation(std::string filename, TooN::SO3<> &rot)
{
  std::ifstream ifs(filename.c_str());
  
  if(!ifs.good())
  {
    ROS_FATAL_STREAM("YAMLParser: Couldn't open "<<filename<<" to get pose information");
    ros::shutdown();
  }

  try {
    YAML::Parser parser(ifs);
    if (!parser) 
    {
      ROS_FATAL("YAMLParser: Unable to create YAML parser for loading pose");
      ros::shutdown();
    }
    YAML::Node doc;
    parser.GetNextDocument(doc);
    
    doc["orientation"] >> rot;
  }
  catch (YAML::Exception& e) {
    ROS_FATAL("YAMLParser: Exception parsing YAML when loading pose:\n%s", e.what());
    ros::shutdown();
  }
}

} // end namespace yaml

#endif
