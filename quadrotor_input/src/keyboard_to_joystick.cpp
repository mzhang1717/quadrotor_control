//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// This program takes keyboard input and translates them to joystick messages
//
// The inspiration for this program was pr2_teleop so much of the code is the same
// or similar. However, this program wasn't tested too thoroughly as operating
// the quadrotor with a keyboard is a pain in the ass and I just use the joystick instead.
//
//=========================================================================================

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <joy/Joy.h>

#define KEYCODE_RIGHT 0x43 
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_C 0x63
#define KEYCODE_U 0x75
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
//#define KEYCODE_X 0x78
//#define KEYCODE_Z 0x7A

class KeyboardToJoystick
{
public:
  KeyboardToJoystick();
  void keyLoop();
  void updateAxis(double &axis, double axis_scale, int dir, double min = -1.0, double max = 1.0);

private:

  ros::NodeHandle nh_;
  double roll, pitch, yaw, throttle, controller, ascent;
  double roll_scale, pitch_scale, yaw_scale, throttle_scale;
  int roll_axis, pitch_axis, yaw_axis, throttle_axis, controller_axis, ascent_axis;
  
  double rate;
  ros::Publisher command_pub_;
  
};

KeyboardToJoystick::KeyboardToJoystick():
  roll(0), pitch(0), yaw(0), throttle(-1), controller(0), ascent(0),
  roll_scale(0.05), pitch_scale(0.05), yaw_scale(0.05), throttle_scale(0.05),
  roll_axis(1), pitch_axis(2), yaw_axis(5), throttle_axis(0), controller_axis(3), ascent_axis(4),
  rate(50)
{
  nh_.param("roll_scale", roll_scale, roll_scale);
  nh_.param("pitch_scale", pitch_scale, pitch_scale);
  nh_.param("yaw_scale", yaw_scale, yaw_scale);
  nh_.param("throttle_scale", throttle_scale, throttle_scale);
  nh_.param("roll_axis", roll_axis, roll_axis);
  nh_.param("pitch_axis", pitch_axis, pitch_axis);
  nh_.param("yaw_axis", yaw_axis, yaw_axis);
  nh_.param("throttle_axis", throttle_axis, throttle_axis);
  nh_.param("controller_axis", controller_axis, controller_axis);
  nh_.param("ascent_axis", ascent_axis, ascent_axis);
  nh_.param("rate", rate, rate);

  command_pub_ = nh_.advertise<joy::Joy>("joy", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_node");
  KeyboardToJoystick kj;

  signal(SIGINT,quit);

  kj.keyLoop();
  
  return(0);
}

void KeyboardToJoystick::updateAxis(double &axis, double axis_scale, int dir, double min, double max)
{
  axis += dir*axis_scale;
  
  if(axis > max)
    axis = max;
  else if(axis < min)
    axis = min;
}

void KeyboardToJoystick::keyLoop()
{
  char c;
  bool dirty=true;

  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 1;
  tcsetattr(kfd, TCSANOW, &raw);

  //puts("Reading from keyboard");
  //puts("---------------------------");
  //puts("Use arrow keys to control Quadrotor roll and pitch, WASD to control yaw and throttle.");
  std::cout<<"Reading from keyboard"<<std::endl;
  std::cout<<"---------------------------"<<std::endl;
  std::cout<<"Use arrow keys to control Quadrotor roll and pitch, WASD to control yaw and throttle"<<std::endl;
  std::cout<<"Z to toggle heigh hold, X to toggle XY hold"<<std::endl;

  ros::Rate loop_rate(rate);

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
    
    // Pull roll, pitch, yaw values back to zero (simulate center spring)
    /*
    if(fabs(roll) > 0)
      roll = (roll/fabs(roll)) * (fabs(roll) - roll_scale);
    if(fabs(pitch) > 0)
      pitch = (pitch/fabs(pitch)) * (fabs(pitch) - pitch_scale);
    if(fabs(yaw) > 0)
      yaw = (yaw/fabs(yaw)) * (fabs(yaw) - yaw_scale);
*/
    // Don't pull throttle back, ie no center spring
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        updateAxis(roll, roll_scale, -1);
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        updateAxis(roll, roll_scale, 1);
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        updateAxis(pitch, pitch_scale, 1);
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        updateAxis(pitch, pitch_scale, -1);
        dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("W");
        updateAxis(throttle, throttle_scale, 1);
        dirty = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("S");
        updateAxis(throttle, throttle_scale, -1);
        dirty = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("A");
        updateAxis(yaw, yaw_scale, 1);
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("D");
        updateAxis(yaw, yaw_scale, -1);
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("U");
        ascent = (ascent != 0 ? 0 : -1);
        dirty = true;
        break;
      case KEYCODE_C:
        ROS_DEBUG("C");
        controller = (controller != 0 ? 0 : -1);
        dirty = true;
        break;
    }
    
    c = '?';
   
    joy::Joy input;
    input.axes.resize(8);
    input.axes[throttle_axis] = -1*throttle;
    input.axes[roll_axis] = -1*roll;
    input.axes[pitch_axis] = -1*pitch;
    input.axes[yaw_axis] = -1*yaw;
    input.axes[controller_axis] = controller;
    input.axes[ascent_axis] = ascent;
    
    command_pub_.publish(input); 
    
    if(dirty ==true)
    {
      std::cout<<"roll: "<<roll<<"  pitch: "<<pitch<<"  yaw: "<<yaw<<"  throttle: "<<throttle<<" ascent: "<<ascent<<" controller: "<<controller<<std::endl;
      dirty=false;
    }
  }


  return;
}



