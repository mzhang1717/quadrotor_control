//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#ifndef QUADROTOR_COMMAND_H_
#define QUADROTOR_COMMAND_H_

namespace gazebo{
  
  /** @brief Holds the 4 commands used to control the quadrotor.
 * 
 * This way the command structure can easily be passed to functions that work on it. */
struct QuadrotorCommand
{    
  public: 
    QuadrotorCommand() : U1(0), U2(0), U3(0), U4(0) {}
    double U1, U2, U3, U4;
};

} // end namespace

#endif
