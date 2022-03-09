//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
//=========================================================================================

#ifndef QUADROTOR_INPUT_H_
#define QUADROTOR_INPUT_H_

namespace gazebo{
  
  /** @brief Holds the latest inputs and maximum inputs in one structure.
 * 
 * This way the input structure can easily be passed to functions that work on it. */
struct QuadrotorInput
{    
  public: 
    QuadrotorInput()
    {
      roll = 0;
      pitch = 0;
      yawRate = 0;
      throttle = 0;
        
      rollMax = 0;
      pitchMax = 0;
      yawRateMax = 0;
      throttleMax = 0;
    }
    
    // All of these take values from -1 to 1
    double roll;
    double pitch;
    double yawRate;
    double throttle;  
    
    double rollMax, pitchMax;  // rad
    double yawRateMax;  // rad/s
    double throttleMax; // Newtons
};

} // end namespace

#endif
