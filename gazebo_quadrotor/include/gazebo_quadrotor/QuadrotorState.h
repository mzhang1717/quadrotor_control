//=========================================================================================
//
// Copyright 2011 Adam Harmat, McGill University
// adam.harmat@mail.mcgill.ca
//
// Revised by Tao Wang, 2013
// extend parameters for x8 against quadrotor
//
//=========================================================================================

#ifndef QUADROTOR_STATE_H_
#define QUADROTOR_STATE_H_

namespace gazebo{
  
/** @brief Holds the current state of the quadrotor  */
struct QuadrotorState{
    
  public: QuadrotorState() : roll(0), pitch(0), yaw(0),
                             rollRate(0), pitchRate(0), yawRate(0),
                             x(0), y(0), z(0),
                             xRate(0), yRate(0), zRate(0),
                             Om1(0), Om2(0), Om3(0), Om4(0), Om5(0), Om6(0), Om7(0), Om8(0),
                             Om1pow2(0), Om2pow2(0), Om3pow2(0), Om4pow2(0), Om5pow2(0), Om6pow2(0), Om7pow2(0), Om8pow2(0)
          {};

  public:
    double roll;   ///< Body frame
    double pitch;   ///< Body frame
    double yaw;   ///< Body frame
    double rollRate;   ///< Body frame
    double pitchRate;   ///< Body frame
    double yawRate;     ///< Body frame
    double x;  ///< World coordinates
    double y;  ///< World coordinates
    double z;  ///< World coordinates
    double xRate; ///< World velocity
    double yRate; ///< World velocity
    double zRate; ///< World velocity
    double Om1;  ///< Rotor speeds
    double Om2;  ///< Rotor speeds
    double Om3;  ///< Rotor speeds
    double Om4;  ///< Rotor speeds
    double Om5;  ///< Rotor speeds
    double Om6;  ///< Rotor speeds
    double Om7;  ///< Rotor speeds
    double Om8;  ///< Rotor speeds
    double Om1pow2;  ///< Rotor speed squared
    double Om2pow2;  ///< Rotor speed squared
    double Om3pow2;  ///< Rotor speed squared
    double Om4pow2;  ///< Rotor speed squared
    double Om5pow2;  ///< Rotor speed squared
    double Om6pow2;  ///< Rotor speed squared
    double Om7pow2;  ///< Rotor speed squared
    double Om8pow2;  ///< Rotor speed squared
 
    
};

} // end namespace

#endif
