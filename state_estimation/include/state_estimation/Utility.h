#ifndef UTILITY_H
#define UTILITY_H

#include <TooN/TooN.h>
#include <TooN/so3.h>
//#include <tag/helpers.h>
#include <tf/transform_datatypes.h>


namespace util{
/*
inline TooN::Matrix<3> getExpIntegral(TooN::Vector<3> omega, double dt)
{
  TooN::Matrix<3> R_integral;
  TooN::Matrix<3> I = TooN::Identity;
  
  double omega_mag = norm(omega);
  TooN::Vector<3> k = omega/omega_mag;
  
  R_integral = I*dt - tag::getCrossProductMatrix(k)*(cos(omega_mag*dt)/omega_mag) + (k.as_col()*k.as_row() - I)*(dt - sin(omega_mag*dt)/omega_mag);
  return R_integral;
}

inline TooN::Matrix<3> getExpDoubleIntegral(TooN::Vector<3> omega, double dt)
{
  TooN::Matrix<3> R_double_integral;
  TooN::Matrix<3> I = TooN::Identity;
  
  double omega_mag = norm(omega);
  TooN::Vector<3> k = omega/omega_mag;
  
  R_double_integral = I*dt*dt/2 - tag::getCrossProductMatrix(k)*(sin(omega_mag*dt)/(omega_mag*omega_mag)) + (k.as_col()*k.as_row() - I)*(dt*dt/2 + cos(omega_mag*dt)/(omega_mag*omega_mag));
  return R_double_integral;
}

inline TooN::Matrix<3> getExpDoubleIntegralExtra(TooN::Vector<3> omega, double dt)
{
  TooN::Matrix<3> R_double_integral_extra;
  TooN::Matrix<3> I = TooN::Identity;
  
  double om = norm(omega);
  double om2 = om*om;
  double om3 = om2*om;
  TooN::Vector<3> k = omega/om;
  
  R_double_integral_extra = I*dt*dt*dt/3 + tag::getCrossProductMatrix(k)*(dt*(-sin(om*dt)/om2) - cos(om*dt)/om3) + (k.as_col()*k.as_row() - I)*(dt*dt*dt/3) 
                      -(k.as_col()*k.as_row() - I)*(dt*(-cos(om*dt)/om2) - (-sin(om*dt)/om3));
                      
  return R_double_integral_extra;
}
*/

inline TooN::SO3<> QuatToSO3(tf::Quaternion q)
{
  TooN::SO3<> so3Rot;
  
  // Convert to Bullet 3x3 matrix
  tf::Matrix3x3 btMat(q);
  
  // Copy data into a TooN 3-Matrix
  TooN::Matrix<3> m3Rot;
  for(unsigned j=0; j < 3; j++)
  {
    for(unsigned i=0; i < 3; i++)
    {
      m3Rot[i][j] = btMat[i][j];
    }
  }
  
  // Set the rotation and we're done
  so3Rot = m3Rot;
  return so3Rot;
}

inline tf::Quaternion SO3ToQuat(TooN::SO3<> so3Rot)
{
  TooN::Matrix<3> m3Rot = so3Rot.get_matrix();
  
  // Initialize 3x3 matrix directly from TooN 3-matrix
  tf::Matrix3x3 btMat(m3Rot(0,0),m3Rot(0,1),m3Rot(0,2),m3Rot(1,0),m3Rot(1,1),m3Rot(1,2),m3Rot(2,0),m3Rot(2,1),m3Rot(2,2));
  tf::Quaternion btQuat;
  btMat.getRotation(btQuat);
  
  return btQuat;
}

inline double AngleDiff(double angle1, double angle2)
{
  double diff = angle1 - angle2;
  
  while(diff > M_PI)
  {
    diff -= 2*M_PI;
  }	
  
  while(diff < -M_PI)
  {
    diff += 2*M_PI;
  }	
  
  return diff;
}

} // end namespace util

#endif
