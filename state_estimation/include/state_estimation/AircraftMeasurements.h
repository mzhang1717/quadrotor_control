#ifndef AIRCRAFT_MEASUREMENTS_H
#define AIRCRAFT_MEASUREMENTS_H

#include <state_estimation/Utility.h>
#include <TooN/TooN.h>
#include <TooN/so3.h>
#include <TooN/se3.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

class MeasurementBase{
public:

  MeasurementBase(){
    pos_offset = TooN::Zeros;
    rot_offset = TooN::SO3<>();
  }

  TooN::Vector<3> pos_offset;  // defined as system=>sensor, ie pos(sensor) = pos(system) + offset
  TooN::SO3<> rot_offset;      // defined as system=>sensor, ie R(sensor) = R(system) * R(offset);
  
  ros::Time stamp;
};

template <class State>
class WorldHeight : public MeasurementBase {
public:

  static const int M_DIMENSION = 1;
  double height;
  TooN::Matrix<M_DIMENSION> covariance;
  TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;

  WorldHeight(void){
      covariance = TooN::Identity;
      height = 0;
      jacobian = TooN::Zeros;
      jacobian(0,State::STATE_DIMENSION-2) = 1;
  }

  const TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state ) const {
      return jacobian;
  }

  const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const {
      //std::cout<<"Covariance:"<<std::endl<<covariance<<std::endl;
      return covariance;
  }

  const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const {

      TooN::Vector<3> pos_from_state = state.position + state.rotation*pos_offset;
      TooN::Vector<M_DIMENSION> innovation = TooN::makeVector(height - pos_from_state[2]);
      std::cout<<"Innovation from height measurement: "<<innovation<<std::endl;
      return innovation;
  }

  void setCovariance( double sigma ){
      covariance = TooN::Identity * sigma;
  }

  void setCovariance( const TooN::Vector<M_DIMENSION> & sigma ){
      covariance = TooN::Zeros;
      for(int i = 0; i < M_DIMENSION; ++i){
          covariance(i,i) = sigma[i];
      }
  }
};

template <class State>
class GroundPlaneDistance : public MeasurementBase {
public:

  static const int M_DIMENSION = 1;
  double distance;
  TooN::Matrix<M_DIMENSION> covariance;
  TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;

  GroundPlaneDistance(void){
      covariance = TooN::Identity;
      distance = 0;
      jacobian = TooN::Zeros;
  }

  const TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state ) {
	  
	  // distance wrt position z
	  jacobian(0,2) = -1.0 / (state.rotation * rot_offset * TooN::makeVector(0,0,1))[2];
	  
	  // distance wrt rotation (numerically)
	  double delta = 1e-6;
	  for(int i=0; i < 3; ++i)
	  {
		  TooN::Vector<3> v3Increment = TooN::Zeros;
		  v3Increment[i] = delta;
		  
		  double distance_plus = -1*(state.position[2] + (state.rotation*TooN::SO3<>::exp(v3Increment)*pos_offset)[2])/(state.rotation*TooN::SO3<>::exp(v3Increment)*rot_offset * TooN::makeVector(0,0,1))[2];
		  double distance_minus = -1*(state.position[2] + (state.rotation*TooN::SO3<>::exp(-1*v3Increment)*pos_offset)[2])/(state.rotation*TooN::SO3<>::exp(-1*v3Increment)*rot_offset * TooN::makeVector(0,0,1))[2];
		  
		  jacobian(0,3+i) = (distance_plus - distance_minus)/(2*delta);
	  }
	  
      return jacobian;
  }

  const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const {
      //std::cout<<"Covariance:"<<std::endl<<covariance<<std::endl;
      return covariance;
  }

  const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const {

      double distance_from_state = -1*(state.position[2] + (state.rotation*pos_offset)[2])/(state.rotation * rot_offset * TooN::makeVector(0,0,1))[2];
      TooN::Vector<M_DIMENSION> innovation = TooN::makeVector(distance - distance_from_state);
      std::cout<<"Innovation from height measurement: "<<innovation<<std::endl;
      return innovation;
  }

  void setCovariance( double sigma ){
      covariance = TooN::Identity * sigma;
  }

  void setCovariance( const TooN::Vector<M_DIMENSION> & sigma ){
      covariance = TooN::Zeros;
      for(int i = 0; i < M_DIMENSION; ++i){
          covariance(i,i) = sigma[i];
      }
  }
};

template <class State>
class BodyXYVelocity : public MeasurementBase {
public:  
  static const int M_DIMENSION = 2;

  TooN::Matrix<M_DIMENSION> covariance;
  TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;
  TooN::Vector<2> velocity;
  
  BodyXYVelocity(void){
      covariance = TooN::Identity;
      jacobian = TooN::Zeros;
  }
  
  TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state )
  {
    // Since measurement is of dimension 2, we'll have to save the jacobians to a temporary 3x3 
    // matrix an take only the first two rows
    TooN::Matrix<3> m3TempJac;
    
    TooN::SO3<> m3RotOffsetInverse = rot_offset.inverse();
    
    // Measurement wrt rotation
    for(unsigned i=0; i < 3; ++i)
      m3TempJac.T()[i] = TooN::SO3<>::generator_field(i, state.velocity) * -1;
        
    jacobian.template slice<0,3,2,3>() = (m3RotOffsetInverse.get_matrix() * m3TempJac).slice<0,0,2,3>();
    
    // Measurement wrt velocity
    m3TempJac = (m3RotOffsetInverse * state.rotation.inverse()).get_matrix();
    
    jacobian.template slice<0,6,2,3>() = m3TempJac.slice<0,0,2,3>();
    
    return jacobian;
  }
  
  const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const 
  {
    return covariance;
  }
  
  const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const 
  {
    TooN::Vector<3> last_angular_velocity = (state.last_gyro - state.gyro_bias);
    
    TooN::Vector<3> velocity_from_state = rot_offset.inverse() * ((state.rotation.inverse() * state.velocity) +  
                                                                  (last_angular_velocity ^ pos_offset) );

    TooN::Vector<M_DIMENSION> innovation;
    innovation = velocity - velocity_from_state.slice<0,2>();
    
    return innovation;
  }
  
};

template <class State>
class WorldRotation : public MeasurementBase {
public:
    static const int M_DIMENSION = 3;

    TooN::Matrix<M_DIMENSION> covariance;
    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;
    TooN::SO3<> rotation;
    
    WorldRotation(void){
        covariance = TooN::Identity;
        jacobian = TooN::Zeros;
    }
    
    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state )
    {
      // Measurement rotation wrt rotation
      jacobian.template slice<0,3,3,3>() = TooN::Identity;
      
      return jacobian;
    }
    
    const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const 
    {
      return covariance;
    }

    const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const 
    {
      TooN::Vector<M_DIMENSION> innovation = (state.rotation.inverse() * rotation).ln();
      
      return innovation;
    }
};

template <class State>
class WorldYaw : public MeasurementBase {
public:
    static const int M_DIMENSION = 1;

    TooN::Matrix<M_DIMENSION> covariance;
    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;
    TooN::SO3<> rotation;
    
    WorldYaw(void){
        covariance = TooN::Identity;
        jacobian = TooN::Zeros;
    }
    
    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state )
    {
      // Measurement rotation wrt rotation
      //jacobian(0,5) = 1;
      
      // yaw wrt rotation (numerically)
      TooN::Vector<3> x_axis = TooN::makeVector(1, 0, 0);
	  double delta = 1e-6;
	  for(int i=0; i < 3; ++i)
	  {
		  TooN::Vector<3> v3Increment = TooN::Zeros;
		  v3Increment[i] = delta;
		  
		  TooN::Vector<3> x_axis_plus = (state.rotation * TooN::SO3<>::exp(v3Increment)) * x_axis;
		  double yaw_plus = atan2(x_axis_plus[1], x_axis_plus[0]);
		  
		  TooN::Vector<3> x_axis_minus = (state.rotation * TooN::SO3<>::exp(-1*v3Increment)) * x_axis;
		  double yaw_minus= atan2(x_axis_minus[1], x_axis_minus[0]);
		 
		  jacobian(0,3+i) = util::AngleDiff(yaw_plus, yaw_minus)/(2*delta);
	  }
      
      return jacobian;
    }
    
    const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const 
    {
      return covariance;
    }

    const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const 
    {
	  TooN::Vector<3> x_axis = TooN::makeVector(1, 0, 0);
	  
	  TooN::Vector<3> x_axis_from_state = state.rotation * x_axis;
	  x_axis_from_state[2] = 0; // project onto XY plane
	  
	  TooN::Vector<3> x_axis_from_meas = rotation * x_axis;
	  x_axis_from_meas[2] = 0; // project onto XY plane
	  
	  TooN::Vector<3> cross_product = x_axis_from_state ^ x_axis_from_meas;
	  
	  double angle = asin(TooN::norm(cross_product));
	  
	  if(cross_product[2] < 0)
	  {
	    angle *= -1;
	  }
	  
	  TooN::Vector<M_DIMENSION> innovation = TooN::makeVector(angle);
		
      //TooN::Vector<M_DIMENSION> innovation = TooN::makeVector((state.rotation.inverse() * rotation).ln()[2]);
      
      return innovation;
    }
};

template <class State>
class RelativePose : public MeasurementBase {
public:
    static const int M_DIMENSION = 6;

    TooN::Matrix<M_DIMENSION> covariance;
    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> jacobian;
    TooN::SO3<> rotation;
    TooN::Vector<3> position;

    RelativePose(void){
        covariance = TooN::Identity;
        jacobian = TooN::Zeros;
    }

    TooN::Matrix<M_DIMENSION,State::STATE_DIMENSION> & getMeasurementJacobian( const State & state ){
      
#ifdef CALIBRATION
      // It's ok to do this here because this function will be called first inside filter(...)
      pos_offset = state.pos_c;
      rot_offset = state.rot_c;
#endif
  
      TooN::Matrix<3> rot_r_inv = (state.rot_r.inverse()).get_matrix();
      TooN::Matrix<3> m3Deriv;
      
      // Measurement position wrt position
      jacobian.template slice<0,0,3,3>() = rot_r_inv ;
      
      // Measurement position wrt rotation
      for(unsigned i=0; i < 3; ++i)
        m3Deriv.T()[i] = state.rotation * TooN::SO3<>::generator_field(i, pos_offset);

      jacobian.template slice<0,3,3,3>() = rot_r_inv * m3Deriv;
      
#ifdef FIXED_REL_WORLD
      // Measurement position wrt rot_r
      jacobian.template slice<0,15,3,3>() = TooN::Zeros;
#else
      // Intermediary variable
      TooN::Vector<3> gamma = rot_r_inv * (state.position + state.rotation * pos_offset);
    
      // Measurement position wrt rot_r
      for(unsigned i=0; i < 3; ++i)
        m3Deriv.T()[i] = TooN::SO3<>::generator_field(i, gamma);
        
      jacobian.template slice<0,15,3,3>() = m3Deriv * -1;
#endif
      
#ifdef CALIBRATION
      // Measurement position wrt pos_c
      jacobian.template slice<0,18,3,3>() = rot_r_inv * state.rotation.get_matrix();
#endif
        
      // Measurement rotation wrt rotation
      jacobian.template slice<3,3,3,3>() = (rot_offset.inverse()).get_matrix(); 
      
      
#ifdef FIXED_REL_WORLD
      // Measurement rotation wrt rot_r
      jacobian.template slice<3,15,3,3>() = TooN::Zeros;
#else
      // Measurement rotation wrt rot_r
      jacobian.template slice<3,15,3,3>() = (rot_offset.inverse() * state.rotation.inverse() * state.rot_r).get_matrix() * -1;
#endif
      
#ifdef CALIBRATION
      // Measurement rotation wrt rot_c
      jacobian.template slice<3,21,3,3>() = TooN::Identity;
#endif
      
      return jacobian;
    }

    const TooN::Matrix<M_DIMENSION> getMeasurementCovariance( const State & state ) const {
      
      /*
      TooN::Matrix<6> covarianceWorld = TooN::Zeros;
      covarianceWorld.slice<0,0,3,3>() = state.rot_r.get_matrix() * covariance.slice<0,0,3,3>() * state.rot_r.inverse().get_matrix();
      covarianceWorld.slice<3,3,3,3>() = state.rot_r.get_matrix() * covariance.slice<3,3,3,3>() * state.rot_r.inverse().get_matrix();
      
      return covarianceWorld;
      */
      
      return covariance;
    }

    const TooN::Vector<M_DIMENSION> getInnovation( const State & state ) const {

      TooN::Vector<3> position_from_state = state.rot_r.inverse() * (state.position + state.rotation*pos_offset);
      TooN::SO3<> rotation_from_state = state.rot_r.inverse() * state.rotation * rot_offset;
    
      TooN::Vector<M_DIMENSION> innovation;
      innovation.slice<0,3>() = position - position_from_state;
      innovation.slice<3,3>() = (rotation_from_state.inverse() * rotation).ln();
      
      return innovation;
    }
};

#endif
