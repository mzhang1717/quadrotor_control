#include <state_estimation/AircraftModel.h>
#include <state_estimation/Utility.h>
#include <iomanip>

#include <visualization_msgs/Marker.h>

// ======================= State ==================================

AircraftState::AircraftState()
{
  reset();
}

void AircraftState::reset()
{
  position = TooN::Zeros;
  rotation = TooN::SO3<>();
  velocity = TooN::Zeros;
  accel_bias = TooN::Zeros;
  gyro_bias = TooN::Zeros;
  rot_r = TooN::SO3<>();
  
#ifdef CALIBRATION
  pos_c = TooN::Zeros;
  rot_c = TooN::SO3<>();
#endif

  covariance = TooN::Identity;
}

void AircraftState::printState()
{
  std::cout<<std::fixed<<std::setprecision(6)<<"Position: "<<position<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Rotation: "<<rotation.ln()<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Velocity: "<<velocity<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Accel bias: "<<accel_bias<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Gyro bias: "<<gyro_bias<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Rot_r: "<<rot_r.ln()<<std::endl;
#ifdef CALIBRATION
  std::cout<<std::fixed<<std::setprecision(6)<<"Pos_c: "<<pos_c<<std::endl;
  std::cout<<std::fixed<<std::setprecision(6)<<"Rot_c: "<<rot_c.ln()<<std::endl;
#endif
  
}

void AircraftState::printCov()
{
  std::cout<<"Covariance: "<<std::endl;
  std::cout<<std::fixed<<std::setprecision(3)<<std::setw(6)<<covariance<<std::endl;
}

void AircraftState::setCovariance(double position_cov, double rotation_cov, double velocity_cov)
{
  covariance.slice<0,0,3,3>() = TooN::Identity * position_cov;
  covariance.slice<3,3,3,3>() = TooN::Identity * rotation_cov;
  covariance.slice<6,6,3,3>() = TooN::Identity * velocity_cov;
}
void AircraftState::setCovariance(double sigma)
{
  covariance = TooN::Identity * sigma;
}

// =================== Input =====================================

AircraftInput::AircraftInput()
{
  accel = TooN::Zeros;
  gyro = TooN::Zeros;
  accel_noise = 0;
  gyro_noise = 0;
  accel_bias_noise = 0;
  gyro_bias_noise = 0;
  gravity = TooN::makeVector(0,0,-9.81);
}

// =================== Model =====================================

AircraftModel::AircraftModel()
: nh_priv("~")
{
  jacobian = TooN::Zeros;
  noise = TooN::Zeros;
  
  nh_priv.param<bool>("fix_xy_output", fix_xy_output, false);
  
  accel_pub = nh.advertise<visualization_msgs::Marker>("gravity_accel",1);
}

const TooN::Matrix<AircraftState::STATE_DIMENSION>& AircraftModel::getJacobian(const AircraftState& state, const AircraftInput& input, const double dt) 
{
  const double dt2 = dt*dt;
  const double dt2_2 = dt2/2;
  
  TooN::Matrix<3> m3Deriv;
  for(unsigned i=0; i < 3; ++i)
    m3Deriv.T()[i] = state.rotation * TooN::SO3<>::generator_field(i, (input.accel-state.accel_bias));
  
  // Position wrt position
  jacobian.slice<0,0,3,3>() = TooN::Identity;
  
  // Position wrt rotation
  jacobian.slice<0,3,3,3>() = m3Deriv * dt2_2;
  
  // Position wrt velocity
  jacobian.slice<0,6,3,3>() = TooN::Identity * dt;
  
  // Position wrt accel bias
  jacobian.slice<0,9,3,3>() = state.rotation.get_matrix() * dt2_2 * -1;

  // Rotation wrt rotation
  jacobian.slice<3,3,3,3>() = (TooN::SO3<>::exp((input.gyro-state.gyro_bias)*dt*-1)).get_matrix();
  
  // Rotation wrt gyro bias
  jacobian.slice<3,12,3,3>() = TooN::Identity * dt * -1; 
  
   // Velocity wrt rotation
  jacobian.slice<6,3,3,3>() = m3Deriv * dt;
  
  // Velocity wrt velocity
  jacobian.slice<6,6,3,3>() = TooN::Identity;
  
  // Velocity wrt accel bias
  jacobian.slice<6,9,3,3>() = state.rotation.get_matrix() * dt * -1;
  
  // Accel bias wrt accel bias
  jacobian.slice<9,9,3,3>() = TooN::Identity;
  
  // Gyro bias wrt gyro bias
  jacobian.slice<12,12,3,3>() = TooN::Identity;
  
  // Rot_r wrt rot_r
  jacobian.slice<15,15,3,3>() = TooN::Identity;
  
#ifdef CALIBRATION
  // Pos_c wrt pos_c
  jacobian.slice<18,18,3,3>() = TooN::Identity;
  
  // Rot_v wrt rot_v
  jacobian.slice<21,21,3,3>() = TooN::Identity;
#endif
  
  return jacobian;
}

void AircraftModel::updateState( AircraftState & state, const AircraftInput& input, const double dt )
{
  TooN::Vector<3> accelInWorld = (state.rotation*(input.accel-state.accel_bias) + input.gravity);
  
  // We're going to visualize the acceleration vector in the world frame for debugging
  visualization_msgs::Marker accel_msg;
  accel_msg.header.frame_id = "kf_world";
  accel_msg.header.stamp = ros::Time::now();
  accel_msg.type = accel_msg.ARROW;
  accel_msg.action = accel_msg.ADD;
  accel_msg.ns = "ACCEL";
  accel_msg.id = 0;
  
  accel_msg.points.resize(2);
  accel_msg.points[0].x = state.position[0];
  accel_msg.points[0].y = state.position[1];
  accel_msg.points[0].z = state.position[2];
  
  if(fix_xy_output)
  {
    accel_msg.points[0].x = 0;
    accel_msg.points[0].y = 0;
  }
  
  accel_msg.color.r = 1.0;
  accel_msg.color.g = 1.0;
  accel_msg.color.b = 1.0;
  accel_msg.color.a = 1.0;
  
  accel_msg.points[1].x = accel_msg.points[0].x + accelInWorld[0];
  accel_msg.points[1].y = accel_msg.points[0].y + accelInWorld[1];
  accel_msg.points[1].z = accel_msg.points[0].z + accelInWorld[2];
  
  accel_msg.scale.x = 0.02;  // shaft diameter
  accel_msg.scale.y = 0.05;  // head diameter
  
  accel_pub.publish(accel_msg);
  // End visualization
  
  state.position = state.position + state.velocity*dt + accelInWorld*dt*dt*0.5; 
  state.velocity = state.velocity + accelInWorld*dt;
  state.rotation = state.rotation * TooN::SO3<>::exp((input.gyro-state.gyro_bias) * dt);
  state.accel_bias = state.accel_bias;
  state.gyro_bias = state.gyro_bias;
  state.rot_r = state.rot_r;
#ifdef CALIBRATION
  state.pos_c = state.pos_c;
  state.rot_c = state.rot_c;
#endif

  state.last_accel = input.accel;
  state.last_gyro = input.gyro;
}

const TooN::Matrix<AircraftState::STATE_DIMENSION>& AircraftModel::getNoiseCovariance(const AircraftInput& input, const double dt )
{      
  const double dt2 = dt*dt;
  const double dt2_2 = dt2/2;
  const double dt3 = dt2*dt;
  const double dt3_3 = dt3/3;
  const double dt3_4 = dt3/4;
  //const double dt3_6 = dt3/6;
  const double dt4 = dt3*dt;
  const double dt4_8 = dt4/8;
  const double dt5 = dt4*dt;
  const double dt5_20 = dt5/20;
  
  TooN::Matrix<3> m3Deriv = jacobian.slice<6,3,3,3>() / dt;
  TooN::Matrix<3> m3Exp = jacobian.slice<3,3,3,3>();
  
  const TooN::Matrix<3> I_accel = TooN::Identity * input.accel_noise;
  const TooN::Matrix<3> I_gyro = TooN::Identity * input.gyro_noise;
  const TooN::Matrix<3> I_accel_bias = TooN::Identity * input.accel_bias_noise;
  const TooN::Matrix<3> I_gyro_bias = TooN::Identity * input.gyro_bias_noise;
  
  const TooN::Matrix<3> I_rot_r = TooN::Identity * input.rot_r_noise;
#ifdef CALIBRATION
  const TooN::Matrix<3> I_pos_c = TooN::Identity * input.pos_c_noise;
  const TooN::Matrix<3> I_rot_c = TooN::Identity * input.rot_c_noise;
#endif
  
  // Position-position
  noise.slice<0,0,3,3>() = I_accel*dt3_3 + I_gyro*dt5_20*m3Deriv*m3Deriv.T() + I_accel_bias*dt5_20;
  
  // Position-rotation
  noise.slice<0,3,3,3>() = I_gyro * m3Deriv * dt3_4 * m3Exp.T();
  
  // Position-velocity
  noise.slice<0,6,3,3>() = I_accel*dt2_2 + I_gyro*dt4_8*m3Deriv*m3Deriv.T() + I_accel_bias*dt4_8;
  
  // Position-accel_bias
  noise.slice<0,9,3,3>() = I_accel_bias*jacobian.slice<0,9,3,3>()*dt*(1/3);
  
  // Rotation-position
  noise.slice<3,0,3,3>()  = noise.slice<0,3,3,3>().T();
  
  // Rotation-rotation
  noise.slice<3,3,3,3>() = I_gyro*dt + I_gyro_bias*dt3_3;
  
  // Rotation-velocity
  noise.slice<3,6,3,3>() = I_gyro*m3Exp*dt2_2*m3Deriv.T();
  
  // Rotation-gyro_bias
  noise.slice<3,12,3,3>() = I_gyro_bias*dt2_2*-1;
  
  // Velocity-position
  noise.slice<6,0,3,3>() = noise.slice<0,6,3,3>().T();
  
  // Velocity-rotation
  noise.slice<6,3,3,3>() = noise.slice<3,6,3,3>().T();
  
  // Velocity-velocity
  noise.slice<6,6,3,3>() = I_accel*dt + I_gyro*dt3_3*m3Deriv*m3Deriv.T() + I_gyro_bias*dt3_3;
  
  // Velocity-accel_bias
  noise.slice<6,9,3,3>() = I_accel_bias*jacobian.slice<0,9,3,3>();
  
  // Accel_bias-position
  noise.slice<9,0,3,3>() = noise.slice<0,9,3,3>().T();
  
  // Accel_bias-velocity
  noise.slice<9,6,3,3>() = noise.slice<6,9,3,3>().T();
  
  // Accel_bias-accel_bias
  noise.slice<9,9,3,3>() = I_accel_bias*dt;
  
  // Gyro_bias-rotation
  noise.slice<12,3,3,3>() = noise.slice<3,12,3,3>().T();
  
  // Gyro_bias-gyro_bias
  noise.slice<12,12,3,3>() = I_gyro_bias*dt;
  
  // Rot_r-Rot_r
  noise.slice<15,15,3,3>() = I_rot_r*dt;
  
#ifdef CALIBRATION
  // pos_c-pos_c
  noise.slice<18,18,3,3>() = I_pos_c*dt;
  
  // rot_c-rot_c
  noise.slice<21,21,3,3>() = I_rot_c*dt;
#endif
  
  return noise;
}

void AircraftModel::updateFromMeasurement( AircraftState & state, const TooN::Vector<AircraftState::STATE_DIMENSION> & innovation )
{
  state.position = state.position + innovation.slice<0,3>();
  state.rotation = state.rotation * TooN::SO3<>::exp(innovation.slice<3,3>());
  state.velocity = state.velocity + innovation.slice<6,3>();
  state.accel_bias = state.accel_bias +  innovation.slice<9,3>();
  state.gyro_bias = state.gyro_bias +  innovation.slice<12,3>();
  
#ifdef FIXED_REL_WORLD
  state.rot_r = state.rot_r;  // just to make it explicit
  //std::cout<<"Would have updated rot_r with: "<<innovation.slice<15,3>()<<std::endl;
#else
  state.rot_r = state.rot_r * TooN::SO3<>::exp(innovation.slice<15,3>());
#endif
  
#ifdef CALIBRATION
  state.pos_c = state.pos_c + innovation.slice<18,3>();
  state.rot_c = state.rot_c * TooN::SO3<>::exp(innovation.slice<21,3>());
#endif

}

