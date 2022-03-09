#include <ros/ros.h>
#include <state_estimation/ExtendedKalmanFilter.h>
#include <state_estimation/AircraftStateMsg.h>
#include <state_estimation/AircraftMeasurements.h>
#include <state_estimation/Utility.h>
#include <state_estimation/YAMLParser.h>

ExtendedKalmanFilter::ExtendedKalmanFilter() 
: FilterBase()
{
  nh_private.param<double>("rot_r_noise", rot_r_noise, 0.0001);
#ifdef CALIBRATION  
  nh_private.param<double>("pos_c_noise", pos_c_noise, 0.0001);
  nh_private.param<double>("rot_c_noise", rot_c_noise, 0.0001);
#endif

  int buffer_size;
  nh_private.param<int>("buffer_size", buffer_size, 50);
  
  cbStates.set_capacity(buffer_size);
  cbMeasurements.set_capacity(buffer_size);
  
  std::string offset_source;
  this->nh_private.param<std::string>("sensor_offset_source", offset_source, "");
  
  if(offset_source != "tf" && offset_source != "file")
  {
    ROS_FATAL("Parameter sensor_offset_source needs to be either 'tf' or 'file'");
    ros::shutdown();
    return;
  }
  
  if(offset_source == "tf")
  {
    ros::Time nowtime;
    // Wait until we have a valid timestamp (ie not zero)
    do
    {
      nowtime = ros::Time::now();
      ros::Rate(10).sleep();
    }
    while(nowtime == ros::Time(0));
    
    ROS_INFO("Gathering TF in buffer");
    ros::Duration(1.0).sleep();
    
    std::string imu_sensor_name;
    nh_private.param<std::string>("imu_sensor_name", imu_sensor_name, "");
    if(imu_sensor_name == "")
    {
      ROS_WARN("Parameter imu_sensor_name not found, assuming 'imu'");
    }
    
    // Get calibrated pose of relative pose sensor
    std::string rel_pose_sensor_name;
    nh_private.param<std::string>("relative_pose_sensor_name", rel_pose_sensor_name, "");
    if(rel_pose_sensor_name == "")
    {
      ROS_WARN("Parameter relative_pose_sensor_name not found, assuming identity transform for offset");
    }
    else
    {
      tf::StampedTransform transform;
      try{
        ROS_INFO_STREAM("Waiting for transform from "<<imu_sensor_name<<" to "<<rel_pose_sensor_name);
        std::string error;
        nowtime = ros::Time::now();
        bool success = this->tf_listener.waitForTransform(imu_sensor_name, rel_pose_sensor_name, nowtime, ros::Duration(1), ros::Duration(0.01), &error);
        if(!success)
        {
          ROS_ERROR_STREAM("Got error: "<<error);
        }
        this->tf_listener.lookupTransform(imu_sensor_name, rel_pose_sensor_name, nowtime, transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      
      this->rel_pose_meas_template.pos_offset[0] = transform.getOrigin().x();
      this->rel_pose_meas_template.pos_offset[1] = transform.getOrigin().y();
      this->rel_pose_meas_template.pos_offset[2] = transform.getOrigin().z();
      this->rel_pose_meas_template.rot_offset = util::QuatToSO3(transform.getRotation());
    }
    
    // Get calibrated pose of body velocity sensor
    std::string body_vel_sensor_name;
    nh_private.param<std::string>("body_velocity_sensor_name", body_vel_sensor_name, "");
    if(body_vel_sensor_name == "")
    {
      ROS_WARN("Parameter body_velocity_sensor_name not found, assuming identity transform for offset");
    }
    else
    {
      tf::StampedTransform transform;
      try{
        ROS_INFO_STREAM("Waiting for transform from "<<imu_sensor_name<<" to "<<body_vel_sensor_name);
        std::string error;
        nowtime = ros::Time::now();
        bool success = this->tf_listener.waitForTransform(imu_sensor_name, body_vel_sensor_name, nowtime, ros::Duration(1), ros::Duration(0.01), &error);
        if(!success)
        {
          ROS_ERROR_STREAM("Got error: "<<error);
        }
        this->tf_listener.lookupTransform(imu_sensor_name, body_vel_sensor_name, nowtime, transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      
      this->body_vel_meas_template.pos_offset[0] = transform.getOrigin().x();
      this->body_vel_meas_template.pos_offset[1] = transform.getOrigin().y();
      this->body_vel_meas_template.pos_offset[2] = transform.getOrigin().z();
      this->body_vel_meas_template.rot_offset = util::QuatToSO3(transform.getRotation());
    }
    
    // Get calibrated pose of ground distance sensor
    std::string ground_distance_sensor_name;
    nh_private.param<std::string>("ground_distance_sensor_name", ground_distance_sensor_name, "");
    if(ground_distance_sensor_name == "")
    {
      ROS_WARN("Parameter ground_distance_sensor_name not found, assuming identity transform for offset");
    }
    else
    {
      tf::StampedTransform transform;
      try{
        ROS_INFO_STREAM("Waiting for transform from "<<imu_sensor_name<<" to "<<ground_distance_sensor_name);
        std::string error;
        nowtime = ros::Time::now();
        bool success = this->tf_listener.waitForTransform(imu_sensor_name, ground_distance_sensor_name, nowtime, ros::Duration(1), ros::Duration(0.01), &error);
        if(!success)
        {
          ROS_ERROR_STREAM("Got error: "<<error);
        }
        this->tf_listener.lookupTransform(imu_sensor_name, ground_distance_sensor_name, nowtime, transform);
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
      }
      
      this->ground_distance_meas_template.pos_offset[0] = transform.getOrigin().x();
      this->ground_distance_meas_template.pos_offset[1] = transform.getOrigin().y();
      this->ground_distance_meas_template.pos_offset[2] = transform.getOrigin().z();
      this->ground_distance_meas_template.rot_offset = util::QuatToSO3(transform.getRotation());
    }
  }
  else // file source instead of tf source
  {
    // Get calibrated pose of relative pose sensor
    std::string rel_pose_sensor_file;
    nh_private.param<std::string>("relative_pose_sensor_file", rel_pose_sensor_file, "");
    if(rel_pose_sensor_file == "")
    {
      ROS_WARN("Parameter relative_pose_sensor_file not found, assuming identity transform for offset");
    }
    else
    {
      yaml::loadPose(rel_pose_sensor_file, this->rel_pose_meas_template.pos_offset, this->rel_pose_meas_template.rot_offset);
      std::cout<<"Relative pose sensor offsets: "<<this->rel_pose_meas_template.pos_offset<<std::endl;
      std::cout<<this->rel_pose_meas_template.rot_offset<<std::endl;
    }
    
    // Get calibrated pose of body velocity sensor
    std::string body_vel_sensor_file;
    nh_private.param<std::string>("body_velocity_sensor_file", body_vel_sensor_file, "");
    if(body_vel_sensor_file == "")
    {
      ROS_WARN("Parameter body_velocity_sensor_file not found, assuming identity transform for offset");
    }
    else
    {
      yaml::loadPose(body_vel_sensor_file, this->body_vel_meas_template.pos_offset, this->body_vel_meas_template.rot_offset);
      std::cout<<"Body velocity sensor offsets: "<<this->body_vel_meas_template.pos_offset<<std::endl;
      std::cout<<this->body_vel_meas_template.rot_offset<<std::endl;
    }
    
    // Get calibrated pose of ground_distance sensor
    std::string ground_distance_sensor_file;
    nh_private.param<std::string>("ground_distance_sensor_file", ground_distance_sensor_file, "");
    if(ground_distance_sensor_file == "")
    {
      ROS_WARN("Parameter ground_distance_sensor_file not found, assuming identity transform for offset");
    }
    else
    {
      yaml::loadPose(ground_distance_sensor_file, this->ground_distance_meas_template.pos_offset, this->ground_distance_meas_template.rot_offset);
      std::cout<<"Ground distance sensor offsets: "<<this->ground_distance_meas_template.pos_offset<<std::endl;
      std::cout<<this->ground_distance_meas_template.rot_offset<<std::endl;
    }
    
#ifdef FIXED_REL_WORLD
    std::string rel_world_file;
    nh_private.param<std::string>("rel_world_file", rel_world_file, "");
    if(rel_world_file == "")
    {
      ROS_FATAL("Parameter rel_world_file not found, if you want a fixed relative world you have to supply this file!");
      ros::shutdown();
      return;
    }
    else
    {
      TooN::Vector<3> v3TempPosition;
      yaml::loadPose(rel_world_file, v3TempPosition, this->filter.state.rot_r);
      std::cout<<"Fixed relative world: "<<v3TempPosition<<std::endl;
      std::cout<<this->filter.state.rot_r<<std::endl;
    }
#endif
  }
  
  // The rotation measurement is assumed to happen at the IMU location.
  // These lines aren't necessary since they are the defaults for the 
  // MeasurementBase class, but I'm making it explicit here.
  this->rot_meas_template.pos_offset = TooN::Zeros;
  this->rot_meas_template.rot_offset = TooN::SO3<>();
  this->yaw_meas_template.pos_offset = TooN::Zeros;
  this->yaw_meas_template.rot_offset = TooN::SO3<>();
  
  this->filter.state.setCovariance(1);
  
  std::cout<<"Start State: "<<std::endl;
  this->filter.state.printState();
  std::cout<<"Start Cov: "<<std::endl;
  this->filter.state.printCov();
  
}

void ExtendedKalmanFilter::imuCallbackInternal(TooN::Vector<3> accel_meas, double accel_cov, double accel_bias_cov, TooN::Vector<3> gyro_meas, double gyro_cov, double gyro_bias_cov, double dt, TooN::Vector<3> gravity, ros::Time stamp)
{
  /*
  // IMU is NEVER allowed to come out of order or late!
  if(!this->cbStates.empty())
  {
    if(stamp < this->cbStates.back().stamp)
    {
      ROS_ERROR_STREAM("Stamp: "<<stamp<<"  back stamp: "<<this->cbStates.back().stamp);
    }
  }
  
  ROS_ASSERT(this->cbStates.empty() || stamp > this->cbStates.back().stamp);
  */
  
  AircraftInput input;

  input.accel = accel_meas; // body frame
  input.gyro = gyro_meas; // body frame
  input.accel_noise = accel_cov;
  input.gyro_noise = gyro_cov;
  input.accel_bias_noise = accel_bias_cov;
  input.gyro_bias_noise = gyro_bias_cov;
  input.rot_r_noise = rot_r_noise;
#ifdef CALIBRATION
  input.pos_c_noise = pos_c_noise;
  input.rot_c_noise = rot_c_noise;
#endif
  input.gravity = gravity;
  input.dt = dt;
  input.stamp = stamp;
  
  int startIdx = saveMeas(input, stamp);
  ROS_ASSERT(startIdx >= 0); // if startIdx < 0, means IMU data came so late that we are off the end of the buffer, very bad
  
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}

void ExtendedKalmanFilter::bodyXYVelCallbackInternal(TooN::Vector<2> vel, TooN::Matrix<2> cov, ros::Time stamp)
{
  if(this->cbStates.empty())
  {
    ROS_WARN("In bodyXYVelCallbackInternal but we don't have any states in the buffer yet!");
    return;
  }	
	
  BodyXYVelocity<AircraftState> body_vel_meas = this->body_vel_meas_template;
  
  body_vel_meas.velocity = vel;
  body_vel_meas.covariance = cov;
  body_vel_meas.stamp = stamp;
  
  int startIdx = saveMeas(body_vel_meas, stamp);
  if(startIdx < 0)
    return;
    
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}


void ExtendedKalmanFilter::groundDistanceCallbackInternal(double distance, double cov, ros::Time stamp)
{
  if(this->cbStates.empty())
  {
    ROS_WARN("In groundDistanceCallbackInternal but we don't have any states in the buffer yet!");
    return;
  }
  
  GroundPlaneDistance<AircraftState> ground_distance_meas = this->ground_distance_meas_template;
  
  ground_distance_meas.distance = distance;
  ground_distance_meas.covariance = TooN::Identity * cov;
  ground_distance_meas.stamp = stamp;
  
  int startIdx = saveMeas(ground_distance_meas, stamp);
  if(startIdx < 0)
    return;
    
  static bool first = true;
  if(first)
  {
    first = false;
    
    this->filter.state.position[2] = -1*( (this->filter.state.rotation * ground_distance_meas.pos_offset)[2] + (this->filter.state.rotation * ground_distance_meas.rot_offset * TooN::makeVector(0,0,ground_distance_meas.distance))[2] );

    // Put the newly updated state back in the buffer
    this->cbStates.back() = this->filter.state;
    
    std::cout<<"First ground distance meas: "<<std::endl<< ground_distance_meas.distance <<std::endl;
    std::cout<<"First ground distance meas pos offset: "<<std::endl<< ground_distance_meas.pos_offset <<std::endl;
    std::cout<<"First ground distance meas rot offset: "<<std::endl<< ground_distance_meas.rot_offset <<std::endl;
    std::cout<<"First state height: "<<this->filter.state.position[2]<<std::endl;
  }
    
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}

void ExtendedKalmanFilter::rotCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp)
{
  // If we're working with a pre-calibrated relative world, don't even bother using these measurements
  // since it'll just cause convergence issues, and frankly we don't care about magnetic north in this case
#ifdef FIXED_REL_WORLD
  return;
#endif
  
  WorldRotation<AircraftState> rot_meas = this->rot_meas_template;
  
  rot_meas.rotation = rot;
  rot_meas.covariance = cov;
  rot_meas.stamp = stamp;
  
  int startIdx = saveMeas(rot_meas, stamp);
  if(startIdx < 0)
    return;
    
  // On receipt of the first rotation measurement, we want to set the current state of the 
  // filter's rotation to be equal to the measurement
  // Should not be necessary to do this, but it helps speed up convergence of other states
  
  static bool first = true;
  if(first)
  {
    first = false;
    
    this->filter.state.rotation = rot_meas.rotation * rot_meas.rot_offset.inverse();

    // Put the newly updated state back in the buffer
    this->cbStates.back() = this->filter.state;
    
    std::cout<<"First meas rot: "<<std::endl<< rot_meas.rotation <<std::endl;
    std::cout<<"First meas rot offset: "<<std::endl<< rot_meas.rot_offset <<std::endl;
    std::cout<<"First state rot: "<<this->filter.state.rotation<<std::endl;
  }
  
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}
    
void ExtendedKalmanFilter::yawCallbackInternal(TooN::SO3<> rot, TooN::Matrix<3> cov, ros::Time stamp)
{
  // If we're working with a pre-calibrated relative world, don't even bother using these measurements
  // since it'll just cause convergence issues, and frankly we don't care about magnetic north in this case
#ifdef FIXED_REL_WORLD
  return;
#endif
  
  WorldYaw<AircraftState> yaw_meas = this->yaw_meas_template;
  
  yaw_meas.rotation = rot;
  yaw_meas.covariance = cov.slice<2,2,1,1>();
  yaw_meas.stamp = stamp;
  
  int startIdx = saveMeas(yaw_meas, stamp);
  if(startIdx < 0)
    return;
    
  // On receipt of the first rotation measurement, we want to set the current state of the 
  // filter's rotation to be equal to the measurement
  // Should not be necessary to do this, but it helps speed up convergence of other states
  // Use the full rotation measurement, not just the yaw
  
  static bool first = true;
  if(first)
  {
    first = false;
    
    this->filter.state.rotation = yaw_meas.rotation * yaw_meas.rot_offset.inverse();

    // Put the newly updated state back in the buffer
    this->cbStates.back() = this->filter.state;
    
    std::cout<<"First meas rot: "<<std::endl<< yaw_meas.rotation <<std::endl;
    std::cout<<"First meas rot offset: "<<std::endl<< yaw_meas.rot_offset <<std::endl;
    std::cout<<"First state rot: "<<this->filter.state.rotation<<std::endl;
  }
  
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}

int ExtendedKalmanFilter::saveMeas(boost::any meas, ros::Time stamp)
{
  // Two cases: cbStates is empty, meaning we haven't done anything yet, or cbStates
  // has something in it, meaning we need to apply the usual rules
  
  if(this->cbStates.empty())
  {
    ROS_ASSERT(this->cbMeasurements.empty());
    this->cbMeasurements.push_back(meas);
    return 0;
  }
  
  // Check if measurement time isn't before our first state in buffer
  // If it is, then measurement is too far in the past to be able to compensate
  // Set a bigger buffer or reduce the measurement time delay!
  if(stamp < this->cbStates.front().stamp)
  {
    ROS_WARN_STREAM("Measurement time of "<<stamp<<" too far in the past, can only compensate up to "<<this->cbStates.front().stamp);
    return -1;
  } 
  
  // Any states past stamp are incorrect, pop them all off
  // We already made sure that stamp is later than the oldest saved timestamp, so 
  // this loop will terminate
  while(stamp < this->cbStates.back().stamp)
    this->cbStates.pop_back();
  
  unsigned insertIdx = this->cbStates.size();
  boost::circular_buffer<boost::any>::iterator insertIter;
  
  if(insertIdx == this->cbStates.capacity()) // If we're alread at capacity, insert at end
    insertIter = this->cbMeasurements.end();
  else  // Otherwise insert at the index that corresponds to the next state we will compute
    insertIter = this->cbMeasurements.begin() + insertIdx;
  
  insertIter = this->cbMeasurements.insert(insertIter, meas);
  
  // Make sure that the measurement was inserted (insert(...) returns
  // an iterator to begin() if insertion didn't do anything because
  // we were trying to insert at the beginning of a full circular buffer)
  // This is only for debugging, as the check to make sure that the stamp
  // is newer than the oldest state timestamp should take care of any issues
  // with trying to insert a measurement into the beginning of a full buffer
  if(this->cbMeasurements.full())
  {
    ROS_ASSERT(insertIter != this->cbMeasurements.begin());
  }
    
  int startIdx = std::distance(this->cbMeasurements.begin(), insertIter);
  //std::cout<<"Inserted relative pose measurement at "<<startIdx<<std::endl;
  
  // Set the state of our filter from the saved states
  this->filter.state = this->cbStates.back();
  
  return startIdx;
}

void ExtendedKalmanFilter::relPoseCallbackInternal(TooN::Vector<3> rel_pos, TooN::SO3<> rel_rot, TooN::Matrix<6> rel_cov, ros::Time stamp)
{
  // This will make sure that the initial imu and mangetometer data have been integrated into the filter
  // before we start doing stuff
  if(this->cbStates.empty())
  {
    ROS_WARN("In relPoseCallbackInternal but we don't have any states in the buffer yet!");
    return;
  }
  
  RelativePose<AircraftState> rel_pose_meas = this->rel_pose_meas_template;
  
  rel_pose_meas.rotation = rel_rot;
  rel_pose_meas.position = rel_pos;
  rel_pose_meas.covariance = rel_cov;
  rel_pose_meas.stamp = stamp;
  
  int startIdx = saveMeas(rel_pose_meas, stamp);
  if(startIdx < 0)
    return;
    
  // On receipt of the first relative pose message, we want to set the current state of the 
  // filter 
  
  static bool first = true;
  if(first)
  {
    first = false;
    
#ifdef FIXED_REL_WORLD
    // rot_r was already loaded in from rel_world_file, so don't change it
    // but we should update state.rotation to take the known rot_r into account
    this->filter.state.rotation = this->filter.state.rot_r * rel_pose_meas.rotation * rel_pose_meas.rot_offset.inverse();
#else
    // otherwise we should compute a good initial value of rot_r, keeping in mind that state.rotation will
    // have been set by the magnetometer rotation in rotCallbackInternal before we ever get here
    this->filter.state.rot_r = this->filter.state.rotation * rel_pose_meas.rot_offset * rel_pose_meas.rotation.inverse();
#endif

    // position needs to be upated regardless of FIXED_REL_WORLD
    this->filter.state.position = this->filter.state.rot_r * rel_pose_meas.position - this->filter.state.rotation * rel_pose_meas.pos_offset;
    
#ifdef CALIBRATION
    this->filter.state.pos_c = rel_pose_meas.pos_offset;
    this->filter.state.rot_c = rel_pose_meas.rot_offset;
#endif

    // Put the newly updated state back in the buffer
    this->cbStates.back() = this->filter.state;
    
    std::cout<<"First rel pos: "<<std::endl<< rel_pose_meas.position <<std::endl;
    std::cout<<"First rel pos offset: "<<std::endl<< rel_pose_meas.pos_offset <<std::endl;
    std::cout<<"First state pos: "<<this->filter.state.position<<std::endl;
    
    std::cout<<"First rel rot: "<<std::endl<< rel_pose_meas.rotation <<std::endl;
    std::cout<<"First rel rot offset: "<<std::endl<< rel_pose_meas.rot_offset <<std::endl;
    std::cout<<"First state rot: "<<this->filter.state.rotation<<std::endl;
  }
  
  // This will go through all measurements starting at startIdx and predict/filter as necessary
  filterFromBuffer(startIdx);
}

void ExtendedKalmanFilter::filterFromBuffer(int startIdx)
{
  boost::circular_buffer<boost::any>::iterator startIter = this->cbMeasurements.begin() + startIdx;
  
  //std::cout<<"Filtering from buffer, contains "<<this->cbMeasurements.size()<<" measurements"<<std::endl;
  //std::cout<<"Starting at "<< std::distance(this->cbMeasurements.begin(), startIter)<<std::endl;
  //std::cout<<" which is "<<std::distance(startIter, this->cbMeasurements.end()) <<" from the end"<<std::endl;
  for(boost::circular_buffer<boost::any>::iterator it = startIter; it != this->cbMeasurements.end(); ++it)
  {
    ros::Time stamp;
    
    if(it->type() == typeid(AircraftInput))  // imu data
    {
      AircraftInput input = boost::any_cast<AircraftInput>(*it);
      stamp = input.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.predict(input, input.dt);
    }
    else if(it->type() == typeid(WorldRotation<AircraftState>))  // magnetometer rotation
    {
      WorldRotation<AircraftState> rot_meas = boost::any_cast<WorldRotation<AircraftState> >(*it);
      stamp = rot_meas.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.filter(rot_meas);
    }
    else if(it->type() == typeid(WorldYaw<AircraftState>))  // magnetometer yaw only
    {
      WorldYaw<AircraftState> yaw_meas = boost::any_cast<WorldYaw<AircraftState> >(*it);
      stamp = yaw_meas.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.filter(yaw_meas);
    }
    else if(it->type() == typeid(BodyXYVelocity<AircraftState>))  // body velocity
    {
      BodyXYVelocity<AircraftState> body_vel_meas = boost::any_cast<BodyXYVelocity<AircraftState> >(*it);
      stamp = body_vel_meas.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.filter(body_vel_meas);
    }
    else if(it->type() == typeid(GroundPlaneDistance<AircraftState>))  // ground plane distance
    {
      GroundPlaneDistance<AircraftState> ground_distance_meas = boost::any_cast<GroundPlaneDistance<AircraftState> >(*it);
      stamp = ground_distance_meas.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.filter(ground_distance_meas);
    }
    else if(it->type() == typeid(RelativePose<AircraftState>)) // vision/vicon data
    {
      //std::cout<<"Processing relative pose"<<std::endl;
      RelativePose<AircraftState> rel_pose_meas = boost::any_cast<RelativePose<AircraftState> >(*it);
      stamp = rel_pose_meas.stamp;
      ROS_ASSERT(stamp >= this->filter.state.stamp);
      this->filter.filter(rel_pose_meas);
    }
    else
    {
      ROS_ERROR("Unknown measurement in buffer!!!");
    }
    
    // Update the stamp on the state
    this->filter.state.stamp = stamp;
    
    // Save it in the buffer
    this->cbStates.push_back(this->filter.state);
  }
  
  // After all the measurements have been processed, the state and measurement buffers should
  // again be the same size
  ROS_ASSERT(this->cbStates.size() == this->cbMeasurements.size());
  
}
    
void ExtendedKalmanFilter::fillStateMessage(state_estimation::AircraftStateMsg &state_msg)
{
  state_msg.position.x = this->filter.state.position[0];
  state_msg.position.y = this->filter.state.position[1];
  state_msg.position.z = this->filter.state.position[2];
  
  tf::Quaternion q = util::SO3ToQuat(this->filter.state.rotation);
  
  state_msg.rotation.x = q.x();
  state_msg.rotation.y = q.y();
  state_msg.rotation.z = q.z();
  state_msg.rotation.w = q.w();
  
  state_msg.velocity.x = this->filter.state.velocity[0];
  state_msg.velocity.y = this->filter.state.velocity[1];
  state_msg.velocity.z = this->filter.state.velocity[2];
  
  state_msg.accel_bias.x = this->filter.state.accel_bias[0];
  state_msg.accel_bias.y = this->filter.state.accel_bias[1];
  state_msg.accel_bias.z = this->filter.state.accel_bias[2];
  
  state_msg.gyro_bias.x = this->filter.state.gyro_bias[0];
  state_msg.gyro_bias.y = this->filter.state.gyro_bias[1];
  state_msg.gyro_bias.z = this->filter.state.gyro_bias[2];
  
  tf::Quaternion q_r = util::SO3ToQuat(this->filter.state.rot_r);
    
  state_msg.rot_r.x = q_r.x();
  state_msg.rot_r.y = q_r.y();
  state_msg.rot_r.z = q_r.z();
  state_msg.rot_r.w = q_r.w();
  
#ifdef CALIBRATION
  // Take pos_c and rot_c from filter state
  
  state_msg.pos_c.x = this->filter.state.pos_c[0];
  state_msg.pos_c.y = this->filter.state.pos_c[1];
  state_msg.pos_c.z = this->filter.state.pos_c[2];
  
  tf::Quaternion q_c = util::SO3ToQuat(this->filter.state.rot_c);
    
  state_msg.rot_c.x = q_c.x();
  state_msg.rot_c.y = q_c.y();
  state_msg.rot_c.z = q_c.z();
  state_msg.rot_c.w = q_c.w();
  
#else
  // Take pos_c and rot_c from the pos and rot offsets loaded into the sensor measurement structs
  // (from an external calibration)
  
  state_msg.pos_c.x = this->rel_pose_meas_template.pos_offset[0];
  state_msg.pos_c.y = this->rel_pose_meas_template.pos_offset[1];
  state_msg.pos_c.z = this->rel_pose_meas_template.pos_offset[2];
  
  tf::Quaternion q_c = util::SO3ToQuat(this->rel_pose_meas_template.rot_offset);
    
  state_msg.rot_c.x = q_c.x();
  state_msg.rot_c.y = q_c.y();
  state_msg.rot_c.z = q_c.z();
  state_msg.rot_c.w = q_c.w();

#endif
  
  int numElems = this->filter.state.STATE_DIMENSION;
  state_msg.covariance.resize(numElems*numElems, 0);
  for(int i=0; i < numElems; ++i)
  {
    for(int j=0; j < numElems; ++j)
    {
      state_msg.covariance[i*numElems + j] = this->filter.state.covariance(i,j);
    }
  }
}
