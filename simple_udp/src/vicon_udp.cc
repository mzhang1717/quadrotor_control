/**
 * Simple UDP Receiver Program.
 * Parameters:
 *    1) Number of doubles in a packet.
 *    2) Local Network Port used to receive the packet.
 * 
 * Outputs: 
 *    1) Data:    Array of doubles.
 *  
 */


/* -------------------------------------------------------------------------
 * Include System Headers.
 * ------------------------------------------------------------------------- */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <boost/circular_buffer.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <fcntl.h>
#include <fstream>
#include <queue>

#ifdef _WIN32
   #include <windows.h>
   #include <time.h>
#else 
#ifdef __QNX__
   #include <unix.h>
   #include <errno.h>
   #include <sys/ioctl.h>
   #include <unistd.h>
#endif
   #include <netinet/in.h>
   #include <netdb.h>
   #include <sys/socket.h>
   #include <sys/time.h>
   #include <sys/select.h>
#endif /* _WIN32 */

/* -------------------------------------------------------------------------
 * Constants.
 * ------------------------------------------------------------------------- */

#define BUFFER_SIZE    64

#define SFCT_NUM_ARGS			(3)
#define SFCT_ARG_BUFFER_LEN		8
#define SFCT_ARG_PORT_ID		16001
#define SFCT_ARG_SAMPLE_TIME		0.01

#define TIMING_BUFFER_SIZE 11  // should be odd to get median value
#define TIMING_BUFFER_MEDIAN_INDEX 5

//#define _DEBUG_


/* =========================================================================
 * TYPEDEF
 * ========================================================================= */

typedef union _Union_
{
   uint32_t aulvalue[2];
   double         dvalue;
} Union;


int main(int argc, char** argv) {
  
  ros::init(argc, argv, "vicon_udp");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher pubCov = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("pose_cov", 1);
  tf::TransformBroadcaster broadcaster;
  
  double posNoise, rotNoise, delayFactor, maxAllowedDelay;
  int publishLag;
  double publishRate;
  
  nh_priv.param<double>("position_noise", posNoise, 0.0);
  nh_priv.param<double>("rotation_noise", rotNoise, 0.0);
  nh_priv.param<double>("delay_factor", delayFactor, 10.0);
  nh_priv.param<double>("max_allowed_delay", maxAllowedDelay, 0.02);
  nh_priv.param<int>("publish_lag", publishLag, 0);
  nh_priv.param<double>("publish_rate", publishRate, 120);  // vicon max rate is 120Hz
  
  if(publishRate > 120)
  {
    std::cout<<"Got publish rate of "<<publishRate<<" Hz, maximum is 120 Hz, clamping"<<std::endl;
    publishRate = 120;
  }
  
  std::cout<<"Applying publish lag of: "<<publishLag<<std::endl;
  std::cout<<"Publishing at a minimum rate of "<<publishRate<<" Hz"<<std::endl;
  
  ros::Duration publishPeriod(1/publishRate);
  ros::Time lastPublishTime;
  
  double* output_data;
  
  /* Simulink work elements. */
  struct sockaddr_in        their_address;
  socklen_t iaddr_len;
  int inumber_of_bytes_receive;
  Union buffer;
  unsigned long ulbuffer;
  struct timeval tv = {0, 0};  /* s and us */
  fd_set readfds;

  struct sockaddr_in listener_address;
  int uisocket;
  double *pdpacket;
  
  boost::circular_buffer<double> timing_buffer(TIMING_BUFFER_SIZE);
  std::vector<double> buffer_vector(TIMING_BUFFER_SIZE);
  bool delayAcceptable;

#ifdef _WIN32
  WSADATA wsaData;      /* Used to initialise the ws2_32.dll link. */
  int ireturn_value;
#endif

#ifdef _WIN32
  if ((ireturn_value = WSAStartup(MAKEWORD(1, 1),&wsaData)) != 0) {
    printf ("WSAStartup failed");
    return 1;
  }
#endif /* _WIN32 */
   
//   uisocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  uisocket = socket(AF_INET, SOCK_DGRAM, 0);
  if (uisocket < 0){
    printf ("Unable to create the UDP socket");
    return 2;
  }
   
  /* Fill in the listener address structure. */
  memset(&listener_address, 0, sizeof(listener_address));   
  listener_address.sin_family = AF_INET;
  listener_address.sin_port = htons(SFCT_ARG_PORT_ID);
  listener_address.sin_addr.s_addr = INADDR_ANY;
   
  if (bind(uisocket, (struct sockaddr *) &listener_address, sizeof(struct sockaddr)) < 0) {
    printf ("Unable to bind the UDP socket");
    return 3;
  }

  /* Allocate memory for the packet buffer. */
  pdpacket = (double *) malloc(sizeof(double) * SFCT_ARG_BUFFER_LEN);
  memset(pdpacket, 0, sizeof(double) * SFCT_ARG_BUFFER_LEN);
  
  output_data = (double *) malloc(sizeof(double) * SFCT_ARG_BUFFER_LEN);
  memset(output_data, 0, sizeof(double) * SFCT_ARG_BUFFER_LEN);
  
  geometry_msgs::PoseStamped poseMsg;
  geometry_msgs::PoseWithCovarianceStamped poseCovMsg;
  tf::Transform transform;
  
  std::queue<geometry_msgs::PoseStamped> poseQueue;
  std::queue<geometry_msgs::PoseWithCovarianceStamped> poseCovQueue;
  std::queue<tf::Transform> transformQueue;
  
  ROS_INFO_STREAM("Starting to listen on port "<<SFCT_ARG_PORT_ID);
  ROS_INFO_STREAM("Accepting data packets with max delay of "<<delayFactor<<" times median delay (window size: "<<TIMING_BUFFER_SIZE<<")");
  ROS_INFO_STREAM("Make sure you are connected to the correct network and that the Vicon streamer is sending TIME instead of VICON SEQUENCE numbers");
  
  //std::cout.precision(10);
  
  //std::ofstream debug_out("vicon_debug.csv");
  
  lastPublishTime = ros::Time::now();
  while(ros::ok()) {
    
    /* Receive the data. */
    iaddr_len = sizeof(struct sockaddr);
    
    FD_ZERO(&readfds);
    FD_SET(uisocket, &readfds);
    
    select(uisocket + 1, &readfds, NULL, NULL, &tv);
    
    if (FD_ISSET(uisocket, &readfds))
    {
      inumber_of_bytes_receive = 
         recvfrom(uisocket,
		          (char *) pdpacket,
		          SFCT_ARG_BUFFER_LEN * sizeof(double),
                  0,//MSG_WAITALL,
                  (struct sockaddr *)&their_address,
                  &iaddr_len);
       
      /*
      for(unsigned i=0; i < 64; ++i)
        std::cout<<std::hex<<(int)((unsigned char*)pdpacket)[i]<<" ";
      */
                  
     // bool invalidPose = false;
      /* Convert from network to host. */
      for (int i = 0; i < SFCT_ARG_BUFFER_LEN; ++i) {
        
        buffer.dvalue = pdpacket[i];
       
        /* Check if we're working on Little Endian platform. */
        /*
        if (1000 != htonl(1000)){
          //8-byte double is seen as two 4-byte ints - first flip each int:
          buffer.aulvalue[0] = ntohl(buffer.aulvalue[0]);
          buffer.aulvalue[1] = ntohl(buffer.aulvalue[1]);
          //swap the two ints
          ulbuffer = buffer.aulvalue[0];
          buffer.aulvalue[0] = buffer.aulvalue[1];
          buffer.aulvalue[1] = ulbuffer;
        }
        */
        
        pdpacket[i] = buffer.dvalue;
        
        //std::cout<<buffer.aulvalue[0]<<"|"<<buffer.aulvalue[1]<<" ";
        
      }
      
      // Check to see if all the data we got is zeros
      // This means that the vicon system lost track of the vehicle, so 
      // we shouldn't output anything
  
      bool invalidPose = pdpacket[1] == 0.0;  // start at idx 1 to skip time data at idx 0
      int i = 2;
      
      //std::cout<<(invalidPose ? "i" : "v");
      
      while(invalidPose && i < SFCT_ARG_BUFFER_LEN)
      {
        if(i == 4)  // quaternion scalar component
          invalidPose &= pdpacket[i] == 1.0;
        else
          invalidPose &= pdpacket[i] == 0.0;
          
        //std::cout<<(invalidPose ? " i" : " v");
          
        ++i;
      }
      
      //std::cout<<std::endl;
      if(invalidPose) 
        continue;
      
      //std::cout<<"Time: "<<pdpacket[0]<<std::endl;
      
      //debug_out<<pdpacket[0]<<", "<<ros::Time::now().toSec()<<"\n";
      
      delayAcceptable = true;
      
      double delay = ros::Time::now().toSec() - pdpacket[0];
      double median, MAD;
      timing_buffer.push_back(delay);
      
      if(timing_buffer.size() == timing_buffer.capacity())  // we've filled the buffer, time to do median filtering
      {
        std::copy(timing_buffer.begin(), timing_buffer.end(), buffer_vector.begin());
        std::nth_element(buffer_vector.begin(), buffer_vector.begin()+TIMING_BUFFER_MEDIAN_INDEX, buffer_vector.end());
        median = buffer_vector[TIMING_BUFFER_MEDIAN_INDEX];
        for(unsigned i=0; i < buffer_vector.size(); ++i)
          buffer_vector[i] = std::fabs(buffer_vector[i] - median);
          
        std::nth_element(buffer_vector.begin(), buffer_vector.begin()+TIMING_BUFFER_MEDIAN_INDEX, buffer_vector.end());
        MAD = buffer_vector[TIMING_BUFFER_MEDIAN_INDEX];
        
        if(delay-median < maxAllowedDelay)
          delayAcceptable = true;
        else if(delay-median > delayFactor*MAD)
          delayAcceptable = false;
      }
      
      ros::Time nowTime = ros::Time::now();
      
      if(delayAcceptable)
      {
        poseMsg.header.stamp = nowTime;
        poseMsg.header.frame_id = "vicon_world";
        poseMsg.pose.position.x = pdpacket[1] / 1000.0;  // convert from mm to m
        poseMsg.pose.position.y = pdpacket[2] / 1000.0;
        poseMsg.pose.position.z = pdpacket[3] / 1000.0;
        poseMsg.pose.orientation.w = pdpacket[4];
        poseMsg.pose.orientation.x = pdpacket[5];
        poseMsg.pose.orientation.y = pdpacket[6];
        poseMsg.pose.orientation.z = pdpacket[7];
        
        poseCovMsg.header.stamp = nowTime;
        poseCovMsg.header.frame_id = "vicon_world";
        poseCovMsg.pose.pose = poseMsg.pose;
        poseCovMsg.pose.covariance[0] = posNoise;
        poseCovMsg.pose.covariance[7] = posNoise;
        poseCovMsg.pose.covariance[14] = posNoise;
        poseCovMsg.pose.covariance[21] = rotNoise;
        poseCovMsg.pose.covariance[28] = rotNoise;
        poseCovMsg.pose.covariance[35] = rotNoise;
        
        tf::poseMsgToTF(poseMsg.pose, transform);
        
        poseQueue.push(poseMsg);
        poseCovQueue.push(poseCovMsg);
        transformQueue.push(transform);
        
        if((int)poseQueue.size() > publishLag) // all queues will be the same size so only check poseQueue
        {
          if(nowTime > lastPublishTime + publishPeriod)  // actually publish
          {
            pub.publish(poseQueue.front());
            pubCov.publish(poseCovQueue.front());
            broadcaster.sendTransform(tf::StampedTransform(transformQueue.front(), nowTime, "vicon_world", "vicon_pose"));
            lastPublishTime = nowTime;
          }
            
          poseQueue.pop();
          poseCovQueue.pop();
          transformQueue.pop();
          
          ROS_ASSERT((int)poseQueue.size() == publishLag);
        }
      }
      else
        std::cout<<"Got delayed message, difference from window median: "<<delay-median<<"  MAD: "<<MAD<<std::endl;
      
    }  //end of "if there is data on port"

  }  // end of big while loop


  /* Close the socket used. */
#ifdef _WIN32
  closesocket(uisocket);
  WSACleanup();
#else
  close(uisocket);
#endif
  
  free(output_data);
  free(pdpacket);
  
  return 0;
};

