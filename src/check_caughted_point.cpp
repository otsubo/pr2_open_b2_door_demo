#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/Segment.h>
#include <jsk_recognition_msgs/SegmentArray.h>
//#include <jsk_recognition_msgs/MarkerArrayStamped.h>
#include <pr2_open_b2_door_demo/MarkerArrayStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <time.h>
#include <vector>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher dist_pub;
ros::Publisher nearest_segment_pub;
boost::mutex mutex_;



void calculate(
               const jsk_recognition_msgs::BoundingBox::ConstPtr& box,
               const jsk_recognition_msgs::SegmentArray::ConstPtr& segment_array)
{
  //init input topic
  std::cout<<"test1"<<std::endl;

  //std::count<<"calculate"<<std::endl;
  jsk_recognition_msgs::BoundingBox new_bbox_msg(*box);
  //visualization_msgs::MarkerArray ma;
  jsk_recognition_msgs::SegmentArray new_segment_msg(*segment_array);
  jsk_recognition_msgs::Segment nearest_segment;
  float dist;
  std::vector<float> dist_vec;
  float min_dist;

  float b_x = new_bbox_msg.pose.position.x;
  float b_y = new_bbox_msg.pose.position.y;
  float b_z = new_bbox_msg.pose.position.z;
  float s_x;
  float s_y;
  float s_z;

  //calculate min distance and return nearest edge
  for(size_t i = 0; i < new_segment_msg.segments.size() ; i++) {
    min_dist = -100;
    s_x = (new_segment_msg.segments[i].start_point.x + new_segment_msg.segments[i].end_point.x) / 2;
    s_y = (new_segment_msg.segments[i].start_point.y + new_segment_msg.segments[i].end_point.y) / 2;
    s_z = (new_segment_msg.segments[i].start_point.z + new_segment_msg.segments[i].end_point.z) / 2;;
    dist = sqrt(pow(s_x - b_x, 2.0) + pow(s_y - b_y, 2.0) + pow(s_z - b_z, 2.0));
    if (dist < min_dist)
      min_dist = dist;
    nearest_segment = new_segment_msg.segments[i];
    std::cout<<"segment"<<i<<" is calculated"<<std::endl;
  }
  // //Publish the data
  //dist_pub.publish (min_dist);
  nearest_segment_pub.publish (nearest_segment);
  std::cout<<"test2"<<std::endl;
}

// void test_calculate(
//                const jsk_recognition_msgs::BoundingBox::ConstPtr& box,
//                const pr2_open_b2_door_demo::MarkerArrayStamped::ConstPtr& marker_array)
// {
//   // //init input topic
//   // std::cout<<"test1"<<std::endl;

//   std::cout<<"calculate"<<std::endl;
//   // jsk_recognition_msgs::BoundingBox new_bbox_msg(*box);
//   // //visualization_msgs::MarkerArray ma;
//   // pr2_open_b2_door_demo::MarkerArrayStamped new_marker_msg(*marker_array);
//   // visualization_msgs::Marker nearest_marker;
//   // float dist;
//   // std::vector<float> dist_vec;
//   // float min_dist;

//   // float b_x = new_bbox_msg.pose.position.x;
//   // float b_y = new_bbox_msg.pose.position.y;
//   // float b_z = new_bbox_msg.pose.position.z;
//   // float m_x;
//   // float m_y;
//   // float m_z;

//   // //calculate min distance and return nearest edge
//   // for(size_t i = 0; i < new_marker_msg.markers.size() ; i++) {
//   //   min_dist = -100;
//   //   m_x = new_marker_msg.markers[i].pose.position.x;
//   //   m_y = new_marker_msg.markers[i].pose.position.y;
//   //   m_z = new_marker_msg.markers[i].pose.position.z;
//   //   dist = sqrt(pow(m_x - b_x, 2.0) + pow(m_y - b_y, 2.0) + pow(m_z - b_z, 2.0));
//   //   if (dist < min_dist)
//   //     min_dist = dist;
//   //   nearest_marker = new_marker_msg.markers[i];
//   //   std::cout<<"marker"<<i<<" is calculated"<<std::endl;
//   // }
//   // // //Publish the data
//   // //dist_pub.publish (min_dist);
//   // nearest_marker_pub.publish (nearest_marker);
//   // std::cout<<"test2"<<std::endl;
//   pr2_open_b2_door_demo::MarkerArrayStamped new_marker_msg(*marker_array);
//   visualization_msgs::Marker nearest_marker;
//   nearest_marker = new_marker_msg.markers[0];
//   nearest_marker_pub.publish (nearest_marker);
// }



int
main(int argc, char** argv)
{

  boost::mutex::scoped_lock lock(mutex_);
  //Initialize ROS
  std::cout<<"initialized"<<std::endl;
  ros::init (argc, argv, "check_node");
  ros::NodeHandle nh;
  typedef message_filters::sync_policies::ApproximateTime<jsk_recognition_msgs::BoundingBox, jsk_recognition_msgs::SegmentArray> MySyncPolicy;

  ros::Rate loop_rate(20);
  while(ros::ok()){

    //message filter
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> box_sub(nh, "input_box",1);
    message_filters::Subscriber<jsk_recognition_msgs::SegmentArray> segment_sub(nh, "input_segment",1);


    message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(5000), box_sub, segment_sub);
    sync_.registerCallback(boost::bind(&calculate, _1, _2));
    std::cout<<"synchronized"<<std::endl;
    //Create ROS publishers for output
    //dist_pub = nh.advertise<geometry_msgs::PointStamped> ("distance", 1);
    nearest_segment_pub = nh.advertise<jsk_recognition_msgs::Segment> ("output_marker", 1);
    std::cout<<"pub"<<std::endl;
    
    //Spin
  // ros::Rate loop_rate(20);
  // while(ros::ok()){
    ros::spinOnce();
    std::cout<<"spin"<<std::endl;
    loop_rate.sleep();
  }
  // ros::spin();
  // std::cout<<"spin"<<std::endl;
  
}
 


