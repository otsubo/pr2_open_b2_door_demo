#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/Edge.h>
#include <jsk_recognition_msgs/EdgeArray.h>
#include <geometry_msgs/PointStamped.h>
#include <time.h>
#include <vector>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


ros::Publisher dist_pub;
ros::Publisher nearest_edge_pub;
boost::mutex mutex_;



void calculate(
               const jsk_recognition_msgs::BoundingBox::ConstPtr& box,
               const jsk_recognition_msgs::EdgeArray::ConstPtr& edge_array)
{
  //init input topic

  //std::count<<"calculate"<<std::endl;
  jsk_recognition_msgs::BoundingBox new_bbox_msg(*box);
  //visualization_msgs::MarkerArray ma;
  jsk_recognition_msgs::EdgeArray new_edge_msg(*edge_array);
  jsk_recognition_msgs::Edge nearest_edge;
  double dist;
  std::vector<double> dist_vec;
  double min_dist;
  // ba.header = box->header;
  // ba.pose = box->pose;
  // ba.dimensions = box->dimensions;
  //ma.markers.header = marker_array->header;
  //ma.markers = marker_array->markers;

  double b_x = new_bbox_msg.pose.position.x;
  double b_y = new_bbox_msg.pose.position.y;
  double b_z = new_bbox_msg.pose.position.z;
  double e_x;
  double e_y;
  double e_z;
  //ba.header = box->header;
  //ba.box = box->box;
  //ma.header = marker_array->header;
  //ma.markers = marker_array->markers;

  for(size_t i = 0; i < new_edge_msg.edges.size() ; i++) {
    min_dist = -100;
    e_x = new_edge_msg.edges[i].start_point.x;
    e_y = new_edge_msg.edges[i].start_point.y;
    e_z = new_edge_msg.edges[i].start_point.z;
    dist = sqrt(pow(e_x - b_x, 2.0) + pow(e_y - b_y, 2.0) + pow(e_z - b_z, 2.0));
    if (dist < min_dist)
      min_dist = dist;
    nearest_edge = new_edge_msg.edges[i];
  }
  //Publish the data
  dist_pub.publish (min_dist);
  nearest_edge_pub.publish (nearest_edge);
}


int
main(int argc, char** argv)
{

  //boost::mutex::scope_lock lock(mutex_);
  //Initialize ROS
  ros::init (argc, argv, "check_node");
  ros::NodeHandle nh;
  typedef message_filters::sync_policies::ApproximateTime<jsk_recognition_msgs::BoundingBox, jsk_recognition_msgs::EdgeArray> MySyncPolicy;
  //boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_;
  //message filter
  message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> box_sub(nh, "input_box",1);
  message_filters::Subscriber<jsk_recognition_msgs::EdgeArray> edge_sub(nh, "input_edge",1);

  //boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> > sync_ = boost::make_shared<message_filters::Synchronizer<MySyncPolicy> >(100);
  //message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10), box_sub, marker_sub);
  //sync_.connectInput(box_sub, edge_sub);
  //sync_.registerCallback(boost::bind(&calculate, _1, _2));

  message_filters::Synchronizer<MySyncPolicy> sync_(MySyncPolicy(10), box_sub, edge_sub);
  sync_.registerCallback(boost::bind(&calculate, _1, _2));

  //Create ROS publishers for output
  dist_pub = nh.advertise<geometry_msgs::PointStamped> ("distance", 1);
  nearest_edge_pub = nh.advertise<jsk_recognition_msgs::Edge> ("output_edge", 1);
  
  //Spin
  // ros::Rate loop_rate(20);
  // while(ros::ok()){
  //   ros::spineOnce ();
  //   loop_rate.sleep();
  //ros::spine();

  return 0;
}
 


