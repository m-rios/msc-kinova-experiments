/*
 * filter_node.cpp
 *
 *  Created on: Mar 6, 2019
 *      Author: rik
 */

#include <ros/ros.h>
#include <string>
#include "point_cloud_filters.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class FilterCloud {

  ros::NodeHandle nh_;
  ros::Subscriber point_cloud_subscriber_;
  ros::Publisher filter_publisher_;
  PointCloudFilters point_cloud_filters_;

  tf::TransformListener transform_listener_;
  size_t counter_;

public:
  FilterCloud(ros::NodeHandle nh) : nh_(nh), counter_(0) {
    std::string point_cloud_topic = "/camera/depth_registered/points";
    std::string filter_topic = "/camera/filter/points";

    point_cloud_subscriber_ = nh_.subscribe(point_cloud_topic, 1, &FilterCloud::pointCloudCallback, this);
    filter_publisher_ = nh_.advertise<PointCloud>(filter_topic, 1);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
    
    if (counter_  == 0) {
      transform_listener_.waitForTransform("m1n6s200_link_base",
                                         cloud_msg->header.frame_id,
                                         ros::Time::now(),
                                         ros::Duration(0.5));

      sensor_msgs::PointCloud2 cloud_msg2;
      pcl_ros::transformPointCloud(std::string("m1n6s200_link_base"), *cloud_msg, cloud_msg2, transform_listener_);
      PointCloud cloud;
      pcl::fromROSMsg(cloud_msg2, cloud);

      float min_x = -0.4;
      float max_x = 0.4;
      float min_y = -0.5;
      float max_y = 0.5;

      point_cloud_filters_.Filter(cloud, min_x, max_x, "x");
      point_cloud_filters_.Filter(cloud, min_y, max_y, "y");
      filter_publisher_.publish(cloud); 
    }
    
    ++counter_;
    
    if (counter_ == 2) {
        counter_ = 0;
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv, std::string("filter_node"));
  ros::NodeHandle nh;
  FilterCloud filter_cloud(nh);

  ros::spin();
}



