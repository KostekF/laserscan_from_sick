#ifndef __laserscanFromSick_h
#define __laserscanFromSick_h


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <iostream>
#include <sensor_msgs/point_cloud2_iterator.h>
#include<cmath>
#include <pcl_ros/point_cloud.h> //supports message passing with PCL native data types. 
//This header allows you to publish and subscribe pcl::PointCloud<T> objects as ROS messages.
#include <pcl/common/projection_matrix.h>
#include <image_transport/image_transport.h>



#include <pcl/io/pcd_io.h>
#include <sensor_msgs/LaserScan.h>
#include <string>

constexpr double cloudCols=924;
constexpr double clourRows=24;
class LaserscanFromSick
{
public:
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input);
 void createLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud);
 
 LaserscanFromSick();
 
private:
    float findSmallestValueInCol(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptr_cloud, int numOfCol);
    
private:
    float range_min_;                    ///< Stores the current minimum range to use[m]
    float range_max_;                    ///< Stores the current maximum range to use[m]
    float angle_min_;//< start angle of the scan [rad] 
    float angle_max_; //< end angle of the scan [rad]
    float angle_increment_; //angular distance between measurements [rad]
    
    float scan_time_;// time between scans [seconds]
    
    float height_min_;                   ///< Stores the current minimum height to use[m]
    float height_max_;                   ///< Stores the current maximum range to use[m]
    
    
  ros::NodeHandle nh_,private_nh_{"~"};
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher scan_pub_; //scan message publisher


  sensor_msgs::LaserScanPtr scan_msg_;  /// Published scan message
    
};

#endif