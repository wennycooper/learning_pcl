#include <ros/ros.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;
//using namespace pcl;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_viewer");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i=0; i<cloud->points.size(); ++i) {
    cloud->points[i].x = i;
    cloud->points[i].y = 0;
    cloud->points[i].z = i;
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;


  pcl::visualization::CloudViewer viewer("PCL viewer");
  viewer.showCloud(cloud);
  
  ros::spin();
  return 0;
}
