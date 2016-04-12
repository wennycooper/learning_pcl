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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Pcl_viewer
{
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub_;
  boost::shared_ptr<pcl::visualization::CloudViewer> viewer;

public:
  Pcl_viewer()
  {
    viewer = createViewer();
    cloud_sub_= nh_.subscribe<PointCloud>("/camera/depth/points", 1, &Pcl_viewer::cloudCb, this);
  }

  ~Pcl_viewer()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
  }

  void cloudCb(const PointCloud::ConstPtr&  cloud)
  {
    viewer->showCloud(cloud);

  }

  // Creates, initializes and returns a new viewer.
  boost::shared_ptr<pcl::visualization::CloudViewer>
  createViewer()
  {
    boost::shared_ptr<pcl::visualization::CloudViewer> v(new pcl::visualization::CloudViewer("PCL viewer"));
    //v->registerKeyboardCallback(keyboardEventOccurred);
 
    return (v);
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_viewer");
  Pcl_viewer pv;
  ros::spin();
  return 0;
}
