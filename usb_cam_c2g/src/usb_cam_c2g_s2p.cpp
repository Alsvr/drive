//#include "usb_cam_c2g_s2p.h"

#include "ros/ros.h" 
#include <sensor_msgs/Image.h>
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h" 
#include "boost/thread.hpp" 
#include "image_transport/image_transport.h"    
   
#include <opencv2/imgproc/imgproc.hpp>    
#include <opencv2/highgui/highgui.hpp>    
#include <iostream> 

using namespace cv;
using namespace std;
/*
  void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)
  {
    cv::Mat img;
    try
    {
      //img = cv_bridge::toCvShare(tem_msg, "mono8")->image;
     // cv::imshow("node_a listener from node_b", cv_bridge::toCvShare(tem_msg, "mono8")->image);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
    }
  }
*/
int main(int argc, char **argv)
{
    //ros::init(argc, argv, "usb_cam_c2g_node ");
    //ros::init(argc, argv, "usb_cam_c2g_node ");
    //ros::NodeHandle  n;
    
   //ROS_INFO("IMAGE \n");
  
    // image_transport::ImageTransport it(n);
    //  image_transport::Subscriber sub;
    //sub= it.subscribe("camera/image", 10, imageCallback);

  return 0;

}
