#include <ros/ros.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc,char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh ;
   /* image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);*/

    cv::Mat image =cv::imread("/home/tanqingge/catkin_ws/src/test_opencv/SimImg/frame0000.jpg");
    if (image.empty())
    {
        printf("error open\n");
    }
    /*sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ros::Rate loop_rate(5);
    while (nh.ok()) 
    {

    }*/
    cv::imshow("test",image);
    cv::waitKey(3000);
    cv::destroyWindow("test");
    ros::spin();
}