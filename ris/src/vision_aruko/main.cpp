#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <ris_msgs/Hello.h>
#include <sensor_msgs/CompressedImage.h>

std::atomic<bool> run = true;
boost::shared_ptr<ros::NodeHandle> node = nullptr;

void hello_message_received(boost::shared_ptr<ris_msgs::Hello> msg)
{
    ROS_INFO("/hello_subscriber received msg: %s", msg->text.c_str());
}

void image_receive(boost::shared_ptr<sensor_msgs::CompressedImage> msg)
{
    ROS_INFO("Number of pixels: %lu", msg->data.size());
}

int main(int argc, char **args)
{
    ros::init(argc, args, "hello_subscriber");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_subscriber");
    ROS_INFO("/hello_subscriber node started");

    ros::Subscriber subscriber = node->subscribe("/hello_publisher/message", 1u, &hello_message_received);
    ros::Subscriber image_sub = node->subscribe("/csi_cam_0/image_raw/compressed", 1u, &image_receive);
    ros::spin();
    ros::shutdown();
    return 0;
}
