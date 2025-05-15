#include "ros/ros.h"
#include "include/laser_ros.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "line_detector_node");

    // Create ROS node handles (global and private)
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Instantiate the LaserDetector object
    line_detection::LaserDetector laser_detector(nh, nh_private);

    return 0;
}
