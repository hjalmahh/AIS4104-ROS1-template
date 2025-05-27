#include "ros/ros.h"
#include "laser_ros.h"


#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sub/line_mapping.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_combined_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Start line detection
    line_detection::LaserDetector detector(nh, nh_private);

    // Start map building
    LineMapBuilder mapper;

    ros::spin();
    return 0;
}
