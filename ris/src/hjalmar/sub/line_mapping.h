#ifndef LINE_SEGMENT_LINE_MAPPING_H
#define LINE_SEGMENT_LINE_MAPPING_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class LineMapBuilder {
public:
    LineMapBuilder();

private:
    void markerCallback(const visualization_msgs::Marker::ConstPtr& msg);

    ros::Subscriber marker_sub_;
    ros::Publisher map_pub_;
    nav_msgs::OccupancyGrid map_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string map_frame_;
    std::string base_frame_;
    float resolution_;
    int width_;
    int height_;
};

#endif // LINE_SEGMENT_LINE_MAPPING_H
