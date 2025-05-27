#include "line_mapping.h"

LineMapBuilder::LineMapBuilder()
    : tf_buffer_(), tf_listener_(tf_buffer_)
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("resolution", resolution_, 0.1f);
    nh_private.param("width", width_, 500);
    nh_private.param("height", height_, 500);
    nh_private.param<std::string>("map_frame", map_frame_, "map");
    nh_private.param<std::string>("base_frame", base_frame_, "laser_frame");

    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("line_map", 1);
    marker_sub_ = nh.subscribe("/publish_linemarkers", 10, &LineMapBuilder::markerCallback, this);


    map_.info.resolution = resolution_;
    map_.info.width = width_;
    map_.info.height = height_;
    map_.info.origin.position.x = -width_ * resolution_ / 2.0;
    map_.info.origin.position.y = -height_ * resolution_ / 2.0;
    map_.info.origin.orientation.w = 1.0;
    map_.header.frame_id = map_frame_;
    map_.data.resize(width_ * height_, -1);
}

void LineMapBuilder::markerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    // 🔍 Debug: Check if callback is triggered and how many points there are
    ROS_INFO("Marker callback triggered with %lu points in frame [%s]", msg->points.size(), msg->header.frame_id.c_str());

    geometry_msgs::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException& ex) {
        ROS_WARN("TF error: %s", ex.what());
        return;
    }

    for (const geometry_msgs::Point& p : msg->points) {
        geometry_msgs::PointStamped in, out;
        in.point = p;
        in.header = msg->header;

        try {
            tf2::doTransform(in, out, transform);
            int mx = (int)((out.point.x - map_.info.origin.position.x) / resolution_);
            int my = (int)((out.point.y - map_.info.origin.position.y) / resolution_);
            if (mx >= 0 && mx < width_ && my >= 0 && my < height_) {
                map_.data[my * width_ + mx] = 100;
            }
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Point TF error: %s", ex.what());
            continue;
        }
    }

    map_.header.stamp = ros::Time::now();
    map_pub_.publish(map_);

    // 🔍 Debug: Confirm map was published
    ROS_INFO("Published line_map with timestamp %f", map_.header.stamp.toSec());
}

