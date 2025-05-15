#ifndef _LASER_ROS_H
#define _LASER_ROS_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include "line_detect.h"

namespace line_detection
{
    class LaserDetector
    {
        public:
        LaserDetector(ros::NodeHandle&,ros::NodeHandle&);
        ~LaserDetector();
        void startgame();

    private:
        void publishMarkerMsg(const std::vector<gline> &, visualization_msgs::Marker &marker_msg);
        void load_params();
        void compute_bearing(const sensor_msgs::LaserScan::ConstPtr&);
        void scanValues (const sensor_msgs::LaserScan::ConstPtr&);


    private:
        bool com_bearing_flag;
        bool show_lines_;
        double m_startAng;
        double m_AngInc;
        line_detection::line_detector line_detector;

        std::string frame_id_;
        std::string scan_topic_;

        std::vector<gline> m_gline;
        std::vector<line_segment> m_line_segment;

        ros::NodeHandle nh_;
        ros::NodeHandle nh_local_;
        ros::Subscriber scan_subscriber_;
        ros::Publisher line_publisher_;
        ros::Publisher marker_publisher_;

    };
}
#endif