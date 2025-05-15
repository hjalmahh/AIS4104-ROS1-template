#include <ros/ros.h>
#include <ros/console.h>
//#include "opencv_thing.h"
#include <ris_msgs/Hello.h>
#include <sensor_msgs/CompressedImage.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>


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
//--------------------------------
void motor_callback(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Motor Throttle (Actual): %.2f", msg->data);
}

void servo_callback(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Steering Angle (Actual): %.2f", msg->data);
}


double last_x = 0, last_y = 0;
double last_vx = 0, last_vy = 0;
const double threshold = 0.01;
void odom_receive(const nav_msgs::Odometry::ConstPtr& msg)
{


    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double vx = msg->twist.twist.linear.x; //mulig overflødig logge bare posisjons endring
    double vy = msg->twist.twist.linear.y; //mulig overflødig logge bare posisjons endring

    bool position_changed = std::abs(x - last_x) > threshold || std::abs(y - last_y) > threshold;
    bool velocity_changed = std::abs(vx - last_vx) > threshold || std::abs(vy - last_vy) > threshold;

    if (position_changed || velocity_changed)
    {
        ROS_INFO("Odom Position: x=%.2f y=%.2f | Velocity: vx=%.2f vy=%.2f", x, y, vx, vy);
        last_x = x;
        last_y = y;
        last_vx = vx;
        last_vy = vy;
    }
}



//oppdatere en gang i sekundet
ros::Time last_log_time;

void imu_receive(const sensor_msgs::Imu::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();
    if ((now - last_log_time).toSec() < 1.0)
        return;  // Not time yet

    last_log_time = now;

    // Convert quaternion to RPY
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("IMU Data:");
    ROS_INFO("  Accel     ax=%.2f ay=%.2f az=%.2f", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    ROS_INFO("  Gyro      gx=%.2f gy=%.2f gz=%.2f", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ROS_INFO("  Orient    qx=%.2f qy=%.2f qz=%.2f qw=%.2f", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    ROS_INFO("  RPY       roll=%.2f pitch=%.2f yaw=%.2f", roll, pitch, yaw);
    ROS_INFO("-------------------------------");
}



void lidar_receive(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    float closest = std::numeric_limits<float>::infinity();
    for (float range : msg->ranges)
    {
        if (std::isfinite(range) && range < closest)
        {
            closest = range;
        }
    }

    ROS_INFO("LiDAR: angle_min=%.2f, angle_max=%.2f | Closest object: %.2f m",
             msg->angle_min, msg->angle_max, closest);
}


int main(int argc, char **args)
{
    ros::init(argc, args, "hello_subscriber");
    ros::start();
    node = boost::make_shared<ros::NodeHandle>("hello_subscriber");
    ROS_INFO("/hello_subscriber node started");

   // ros::Subscriber subscriber = node->subscribe("/hello_publisher/message", 1u, &hello_message_received);
    //ros::Subscriber image_sub = node->subscribe("/csi_cam_0/image_raw/compressed", 1u, &image_receive);
    //ros::Subscriber odom_sub = node->subscribe("/odom_raw", 10, odom_receive);
    ros::Subscriber lidar_sub = node->subscribe("/scan", 10, lidar_receive);
    //ros::Subscriber imu_sub = node->subscribe("/imu", 10, imu_receive);
    //ros::Subscriber motor_sub = node->subscribe("/motor_actual", 10, motor_callback);
    //ros::Subscriber servo_sub = node->subscribe("/servo_actual", 10, servo_callback);





    ros::spin();
    ros::shutdown();
    return 0;
}
