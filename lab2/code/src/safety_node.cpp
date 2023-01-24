#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Bool.h>

#include "safety.h"

class SafetyNode {
// The class that handles emergency braking
private:
    ros::NodeHandle n;

    // Create ROS subscribers and publishers
    ros::Subscriber scan_subscriber;
    ros::Subscriber odom_subscriber;

    ros::Publisher brake_publisher;
    ros::Publisher brake_bool_publisher;

    Safety safety;

public:
    SafetyNode() : safety(0.5f) {
        std::string scan_topic;
        std::string odom_topic;
        std::string brake_bool_topic;
        std::string brake_topic;

        int publisher_queue_size;
        int subscriber_queue_size;

        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.

        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.

        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        // Retrieve ROS parameters
        n.param<std::string>("scan_topic", scan_topic, "scan");
        n.param<std::string>("odom_topic", odom_topic, "odom");
        n.param<std::string>("brake_bool_topic", brake_bool_topic, "brake_bool");
        n.param<std::string>("brake_topic", brake_topic, "brake");

        n.param("publisher_queue_size", publisher_queue_size, 1000);
        n.param("subscriber_queue_size", subscriber_queue_size, 1);

        // Create ROS subscribers and publishers
        scan_subscriber = n.subscribe<sensor_msgs::LaserScan>(scan_topic, subscriber_queue_size, &SafetyNode::scan_callback, this);
        odom_subscriber = n.subscribe<nav_msgs::Odometry>(odom_topic, subscriber_queue_size, &SafetyNode::odom_callback, this);

        brake_bool_publisher = n.advertise<std_msgs::Bool>(brake_bool_topic, publisher_queue_size);
        brake_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>(brake_topic, publisher_queue_size);
    }

    ~SafetyNode() {
        n.shutdown();
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {

        // Update the measured speed of the vehicle
        safety.UpdateSpeed(odom_msg->twist.twist.linear.x, 0.0f, 0.0f);
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
        std::vector<float> angles;
        bool should_stop;

        std_msgs::Bool brake_bool_msg;
        ackermann_msgs::AckermannDriveStamped brake_msg;

        angles = safety.GenerateAngles(scan_msg->angle_min, scan_msg->angle_max, scan_msg->ranges.size());
        should_stop = safety.SafetyCheck(scan_msg->ranges, angles);

        if(should_stop) {
            brake_bool_msg.data = true;

            brake_msg.header.stamp = ros::Time::now();
            brake_msg.drive.speed = 0.0;

            brake_publisher.publish(brake_msg);
            brake_bool_publisher.publish(brake_bool_msg);

            should_stop = false;
        }
    }

};

int main(int argc, char ** argv) {

    ros::init(argc, argv, "safety_node");

    // Construct ROS node
    SafetyNode sn;

    // Spin ROS node
    ros::spin();
    return 0;
}