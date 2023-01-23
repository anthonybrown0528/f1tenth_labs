#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>

#include <string>
#include <vector>
#include <algorithm>

#include "sensor_processor.h"

class Node {
    public:

        Node(int argc, char** argv, std::string name) {

            // Initialize ROS node
            ros::init(argc, argv, name);

            // Declare ROS topic names
            std::string farthest_point_topic;
            std::string nearest_point_topic;

            std::string subscribe_topic;

            // Queue size for publisher
            int publisher_queue_size;
            int subscriber_queue_size;

            // Retrieve ROS parameters
            node_handle.param<std::string>("farthest_point", farthest_point_topic, "farthest_point");
            node_handle.param<std::string>("nearest_point", nearest_point_topic, "closest_point");
            node_handle.param<std::string>("subscribe_topic", subscribe_topic, "scan");
            
            node_handle.param("publisher_queue_size", publisher_queue_size, 1000); 
            node_handle.param("subscriber_queue_size", subscriber_queue_size, 1); 

            // Construct the node handle
            node_handle = ros::NodeHandle();

            // Construct publishers and subscribers
            scan_farthest_publisher = node_handle.advertise<std_msgs::Float64>(farthest_point_topic, publisher_queue_size);
            scan_nearest_publisher = node_handle.advertise<std_msgs::Float64>(nearest_point_topic, publisher_queue_size);
            
            scan_subscriber = node_handle.subscribe<sensor_msgs::LaserScan>(subscribe_topic, subscriber_queue_size, &Node::SubscribeCallback, this);
        
            // Spin ROS node
            ros::spin();
        }

        void SubscribeCallback(const sensor_msgs::LaserScanConstPtr &msg) {

            // Messages to publish
            std_msgs::Float64 nearest_range_msg;
            std_msgs::Float64 farthest_range_msg;

            // Range of scans
            std::vector<float> range;

            // Calculate the minimum and maximum ranges
            range = SensorProcessor::ProcessSensor(msg->ranges);

            // Store results in ROS messages
            nearest_range_msg.data = range[0];
            farthest_range_msg.data = range[1];

            // Publish messages
            scan_nearest_publisher.publish(nearest_range_msg);
            scan_farthest_publisher.publish(farthest_range_msg);
        }

    private:

        // Declare node handle
        ros::NodeHandle node_handle;

        // Declare publishers and subscribers
        ros::Publisher scan_farthest_publisher;
        ros::Publisher scan_nearest_publisher;

        ros::Subscriber scan_subscriber;
};



int main(int argc, char** argv) {
    
    // Construct ROS node and spin it
    Node node(argc, argv, "sensor_processor_node");
    return 0;
}
