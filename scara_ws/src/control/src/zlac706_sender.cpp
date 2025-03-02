#include "ros/ros.h"
#include "std_msgs/String.h"

// Declare a global publisher
ros::Publisher pub;

// Callback function that will be called every time a new message is published on the subscribed topic
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    // Log the received message
    ROS_INFO("Received message: [%s]", msg->data.c_str());

    // Create a new message to send
    std_msgs::String new_msg;
    new_msg.data = msg->data; // Copy the received message data

    // Publish the new message to the "zlac706_receiver/chatter" topic
    pub.publish(new_msg);
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "zlac706_sender");
    ros::NodeHandle nh;

    // Create a subscriber for the "zlac706_sender/chatter" topic
    ros::Subscriber chatter_sub = nh.subscribe("zlac706_receiver/chatter", 1000, chatterCallback);

    // Create a publisher for the "zlac706_receiver/chatter" topic
    pub = nh.advertise<std_msgs::String>("zlac706_sender/chatter", 1000);

    // Spin to continuously call the callback function when a new message is received
    ros::spin();

    return 0;
}
