#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h" // For motor1 position, speed, current
#include "can_reader.h"
#include <sstream>
#include <linux/can.h>

int32_t pulse_hex_to_int(uint32_t value) {
    if (value >= 0x80000000) {
        value -= (1 << 32);  // Convert to a signed 32-bit integer
    }
    return static_cast<int32_t>(value); // Return as signed integer
}

double pulse_to_degree(int32_t pulses) {
    return (360.0 / 10000.0) * pulses; // Convert pulses to degrees
}

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "zlac706_receiver");
    ros::NodeHandle nh;

    // Create publisher for motor1 current position on "motor1_position"
    ros::Publisher motor1_position_pub = nh.advertise<std_msgs::Float64>("motor1/position", 1000);

    // Create publisher for motor1 speed on "motor1_speed"
    ros::Publisher motor1_speed_pub = nh.advertise<std_msgs::Float64>("motor1/speed", 1000);

    // Create publisher for motor1 current on "motor1_current"
    ros::Publisher motor1_current_pub = nh.advertise<std_msgs::Float64>("motor1/current", 1000);

    // Create an instance of CanBusReader
    CanReader canReader("can1"); // Replace with your CAN interface name

    try {
        canReader.open();
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to open CAN interface: %s", e.what());
        return 1;
    }

    while (ros::ok())
    {
        can_frame frame;
        if (canReader.read(frame)) {
            // Check if data[3] indicates position (20) or speed/current (21)
            if (frame.data[1] == 0xfe) // if heartbeat
            {
            
		    if (frame.data[3] == 0x20) {
		        // Combine data[4] to data[7] to form the position value
		        uint32_t position_raw = (frame.data[4] << 24) | (frame.data[5] << 16) | (frame.data[6] << 8) | frame.data[7];
		        
		        // Convert raw position to signed int and then to degrees
		        int32_t position_signed = pulse_hex_to_int(position_raw);
		        std_msgs::Float64 motor1_position;
		        motor1_position.data = pulse_to_degree(position_signed); // Convert to degrees
		        
		        motor1_position_pub.publish(motor1_position);

		    } else if (frame.data[3] == 0x21) {
		        // Extract and combine speed (data[6] and data[7])
		        int16_t speed_raw = (frame.data[6] << 8) | frame.data[7]; // Speed as a 16-bit integer

		        // Convert speed using the formula
		        std_msgs::Float64 motor1_speed;
		        motor1_speed.data = ((static_cast<double>(speed_raw) / 8192.0) * 3000.0); // Convert to RPM
		        
		        // Extract and combine current (data[4] and data[5])
		        int16_t current_raw = (frame.data[4] << 8) | frame.data[5]; // Current as a 16-bit integer

		        // Convert current using the formula
		        std_msgs::Float64 motor1_current;
		        motor1_current.data = static_cast<double>(current_raw) / 100.0; // Convert to Amps

		        motor1_speed_pub.publish(motor1_speed);
		        motor1_current_pub.publish(motor1_current);
		    }
	    }
        }

        ros::spinOnce();  // Keep ROS responsive without sleeping
    }

    canReader.close();

    return 0;
}
