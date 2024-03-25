/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo Classic SITL
 */

// ROS libraries and dependencies
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// MQTT libraries and dependencies
#include <mqtt/client.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{

    // Initialize ros node and make basic comprobations
    ros::init(argc,argv,"MAVROS_Offboard_Control_Node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0); //Publishing rate faster than 2Hz for setpoint publising

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS is functioning correctly and is connected to MAVROS");

    // Create MQTT client and make basic comprobations
    mqtt::client client("127.0.0.1:1883", "ROS-consumer");
    client.set_timeout(5000);

    ROS_INFO("Connecting to MQTT broker...");
    client.connect();
    ROS_INFO("Connected to MQTT broker");

    client.subscribe("dlstreamer-publisher");
    client.start_consuming(); 
    ROS_INFO("Subscribed to MQTT topic and ready for consumption");

    // Create poses for the offboard mode 
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0; pose.pose.position.y = 0; pose.pose.position.z = 2;

    for(int i = 100; ros::ok() && i>0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Local position setpoint sent");

    // Set offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ROS_INFO("OFFBOARD mode enabled");

    // Set arm command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ROS_INFO("Vehicle armed");

    ROS_INFO("Starting simulation loop");
    mqtt::const_message_ptr messagePointer; // Construct a message pointer to hold an incoming message.
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Starting offboard node and running for 5 seconds");
    while(ros::ok()){

        if(current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0))){

            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                ROS_INFO("Arming vehicle and running arm service for 5 seconds");
            }

        last_request = ros::Time::now();

        }else{

            if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){

                if(arming_client.call(arm_cmd)&&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

            last_request = ros::Time::now();

            }
        }
 
        // Try to consume a message, passing messagePointer by reference.
        // If a message is consumed, the function will return `true`, 
        // allowing control to enter the if-statement body.
        if (client.try_consume_message(&messagePointer)){

            // Construct a string from the message payload.
            std::string messageString = messagePointer -> get_payload_str();
            // Print payload string to console (debugging).
            std::cout << messageString << std::endl;
 
            // Perform processing on the string.
            // This is where message processing can be passed onto different
            // functions for parsing. 
            // Here, we break the loop and exit the program if a `quit` is received.
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}