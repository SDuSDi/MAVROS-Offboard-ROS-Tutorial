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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>

// MQTT library
#include <mqtt/async_client.h>
// JSON library
#include <nlohmann/json.hpp>
using json = nlohmann::json; // for convenience
// Callback file
#include "classes.cpp"

mavros_msgs::State current_state;
nav_msgs::Odometry current_odometry;

// MQTT message storage and access
mqtt::async_client *client = NULL;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg){
    current_odometry = *msg;
}

int main(int argc, char **argv)
{

    // Initialize ros node
    ros::init(argc,argv,"MAVROS_Offboard_Control_Node");
    ros::NodeHandle nh;

    // Subscribe to state and odometry
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Subscriber odometry_sub = nh.subscribe<nav_msgs::Odometry>("mavros/odometry/in",10, odometry_cb);

    // Publish to setpoint and velocity
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",10);

    // Create service clients for arming and set mode
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0); //Publishing rate faster than 2Hz for setpoint publising

    // Wait for connection to be established
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("ROS is functioning correctly and is connected to MAVROS");

    // Create MQTT client and connect
    client = new mqtt::async_client("127.0.0.1:1883", "ROS-consumer", 0);

    // Set connection options
	mqtt::connect_options connOpts;
	connOpts.set_clean_session(false);

    // Create callback object and set callback for use
    callback mqtt_cb(*client, connOpts);
    client->set_callback(mqtt_cb);

    client->start_consuming();
    ROS_INFO("MQTT client ready for consumption");

    ROS_INFO("Connecting to MQTT broker...");
    client->connect(connOpts);
    ROS_INFO("Connected to MQTT broker");

    while(!client->is_connected()){
        ROS_INFO("Failed to connect to MQTT broker. Retrying...");
        rate.sleep();
    }

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
    ROS_INFO("Vehicle arm command prepared");

    ROS_INFO("Starting simulation loop");
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Starting offboard node and running for 5 seconds");
    while(ros::ok()){

        if(current_state.mode!="OFFBOARD"&&(ros::Time::now()-last_request>ros::Duration(5.0))){

            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
                ROS_INFO("Arming vehicle and running arm service for 5 seconds");
            }

        last_request = ros::Time::now();
        local_pos_pub.publish(pose);

        }else{

            if(!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){

                if(arming_client.call(arm_cmd)&&arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }

            last_request = ros::Time::now();
            local_pos_pub.publish(pose);

            }else{

                /*std::cout<<"X_max: "<<data["objects"][0]["detection"]["bounding_box"]["x_max"];
                std::cout<<" ; X_min: "<<data["objects"][0]["detection"]["bounding_box"]["x_min"];
                std::cout<<" ; Y_max: "<<data["objects"][0]["detection"]["bounding_box"]["y_max"];
                std::cout<<" ; Y_min: "<<data["objects"][0]["detection"]["bounding_box"]["y_min"]<<std::endl;*/

                pose.pose.position.x = std::round(current_odometry.pose.pose.position.x*10)/10; 
                pose.pose.position.y = std::round(current_odometry.pose.pose.position.y*10)/10; 
                pose.pose.position.z = 2;

                if(data["objects"][0]["detection"]["bounding_box"]["y_max"]>0.9){
                    pose.pose.position.x = current_odometry.pose.pose.position.x - 0.1; 

                }else{if(data["objects"][0]["detection"]["bounding_box"]["y_min"]<0.1){  
                    pose.pose.position.x = current_odometry.pose.pose.position.x + 0.1; 

                    }
                }

                if(data["objects"][0]["detection"]["bounding_box"]["x_max"]>0.9){
                    pose.pose.position.x = current_odometry.pose.pose.position.y - 0.1; 

                }else{if(data["objects"][0]["detection"]["bounding_box"]["x_min"]<0.1){  
                    pose.pose.position.x = current_odometry.pose.pose.position.y + 0.1; 

                    }
                }

                local_pos_pub.publish(pose);

            }
        }

        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}